#! /usr/bin/env python
import pdb
import rosnode
from typing import Optional, Callable, List, Tuple

import actionlib
import rospy
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from arm_robots.robot import MoveitEnabledRobot
from franka_gripper.msg import GraspAction, GraspGoal, MoveAction, MoveGoal, HomingAction, HomingGoal, StopAction, \
    StopGoal
from franka_msgs.msg import ErrorRecoveryAction, ErrorRecoveryGoal, FrankaState
from rosgraph.names import ns_join
from sensor_msgs.msg import JointState


class Panda(MoveitEnabledRobot):

    def __init__(self, robot_namespace: str = '', force_trigger: float = -0.0, **kwargs):
        MoveitEnabledRobot.__init__(self,
                                    robot_namespace=robot_namespace,
                                    robot_description=ns_join(robot_namespace, 'robot_description'),
                                    arms_controller_name=None,
                                    force_trigger=force_trigger,
                                    **kwargs)
        self.panda_1 = 'panda_1'
        self.panda_2 = 'panda_2'
        self.panda_1_EE = 'panda_1_link_planning_EE'
        self.panda_2_EE = 'panda_2_link_planning_EE'
        self.display_goals = False

        active_nodes = rosnode.get_node_names()
        if ns_join('/', f'{self.panda_1}/franka_gripper') in active_nodes:
            self.gripper_1 = PandaGripper(self.robot_namespace, self.panda_1)
        if ns_join('/', f'{self.panda_2}/franka_gripper') in active_nodes:
            self.gripper_2 = PandaGripper(self.robot_namespace, self.panda_2)

    def follow_arms_joint_trajectory(self, trajectory: JointTrajectory, stop_condition: Optional[Callable] = None,
                                     group_name: Optional[str] = None):
        if self.execute:
            move_group = self.get_move_group_commander(group_name)
            plan_msg = RobotTrajectory()
            plan_msg.joint_trajectory = trajectory
            move_group.execute(plan_msg)

    def send_joint_command(self, joint_names: List[str], trajectory_point: JointTrajectoryPoint) -> Tuple[bool, str]:
        pass
    # TODO: Add control mode setter/getter.


class PandaGripper:
    def __init__(self, robot_ns, arm_id):
        self.gripper_ns = ns_join(robot_ns, f'{arm_id}/franka_gripper')
        self.grasp_client = actionlib.SimpleActionClient(ns_join(self.gripper_ns, 'grasp'), GraspAction)
        self.move_client = actionlib.SimpleActionClient(ns_join(self.gripper_ns, 'move'), MoveAction)
        self.homing_client = actionlib.SimpleActionClient(ns_join(self.gripper_ns, 'homing'), HomingAction)
        self.stop_client = actionlib.SimpleActionClient(ns_join(self.gripper_ns, 'stop'), StopAction)
        self.grasp_client.wait_for_server()
        self.move_client.wait_for_server()
        self.homing_client.wait_for_server()
        self.stop_client.wait_for_server()
        self.gripper_width = None
        rospy.Subscriber(ns_join(self.gripper_ns, 'joint_states'), JointState, self.gripper_cb)
        self.MIN_FORCE = 0.05
        self.MAX_FORCE = 50  # documentation says up to 70N is possible as continuous force
        self.MIN_WIDTH = 0.0
        self.MAX_WIDTH = 0.08
        self.DEFAULT_EPSILON = 0.005
        self.DEFAULT_SPEED = 0.02
        self.DEFAULT_FORCE = 10

    def gripper_cb(self, data):
        self.gripper_width = data.position[0] + data.position[1]

    def grasp(self, width, speed=None, epsilon_outer=None, epsilon_inner=None, force=None, wait_for_result=True):
        if width > self.gripper_width:
            self.move(self.MAX_WIDTH, wait_for_result=True)
        goal = GraspGoal()
        goal.width = width
        goal.epsilon.outer = self.DEFAULT_EPSILON if not epsilon_outer else epsilon_outer
        goal.epsilon.inner = self.DEFAULT_EPSILON if not epsilon_inner else epsilon_inner
        goal.speed = self.DEFAULT_SPEED if not speed else speed
        goal.force = self.DEFAULT_FORCE if not force else force
        self.grasp_client.send_goal(goal)
        if wait_for_result:
            result = self.grasp_client.wait_for_result(rospy.Duration(10))
            return result
        return True

    def move(self, width, speed=None, wait_for_result=True):
        goal = MoveGoal()
        goal.width = width
        goal.speed = self.DEFAULT_SPEED if not speed else speed
        self.move_client.send_goal(goal)
        if wait_for_result:
            result = self.move_client.wait_for_result(rospy.Duration(10))
            return result
        return True

    def homing(self, wait_for_result=True):
        goal = HomingGoal()
        self.homing_client.send_goal(goal)
        if wait_for_result:
            result = self.homing_client.wait_for_result(rospy.Duration(10))
            return result
        return True

    def stop(self, wait_for_result=True):
        goal = StopGoal()
        self.stop_client.send_goal(goal)
        if wait_for_result:
            result = self.stop_client.wait_for_result(rospy.Duration(10))
            return result
        return True

    def open(self):
        self.move(self.MAX_WIDTH)

    def close(self):
        self.move(self.MIN_WIDTH)
