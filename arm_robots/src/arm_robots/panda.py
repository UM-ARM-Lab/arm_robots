#! /usr/bin/env python
import pdb
from typing import Optional, Callable, List, Tuple

import actionlib
import rospy
import numpy as np
from geometry_msgs.msg import WrenchStamped
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from arm_robots.robot import MoveitEnabledRobot
from franka_gripper.msg import GraspAction, GraspGoal, MoveAction, MoveGoal, HomingAction, HomingGoal, StopAction, \
    StopGoal
from netft_rdt_driver.srv import Zero
from control_msgs.msg import GripperCommandActionGoal, GripperCommand, GripperCommandGoal
from franka_msgs.msg import ErrorRecoveryAction, ErrorRecoveryGoal, FrankaState
from rosgraph.names import ns_join
from sensor_msgs.msg import JointState
from arm_robots.robot_utils import ExecutionResult


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
        self.panda_1_command_pub = rospy.Publisher(ns_join(self.panda_1, 'panda_1_position_joint_trajectory_controller/command'), JointTrajectory, queue_size=10)
        self.panda_2_command_pub = rospy.Publisher(ns_join(self.panda_2, 'panda_2_position_joint_trajectory_controller/command'), JointTrajectory, queue_size=10)
        self.display_goals = False
        self.panda_1_gripper = PandaGripper(self.robot_namespace, self.panda_1)
        self.panda_2_gripper = PandaGripper(self.robot_namespace, self.panda_2)
        self.panda_1_netft = PandaNetft(self.robot_namespace, self.panda_1)
        self.panda_2_netft = PandaNetft(self.robot_namespace, self.panda_2)

        self.panda_1_client = self.setup_joint_trajectory_controller_client(ns_join(self.panda_1, f'{self.panda_1}_position_joint_trajectory_controller'))
        self.panda_2_client = self.setup_joint_trajectory_controller_client(ns_join(self.panda_2, f'{self.panda_2}_position_joint_trajectory_controller'))

    def follow_arms_joint_trajectory(self, trajectory: JointTrajectory, stop_condition: Optional[Callable] = None,
                                     group_name: Optional[str] = None):
        if group_name == self.panda_1:
            return self.follow_joint_trajectory(trajectory, self.panda_1_client, stop_condition=stop_condition)
        elif group_name == self.panda_2:
            return self.follow_joint_trajectory(trajectory, self.panda_2_client, stop_condition=stop_condition)

    def send_joint_command(self, joint_names: List[str], trajectory_point: JointTrajectoryPoint, group_name: Optional[str] = None):
        robot_command = JointTrajectory()
        robot_command.joint_names = joint_names
        robot_command.points.append(trajectory_point)
        pdb.set_trace()
        if group_name == self.panda_1:
            self.panda_1_command_pub.publish(robot_command)
        elif group_name == self.panda_2:
            self.panda_2_command_pub.publish(robot_command)



class PandaNetft:
    def __init__(self, robot_ns, arm_id, stop_force=3.0, stop_torque=0.3):
        self.netft_ns = ns_join(robot_ns, f'{arm_id}_netft')
        self.netft_zero = rospy.ServiceProxy(ns_join(self.netft_ns, 'zero'), Zero)
        self.netft_data = None
        self.netft_data_sub = rospy.Subscriber(ns_join(self.netft_ns, 'netft_data'), WrenchStamped, self.netft_data_cb, queue_size=None)
        self.stop_force = stop_force
        self.stop_torque = stop_torque

    def zero_netft(self):
        self.netft_zero()

    def netft_data_cb(self, wrench_msg):
        self.netft_data = np.array((wrench_msg.wrench.force.x, wrench_msg.wrench.force.y, wrench_msg.wrench.force.z, wrench_msg.wrench.torque.x, wrench_msg.wrench.torque.y, wrench_msg.wrench.torque.z))

    def stop_condition(self, feedback):
        force_magnitude = np.linalg.norm(self.netft_data[:3])
        return force_magnitude > self.stop_force


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

    def grasp(self, width, speed=None, epsilon_outer=None, epsilon_inner=None, force=None, wait_for_result=False):
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

    def move(self, width, speed=None, wait_for_result=False):
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

    def stop(self, wait_for_result=False):
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
