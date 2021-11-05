#! /usr/bin/env python
from copy import Error
from rosgraph.names import ns_join
from typing import List, Tuple

from arm_robots.robot import MoveitEnabledRobot
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState
from franka_msgs.msg import ErrorRecoveryAction, ErrorRecoveryGoal, FrankaState
from franka_gripper.msg import GraspAction, GraspGoal, MoveAction, MoveGoal, HomingAction, HomingGoal, StopAction, StopGoal
import rospy
import actionlib
import pdb

class Panda(MoveitEnabledRobot):

    def __init__(self, robot_namespace: str = 'combined_panda', force_trigger: float = -0.0, **kwargs):
        MoveitEnabledRobot.__init__(self,
                                    robot_namespace=robot_namespace,
                                    robot_description=ns_join(robot_namespace, 'robot_description'),
                                    arms_controller_name='effort_joint_trajectory_controller',
                                    force_trigger=force_trigger,
                                    **kwargs)
        self.nebula_arm = "nebula_arm"
        self.nebula_id = "panda_1"
        self.rocket_arm = "rocket_arm"
        self.rocket_id = "panda_2"
        self.nebula_wrist = "panda_1_link8"
        self.rocket_wrist = "panda_2_link8"
        self.nebula_gripper = PandaGripper(self.robot_namespace, self.nebula_id)
        self.rocket_gripper = PandaGripper(self.robot_namespace, self.rocket_id)
        rospy.Subscriber("/"+self.robot_namespace+"/"+self.nebula_id+"_state_controller/franka_states", FrankaState, self.nebula_error_cb)
        rospy.Subscriber("/"+self.robot_namespace+"/"+self.rocket_id+"_state_controller/franka_states", FrankaState, self.rocket_error_cb)

    def open_nebula_gripper(self):
        self.nebula_gripper.move(self.nebula_gripper.MAX_WIDTH)

    def close_nebula_gripper(self):
        self.nebula_gripper.move(self.nebula_gripper.MIN_WIDTH)

    def open_rocket_gripper(self):
        self.rocket_gripper.move(self.rocket_gripper.MAX_WIDTH)

    def close_rocket_gripper(self):
        self.rocket_gripper.move(self.rocket_gripper.MIN_WIDTH)

    def nebula_error_cb(self, data):
        if data.current_errors.communication_constraints_violation or data.last_motion_errors.communication_constraints_violation:
            self.clear_errors(wait_for_result=True)

    def rocket_error_cb(self, data):
        if data.current_errors.communication_constraints_violation or data.last_motion_errors.communication_constraints_violation:
            self.clear_errors(wait_for_result=True)

    def clear_errors(self, wait_for_result=False):
        self.error_client = actionlib.SimpleActionClient("/"+self.robot_namespace+"/error_recovery", ErrorRecoveryAction)
        self.error_client.wait_for_server()
        goal = ErrorRecoveryGoal()
        self.error_client.send_goal(goal)
        if wait_for_result:
            result = self.error_client.wait_for_result(rospy.Duration(15.))
            return result
        return True

    # TODO: Add control mode setter/getter.

class PandaGripper:
    def __init__(self, robot_ns, arm_id):
        self.gripper_ns = "/"+robot_ns+"/"+arm_id+"/franka_gripper/"
        self.grasp_client = actionlib.SimpleActionClient(self.gripper_ns+"grasp", GraspAction)
        self.move_client = actionlib.SimpleActionClient(self.gripper_ns+"move", MoveAction)
        self.homing_client = actionlib.SimpleActionClient(self.gripper_ns+"homing", HomingAction)
        self.stop_client = actionlib.SimpleActionClient(self.gripper_ns+"stop", StopAction)
        self.grasp_client.wait_for_server()
        self.move_client.wait_for_server()
        self.homing_client.wait_for_server()
        self.stop_client.wait_for_server()
        self.gripper_width = None
        rospy.Subscriber(self.gripper_ns+"joint_states", JointState, self.gripper_cb)
        self.MIN_FORCE = 0.05
        self.MAX_FORCE = 50 # documentation says up to 70N is possible as continuous force
        self.MIN_WIDTH = 0.0
        self.MAX_WIDTH = 0.08
        self.DEFAULT_EPSILON = 0.005
        self.DEFAULT_SPEED = 0.02
        self.DEFAULT_FORCE = 10
        # TODO: add to cfg file

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
            result = self.grasp_client.wait_for_result(rospy.Duration(15.))
            return result
        return True

    def move(self, width, speed=None, wait_for_result=False):
        goal = MoveGoal()
        goal.width = width
        goal.speed = self.DEFAULT_SPEED if not speed else speed
        self.move_client.send_goal(goal)
        if wait_for_result:
            result = self.move_client.wait_for_result(rospy.Duration(15.))
            return result
        return True

    def homing(self, wait_for_result=True):
        goal = HomingGoal()
        self.homing_client.send_goal(goal)
        if wait_for_result:
            result = self.homing_client.wait_for_result(rospy.Duration(15.))
            return result
        return True

    def stop(self, wait_for_result=False):
        goal = StopGoal()
        self.stop_client.send_goal(goal)
        if wait_for_result:
            result = self.stop_client.wait_for_result(rospy.Duration(15.))
            return result
        return True