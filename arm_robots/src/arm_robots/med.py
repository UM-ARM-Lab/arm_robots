#! /usr/bin/env python
import collections
from typing import List, Dict, Tuple, Sequence

import numpy as np
from colorama import Fore
from scipy import signal

import moveit_commander
import rospy
from actionlib import SimpleActionClient
from arc_utilities.conversions import convert_to_pose_msg, normalize_quaternion, convert_to_positions
from arc_utilities.listener import Listener
from arm_robots.base_robot import DualArmRobot, BaseRobot
from arm_robots.config.victor_config import NAMED_POSITIONS, default_robotiq_command, \
    KUKA_MIN_PATH_JOINT_POSITION_TOLERANCE, KUKA_FULL_SPEED_PATH_JOINT_POSITION_TOLERANCE, LEFT_GRIPPER_JOINT_NAMES, \
    RIGHT_GRIPPER_JOINT_NAMES, LEFT_ARM_JOINT_NAMES, RIGHT_ARM_JOINT_NAMES, BOTH_ARM_JOINT_NAMES, ALL_JOINT_NAMES, \
    KUKA_GOAL_JOINT_POSITION_TOLERANCE, KUKA_MIN_PATH_JOINT_IMPEDANCE_TOLERANCE, \
    KUKA_FULL_SPEED_PATH_JOINT_IMPEDANCE_TOLERANCE, KUKA_GOAL_JOINT_IMPEDANCE_TOLERANCE
from arm_robots.config.med_config import ARM_JOINT_NAMES
from arm_robots.robot import MoveitEnabledRobot
from arm_robots.robot_utils import make_joint_tolerance
from control_msgs.msg import FollowJointTrajectoryFeedback, FollowJointTrajectoryGoal, FollowJointTrajectoryResult
from moveit_msgs.msg import DisplayRobotState
from std_msgs.msg import String, Float32
from trajectory_msgs.msg import JointTrajectoryPoint
from victor_hardware_interface.victor_utils import get_control_mode_params, list_to_jvq, jvq_to_list, \
    default_gripper_command #, gripper_status_to_list
from victor_hardware_interface_msgs.msg import ControlMode, MotionStatus, MotionCommand, Robotiq3FingerCommand, \
    Robotiq3FingerStatus
from victor_hardware_interface_msgs.srv import SetControlMode, GetControlMode, GetControlModeRequest, \
    GetControlModeResponse, SetControlModeResponse
from wsg_50_common.srv import Move, Conf

# TODO: Since we have only one set of arms, this really just makes sure everythings in the right order. Could probably simplify but I'll keep it for now.
def delegate_to_arms(positions: List, joint_names: Sequence[str]) -> Tuple[Dict[str, List], bool, str]:
    """
    Given a list (e.g. of positions) and a corresponding list of joint names,
    assign and order by victor's joint groups.

    Args:
        positions: values to delegate
        joint_names: list of joint_names

    Returns:
        object: (map from joint_names to values, abort?, abort_msg)
    """
    assert len(positions) == len(joint_names), "positions and joint_names must be same length"

    # TODO: Why can't joint_names be a combination of arm and gripper joints?
    ok = set(joint_names) in [set(ARM_JOINT_NAMES)]

    if not ok:
        blank_positions = {n: None for n in ['arm']}
        return blank_positions, True, f"Invalid joint_names {joint_names}"

    joint_position_of = {name: pos for name, pos in zip(joint_names, positions)}

    def fill_using(joint_ordering: Sequence[str]):
        if not all(j in joint_position_of for j in joint_ordering):
            return None
        return [joint_position_of[name] for name in joint_ordering]

    positions_by_interface = {
        'arm'    : fill_using(ARM_JOINT_NAMES),
    }
    # set equality ignores order

    return positions_by_interface, False, ""

class BaseMed(BaseRobot):

    def __init__(self, robot_namespace: str):
        BaseRobot.__init__(self, robot_namespace=robot_namespace)

        self.arm_command_pub = rospy.Publisher(self.ns("left_arm/motion_command"), MotionCommand, queue_size=10)
        self.set_control_mode_srv = rospy.ServiceProxy(self.ns("left_arm/set_control_mode_service"), SetControlMode)
        self.get_control_mode_srv = rospy.ServiceProxy(self.ns("left_arm/get_control_mode_service"), GetControlMode)
        self.arm_status_listener = Listener(self.ns("left_arm/motion_status"), MotionStatus)
        self.waypoint_state_pub = rospy.Publisher(self.ns("waypoint_robot_state"), DisplayRobotState, queue_size=10)

    def send_joint_command(self, joint_names: Sequence[str], trajectory_point: JointTrajectoryPoint) -> Tuple[bool, str]:
        # TODO: in victor's impedance mode, we want to modify the setpoint so that there is a limit
        #  on the force we will apply
        positions, abort, msg = delegate_to_arms(trajectory_point.positions, joint_names)
        if abort:
            return True, msg

        velocities, _, _ = delegate_to_arms([0.0] * len(ARM_JOINT_NAMES), ARM_JOINT_NAMES)
        if len(trajectory_point.velocities) != 0:
            velocities, abort, msg = delegate_to_arms(trajectory_point.velocities, joint_names)
        if abort:
            return True, msg

        # This is a bit silly.
        positions = positions['arm']
        velocities = velocities['arm']

        control_mode = self.get_control_mode()

        def trunc(values, decs=0):
            return np.trunc(values * 10 ** decs) / (10 ** decs)

        velocities = trunc(np.array(velocities), 3)  # Kuka does not like sending small but non-zero velocity commands
        # FIXME: what if we allow the BaseRobot class to use moveit, but just don't have it require that
        # any actions are running?
        # NOTE: why are these values not checked by the lower-level code? the Java code knows what the joint limits
        # are so why does it not enforce them?

        # TODO: use enforce bounds? https://github.com/ros-planning/moveit/pull/2356
        limit_enforced_positions = []
        for i, joint_name in enumerate(ARM_JOINT_NAMES):
            joint: moveit_commander.RobotCommander.Joint = self.robot_commander.get_joint(joint_name)
            limit_enforced_position = np.clip(positions[i], joint.min_bound() + 1e-2, joint.max_bound() - 1e-2)
            limit_enforced_positions.append(limit_enforced_position)

        # TODO: enforce velocity limits
        cmd = MotionCommand(joint_position=list_to_jvq(limit_enforced_positions),
                            joint_velocity=list_to_jvq(velocities),
                            control_mode=control_mode)
        cmd.header.stamp = rospy.Time.now()
        self.arm_command_pub.publish(cmd)

        return False, ""

    def get_control_mode(self):
        control_mode_res: GetControlModeResponse = self.get_control_mode_srv(GetControlModeRequest())
        return control_mode_res.active_control_mode.control_mode

    def set_control_mode(self, control_mode: ControlMode, vel, **kwargs):
        new_control_mode = get_control_mode_params(control_mode, vel=vel, **kwargs)
        res: SetControlModeResponse = self.set_control_mode_srv(new_control_mode)

        if not res.success:
            rospy.logerr("Failed to switch arm to control mode: " + str(control_mode))
            rospy.logerr(res.message)
        return res

    def get_arm_status(self):
        return self.arm_status_listener.get()

    def send_cartesian_command(self, pose_stamped):
        pose_stamped = convert_to_pose_msg(pose_stamped)
        pose_stamped.pose.orientation = normalize_quaternion(pose_stamped.pose.orientation)

        left_arm_command = MotionCommand()
        left_arm_command.header.frame_id = 'med_base'
        left_arm_command.cartesian_pose = pose_stamped.pose
        left_arm_command.control_mode = self.get_control_mode()
        while self.arm_command_pub.get_num_connections() < 1:
            rospy.sleep(0.01)

        self.arm_command_pub.publish(left_arm_command)

    # TODO: send_delta_cartesian_command

    def get_joint_positions_map(self) -> Dict[str, float]:
        arm_joint_vals = jvq_to_list(self.arm_status_listener.get().measured_joint_position)
        return {n: val for n, val in zip(ARM_JOINT_NAMES, arm_joint_vals)}

    def get_joint_positions(self, joint_names: Sequence[str] = ARM_JOINT_NAMES):
        """
        :args joint_names an optional list of names if you want to have a specific order or a subset
        """
        position_of_joint = self.get_joint_positions_map()
        print(position_of_joint)
        return [position_of_joint[name] for name in joint_names]


class Med(MoveitEnabledRobot, BaseMed):

    def __init__(self, robot_namespace: str = 'victor', force_trigger: float= -0.0, manual_execute: bool = True):
        MoveitEnabledRobot.__init__(self,
                                    robot_namespace=robot_namespace,
                                    arms_controller_name='arm_trajectory_controller',
                                    force_trigger=force_trigger,
                                    manual_execute=manual_execute)
        BaseMed.__init__(self, robot_namespace=robot_namespace)
        self.arm_group = 'kuka_arm'
        self.wrist = 'med_kuka_link_ee'

    def get_arm_joints(self):
        return ARM_JOINT_NAMES

    def set_control_mode(self, control_mode: ControlMode, vel=0.1, **kwargs):
        super().set_control_mode(control_mode, vel, **kwargs)
        self.max_velocity_scale_factor = vel

    def set_grasping_force(self, force):
        if type(force) != float or force > 80.0 or force <= 0.0:
            print("Bad grasp force value provided! Not setting grasping force.")
            return None

        rospy.wait_for_service('/wsg_50_driver/set_force')
        try:
            force_proxy = rospy.ServiceProxy('/wsg_50_driver/set_force', Conf)
            force_resp = force_proxy(force)
            return force_resp.error
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

    def grasp(self, width, speed=50.0):
        rospy.wait_for_service('/wsg_50_driver/grasp')
        try:
            close_proxy = rospy.ServiceProxy('/wsg_50_driver/grasp', Move)
            grasp_resp = close_proxy(width, speed)
            return grasp_resp.error
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

    def open_gripper(self):
        rospy.wait_for_service('/wsg_50_driver/move')
        try:
            move_proxy = rospy.ServiceProxy('/wsg_50_driver/move', Move)
            move_resp = move_proxy(100.0, 50.0)
            return move_resp.error
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

    def release(self, width=110.0, speed=50.0):
        rospy.wait_for_service('/wsg_50_driver/release')
        try:
            release_proxy = rospy.ServiceProxy('/wsg_50_driver/release', Move)
            release_resp = release_proxy(width, speed)
            return release_resp.error
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)
