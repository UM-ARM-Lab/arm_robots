#! /usr/bin/env python
import collections
from typing import List, Dict, Tuple, Sequence

import numpy as np
from colorama import Fore
from scipy import signal

import moveit_commander
import rospy
from arc_utilities.conversions import convert_to_pose_msg, normalize_quaternion
from arc_utilities.listener import Listener
from arm_robots.base_robot import BaseRobot
from arm_robots.config.med_config import ARM_JOINT_NAMES
from arm_robots.robot import MoveitEnabledRobot
from moveit_msgs.msg import DisplayRobotState
from trajectory_msgs.msg import JointTrajectoryPoint
from victor_hardware_interface.victor_utils import get_control_mode_params, list_to_jvq, jvq_to_list
from victor_hardware_interface_msgs.msg import ControlMode, MotionStatus, MotionCommand
from victor_hardware_interface_msgs.srv import SetControlMode, GetControlMode, GetControlModeRequest, \
    GetControlModeResponse, SetControlModeResponse
from wsg_50_utils.wsg_50_gripper import WSG50Gripper


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

    joint_position_of = dict(zip(joint_names, positions))

    def fill_using(joint_ordering: Sequence[str]):
        if not all(j in joint_position_of for j in joint_ordering):
            return None
        return [joint_position_of[name] for name in joint_ordering]

    positions_by_interface = {
        'arm': fill_using(ARM_JOINT_NAMES),
    }
    # set equality ignores order

    return positions_by_interface, False, ""


class BaseMed(BaseRobot):

    def __init__(self, robot_namespace: str, cartesian_impedance_controller_kwargs=None):
        BaseRobot.__init__(self, robot_namespace=robot_namespace)

        self.arm_command_pub = rospy.Publisher(self.ns("motion_command"), MotionCommand, queue_size=10)
        self.set_control_mode_srv = rospy.ServiceProxy(self.ns("set_control_mode_service"), SetControlMode)
        self.get_control_mode_srv = rospy.ServiceProxy(self.ns("get_control_mode_service"), GetControlMode)
        self.arm_status_listener = Listener(self.ns("motion_status"), MotionStatus)
        self.waypoint_state_pub = rospy.Publisher(self.ns("waypoint_robot_state"), DisplayRobotState, queue_size=10)
        kwargs = cartesian_impedance_controller_kwargs or {}
        self.create_cartesian_impedance_controller([self.arm_status_listener], [self.arm_command_pub], ARM_JOINT_NAMES,
                                                   "med_base", **kwargs)

    def send_joint_command(self, joint_names: Sequence[str], trajectory_point: JointTrajectoryPoint) -> Tuple[
        bool, str]:
        # TODO: in med's impedance mode, we want to modify the setpoint so that there is a limit
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
        low, high = self.get_joint_limits(ARM_JOINT_NAMES, safety_margin=1e-2)
        limit_enforced_positions = np.clip(positions, low, high)

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

        return [position_of_joint[name] for name in joint_names]


class Med(BaseMed, MoveitEnabledRobot):

    def __init__(self, robot_namespace: str = 'med', force_trigger: float = -0.0, base_kwargs=None, **kwargs):
        MoveitEnabledRobot.__init__(self,
                                    robot_namespace=robot_namespace,
                                    arms_controller_name='arm_trajectory_controller',
                                    force_trigger=force_trigger,
                                    **kwargs)
        base_kwargs = base_kwargs or {}
        BaseMed.__init__(self, robot_namespace=robot_namespace, **base_kwargs)
        self.arm_group = 'kuka_arm'
        self.wrist = 'med_kuka_link_ee'
        self.gripper = WSG50Gripper()

    def get_arm_joints(self):
        return ARM_JOINT_NAMES

    def set_control_mode(self, control_mode: ControlMode, vel=0.1, **kwargs):
        super().set_control_mode(control_mode, vel, **kwargs)
        self._max_velocity_scale_factor = vel

    def set_grasping_force(self, force):
        return self.gripper.set_grasping_force(force)

    def grasp(self, width, speed=50.0):
        return self.gripper.grasp(width, speed=speed)

    def open_gripper(self):
        return self.gripper.open_gripper()

    def release(self, width=110.0, speed=50.0):
        self.gripper.release(width=width, speed=speed)


