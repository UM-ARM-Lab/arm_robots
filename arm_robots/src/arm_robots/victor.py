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
from arm_robots.base_robot import DualArmRobot
from arm_robots.config.victor_config import NAMED_POSITIONS, default_robotiq_command, \
    KUKA_MIN_PATH_JOINT_POSITION_TOLERANCE, KUKA_FULL_SPEED_PATH_JOINT_POSITION_TOLERANCE, LEFT_GRIPPER_JOINT_NAMES, \
    RIGHT_GRIPPER_JOINT_NAMES, LEFT_ARM_JOINT_NAMES, RIGHT_ARM_JOINT_NAMES, BOTH_ARM_JOINT_NAMES, ALL_JOINT_NAMES, \
    KUKA_GOAL_JOINT_POSITION_TOLERANCE, KUKA_MIN_PATH_JOINT_IMPEDANCE_TOLERANCE, \
    KUKA_FULL_SPEED_PATH_JOINT_IMPEDANCE_TOLERANCE, KUKA_GOAL_JOINT_IMPEDANCE_TOLERANCE
from arm_robots.robot import MoveitEnabledRobot
from arm_robots.robot_utils import make_joint_tolerance
from control_msgs.msg import FollowJointTrajectoryFeedback, FollowJointTrajectoryGoal, FollowJointTrajectoryResult
from moveit_msgs.msg import DisplayRobotState
from std_msgs.msg import String, Float32
from trajectory_msgs.msg import JointTrajectoryPoint
from victor_hardware_interface.victor_utils import get_control_mode_params, list_to_jvq, jvq_to_list, \
    default_gripper_command, gripper_status_to_list
from victor_hardware_interface_msgs.msg import ControlMode, MotionStatus, MotionCommand, Robotiq3FingerCommand, \
    Robotiq3FingerStatus
from victor_hardware_interface_msgs.srv import SetControlMode, GetControlMode, GetControlModeRequest, \
    GetControlModeResponse, SetControlModeResponse


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
    ok = set(joint_names) in [set(LEFT_ARM_JOINT_NAMES),
                              set(RIGHT_ARM_JOINT_NAMES),
                              set(BOTH_ARM_JOINT_NAMES),
                              set(LEFT_GRIPPER_JOINT_NAMES),
                              set(RIGHT_GRIPPER_JOINT_NAMES),
                              set(ALL_JOINT_NAMES)]

    if not ok:
        blank_positions = {n: None for n in ['right_arm', 'left_arm', 'right_gripper', 'left_gripper']}
        return blank_positions, True, f"Invalid joint_names {joint_names}"

    joint_position_of = {name: pos for name, pos in zip(joint_names, positions)}

    def fill_using(joint_ordering: Sequence[str]):
        if not all(j in joint_position_of for j in joint_ordering):
            return None
        return [joint_position_of[name] for name in joint_ordering]

    positions_by_interface = {
        'right_arm'    : fill_using(RIGHT_ARM_JOINT_NAMES),
        'left_arm'     : fill_using(LEFT_ARM_JOINT_NAMES),
        'right_gripper': fill_using(RIGHT_GRIPPER_JOINT_NAMES),
        'left_gripper' : fill_using(LEFT_GRIPPER_JOINT_NAMES),
    }
    # set equality ignores order

    return positions_by_interface, False, ""


class BaseVictor(DualArmRobot):

    def __init__(self, robot_namespace: str):
        DualArmRobot.__init__(self, robot_namespace=robot_namespace)

        self.left_arm_command_pub = rospy.Publisher(self.ns("left_arm/motion_command"), MotionCommand, queue_size=10)
        self.right_arm_command_pub = rospy.Publisher(self.ns("right_arm/motion_command"), MotionCommand, queue_size=10)

        self.left_gripper_command_pub = rospy.Publisher(self.ns("left_arm/gripper_command"), Robotiq3FingerCommand,
                                                        queue_size=10)
        self.right_gripper_command_pub = rospy.Publisher(self.ns("right_arm/gripper_command"), Robotiq3FingerCommand,
                                                         queue_size=10)

        self.left_set_control_mode_srv = rospy.ServiceProxy(self.ns("left_arm/set_control_mode_service"),
                                                            SetControlMode)
        self.right_set_control_mode_srv = rospy.ServiceProxy(self.ns("right_arm/set_control_mode_service"),
                                                             SetControlMode)

        self.left_get_control_mode_srv = rospy.ServiceProxy(self.ns("left_arm/get_control_mode_service"),
                                                            GetControlMode)
        self.right_get_control_mode_srv = rospy.ServiceProxy(self.ns("right_arm/get_control_mode_service"),
                                                             GetControlMode)

        self.left_arm_status_listener = Listener(self.ns("left_arm/motion_status"), MotionStatus)
        self.right_arm_status_listener = Listener(self.ns("right_arm/motion_status"), MotionStatus)

        self.left_gripper_status_listener = Listener(self.ns("left_arm/gripper_status"), Robotiq3FingerStatus)
        self.right_gripper_status_listener = Listener(self.ns("right_arm/gripper_status"), Robotiq3FingerStatus)

        self.waypoint_state_pub = rospy.Publisher(self.ns("waypoint_robot_state"), DisplayRobotState, queue_size=10)

    def send_joint_command(self, joint_names: Sequence[str], trajectory_point: JointTrajectoryPoint) -> Tuple[
        bool, str]:
        # TODO: in victor's impedance mode, we want to modify the setpoint so that there is a limit
        #  on the force we will apply
        positions, abort, msg = delegate_to_arms(trajectory_point.positions, joint_names)
        if abort:
            return True, msg

        velocities, _, _ = delegate_to_arms([0.0] * len(ALL_JOINT_NAMES), ALL_JOINT_NAMES)
        if len(trajectory_point.velocities) != 0:
            velocities, abort, msg = delegate_to_arms(trajectory_point.velocities, joint_names)
        if abort:
            return True, msg

        # Get the current control mode
        control_mode = self.get_control_modes()
        left_arm_control_mode = control_mode['left']
        right_arm_control_mode = control_mode['right']

        self.send_arm_command(self.left_arm_command_pub, left_arm_control_mode,
                              positions['left_arm'], velocities['left_arm'])
        self.send_arm_command(self.right_arm_command_pub, right_arm_control_mode,
                              positions['right_arm'], velocities['right_arm'])
        self.send_gripper_command(self.left_gripper_command_pub, positions['left_gripper'])
        self.send_gripper_command(self.right_gripper_command_pub, positions['right_gripper'])

        return False, ""

    def send_arm_command(self, command_pub: rospy.Publisher, control_mode: ControlMode,
                         positions, velocities=(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)):
        if positions is None:
            return

        def trunc(values, decs=0):
            return np.trunc(values * 10 ** decs) / (10 ** decs)

        velocities = trunc(np.array(velocities), 3)  # Kuka does not like sending small but non-zero velocity commands
        # FIXME: what if we allow the BaseRobot class to use moveit, but just don't have it require that
        # any actions are running?
        # NOTE: why are these values not checked by the lower-level code? the Java code knows what the joint limits
        # are so why does it not enforce them?

        # TODO: use enforce bounds? https://github.com/ros-planning/moveit/pull/2356
        limit_enforced_positions = []
        for i, joint_name in enumerate(RIGHT_ARM_JOINT_NAMES):
            joint: moveit_commander.RobotCommander.Joint = self.robot_commander.get_joint(joint_name)
            limit_enforced_position = np.clip(positions[i], joint.min_bound() + 1e-2, joint.max_bound() - 1e-2)
            limit_enforced_positions.append(limit_enforced_position)

        # TODO: enforce velocity limits
        cmd = MotionCommand(joint_position=list_to_jvq(limit_enforced_positions),
                            joint_velocity=list_to_jvq(velocities),
                            control_mode=control_mode)
        cmd.header.stamp = rospy.Time.now()
        command_pub.publish(cmd)

    def get_gripper_statuses(self):
        return {'left' : self.get_left_gripper_status(),
                'right': self.get_right_gripper_status()}

    def get_right_gripper_status(self) -> Robotiq3FingerStatus:
        return self.right_gripper_status_listener.get()

    def get_left_gripper_status(self) -> Robotiq3FingerStatus:
        return self.left_gripper_status_listener.get()

    def is_left_gripper_closed(self):
        status = self.get_left_gripper_status()
        return self.is_gripper_closed(status)

    def is_right_gripper_closed(self):
        status = self.get_right_gripper_status()
        return self.is_gripper_closed(status)

    def is_gripper_closed(self, status: Robotiq3FingerStatus):
        # [0.5, 0.4, 0.4, 0.8]
        finger_a_closed = status.finger_a_status.position > 0.5 - 1e-2
        finger_b_closed = status.finger_b_status.position > 0.5 - 1e-2
        finger_c_closed = status.finger_c_status.position > 0.5 - 1e-2
        return finger_a_closed and finger_b_closed and finger_c_closed

    def get_arms_statuses(self):
        return {'left' : self.get_left_arm_status(),
                'right': self.get_right_arm_status()}

    def get_right_arm_status(self) -> MotionStatus:
        return self.right_arm_status_listener.get()

    def get_left_arm_status(self) -> MotionStatus:
        return self.left_arm_status_listener.get()

    def get_control_modes(self):
        return {'left': self.get_left_arm_control_mode(), 'right': self.get_right_arm_control_mode()}

    def get_control_mode_for_joint(self, joint_name):
        if joint_name in LEFT_ARM_JOINT_NAMES:
            return self.get_left_arm_control_mode()
        if joint_name in RIGHT_ARM_JOINT_NAMES:
            return self.get_right_arm_control_mode()
        return None

    def set_control_mode(self, control_mode: ControlMode, vel, **kwargs):
        left_res = self.set_left_arm_control_mode(control_mode, vel=vel, **kwargs)
        right_res = self.set_right_arm_control_mode(control_mode, vel=vel, **kwargs)
        return left_res, right_res

    def get_left_arm_control_mode(self):
        left_control_mode_res: GetControlModeResponse = self.left_get_control_mode_srv(GetControlModeRequest())
        return left_control_mode_res.active_control_mode.control_mode

    def get_right_arm_control_mode(self):
        right_control_mode_res: GetControlModeResponse = self.right_get_control_mode_srv(GetControlModeRequest())
        return right_control_mode_res.active_control_mode.control_mode

    def set_right_arm_control_mode(self, control_mode: ControlMode, **kwargs):
        new_control_mode = get_control_mode_params(control_mode, **kwargs)
        res: SetControlModeResponse = self.right_set_control_mode_srv(new_control_mode)

        if not res.success:
            rospy.logerr("Failed to switch right arm to control mode: " + str(control_mode))
            rospy.logerr(res.message)
        return res

    def set_left_arm_control_mode(self, control_mode: ControlMode, **kwargs):
        new_control_mode = get_control_mode_params(control_mode, **kwargs)
        res: SetControlModeResponse = self.left_set_control_mode_srv(new_control_mode)

        if not res.success:
            rospy.logerr("Failed to switch left arm to control mode: " + str(control_mode))
            rospy.logerr(res.message)
        return res

    def send_cartesian_command(self, poses: Dict):
        """ absolute """
        self.send_left_arm_cartesian_command(poses['left'])
        self.send_right_arm_cartesian_command(poses['right'])

    def send_left_arm_cartesian_command(self, pose_stamped):
        pose_stamped = convert_to_pose_msg(pose_stamped)
        pose_stamped.pose.orientation = normalize_quaternion(pose_stamped.pose.orientation)

        left_arm_command = MotionCommand()
        left_arm_command.header.frame_id = 'victor_left_arm_world_frame_kuka'
        left_arm_command.cartesian_pose = pose_stamped.pose
        left_arm_command.control_mode = self.get_left_arm_control_mode()
        while self.left_arm_command_pub.get_num_connections() < 1:
            rospy.sleep(0.01)

        self.left_arm_command_pub.publish(left_arm_command)

    def send_right_arm_cartesian_command(self, pose_stamped):
        pose_stamped = convert_to_pose_msg(pose_stamped)
        pose_stamped.pose.orientation = normalize_quaternion(pose_stamped.pose.orientation)

        right_arm_command = MotionCommand()
        right_arm_command.header.frame_id = 'victor_right_arm_world_frame_kuka'
        right_arm_command.cartesian_pose = pose_stamped.pose
        right_arm_command.control_mode = self.get_right_arm_control_mode()
        while self.right_arm_command_pub.get_num_connections() < 1:
            rospy.sleep(0.01)

        self.right_arm_command_pub.publish(right_arm_command)

    def send_delta_cartesian_command(self, delta_positions):
        delta_positions = convert_to_positions(delta_positions)
        statuses = self.get_arms_statuses()

        current_left_pose = statuses['left'].measured_cartesian_pose
        desired_left_pose = current_left_pose
        desired_left_pose.position.x += delta_positions['left'].x
        desired_left_pose.position.y += delta_positions['left'].y
        desired_left_pose.position.z += delta_positions['left'].z

        current_right_pose = statuses['right'].measured_cartesian_pose
        desired_right_pose = current_right_pose
        desired_right_pose.position.x += delta_positions['right'].x
        desired_right_pose.position.y += delta_positions['right'].y
        desired_right_pose.position.z += delta_positions['right'].z

        poses = {
            'left' : desired_left_pose,
            'right': desired_right_pose,
        }
        self.send_cartesian_command(poses)

    @staticmethod
    def send_gripper_command(command_pub: rospy.Publisher, positions):
        if positions is not None:
            cmd = default_gripper_command()
            cmd.finger_a_command.position = positions[0]
            cmd.finger_b_command.position = positions[1]
            cmd.finger_c_command.position = positions[2]
            cmd.scissor_command.position = positions[3]
            command_pub.publish(cmd)

    def get_joint_positions_map(self) -> Dict[str, float]:
        all_joint_vals = jvq_to_list(self.left_arm_status_listener.get().measured_joint_position)
        all_joint_vals += jvq_to_list(self.right_arm_status_listener.get().measured_joint_position)
        all_joint_vals += gripper_status_to_list(self.left_gripper_status_listener.get())
        all_joint_vals += gripper_status_to_list(self.right_gripper_status_listener.get())
        return {n: val for n, val in zip(ALL_JOINT_NAMES, all_joint_vals)}

    def get_joint_positions(self, joint_names: Sequence[str] = ALL_JOINT_NAMES):
        """
        :args joint_names an optional list of names if you want to have a specific order or a subset
        """
        position_of_joint = self.get_joint_positions_map()
        return [position_of_joint[name] for name in joint_names]

    def get_right_gripper_command_pub(self):
        return self.right_gripper_command_pub

    def get_left_gripper_command_pub(self):
        return self.left_gripper_command_pub

    def get_open_gripper_msg(self):
        cmd = default_robotiq_command()
        cmd.finger_a_command.position = 0.25
        cmd.finger_b_command.position = 0.25
        cmd.finger_c_command.position = 0.25
        cmd.scissor_command.position = 0.8
        return cmd

    def get_close_gripper_msg(self):
        cmd = default_robotiq_command()
        cmd.finger_a_command.position = 0.65
        cmd.finger_b_command.position = 0.65
        cmd.finger_c_command.position = 0.5
        cmd.scissor_command.position = 0.8
        return cmd


# TODO: undo this multiple inheritance and use composition
class Victor(BaseVictor, MoveitEnabledRobot):
    trajectory_delay_in_s_before_log = 0.1

    def __init__(self, robot_namespace: str = 'victor', force_trigger: float = -0.0, **kwargs):
        MoveitEnabledRobot.__init__(self,
                                    robot_namespace=robot_namespace,
                                    arms_controller_name='both_arms_trajectory_controller',
                                    **kwargs)
        BaseVictor.__init__(self, robot_namespace=robot_namespace)
        self.left_arm_group = 'left_arm'
        self.right_arm_group = 'right_arm'
        self.left_tool_name = 'left_tool'
        self.right_tool_name = 'right_tool'
        self.left_force_change_sub = rospy.Publisher(self.ns("left_force_change"), Float32, queue_size=10)
        self.right_force_change_sub = rospy.Publisher(self.ns("right_force_change"), Float32, queue_size=10)
        self.polly_pub = rospy.Publisher("/polly", String, queue_size=10)
        self.use_force_trigger = force_trigger >= 0
        self.force_trigger = force_trigger
        self.feedback_callbacks.append(self._log_trajectory_goal_not_reached)

    def _log_trajectory_goal_not_reached(self, client, goal, feedback: FollowJointTrajectoryFeedback):
        time_error = (feedback.actual.time_from_start - feedback.desired.time_from_start).secs
        if time_error > self.trajectory_delay_in_s_before_log:
            act = np.array(feedback.actual.positions)
            des = np.array(feedback.desired.positions)
            error = [(name, err) for name, err in zip(feedback.joint_names, act - des)]
            error_str = '\n'.join(f"{name}: {err:.2f}" for name, err in error)
            rospy.logwarn_throttle(0.5, f"Robot is late in reaching goal. Error is:\n{error_str}")

    def set_control_mode(self, control_mode: ControlMode, vel=0.1, **kwargs):
        super().set_control_mode(control_mode, vel, **kwargs)
        self.max_velocity_scale_factor = vel

    def move_to_impedance_switch(self, actually_switch: bool = True, new_relative_velocity=0.1):
        self.move_to("impedance switch")
        if actually_switch:
            return self.set_control_mode(ControlMode.JOINT_IMPEDANCE, vel=new_relative_velocity)
        return True

    def move_to(self, position_name: str):
        if position_name not in NAMED_POSITIONS:
            raise ValueError(f'{position_name} is not a known position for victor')

        position = NAMED_POSITIONS[position_name]
        if "left_arm" in position and "right_arm" in position:
            self.plan_to_joint_config("both_arms", self.both_arm_config(left_arm_config=position["left_arm"],
                                                                        right_arm_config=position["right_arm"]))

        else:
            for joint_group, configs in position.items():
                self.plan_to_joint_config(joint_group, configs)

    def move_to_config(self, joint_group, configs):
        _, res, _ = self.plan_to_joint_config(joint_group, configs)
        if res.error_code != FollowJointTrajectoryResult.SUCCESSFUL:
            rospy.logwarn("Failed to move to config: ")
            raise Exception(f"Failed to move to config: {res.error_string}")

    @staticmethod
    def both_arm_config(left_arm_config, right_arm_config):
        assert len(left_arm_config) == 7, "Left arm config must be length 7"
        assert len(right_arm_config) == 7, "Right arm config must be length 7"
        return left_arm_config + right_arm_config

    def get_right_gripper_joints(self):
        return RIGHT_GRIPPER_JOINT_NAMES

    def get_left_gripper_joints(self):
        return LEFT_GRIPPER_JOINT_NAMES

    # def get_joint_positions(self, joint_names: Optional[List[str]] = None):
    #     return self.get_joint_positions(joint_names)

    def speak(self, message: str):
        rospy.loginfo(f"Victor says: {message}")
        self.polly_pub.publish(String(data=message))

    def get_right_arm_joints(self):
        return RIGHT_ARM_JOINT_NAMES

    def get_left_arm_joints(self):
        return LEFT_ARM_JOINT_NAMES

    def follow_joint_trajectory_feedback_cb(self,
                                            client: SimpleActionClient,
                                            goal: FollowJointTrajectoryGoal,
                                            feedback: FollowJointTrajectoryFeedback):
        for feedback_callback in self.feedback_callbacks:
            feedback_callback(client, goal, feedback)

    def make_follow_joint_trajectory_goal(self, trajectory) -> FollowJointTrajectoryGoal:
        goal = FollowJointTrajectoryGoal(trajectory=trajectory,
                                         goal_time_tolerance=rospy.Duration(nsecs=500_000_000))

        def kuka_limits_list_to_map(limits):
            assert len(limits) == 7, "Kuka limits must be length 7"
            return {n: val for n, val in zip(LEFT_ARM_JOINT_NAMES + RIGHT_ARM_JOINT_NAMES, limits * 2)}

        min_position_path_tol_of = kuka_limits_list_to_map(KUKA_MIN_PATH_JOINT_POSITION_TOLERANCE)
        full_speed_position_path_tol_of = kuka_limits_list_to_map(KUKA_FULL_SPEED_PATH_JOINT_POSITION_TOLERANCE)
        position_goal_tol_of = kuka_limits_list_to_map(KUKA_GOAL_JOINT_POSITION_TOLERANCE)

        min_impedance_path_tol_of = kuka_limits_list_to_map(KUKA_MIN_PATH_JOINT_IMPEDANCE_TOLERANCE)
        full_speed_impedance_path_tol_of = kuka_limits_list_to_map(KUKA_FULL_SPEED_PATH_JOINT_IMPEDANCE_TOLERANCE)
        impedance_goal_tol_of = kuka_limits_list_to_map(KUKA_GOAL_JOINT_IMPEDANCE_TOLERANCE)

        vel_fraction = self.max_velocity_scale_factor
        assert 0 <= vel_fraction <= 1, "Invalid velocity command"

        control_modes = self.get_control_modes()

        def _get_control_mode_for_joint(joint_name):
            if joint_name in LEFT_ARM_JOINT_NAMES:
                return control_modes['left']
            if joint_name in RIGHT_ARM_JOINT_NAMES:
                return control_modes['right']
            return None

        def path_tol(joint_name):
            control_mode = _get_control_mode_for_joint(joint_name).mode
            if control_mode == ControlMode.JOINT_POSITION or control_mode == ControlMode.CARTESIAN_POSE:
                tol = max(vel_fraction * full_speed_position_path_tol_of[joint_name],
                          min_position_path_tol_of[joint_name])
                return make_joint_tolerance(tol, joint_name)
            if control_mode == ControlMode.JOINT_IMPEDANCE or control_mode == ControlMode.CARTESIAN_IMPEDANCE:
                tol = max(vel_fraction * full_speed_impedance_path_tol_of[joint_name],
                          min_impedance_path_tol_of[joint_name])
                return make_joint_tolerance(tol, joint_name)
            return make_joint_tolerance(0.1, joint_name)

        def goal_tol(joint_name):
            control_mode = _get_control_mode_for_joint(joint_name).mode
            if control_mode == ControlMode.JOINT_POSITION or control_mode == ControlMode.CARTESIAN_POSE:
                tol = position_goal_tol_of[joint_name]
                return make_joint_tolerance(tol, joint_name)
            if control_mode == ControlMode.JOINT_IMPEDANCE or control_mode == ControlMode.CARTESIAN_IMPEDANCE:
                tol = impedance_goal_tol_of[joint_name]
                return make_joint_tolerance(tol, joint_name)
            return make_joint_tolerance(0.05, joint_name)

        goal.path_tolerance = [path_tol(n) for n in trajectory.joint_names]
        goal.goal_tolerance = [goal_tol(n) for n in trajectory.joint_names]
        return goal

    @staticmethod
    def get_force_norm(status: MotionStatus):
        f = np.array([status.estimated_external_wrench.x,
                      status.estimated_external_wrench.y,
                      status.estimated_external_wrench.z])
        return np.linalg.norm(f)

    def get_median_filtered_left_force(self):
        status = self.get_left_arm_status()
        f = self.get_force_norm(status)
        self.get_median_filtered_left_force.queue.append(f)
        current_history = np.array(self.get_median_filtered_left_force.queue)
        filtered_force = signal.medfilt(current_history)[-1]
        return filtered_force

    get_median_filtered_left_force.queue = collections.deque(maxlen=100)

    def get_median_filtered_right_force(self):
        status = self.get_right_arm_status()
        f = self.get_force_norm(status)
        self.get_median_filtered_right_force.queue.append(f)
        current_history = np.array(self.get_median_filtered_right_force.queue)
        filtered_force = signal.medfilt(current_history)[-1]
        return filtered_force

    get_median_filtered_right_force.queue = collections.deque(maxlen=100)

    def stop_on_force_cb(self, client, feedback):
        rospy.logwarn("wrong cb 1")
        if self.use_force_trigger:
            status = self.get_arms_statuses()
            left_force = self.get_force_norm(status['left'])
            right_force = self.get_force_norm(status['right'])
            left_force_change = np.abs(left_force - self.get_median_filtered_left_force())
            right_force_change = np.abs(right_force - self.get_median_filtered_right_force())

            rospy.logerr(f"{left_force_change} {right_force_change}")
            left_force_change_msg = Float32()
            left_force_change_msg.data = left_force_change
            self.left_force_change_sub.publish(left_force_change_msg)

            rospy.logerr(f"{right_force_change} {right_force_change}")
            right_force_change_msg = Float32()
            right_force_change_msg.data = right_force_change
            self.right_force_change_sub.publish(right_force_change_msg)

            stop = left_force_change > self.force_trigger or right_force_change > self.force_trigger
            if stop:
                rospy.logwarn("CANCELING!")
                client.cancel_all_goals()

    def connect(self):
        super().connect()
        rospy.loginfo(Fore.GREEN + "Victor ready!" + Fore.RESET)
