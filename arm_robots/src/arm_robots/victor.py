#! /usr/bin/env python
import collections
from typing import List, Dict, Tuple, Optional

import numpy as np
from colorama import Fore
from scipy import signal

import moveit_commander
import rospy
from actionlib import SimpleActionClient
from arc_utilities.conversions import convert_to_pose_msg, normalize_quaternion, convert_to_positions
from arc_utilities.listener import Listener
from arm_robots.base_robot import DualArmRobot
from arm_robots.robot import MoveitEnabledRobot
from control_msgs.msg import FollowJointTrajectoryFeedback, FollowJointTrajectoryGoal
from moveit_msgs.msg import DisplayRobotState
from sensor_msgs.msg import JointState
from std_msgs.msg import String, Float32
from trajectory_msgs.msg import JointTrajectoryPoint
from victor_hardware_interface.victor_utils import get_control_mode_params, list_to_jvq, default_gripper_command
from victor_hardware_interface_msgs.msg import ControlMode, MotionStatus, MotionCommand, Robotiq3FingerCommand, \
    Robotiq3FingerStatus
from victor_hardware_interface_msgs.srv import SetControlMode, GetControlMode, GetControlModeRequest, \
    GetControlModeResponse, SetControlModeResponse

# NOTE: moveit has all of this stuff specified, can we use moveit here? without relying on its execution capabilities?
left_gripper_joints = [
    "left_finger_a",
    "left_finger_b",
    "left_finger_c",
    "left_scissor",
]

right_gripper_joints = [
    "right_finger_a",
    "right_finger_b",
    "right_finger_c",
    "right_scissor",
]

left_arm_joints = [
    'victor_left_arm_joint_1',
    'victor_left_arm_joint_2',
    'victor_left_arm_joint_3',
    'victor_left_arm_joint_4',
    'victor_left_arm_joint_5',
    'victor_left_arm_joint_6',
    'victor_left_arm_joint_7',
]

right_arm_joints = [
    'victor_right_arm_joint_1',
    'victor_right_arm_joint_2',
    'victor_right_arm_joint_3',
    'victor_right_arm_joint_4',
    'victor_right_arm_joint_5',
    'victor_right_arm_joint_6',
    'victor_right_arm_joint_7',
]

both_arm_joints = left_arm_joints + right_arm_joints


def default_robotiq_command():
    cmd = Robotiq3FingerCommand()
    cmd.finger_a_command.speed = 1.0
    cmd.finger_a_command.force = 1.0
    cmd.finger_b_command.speed = 1.0
    cmd.finger_b_command.force = 1.0
    cmd.finger_c_command.speed = 1.0
    cmd.finger_c_command.force = 1.0
    cmd.scissor_command.speed = 1.0
    cmd.scissor_command.force = 1.0
    return cmd


def delegate_positions_to_arms(positions, joint_names: List[str]):
    """
    Given the list of positions, assumed to be in the same order as the list of joint names,
    determine which actual robot command things it goes to
    """
    msg = ""
    abort = False
    left_arm_positions = None
    right_arm_positions = None
    left_gripper_positions = None
    right_gripper_positions = None
    # set equality ignores order
    if set(joint_names) == set(left_arm_joints):
        left_arm_positions = []
        for joint_name, joint_value in zip(joint_names, positions):
            if joint_name in left_arm_joints:
                left_arm_positions.append(joint_value)
    elif set(joint_names) == set(right_arm_joints):
        right_arm_positions = []
        for joint_name, joint_value in zip(joint_names, positions):
            if joint_name in right_arm_joints:
                right_arm_positions.append(joint_value)
    elif set(joint_names) == set(both_arm_joints):
        left_arm_positions = []
        right_arm_positions = []
        for joint_name, joint_value in zip(joint_names, positions):
            if joint_name in left_arm_joints:
                left_arm_positions.append(joint_value)
        for joint_name, joint_value in zip(joint_names, positions):
            if joint_name in right_arm_joints:
                right_arm_positions.append(joint_value)
    elif set(joint_names) == set(left_gripper_joints):
        left_gripper_positions = []
        for joint_name, joint_value in zip(joint_names, positions):
            if joint_name in left_gripper_joints:
                left_gripper_positions.append(joint_value)
    elif set(joint_names) == set(right_gripper_joints):
        right_gripper_positions = []
        for joint_name, joint_value in zip(joint_names, positions):
            if joint_name in right_gripper_joints:
                right_gripper_positions.append(joint_value)
    else:
        msg = f"Invalid joint_names [{joint_names}]"
        abort = True
    positions_by_interface = {
        'right_arm': right_arm_positions,
        'left_arm': left_arm_positions,
        'right_gripper': right_gripper_positions,
        'left_gripper': left_gripper_positions,
    }
    return positions_by_interface, abort, msg


left_impedance_switch_config = [-0.694, 0.14, -0.229, -1.11, -0.512, 1.272, 0.077]
right_impedance_switch_config = [0.724, 0.451, 0.94, -1.425, 0.472, 0.777, -0.809]


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

    def send_joint_command(self, joint_names: List[str], trajectory_point: JointTrajectoryPoint) -> Tuple[bool, str]:
        # TODO: in victor's impedance mode, we want to modify the setpoint so that there is a limit
        #  on the force we will apply
        positions_by_interface, abort, msg = delegate_positions_to_arms(trajectory_point.positions, joint_names)
        left_arm_positions = positions_by_interface['left_arm']
        right_arm_positions = positions_by_interface['right_arm']
        left_gripper_positions = positions_by_interface['left_gripper']
        right_gripper_positions = positions_by_interface['right_gripper']

        if abort:
            return True, msg

        # Get the current control mode
        control_mode = self.get_control_modes()
        left_arm_control_mode = control_mode['left']
        right_arm_control_mode = control_mode['right']

        self.send_arm_command(self.left_arm_command_pub, left_arm_control_mode, left_arm_positions)
        self.send_arm_command(self.right_arm_command_pub, right_arm_control_mode, right_arm_positions)
        self.send_gripper_command(self.left_gripper_command_pub, left_gripper_positions)
        self.send_gripper_command(self.right_gripper_command_pub, right_gripper_positions)

        return False, ""

    def send_arm_command(self, command_pub: rospy.Publisher, right_arm_control_mode: ControlMode, positions):
        if positions is not None:
            # FIXME: what if we allow the BaseRobot class to use moveit, but just don't have it require that
            # any actions are running?
            # NOTE: why are these values not checked by the lower-level code? the Java code knows what the joint limits
            # are so why does it not enforce them?
            
            # TODO: use enforce bounds? https://github.com/ros-planning/moveit/pull/2356
            limit_enforced_positions = []
            for i, joint_name in enumerate(right_arm_joints):
                joint: moveit_commander.RobotCommander.Joint = self.robot_commander.get_joint(joint_name)
                limit_enforced_position = np.clip(positions[i], joint.min_bound() + 1e-2, joint.max_bound() - 1e-2)
                limit_enforced_positions.append(limit_enforced_position)
            cmd = MotionCommand()
            cmd.header.stamp = rospy.Time.now()
            cmd.joint_position = list_to_jvq(limit_enforced_positions)
            cmd.control_mode = right_arm_control_mode
            command_pub.publish(cmd)

    def get_gripper_statuses(self):
        return {'left': self.get_left_gripper_status(),
                'right': self.get_right_gripper_status()}

    def get_right_gripper_status(self):
        right_status: Robotiq3FingerStatus = self.right_gripper_status_listener.get()
        return right_status

    def get_left_gripper_status(self):
        left_status: Robotiq3FingerStatus = self.left_gripper_status_listener.get()
        return left_status

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
        return {'left': self.get_left_arm_status(),
                'right': self.get_right_arm_status()}

    def get_right_arm_status(self):
        right_status: MotionStatus = self.right_arm_status_listener.get()
        return right_status

    def get_left_arm_status(self):
        left_status: MotionStatus = self.left_arm_status_listener.get()
        return left_status

    def get_control_modes(self):
        return {'left': self.get_left_arm_control_mode(), 'right': self.get_right_arm_control_mode()}

    def set_control_mode(self, control_mode: ControlMode, **kwargs):
        left_res = self.set_left_arm_control_mode(control_mode, **kwargs)
        right_res = self.set_right_arm_control_mode(control_mode, **kwargs)
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
            'left': desired_left_pose,
            'right': desired_right_pose,
        }
        self.send_cartesian_command(poses)

    @staticmethod
    def send_gripper_command(command_pub: rospy.Publisher, positions):
        if positions is not None:
            cmd = default_gripper_command()
            cmd.header.stamp = rospy.Time.now()
            cmd.finger_a_command.position = positions[0]
            cmd.finger_b_command.position = positions[1]
            cmd.finger_c_command.position = positions[2]
            cmd.scissor_command.position = positions[3]
            command_pub.publish(cmd)

    def get_joint_positions(self, joint_names: Optional[List[str]] = None):
        """
        :args joint_names an optional list of names if you want to have a specific order or a subset
        """
        joint_state: JointState = self.joint_state_listener.get()
        if joint_names is None:
            return joint_state.position

        gripper_statuses = self.get_gripper_statuses()
        current_joint_positions = []
        for name in joint_names:
            if name in joint_state.name:
                idx = joint_state.name.index(name)
                pos = joint_state.position[idx]
                current_joint_positions.append(pos)
            elif name == 'left_finger_a':
                current_joint_positions.append(gripper_statuses['left'].finger_a_status.position)
            elif name == 'left_finger_b':
                current_joint_positions.append(gripper_statuses['left'].finger_b_status.position)
            elif name == 'left_finger_c':
                current_joint_positions.append(gripper_statuses['left'].finger_c_status.position)
            elif name == 'left_scissor':
                current_joint_positions.append(gripper_statuses['left'].scissor_status.position)
            elif name == 'right_finger_a':
                current_joint_positions.append(gripper_statuses['right'].finger_a_status.position)
            elif name == 'right_finger_b':
                current_joint_positions.append(gripper_statuses['right'].finger_b_status.position)
            elif name == 'right_finger_c':
                current_joint_positions.append(gripper_statuses['right'].finger_c_status.position)
            elif name == 'right_scissor':
                current_joint_positions.append(gripper_statuses['right'].scissor_status.position)
            else:
                raise ValueError(f"Could not get joint {name}")

        # try looking at the status messages
        return current_joint_positions

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

    def __init__(self, robot_namespace: str = 'victor', force_trigger: float = -0.0):
        MoveitEnabledRobot.__init__(self,
                                    robot_namespace=robot_namespace,
                                    arms_controller_name='both_arms_trajectory_controller')
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

    def move_to_impedance_switch(self, actually_switch: bool = True):
        self.plan_to_joint_config("right_arm", right_impedance_switch_config)
        self.plan_to_joint_config("left_arm", left_impedance_switch_config)
        if actually_switch:
            return self.set_control_mode(ControlMode.JOINT_IMPEDANCE)
        return True

    def get_right_gripper_joints(self):
        return right_gripper_joints

    def get_left_gripper_joints(self):
        return left_gripper_joints

    def get_joint_positions(self, joint_names: Optional[List[str]] = None):
        return self.get_joint_positions(joint_names)

    def speak(self, message: str):
        msg = String()
        msg.data = message
        self.polly_pub.publish(msg)

    def get_right_arm_joints(self):
        return right_arm_joints

    def get_left_arm_joints(self):
        return left_arm_joints

    def follow_joint_trajectory_feedback_cb(self,
                                            client: SimpleActionClient,
                                            goal: FollowJointTrajectoryGoal,
                                            feedback: FollowJointTrajectoryFeedback):
        for feedback_callback in self.feedback_callbacks:
            feedback_callback(client, goal, feedback)

    def get_force_norm(self, status: MotionStatus):
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
        rospy.loginfo(Fore.GREEN + "Victor ready!")
