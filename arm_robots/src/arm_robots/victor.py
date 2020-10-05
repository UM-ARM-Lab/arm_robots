#! /usr/bin/env python
from typing import List, Dict, Tuple

from colorama import Fore

import rospy
from arc_utilities.conversions import convert_to_pose_msg, normalize_quaternion, convert_to_positions
from arc_utilities.ros_helpers import Listener
from arm_robots.base_robot import BaseRobot
from arm_robots.robot import MoveitEnabledRobot
from moveit_msgs.msg import DisplayRobotState
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint
from victor_hardware_interface.victor_utils import get_new_control_mode, list_to_jvq
from victor_hardware_interface_msgs.msg import ControlMode, MotionStatus, MotionCommand
from victor_hardware_interface_msgs.srv import SetControlMode, GetControlMode, GetControlModeRequest, \
    GetControlModeResponse

left_gripper_joint_names = [
    "victor_left_gripper_fingerA_joint_2",
    "victor_left_gripper_fingerA_joint_3",
    "victor_left_gripper_fingerA_joint_4",
    "victor_left_gripper_fingerB_joint_2",
    "victor_left_gripper_fingerB_joint_3",
    "victor_left_gripper_fingerB_joint_4",
    "victor_left_gripper_fingerB_knuckle",
    "victor_left_gripper_fingerC_joint_2",
    "victor_left_gripper_fingerC_joint_3",
    "victor_left_gripper_fingerC_joint_4",
    "victor_left_gripper_fingerC_knuckle",
]

right_gripper_joint_names = [
    "victor_right_gripper_fingerA_joint_2",
    "victor_right_gripper_fingerA_joint_3",
    "victor_right_gripper_fingerA_joint_4",
    "victor_right_gripper_fingerB_joint_2",
    "victor_right_gripper_fingerB_joint_3",
    "victor_right_gripper_fingerB_joint_4",
    "victor_right_gripper_fingerB_knuckle",
    "victor_right_gripper_fingerC_joint_2",
    "victor_right_gripper_fingerC_joint_3",
    "victor_right_gripper_fingerC_joint_4",
    "victor_right_gripper_fingerC_knuckle",
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


def delegate_positions_to_arms(positions, joint_names: List[str]):
    """
    Given the list of positions, assumed to be in the same order as the list of joint names,
    determine whether it's meant for the left arm, right arm, or both arms, and error if it doesn't perfectly match any
    """
    msg = ""
    abort = False
    left_arm_positions = None
    right_arm_positions = None
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
    else:
        msg = "Invalid joint_names, didn't match the left arm joints, right arm joints, or all joints"
        abort = True
    return right_arm_positions, left_arm_positions, abort, msg


left_impedance_switch_config = [-0.694, 0.14, -0.229, -1.11, -0.512, 1.272, 0.077]
right_impedance_switch_config = [0.724, 0.451, 0.94, -1.425, 0.472, 0.777, -0.809]


class BaseVictor(BaseRobot):

    def __init__(self, robot_namespace: str):
        super().__init__(robot_namespace=robot_namespace)

        self.left_arm_motion_command_pub = rospy.Publisher("left_arm/motion_command", MotionCommand, queue_size=10)
        self.right_arm_motion_command_pub = rospy.Publisher("right_arm/motion_command", MotionCommand, queue_size=10)

        self.left_set_control_mode_srv = rospy.ServiceProxy("/left_arm/set_control_mode_service", SetControlMode)
        self.right_set_control_mode_srv = rospy.ServiceProxy("/right_arm/set_control_mode_service", SetControlMode)

        self.left_get_control_mode_srv = rospy.ServiceProxy("/left_arm/get_control_mode_service", GetControlMode)
        self.right_get_control_mode_srv = rospy.ServiceProxy("/right_arm/get_control_mode_service", GetControlMode)

        self.left_motion_status_listener = Listener("/left_arm/motion_status", MotionStatus)
        self.right_motion_status_listener = Listener("/right_arm/motion_status", MotionStatus)

        self.waypoint_state_pub = rospy.Publisher("waypoint_robot_state", DisplayRobotState, queue_size=10)

    def send_joint_command(self,
                           now: rospy.Time,
                           joint_names: List[str],
                           trajectory_point: JointTrajectoryPoint) -> Tuple[bool, str]:
        # TODO: in victor's impedance mode, we want to modify the setpoint so that there is a limit
        #  on the force we will apply
        right_arm_positions, left_arm_positions, abort, msg = delegate_positions_to_arms(trajectory_point.positions,
                                                                                         joint_names)
        if abort is not None:
            return False, msg

        # Get the current control mode
        control_mode = self.get_control_mode()
        left_arm_control_mode = control_mode['left']
        right_arm_control_mode = control_mode['right']

        # command waypoint
        waypoint_joint_state = JointState()
        waypoint_joint_state.header.stamp = now
        if left_arm_positions is not None:
            left_arm_command = MotionCommand()
            left_arm_command.header.stamp = now
            left_arm_command.joint_position = list_to_jvq(left_arm_positions)
            left_arm_command.control_mode = left_arm_control_mode
            self.left_arm_motion_command_pub.publish(left_arm_command)
            waypoint_joint_state.name.extend(left_arm_joints)
            waypoint_joint_state.position.extend(left_arm_positions)

        if right_arm_positions is not None:
            right_arm_command = MotionCommand()
            right_arm_command.header.stamp = now
            right_arm_command.joint_position = list_to_jvq(right_arm_positions)
            right_arm_command.control_mode = right_arm_control_mode
            self.right_arm_motion_command_pub.publish(right_arm_command)
            waypoint_joint_state.name.extend(right_arm_joints)
            waypoint_joint_state.position.extend(right_arm_positions)

        # waypoint_state = DisplayRobotState()
        # waypoint_state.state.joint_state = waypoint_joint_state
        # # TODO: need a non-moveit way to get list of links... urdf_py from rosparam?
        # for link_name in self.robot_commander.get_link_names():
        #     object_color = ObjectColor()
        #     object_color.id = link_name
        #     object_color.color = make_color(0, 1, 0, 1)
        #     waypoint_state.highlight_links.append(object_color)
        #
        # self.waypoint_state_pub.publish(waypoint_state)

        return True, ""

    def get_motion_status(self) -> Dict[str, MotionStatus]:
        return {'left': self.left_motion_status_listener.get(),
                'right': self.right_motion_status_listener.get()}

    def get_right_motion_status(self) -> MotionStatus:
        right_status = self.right_motion_status_listener.get()
        return right_status

    def get_left_motion_status(self) -> MotionStatus:
        left_status = self.left_motion_status_listener.get()
        return left_status

    def get_control_mode(self) -> Dict[str, ControlMode]:
        return {'left': self.get_left_arm_control_mode(), 'right': self.get_right_arm_control_mode()}

    def set_control_mode(self, control_mode: ControlMode, **kwargs):
        self.set_left_arm_control_mode(control_mode, **kwargs)
        self.set_right_arm_control_mode(control_mode, **kwargs)

    def get_left_arm_control_mode(self) -> ControlMode:
        left_control_mode_res: GetControlModeResponse = self.left_get_control_mode_srv(GetControlModeRequest())
        return left_control_mode_res.active_control_mode.control_mode

    def get_right_arm_control_mode(self) -> ControlMode:
        right_control_mode_res: GetControlModeResponse = self.right_get_control_mode_srv(GetControlModeRequest())
        return right_control_mode_res.active_control_mode.control_mode

    def set_right_arm_control_mode(self, control_mode: ControlMode, **kwargs):
        new_control_mode = get_new_control_mode(control_mode, **kwargs)
        res = self.right_set_control_mode_srv(new_control_mode)

        if not res.success:
            rospy.logerr("Failed to switch right arm to control mode: " + str(control_mode))
            rospy.logerr(res.message)

    def set_left_arm_control_mode(self, control_mode: ControlMode, **kwargs):
        new_control_mode = get_new_control_mode(control_mode, **kwargs)
        res = self.left_set_control_mode_srv(new_control_mode)

        if not res.success:
            rospy.logerr("Failed to switch left arm to control mode: " + str(control_mode))
            rospy.logerr(res.message)

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
        while self.left_arm_motion_command_pub.get_num_connections() < 1:
            rospy.sleep(0.01)

        self.left_arm_motion_command_pub.publish(left_arm_command)

    def send_right_arm_cartesian_command(self, pose_stamped):
        pose_stamped = convert_to_pose_msg(pose_stamped)
        pose_stamped.pose.orientation = normalize_quaternion(pose_stamped.pose.orientation)

        right_arm_command = MotionCommand()
        right_arm_command.header.frame_id = 'victor_right_arm_world_frame_kuka'
        right_arm_command.cartesian_pose = pose_stamped.pose
        right_arm_command.control_mode = self.get_right_arm_control_mode()
        while self.right_arm_motion_command_pub.get_num_connections() < 1:
            rospy.sleep(0.01)

        self.right_arm_motion_command_pub.publish(right_arm_command)

    def send_delta_cartesian_command(self, delta_positions):
        delta_positions = convert_to_positions(delta_positions)
        motion_status = self.get_motion_status()

        current_left_pose = motion_status['left'].measured_cartesian_pose
        desired_left_pose = current_left_pose
        desired_left_pose.position.x += delta_positions['left'].x
        desired_left_pose.position.y += delta_positions['left'].y
        desired_left_pose.position.z += delta_positions['left'].z

        current_right_pose = motion_status['right'].measured_cartesian_pose
        desired_right_pose = current_right_pose
        desired_right_pose.position.x += delta_positions['right'].x
        desired_right_pose.position.y += delta_positions['right'].y
        desired_right_pose.position.z += delta_positions['right'].z

        poses = {
            'left': desired_left_pose,
            'right': desired_right_pose,
        }
        self.send_cartesian_command(poses)


class Victor(MoveitEnabledRobot):

    def __init__(self, robot_namespace: str = 'victor'):
        self.base_victor = BaseVictor(robot_namespace)
        super().__init__(self.base_victor,
                         arms_controller_name='both_arms_trajectory_controller',
                         left_gripper_controller_name='left_hand_trajectory_controller',
                         right_gripper_controller_name='right_hand_trajectory_controller',
                         )
        rospy.loginfo(Fore.GREEN + "Victor ready!")

    def move_to_impedance_switch(self, actually_switch: bool = True):
        self.plan_to_joint_config("right_arm", right_impedance_switch_config)
        self.plan_to_joint_config("left_arm", left_impedance_switch_config)
        if actually_switch:
            self.base_victor.set_control_mode(ControlMode.JOINT_IMPEDANCE)

    def get_joint_trajectory_controller_name(self):
        return "both_arms_trajectory_controller"

    def get_right_gripper_joints(self):
        return right_gripper_joint_names

    def get_left_gripper_joints(self):
        return left_gripper_joint_names

    def get_gripper_closed_positions(self):
        return [1, -0.25, -0.5,
                1, -0.25, -0.5, 0.15,
                1, -0.25, -0.5, 0.15]

    def get_gripper_open_positions(self):
        return [0, 0, 0,
                0, 0, 0, 0,
                0, 0, 0, 0]
