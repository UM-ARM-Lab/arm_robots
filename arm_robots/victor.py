#! /usr/bin/env python
from typing import Sequence, Optional

from rclpy.node import Node
from sensor_msgs.msg import JointState

from arm_utilities.listener import Listener
from victor_hardware.victor_utils import get_control_mode_params, is_gripper_closed, DEFAULT_JOINT_VEL, \
    get_gripper_open_fraction_msg
from victor_hardware_interfaces.msg import MotionCommand, MotionStatus, Robotiq3FingerStatus, Robotiq3FingerCommand, ControlMode
from victor_hardware_interfaces.srv import SetControlMode, GetControlMode


class Victor:
    def __init__(self, node: Node):
        self.node = node

        self.left_arm_cmd_pub = node.create_publisher(MotionCommand, "victor/left_arm/motion_command", 10)
        self.right_arm_cmd_pub = node.create_publisher(MotionCommand, "victor/right_arm/motion_command", 10)
        self.left_gripper_cmd_pub = node.create_publisher(Robotiq3FingerCommand, "victor/left_arm/gripper_command", 10)
        self.right_gripper_cmd_pub = node.create_publisher(Robotiq3FingerCommand, "victor/right_arm/gripper_command", 10)

        self.left_set_control_mode_srv = node.create_client(SetControlMode, "victor/left_arm/set_control_mode_service")
        self.right_set_control_mode_srv = node.create_client(SetControlMode, "victor/right_arm/set_control_mode_service")
        self.left_get_control_mode_srv = node.create_client(SetControlMode, "victor/left_arm/get_control_mode_service")
        self.right_get_control_mode_srv = node.create_client(SetControlMode, "victor/right_arm/get_control_mode_service")

        self.joint_states_listener = Listener(node, JointState, "joint_states", 10)
        self.left_arm_status_listener = Listener(node, MotionStatus, "victor/left_arm/motion_status", 10)
        self.right_arm_status_listener = Listener(node, MotionStatus, "victor/right_arm/motion_status", 10)
        self.left_gripper_status_listener = Listener(node, Robotiq3FingerStatus, "victor/left_arm/gripper_status", 10)
        self.right_gripper_status_listener = Listener(node, Robotiq3FingerStatus, "victor/right_arm/gripper_status", 10)

        # kwargs = cartesian_impedance_controller_kwargs or {}
        # self.create_cartesian_impedance_controller([self.left_arm_status_listener, self.right_arm_status_listener],
        #                                            [self.left_arm_cmd_pub, self.right_arm_cmd_pub],
        #                                            RIGHT_ARM_JOINT_NAMES, "victor_root",
        #                                            sensor_frame_names=["victor_left_arm_world_frame_kuka",
        #                                                                "victor_right_arm_world_frame_kuka"], **kwargs)

    def open_left_gripper(self, position=1.):
        self.left_gripper_cmd_pub.publish(get_gripper_open_fraction_msg(position))

    def open_right_gripper(self, position=1.):
        self.right_gripper_cmd_pub.publish(get_gripper_open_fraction_msg(position))

    def close_left_gripper(self):
        # TODO: implementing blocking grasping
        self.left_gripper_cmd_pub.publish(get_gripper_open_fraction_msg(0.0))

    def close_right_gripper(self):
        self.right_gripper_cmd_pub.publish(get_gripper_open_fraction_msg(0.0))

    def get_joint_states(self):
        return self.joint_states_listener.get()

    def get_right_gripper_status(self) -> Robotiq3FingerStatus:
        return self.right_gripper_status_listener.get()

    def get_left_gripper_status(self) -> Robotiq3FingerStatus:
        return self.left_gripper_status_listener.get()

    def is_left_gripper_closed(self):
        return self.is_gripper_closed('left')

    def is_right_gripper_closed(self):
        return self.is_gripper_closed('right')

    def is_gripper_closed(self, gripper: str):
        if gripper == 'left':
            status = self.get_left_gripper_status()
        elif gripper == 'right':
            status = self.get_right_gripper_status()
        else:
            raise NotImplementedError(f"invalid gripper {gripper}")
        return is_gripper_closed(status)

    def get_arms_statuses(self):
        return {'left': self.get_left_arm_status(),
                'right': self.get_right_arm_status()}

    def get_right_arm_status(self) -> MotionStatus:
        return self.right_arm_status_listener.get()

    def get_left_arm_status(self) -> MotionStatus:
        return self.left_arm_status_listener.get()

    def get_control_modes(self):
        return {'left': self.get_left_arm_control_mode(), 'right': self.get_right_arm_control_mode()}

    def set_control_mode(self, control_mode: ControlMode, vel=DEFAULT_JOINT_VEL, **kwargs):
        left_res = self.set_left_arm_control_mode(control_mode, vel=vel, **kwargs)
        right_res = self.set_right_arm_control_mode(control_mode, vel=vel, **kwargs)
        return left_res, right_res

    def get_left_arm_control_mode(self):
        left_control_mode_res: GetControlMode.Response = self.left_get_control_mode_srv.call(GetControlMode.Request())
        return left_control_mode_res.active_control_mode.control_mode

    def get_right_arm_control_mode(self):
        right_control_mode_res: GetControlMode.Response = self.right_get_control_mode_srv.call(GetControlMode.Request())
        return right_control_mode_res.active_control_mode.control_mode

    def set_right_arm_control_mode(self, control_mode: ControlMode, **kwargs):
        new_control_mode = get_control_mode_params(control_mode, **kwargs)
        res: SetControlMode.Response = self.right_set_control_mode_srv.call(new_control_mode)

        if not res.success:
            print("Failed to switch right arm to control mode: " + str(control_mode))
            print(res.message)
        return res

    def set_left_arm_control_mode(self, control_mode: ControlMode, **kwargs):
        new_control_mode = get_control_mode_params(control_mode, **kwargs)
        res: SetControlMode.Response = self.left_set_control_mode_srv.call(new_control_mode)

        if not res.success:
            print("Failed to switch left arm to control mode: " + str(control_mode))
            print(res.message)
        return res

    def get_joint_positions(self, joint_names: Optional[Sequence[str]] = None):
        position_of_joint = self.get_joint_positions_dict()
        return [position_of_joint[name] for name in joint_names]

    def get_joint_positions_dict(self):
        joint_state = self.get_joint_states()
        joint_positions = dict(zip(joint_state.name, joint_state.position))
        return joint_positions
