#! /usr/bin/env python
import pdb

import rospy
from geometry_msgs.msg import PoseStamped
from rosgraph.names import ns_join
from typing import List, Tuple

from arm_robots.robot import MoveitEnabledRobot
from trajectory_msgs.msg import JointTrajectoryPoint
from franka_msgs.srv import SetJointImpedance, SetLoad
from controller_manager_msgs.srv import LoadController, SwitchController
from moveit_msgs.srv import GetPositionIK
from moveit_msgs.msg import PositionIKRequest, RobotState, MoveItErrorCodes

DEFAULT_JOINT_IMPEDANCE = [3000.0, 3000.0, 3000.0, 2500.0, 2500.0, 2000.0, 2000.0]
SOFT_JOINT_IMPEDANCE = [100.0, 100.0, 100.0, 100.0, 50.0, 50.0, 10.0]
POSITION_JOINT_TRAJECTORY_CONTROLLER_NAME = 'position_joint_trajectory_controller'


class Panda(MoveitEnabledRobot):

    def __init__(self, robot_namespace: str = 'combined_panda', force_trigger: float = -0.0, **kwargs):
        MoveitEnabledRobot.__init__(self,
                                    robot_namespace=robot_namespace,
                                    robot_description=ns_join(robot_namespace, 'robot_description'),
                                    arms_controller_name='position_joint_trajectory_controller',
                                    force_trigger=force_trigger,
                                    **kwargs)

        # Panda HW Services - for setting internal controller parameters.
        self.joint_impedance_srv = rospy.ServiceProxy(self.ns('franka_control/set_joint_impedance'), SetJointImpedance)
        self.set_load_srv = rospy.ServiceProxy(self.ns('franka_control/set_load'), SetLoad)

        # Controller Manager Services - for loading/unloading/switching controllers.
        self.load_controller_srv = rospy.ServiceProxy(self.ns('controller_manager/load_controller'), LoadController)
        self.switch_controller_srv = rospy.ServiceProxy(self.ns('controller_manager/switch_controller'),
                                                        SwitchController)

        # IK Service.
        self.ik_srv = rospy.ServiceProxy(self.ns('compute_ik'), GetPositionIK)

        # Default position joint trajectory controller.
        self.active_controller_name = POSITION_JOINT_TRAJECTORY_CONTROLLER_NAME

    def send_joint_command(self, joint_names: List[str], trajectory_point: JointTrajectoryPoint) -> Tuple[bool, str]:
        # TODO: Fill in to send set point to controller.
        pass

    # TODO: Add gripper helpers.

    def switch_controller(self, start_controllers=None, stop_controllers=None) -> bool:
        if stop_controllers is None:
            stop_controllers = []
        if start_controllers is None:
            start_controllers = []

        try:
            switch_controller_resp = self.switch_controller_srv(start_controllers=start_controllers,
                                                                stop_controllers=stop_controllers,
                                                                strictness=2)
        except rospy.ServiceException as e:
            raise Exception("Service call failed: %s" % e)

        return switch_controller_resp.ok

    def load_controller(self, controller_name: str) -> bool:
        try:
            load_controller_resp = self.load_controller_srv(controller_name)
        except rospy.ServiceException as e:
            raise Exception("Service call failed: %s" % e)

        return load_controller_resp.ok

    def set_joint_impedance(self, joint_impedance: List[float]) -> bool:
        # Start by switching off the current controller, if active.
        if self.active_controller_name is not None:
            self.switch_controller(stop_controllers=[self.active_controller_name])

        # Change joint impedance.
        try:
            set_joint_imped_resp = self.joint_impedance_srv(joint_impedance)
        except rospy.ServiceException as e:
            raise Exception("Failed to set impedance: %s" % e)

        if not set_joint_imped_resp.success:
            raise Exception("Failed to set impedance: %s" % set_joint_imped_resp.error)

        # Switch to joint position controller.
        self.switch_controller(start_controllers=[POSITION_JOINT_TRAJECTORY_CONTROLLER_NAME])
        self.active_controller_name = POSITION_JOINT_TRAJECTORY_CONTROLLER_NAME

        return set_joint_imped_resp.success

    def set_load(self, mass: float, load_com: List[float], load_inertia: List[float]) -> bool:
        """
        See also: setLoad() in libfranka Robot class.

        Parameters
        ----------
        mass - load mass in kg
        load_com - Translation from flange to center of mass in meters
        load_inertia - Inertia matrix in kgxm^2, *column major*.

        Returns
        -------
        bool - whether service completed successfully.
        """
        # Start by switching off the current controller, if active.
        if self.active_controller_name is not None:
            self.switch_controller(stop_controllers=[self.active_controller_name])

        if len(load_com) != 3 or len(load_inertia) != 9:
            raise Exception("Invalid load information provided.")

        try:
            set_load_resp = self.set_load_srv(mass, load_com, load_inertia)
        except rospy.ServiceException as e:
            raise Exception("Failed to set load info: %s" % e)

        # Switch to joint position controller.
        self.switch_controller(start_controllers=[POSITION_JOINT_TRAJECTORY_CONTROLLER_NAME])
        self.active_controller_name = POSITION_JOINT_TRAJECTORY_CONTROLLER_NAME

        return set_load_resp.success

    def get_ik(self, pose: PoseStamped, frame: str = "panda_link8"):
        ik_request = PositionIKRequest()
        # TODO: Select move group.
        ik_request.group_name = "panda_arm"
        ik_request.robot_state = self.get_state(group_name="panda_arm")
        ik_request.ik_link_name = frame
        ik_request.pose_stamped = pose
        ik_request.timeout.secs = 10

        try:
            ik_response = self.ik_srv(ik_request=ik_request)
        except rospy.ServiceException as e:
            raise Exception("Failed to get ik response: %s" % e)

        if ik_response.error_code.val != MoveItErrorCodes.SUCCESS:
            rospy.loginfo("IK call failed with code: %d" % ik_response.error_code.val)
            return None

        return ik_response.solution.joint_state.position[:7]
