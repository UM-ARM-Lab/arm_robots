from typing import List, Tuple, Optional

import moveit_commander
import rospy
from arc_utilities import ros_helpers
from arc_utilities.ros_helpers import Listener, TF2Wrapper
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint


class BaseRobot:
    def __init__(self, robot_namespace: str = ''):
        self.robot_namespace = robot_namespace
        # the robot namespace will be prepended by setting ROS_NAMESPACE environment variable or the ns="" in roslaunch
        joint_states_topic = 'joint_states'
        self.joint_state_listener = Listener(joint_states_topic, JointState)

        self.tf_wrapper = TF2Wrapper()

        self.robot_commander = moveit_commander.RobotCommander()

    def send_joint_command(self, joint_names: List[str], trajectory_point: JointTrajectoryPoint) -> Tuple[bool, str]:
        raise NotImplementedError()

    def get_joint_positions(self, joint_names: Optional[List[str]] = None):
        """
        :args joint_names an optional list of names if you want to have a specific order or a subset
        """
        joint_state: JointState = self.joint_state_listener.get()
        if joint_names is None:
            return joint_state.position

        current_joint_positions = []
        for name in joint_names:
            if name not in joint_state.name:
                ros_helpers.logfatal(ValueError, f"Joint {name} not found in joint states")
            idx = joint_state.name.index(name)
            pos = joint_state.position[idx]
            current_joint_positions.append(pos)
        return current_joint_positions

    def check_inputs(self, group_name: str, ee_link_name: str):
        links = self.robot_commander.get_link_names()
        if ee_link_name not in links:
            rospy.logwarn_throttle(1, f"Link [{ee_link_name}] does not exist. Existing links are:")
            rospy.logwarn_throttle(1, links)

        groups = self.robot_commander.get_group_names()
        if group_name not in groups:
            rospy.logwarn_throttle(1, f"Group [{group_name}] does not exist. Existing groups are:")
            rospy.logwarn_throttle(1, groups)

    def get_right_gripper_links(self):
        return self.robot_commander.get_link_names("right_gripper")

    def get_left_gripper_links(self):
        return self.robot_commander.get_link_names("left_gripper")
