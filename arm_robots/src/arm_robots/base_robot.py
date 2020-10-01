import pathlib
from typing import List, Tuple, Optional

import rospy
from arc_utilities import ros_helpers
from arc_utilities.ros_helpers import Listener, TF2Wrapper
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint


class BaseRobot:
    def __init__(self, robot_namespace: str = ''):
        self.robot_namespace = robot_namespace
        joint_states_topic = pathlib.Path(self.robot_namespace) / 'joint_states'
        self.joint_state_listener = Listener(joint_states_topic.as_posix(), JointState)

        self.tf_wrapper = TF2Wrapper()

    def send_joint_command(self,
                           now: rospy.Time,
                           joint_names: List[str],
                           trajectory_point: JointTrajectoryPoint) -> Tuple[bool, str]:
        raise NotImplementedError()

    def get_joint_positions(self, joint_names: Optional[List[str]] = None):
        """
        :args joint_names an optional list of names if you want to have a specific order or a subset
        """
        joint_state: JointState = self.joint_state_listener.get()
        current_joint_positions = []
        for name in joint_names:
            if name not in joint_state.name:
                ros_helpers.logfatal(ValueError, f"Joint {name} not found in joint states")
            idx = joint_state.name.index(name)
            pos = joint_state.position[idx]
            current_joint_positions.append(pos)
        return current_joint_positions