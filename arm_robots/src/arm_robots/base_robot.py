from typing import List, Tuple, Optional

import moveit_commander
import rospy
from arc_utilities import ros_helpers
from arc_utilities.listener import Listener
from arc_utilities.tf2wrapper import TF2Wrapper
from rosgraph.names import ns_join
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint


class BaseRobot:

    def __init__(self, robot_namespace: str = '', robot_description: str = 'robot_description'):
        """
        This class is designed around the needs of the trajectory_follower.TrajectoryFollower
        This class really only contains API that is needed by that class. The trajectory follower only needs to know about the
        basic ROS API -- how to send and get joint commands. However, because MoveIt trajectory execution relies (for now) on that
        TrajectoryFollower class, we do not want BaseRobot to use any MoveIt TrajectoryExeuction or rely on any trajectory
        execution services.
        """
        self.robot_namespace = robot_namespace
        self.robot_description = rospy.resolve_name(robot_description)
        # the robot namespace will be prepended by setting ROS_NAMESPACE environment variable or the ns="" in roslaunch
        self.joint_states_topic = ns_join(self.robot_namespace, 'joint_states')
        self._joint_state_listener = Listener(self.joint_states_topic, JointState)

        self.tf_wrapper = TF2Wrapper()
        try:
            self.robot_commander = moveit_commander.RobotCommander(ns=self.robot_namespace,
                                                                   robot_description=self.robot_description)
        except RuntimeError as e:
            rospy.logerr("You may need to load the moveit planning context and robot description")
            print(e)

    def __del__(self):
        self.disconnect()

    def connect(self):
        pass

    def disconnect(self):
        pass

    def ns(self, name: str):
        return ns_join(self.robot_namespace, name)

    def get_joint_state_listener(self):
        return self._joint_state_listener

    def send_joint_command(self, joint_names: List[str], trajectory_point: JointTrajectoryPoint) -> Tuple[bool, str]:
        raise NotImplementedError()

    def get_joint_velocities(self, joint_names: Optional[List[str]] = None):
        """
        :args joint_names an optional list of names if you want to have a specific order or a subset
        """
        joint_state: JointState = self._joint_state_listener.get()
        if joint_names is None:
            return joint_state.velocity

        current_joint_velocities = []
        for name in joint_names:
            if name not in joint_state.name:
                ros_helpers.logfatal(ValueError, f"Joint {name} not found in joint states")
            idx = joint_state.name.index(name)
            pos = joint_state.position[idx]
            current_joint_velocities.append(pos)
        return current_joint_velocities

    def get_joint_positions(self, joint_names: Optional[List[str]] = None):
        """
        :args joint_names an optional list of names if you want to have a specific order or a subset
        """
        joint_state: JointState = self._joint_state_listener.get()
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
