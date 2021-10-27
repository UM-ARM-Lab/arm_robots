from typing import List, Tuple, Optional

import moveit_commander
import rospy
from arc_utilities import ros_helpers
from arc_utilities.listener import Listener
from arc_utilities.tf2wrapper import TF2Wrapper
from rosgraph.names import ns_join
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint
from arm_robots.cartesian import CartesianImpedanceController


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
            self.cartesian = None

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

    def get_joint_limits(self, joint_names: List[str], safety_margin=1e-2):
        """
        Get joint limits in radians with a safety margin
        """
        lower, upper = [], []
        for joint_name in joint_names:
            joint: moveit_commander.RobotCommander.Joint = self.robot_commander.get_joint(joint_name)
            lower.append(joint.min_bound() + safety_margin)
            upper.append(joint.max_bound() - safety_margin)
        return lower, upper

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

    def create_cartesian_impedance_controller(self, motion_status_listeners: List[Listener],
                                              motion_command_publishers: List[rospy.Publisher],
                                              joint_names: List[str],
                                              world_frame_name: str,
                                              **kwargs):
        """
        Create the cartesian impedance controller. The number of listeners should match the number of publishers and
        each element corresponds to 1 arm. We assume all arms share the same joint properties, so only 1 list of joint
        names should be supplied.
        """
        lower, upper = self.get_joint_limits(joint_names, safety_margin=0)
        self.cartesian = CartesianImpedanceController(self.tf_wrapper.tf_buffer,
                                                      motion_status_listeners,
                                                      motion_command_publishers,
                                                      lower, upper, world_frame_name, **kwargs)

    def move_delta_cartesian_impedance(self, arm, dx, dy, target_z=None, target_orientation=None,
                                       step_size=0.005, blocking=True):
        if self.cartesian is None:
            return False

        self.cartesian.set_active_arm(arm)
        if not self.cartesian.set_relative_goal_2d(dx, dy, target_z=target_z, target_orientation=target_orientation):
            return False
        succeeded = self.cartesian.step(step_size)
        if blocking:
            # TODO add a rospy.Rate and sleep here?
            while self.cartesian.target_pose is not None:
                succeeded = self.cartesian.step(step_size)
        return succeeded
