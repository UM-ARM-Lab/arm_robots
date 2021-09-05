import copy
import enum

import numpy as np
import rospy
import ros_numpy
from geometry_msgs.msg import PoseStamped, Quaternion, Pose
from victor_hardware_interface_msgs.msg import ControlMode, MotionCommand


def quaternion_angle_diff(q1: Quaternion, q2: Quaternion):
    """Angle of rotation required to get from one orientation to another
    see https://math.stackexchange.com/a/90098/184113
    """
    inner = q1.x * q2.x + q1.y * q2.y + q1.z * q2.z + q1.w * q2.w
    return np.arccos(2 * inner ** 2 - 1)


def pose_distance(a: Pose, b: Pose, rot_weight=0):
    pos_distance = np.linalg.norm(ros_numpy.numpify(a.position) - ros_numpy.numpify(b.position))
    rot_distance = 0 if rot_weight == 0 else rot_weight * quaternion_angle_diff(a.orientation, b.orientation)
    return pos_distance + rot_distance


class ArmSide(enum.IntEnum):
    LEFT = 0
    RIGHT = 1


class CartesianImpedanceController:
    def __init__(self, tf_buffer, motion_status_listeners, motion_command_publisher, joint_lim_low, joint_lim_high,
                 position_close_enough=0.0025, timeout_per_1m=500, intermediate_acceptance_factor=7.,
                 joint_limit_boundary=0.03):
        """

        :param tf_buffer: tf2 Buffer object
        :param motion_status_listeners: ROS listener (wrapper around subscriber) for status messages
        :param motion_command_publisher: ROS publisher for arm commands
        :param joint_lim_low: lower joint limits in degrees
        :param joint_lim_high: upper joint limits in degrees
        :param position_close_enough: Distance (m) to target position to be considered close enough
        :param timeout_per_1m: Allowed time (s) to execute before timing out per 1m of travel
        :param joint_limit_boundary: Angle (radian or list of radian) boundary of each joint limit to avoid by
        returning to the previous pose for any entering. If this boundary is larger than what any single motion command
        will step, then we will not receive exceptions on the robot side.
        """
        self.target_pose = None
        # for users to read after reaching goal
        self.reached_joint_limit = False
        self.timed_out = False
        # joint limits
        self.joint_lim_low = joint_lim_low
        self.joint_lim_high = joint_lim_high

        # tf
        self.tf_buffer = tf_buffer
        # what frames the measured cartesian pose is given in
        self.sensor_frames = ["victor_left_arm_world_frame_kuka", "victor_right_arm_world_frame_kuka"]
        self.world_frame = "victor_root"

        # goal parameters
        self._intermediate_target = None
        self._dists_to_goal = []
        self.position_close_enough = position_close_enough
        self._intermediate_close_enough = position_close_enough * intermediate_acceptance_factor
        self._this_target_start_time = None
        self._goal_start_time = None
        self._init_goal_dist = None
        self._timeout_per_1m = timeout_per_1m

        # safety parameters
        self._joint_boundary = joint_limit_boundary
        if isinstance(self._joint_boundary, float):
            self._joint_boundary = [self._joint_boundary for _ in self.joint_lim_low]
        assert len(self._joint_boundary) == len(self.joint_lim_low)
        self._joint_boundary = np.array(self._joint_boundary)
        self._intermediate_target_start_pose = None
        self._check_joint_limits = True
        self._start_violation = 0

        self.active_arm = ArmSide.LEFT
        # ROS subscribers
        self.motion_status_listeners = motion_status_listeners

        # ROS publishers
        self.motion_command_publisher = motion_command_publisher

        # TODO should probably move control loop onto separate thread

    def set_active_arm(self, active_arm: ArmSide):
        self.active_arm = active_arm

    def reset(self):
        self.abort_goal()

    def abort_goal(self):
        self.target_pose = None
        self._intermediate_target = None
        self._dists_to_goal = []
        self._goal_start_time = None
        self._init_goal_dist = None
        self._check_joint_limits = True
        self._start_violation = 0

    def current_pose_in_world_frame(self, arm):
        current_pose = self.motion_status_listeners[arm].get().measured_cartesian_pose
        if current_pose is None:
            return None
        tf_current_pose = PoseStamped()
        tf_current_pose.pose = current_pose
        tf_current_pose.header.frame_id = self.sensor_frames[arm]
        return self.tf_buffer.transform(tf_current_pose, self.world_frame)

    def set_relative_goal_2d(self, dx, dy, target_z=None, target_orientation=None):
        cp = self.current_pose_in_world_frame(self.active_arm)
        if cp is None:
            rospy.logwarn("Trying to set relative goal when we do not have current pose")
            return False

        target_pose = copy.deepcopy(cp)
        target_pose.pose.position.x += dx
        target_pose.pose.position.y += dy
        if target_z is not None:
            target_pose.pose.position.z = target_z
        if target_orientation is not None:
            target_pose.pose.orientation = target_orientation

        self.set_target_pose(target_pose, current_pose=cp)

        rospy.logdebug("Current {}".format(cp.pose))
        return True

    def set_target_pose(self, target_pose, current_pose=None):
        if current_pose is None:
            current_pose = self.current_pose_in_world_frame(self.active_arm)
        self.target_pose = copy.deepcopy(target_pose)
        self._init_goal_dist = pose_distance(current_pose.pose, self.target_pose.pose)
        self._start_violation = self.joint_boundary_violation_amount()
        self._goal_start_time = rospy.get_time()
        self.timed_out = False
        self.reached_joint_limit = False
        rospy.logdebug("Target {}".format(self.target_pose.pose))

    def joint_boundary_violation_amount(self):
        q = self.motion_status_listeners[self.active_arm].get().measured_joint_position
        q = np.array([q.joint_1, q.joint_2, q.joint_3, q.joint_4, q.joint_5, q.joint_6, q.joint_7])
        low = self.joint_lim_low - (q - self._joint_boundary)
        low_violation = low[low > 0].sum()
        high = (q + self._joint_boundary) - self.joint_lim_high
        high_violation = high[high > 0].sum()
        return low_violation + high_violation

    def step(self, step_size=0.005):
        """Take a non-blocking step and return whether false if we timed out; otherwise true"""
        if self.target_pose is None:
            return True

        cp = self.current_pose_in_world_frame(self.active_arm)
        dist_to_goal = pose_distance(cp.pose, self.target_pose.pose)
        self._dists_to_goal.append(dist_to_goal)
        # rospy.loginfo("Dist to goal {}".format(dist_to_goal))
        if dist_to_goal < self.position_close_enough:
            self.abort_goal()
            rospy.logdebug("Reached target {}".format(cp.pose.position))
            return True

        if self._intermediate_target is None or pose_distance(cp.pose,
                                                              self._intermediate_target.pose) < self._intermediate_close_enough:
            # take step along direction to goal
            diff = ros_numpy.numpify(self.target_pose.pose.position) - ros_numpy.numpify(cp.pose.position)
            diff_norm = np.linalg.norm(diff)
            this_step = min(step_size, diff_norm)
            diff *= this_step / diff_norm
            # copy the target pose to get orientation
            self._intermediate_target = copy.deepcopy(self.target_pose)
            self._intermediate_target.pose.position.x = cp.pose.position.x + diff[0]
            self._intermediate_target.pose.position.y = cp.pose.position.y + diff[1]
            self._intermediate_target.pose.position.z = cp.pose.position.z + diff[2]
            # rospy.loginfo("Current position {}".format(cp.pose.position))
            # rospy.loginfo("Set intermediate goal {}".format(self._intermediate_target.pose.position))
            self._intermediate_target_start_pose = cp
            self._this_target_start_time = rospy.get_time()

        now = rospy.get_time()
        # return to start of pose if we enter a joint limit boundary
        # allow violating joint boundaries at start of motion since sometimes we can drift into it
        if self._check_joint_limits and (now - self._goal_start_time) > 0.1:
            joint_violation = self.joint_boundary_violation_amount()
            if joint_violation > self._start_violation:
                rospy.logwarn(
                    "Entering joint limit boundary, returning to previous pose (start violation %f after move violation %f)",
                    self._start_violation, joint_violation)
                # explicitly do not change the intermediate target start pose in case we still remain in the boundary
                self.set_target_pose(self._intermediate_target_start_pose, current_pose=cp)
                self._intermediate_target = self._intermediate_target_start_pose
                self._this_target_start_time = rospy.get_time()
                # don't check joint limits while moving out of joint boundary (otherwise we'll get stuck here)
                self._check_joint_limits = False
                self.reached_joint_limit = True

        # abort if we take too long
        if (now - self._this_target_start_time) > self._timeout_per_1m * step_size or \
                (now - self._goal_start_time) > self._timeout_per_1m * self._init_goal_dist:
            rospy.loginfo("Goal aborted due to timeout")
            self.timed_out = True
            self.abort_goal()
            return False

        self.command_cartesian_pose(self._intermediate_target)
        return True

    def command_cartesian_pose(self, target_pose):
        """
        Send command to go to a pose in cartesian impedance mode (lowest level API)
        :param target_pose: PoseStamped in world frame
        :return:
        """
        motion_command = MotionCommand()
        command_frame = self.sensor_frames[self.active_arm]
        motion_command.header.frame_id = command_frame
        motion_command.control_mode.mode = ControlMode.CARTESIAN_IMPEDANCE

        target_in_arm_frame = self.tf_buffer.transform(target_pose, command_frame)
        motion_command.cartesian_pose = target_in_arm_frame.pose
        pub = self.motion_command_publisher[self.active_arm]
        while pub.get_num_connections() < 1:
            rospy.sleep(0.01)
        pub.publish(motion_command)
