import copy

import numpy as np
import rospy
import ros_numpy
from geometry_msgs.msg import PoseStamped, Quaternion, Pose, WrenchStamped
from victor_hardware_interface_msgs.msg import ControlMode, MotionCommand
from tf.transformations import quaternion_from_euler, quaternion_slerp
from collections import deque


def quaternion_angle_diff(q1: Quaternion, q2: Quaternion):
    """Angle of rotation required to get from one orientation to another
    see https://math.stackexchange.com/a/90098/184113
    """
    inner = q1.x * q2.x + q1.y * q2.y + q1.z * q2.z + q1.w * q2.w
    return np.arccos(np.clip(2 * inner ** 2 - 1, -1, 1))


def pos_distance(a: Pose, b: Pose):
    return np.linalg.norm(ros_numpy.numpify(a.position) - ros_numpy.numpify(b.position))


def rot_distance(a: Pose, b: Pose):
    return quaternion_angle_diff(a.orientation, b.orientation)


class ControllerStatus:
    # for users to read after reaching goal
    def __init__(self):
        self.reached_joint_limit = False
        self.reached_force_threshold = False
        self.callback_stopped = False
        self.timed_out = False

    def reset(self):
        self.reached_joint_limit = False
        self.reached_force_threshold = False
        self.callback_stopped = False
        self.timed_out = False


class MotionFrameTransformer:
    def motion_status_to_ee(self, current_pose: PoseStamped) -> PoseStamped:
        raise NotImplementedError

    def motion_status_to_ee_wrench(self, current_wrench: WrenchStamped) -> WrenchStamped:
        raise NotImplementedError

    def ee_to_motion_command(self, target_pose: PoseStamped) -> PoseStamped:
        raise NotImplementedError


class CartesianImpedanceController:
    def __init__(self, tf_buffer, motion_status_listeners, motion_command_publisher, joint_lim_low, joint_lim_high,
                 world_frame_name, sensor_frame_names=None,
                 position_close_enough=0.0025, rotation_close_enough=0.01,
                 timeout_per_m=500,
                 timeout_per_radian=100,
                 timeout_min_m_per_s=0.001,
                 timeout_min_radian_per_s=0.01,
                 intermediate_acceptance_factor=7.,
                 joint_limit_boundary=0.03):
        """

        :param tf_buffer: tf2 Buffer object
        :param motion_status_listeners: ROS listener (wrapper around subscriber) for status messages
        :param motion_command_publisher: ROS publisher for arm commands
        :param joint_lim_low: lower joint limits in radians
        :param joint_lim_high: upper joint limits in radians
        :param world_frame_name: Name of the world frame
        :param sensor_frame_names: Name of the sensor frames for each arm
        :param position_close_enough: Distance (m) to target position to be considered close enough
        :param rotation_close_enough: Angle (radian) to target orientation to be considered close enough
        :param timeout_per_m: Allowed time (s) to execute before timing out per 1m of travel
        :param timeout_min_m_per_s: Minimum speed (m/s) to avoid timing out
        :param timeout_min_radian_per_s: Minimum speed (radian/s) to avoid timing out
        :param joint_limit_boundary: Angle (radian or list of radian) boundary of each joint limit to avoid by
        returning to the previous pose for any entering. If this boundary is larger than what any single motion command
        will step, then we will not receive exceptions on the robot side.
        """
        self.target_pose = None
        # for users to read after reaching goal
        self.status = ControllerStatus()
        # joint limits, store as radians
        self.joint_lim_low = np.array(joint_lim_low)
        self.joint_lim_high = np.array(joint_lim_high)
        if np.any(self.joint_lim_low < -np.pi * 2) or np.any(self.joint_lim_high > np.pi * 2):
            rospy.logwarn(f"Joint limits supplied may be invalid radians: {self.joint_lim_low} {self.joint_lim_high}")

        # tf
        self.tf_buffer = tf_buffer
        # what frames the measured cartesian pose is given in
        self.world_frame = world_frame_name
        if sensor_frame_names is None:
            self.sensor_frames = [self.world_frame for _ in motion_status_listeners]
        else:
            self.sensor_frames = sensor_frame_names

        # goal parameters
        self._intermediate_target = None
        self._dists_to_goal = []
        self.position_close_enough = position_close_enough
        self.rotation_close_enough = rotation_close_enough
        self._intermediate_acceptance_factor = intermediate_acceptance_factor
        self._this_target_start_time = None
        self._goal_start_time = None
        self._init_goal_dist = None
        self._timeout_per_m = timeout_per_m
        self._timeout_per_radian = timeout_per_radian

        # min speed timeout
        self._pose_history = deque(maxlen=5)
        self._min_m_per_s = timeout_min_m_per_s
        self._min_radian_per_s = timeout_min_radian_per_s

        # safety parameters
        self._joint_boundary = joint_limit_boundary
        if isinstance(self._joint_boundary, float):
            self._joint_boundary = [self._joint_boundary for _ in self.joint_lim_low]
        assert len(self._joint_boundary) == len(self.joint_lim_low)
        self._joint_boundary = np.array(self._joint_boundary)
        self._intermediate_target_start_pose = None
        self._check_joint_limits = True
        self._start_violation = 0

        self.active_arm = 0
        self.motion_status_listeners = motion_status_listeners
        self.motion_command_publisher = motion_command_publisher

    def set_active_arm(self, active_arm):
        if self.target_pose is not None:
            rospy.logwarn(f"Resetting active arm with an active target pose; aborting that target {self.target_pose}")
            self.abort_goal()

        self.active_arm = active_arm

    def reset(self):
        self.abort_goal()

    def abort_goal(self):
        # otherwise continues executing last goal
        self.command_cartesian_pose(
            self.current_pose_in_frame(self.active_arm, reference_frame=self.target_pose.header.frame_id))
        self.target_pose = None
        self._intermediate_target = None
        self._dists_to_goal = []
        self._pose_history.clear()
        self._goal_start_time = None
        self._init_goal_dist = None
        self._check_joint_limits = True
        self._start_violation = 0

    def current_pose_in_frame(self, arm, reference_frame=None):
        # clear potentially stale messages
        self.motion_status_listeners[arm].data = None
        current_pose = self.motion_status_listeners[arm].get().measured_cartesian_pose

        if current_pose is None:
            return None
        # current pose is actually in ee_frame
        tf_current_pose = PoseStamped()
        tf_current_pose.pose = current_pose
        tf_current_pose.header.frame_id = self.sensor_frames[arm]
        if reference_frame is None:
            reference_frame = self.world_frame
        return self.tf_buffer.transform(tf_current_pose, reference_frame)

    def set_goal(self, dx=0, dy=0, dz=0, target_x=None, target_y=None, target_z=None, target_orientation=None,
                 reference_frame=None, motion_frame_transformer: MotionFrameTransformer = None):
        """
        Set position and optionally orientation goals specified in the given reference frame (default to world frame)
        :param dx: desired change in x
        :param dy: desired change in y
        :param dz: desired change in z
        :param target_x: desired absolute x, overriding any dx
        :param target_y: desired absolute y, overriding any dy
        :param target_z: desired absolute z, overriding any dz
        :param target_orientation:
        :param reference_frame:
        :param motion_frame_transformer: If given, will transform between the measured motion status frame and an
            end effector frame such that motion commands are sent in the motion status frame, but goals are set in the
            end effector frame. Note that target positions and orientations are directly specified in the end effector
            frame. The end effector frame is whatever frame the motion transformer transforms to.
        :return: Whether the goal was successfully set
        """

        cp = self.current_pose_in_frame(self.active_arm, reference_frame=reference_frame)
        if cp is None:
            rospy.logwarn("Trying to set relative goal when we do not have current pose")
            return False

        target_pose = copy.deepcopy(cp)
        if motion_frame_transformer is not None:
            target_pose = motion_frame_transformer.motion_status_to_ee(target_pose)

        target_pose.pose.position.x += dx
        target_pose.pose.position.y += dy
        target_pose.pose.position.z += dz
        if target_x is not None:
            target_pose.pose.position.x = target_x
        if target_y is not None:
            target_pose.pose.position.y = target_y
        if target_z is not None:
            target_pose.pose.position.z = target_z

        if target_orientation is not None:
            orientation = target_orientation
            if type(orientation) != Quaternion:
                if len(orientation) == 3:
                    orientation = quaternion_from_euler(*orientation)
                if len(orientation) == 4:
                    orientation = Quaternion(*orientation)
            target_pose.pose.orientation = orientation

        if motion_frame_transformer is not None:
            target_pose = motion_frame_transformer.ee_to_motion_command(target_pose)
            target_pose.header.frame_id = reference_frame or self.world_frame

        self.set_target_pose(target_pose, current_pose=None)  # re-get current pose in world frame
        return True

    def set_target_pose(self, target_pose, current_pose=None):
        # convert the commanded target into a world frame target
        # this avoids the issue of tracking moving reference frames; it will use the frame at the initial call time
        target_pose = self.tf_buffer.transform(target_pose, self.world_frame)

        if current_pose is None:
            current_pose = self.current_pose_in_frame(self.active_arm, reference_frame=target_pose.header.frame_id)
        if target_pose.header.frame_id != current_pose.header.frame_id:
            raise RuntimeError("Target and current poses are given in different frames: "
                               f"target {target_pose.header.frame_id} current {current_pose.header.frame_id}")

        self.target_pose = copy.deepcopy(target_pose)
        a = current_pose.pose
        b = self.target_pose.pose
        self._init_goal_dist = (pos_distance(a, b), rot_distance(a, b))
        self._start_violation = self.joint_boundary_violation_amount()
        self._goal_start_time = rospy.get_time()
        self._pose_history.clear()
        self.status.reset()
        rospy.logdebug("Target\n{}".format(str(self.target_pose.pose).replace('\n', ' ')))

    def joint_boundary_violation_amount(self):
        q = self.motion_status_listeners[self.active_arm].get().measured_joint_position
        q = np.array([q.joint_1, q.joint_2, q.joint_3, q.joint_4, q.joint_5, q.joint_6, q.joint_7])
        low = self.joint_lim_low - (q - self._joint_boundary)
        low_violation = low[low > 0].sum()
        high = (q + self._joint_boundary) - self.joint_lim_high
        high_violation = high[high > 0].sum()
        return low_violation + high_violation

    def step(self, step_size=0.005, stop_on_force_threshold=None, stop_callback=None, step_quaternion_size=0.1):
        """Take a non-blocking step and return whether false if we timed out; otherwise true"""
        if self.target_pose is None:
            return True

        cp = self.current_pose_in_frame(self.active_arm, reference_frame=self.target_pose.header.frame_id)

        a = cp.pose
        b = self.target_pose.pose
        dist_to_goal = (pos_distance(a, b), rot_distance(a, b))
        self._dists_to_goal.append(dist_to_goal)
        # rospy.loginfo("Dist to goal {}".format(dist_to_goal))
        if dist_to_goal[0] < self.position_close_enough and dist_to_goal[1] < self.rotation_close_enough:
            self.abort_goal()
            rospy.logdebug("Reached target\n{}".format(str(cp.pose.position).replace('\n', ' ')))
            return True

        if self._intermediate_target is not None:
            b = self._intermediate_target.pose
        if self._intermediate_target is None or (
                (pos_distance(a, b) < self.position_close_enough * self._intermediate_acceptance_factor) and (
                rot_distance(a, b) < self.rotation_close_enough * self._intermediate_acceptance_factor)):
            # linearly interpolate both the position and orientation
            # take step along direction to goal
            diff = ros_numpy.numpify(self.target_pose.pose.position) - ros_numpy.numpify(cp.pose.position)
            diff_norm = np.linalg.norm(diff)
            this_step = min(step_size, diff_norm)
            diff *= this_step / diff_norm

            self._intermediate_target = copy.deepcopy(self.target_pose)
            self._intermediate_target.pose.position.x = cp.pose.position.x + diff[0]
            self._intermediate_target.pose.position.y = cp.pose.position.y + diff[1]
            self._intermediate_target.pose.position.z = cp.pose.position.z + diff[2]
            # interpolate orientation
            q1 = cp.pose.orientation
            q2 = self.target_pose.pose.orientation
            quat_diff = quaternion_angle_diff(q1, q2)
            this_quaternion_step = min(step_quaternion_size, quat_diff)
            # if quat_diff < step_quaternion_size:
            #     self._intermediate_target.pose.orientation = q2
            # else:
            #     # interpolate by a fixed step size
            #     q1 = ros_numpy.numpify(q1)
            #     q2 = ros_numpy.numpify(q2)
            #     q2 = quaternion_slerp(q1, q2, step_quaternion_size / quat_diff)
            #     q2 = ros_numpy.msgify(Quaternion, q2)
            #     self._intermediate_target.pose.orientation = q2
            q1 = ros_numpy.numpify(q1)
            q2 = ros_numpy.numpify(q2)
            q2 = quaternion_slerp(q1, q2, this_quaternion_step / quat_diff)
            q2 = ros_numpy.msgify(Quaternion, q2)
            self._intermediate_target.pose.orientation = q2

            # rospy.loginfo(
            #     f"Intermediate target position t={this_step / diff_norm:.3f} rotation t={this_quaternion_step / quat_diff:.3f} pos diff {diff_norm:.3f} rot diff {quat_diff:.3f}")

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
                self.status.reached_joint_limit = True

        # abort if we take too long
        time_since_this_target = now - self._this_target_start_time
        timeout_this_target = (time_since_this_target > self._timeout_per_m * step_size) and (
                time_since_this_target > self._timeout_per_radian * step_quaternion_size)
        time_since_goal = now - self._goal_start_time
        timeout_goal = (time_since_goal > self._timeout_per_m * self._init_goal_dist[0]) and (
                time_since_goal > self._timeout_per_radian * self._init_goal_dist[1])
        if timeout_this_target or timeout_goal:
            rospy.loginfo("Goal aborted due to timeout: \ngoal    {} \ncurrent {}\ndist {}".format(
                str(self._intermediate_target.pose).replace('\n', ' '),
                str(cp.pose).replace('\n', ' '), dist_to_goal))
            self.status.timed_out = True
            self.abort_goal()
            return False

        # abort if we are too slow
        if len(self._pose_history) == self._pose_history.maxlen:
            prev_t, prev_a = self._pose_history.pop()
            dt = now - prev_t
            dp = pos_distance(prev_a, a)
            dr = rot_distance(prev_a, a)
            if dp / dt < self._min_m_per_s and dr / dt < self._min_radian_per_s:
                rospy.loginfo("Goal aborted due to being too slow")
                self.timed_out = False
                self.abort_goal()
                return False
        self._pose_history.appendleft((now, a))

        # abort if there is a set wrench threshold and we reached it
        if stop_on_force_threshold is not None:
            status = self.motion_status_listeners[self.active_arm].get()
            w = status.estimated_external_wrench
            f = np.array([w.x, w.y, w.z])
            f_mag = np.linalg.norm(f)
            if f_mag > stop_on_force_threshold:
                rospy.loginfo("Goal aborted due to exceeding force threshold (%f) with measured %f",
                              stop_on_force_threshold, f_mag)
                self.status.reached_force_threshold = True
                self.abort_goal()
                return False
        if stop_callback is not None:
            to_stop = stop_callback()
            if to_stop:
                rospy.loginfo("Goal aborted due to stop callback activating")
                self.status.callback_stopped = True
                self.abort_goal()
                return False

        self.command_cartesian_pose(self._intermediate_target)
        return True

    def command_cartesian_pose(self, target_pose):
        """
        Send command to go to a pose in cartesian impedance mode (lowest level API)
        :param target_pose: PoseStamped in previously specified reference frame
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
