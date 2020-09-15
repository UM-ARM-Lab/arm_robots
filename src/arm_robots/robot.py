#! /usr/bin/env python

import rospy


class ARMRobot:
    def __init__(self):
        rospy.loginfo("MotionEnabledRobot ready")

    def traj_from_path_msg(self, path_msg):
        raise NotImplementedError()

    def execute_trajectory(self, trajectory, blocking=True):
        raise NotImplementedError()

    def move_to_home(self, blocking=True):
        raise NotImplementedError()

    def plan_to_configuration(self, target_config, execute=False, steplength=0.01, blocking=True, **kwargs):
        raise NotImplementedError()

    def plan_to_configuration_both_arms(self, right_config, left_config, execute=False, steplength=0.01, blocking=True):
        raise NotImplementedError()

    def plan_to_configuration_whole_body(self, right_config, left_config, other_config=None, execute=False,
                                         steplength=0.01, blocking=True):
        raise NotImplementedError()

    def plan_to_relative_pose(self, relative_pose, execute=False, **kwargs):
        raise NotImplementedError()

    def plan_to_pose(self, target_pose, execute=False, **kwargs):
        raise NotImplementedError()

    def plan_to_pose_both_arms(self, right_pose, left_pose, execute=False, blocking=True, **kwargs):
        raise NotImplementedError()

    def wait_gripper(self, gripper):
        raise NotImplementedError()

    def close_gripper(self, gripper_name, blocking=True, continuous=False):
        raise NotImplementedError()

    def open_gripper(self, gripper_name, blocking=True, continuous=False):
        raise NotImplementedError()

    def set_gripper(self, gripper_name, pos, blocking=True, continuous=False):
        """
        Sets the gripper position

        Parameters:
        gripper_name  string  - "left" or "right"
        pos       float[]     - position values for fingers
                                [a, b, c] for Victor, [0 to 1]
                                [pos] for Val, limits unknown
        blocking      bool    - Block until gripper motion finishes
        continuous    bool    - if True keeps sending command every X sec (no blocking possible) (only applicable for Victor)
        """
        raise NotImplementedError("set_gripper is a abstract method. It should never be called.")

    def move_both_hands_straight(self,
                                 right_moving_direction=None,
                                 left_moving_direction=None,
                                 right_moving_distance=0.0,
                                 left_moving_distance=0.0,
                                 right_step_size=0.005,
                                 left_step_size=0.005,
                                 execute=False,
                                 blocking=True):
        """
        Wrapper around GeneralIK to perform straight gripper motion. It is assumed that the environment is already
        locked if it needs to be before this function is called.

        moving_direction: [x,y,z] vector in world frame. 'None' indicates that this gripper is not moving.
        moving_distance: moving distance in meters
        step_size: IK interpolation step size in meters
        execute: if True, will execute the path using the controller
        blocking: if True will sleep until the robot controller to finish trajectory
        """
        raise NotImplementedError()

    def rotate_both_hands_straight(self,
                                   right_moving_axis=None,
                                   left_moving_axis=None,
                                   right_moving_radians=0.0,
                                   left_moving_radians=0.0,
                                   right_step_size=0.005,
                                   left_step_size=0.005,
                                   execute=False,
                                   blocking=True):
        """
        Wrapper around GeneralIK to perform straight gripper motion. It is assumed that the environment is already
        locked if it needs to be before this function is called.

        moving_axis: [x,y,z] vector in world frame. 'None' indicates that this gripper is not moving.
        moving_radians: moving distance in radians
        step_size: IK interpolation step size in radians
        execute: if True, will execute the path using the controller
        blocking: if True will sleep until the robot controller to finish trajectory
        """
        raise NotImplementedError()

    def object_grasped(self, gripper):
        """
        Returns boolean whether an object is in the grippers.
        This must be run just after a close_gripper command.
        Incorrect results may be returned for very thin objects.
        """
        rospy.logerr("object_grasped is a abstract method. It should never be called.")
        raise NotImplementedError()
