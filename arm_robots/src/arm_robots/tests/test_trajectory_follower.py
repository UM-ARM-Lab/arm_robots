#! /usr/bin/env python
import pathlib
import unittest

import rospkg
import yaml

import actionlib
import rospy
from arm_robots.robot_utils import interpolate_joint_trajectory_points
from control_msgs.msg import FollowJointTrajectoryGoal, FollowJointTrajectoryAction
from trajectory_msgs.msg import JointTrajectoryPoint
from genpy.message import fill_message_args
from typing import List


class TestRosTrajectoryForwarder(unittest.TestCase):

    def check_trajectories_equal(self, expected: List[JointTrajectoryPoint], actual: List[JointTrajectoryPoint]):
        self.assertEqual(len(actual), len(expected))
        for act, exp in zip(actual, expected):
            self.assertEqual(len(act.positions), len(exp.positions))
            for i in range(len(act.positions)):
                self.assertAlmostEqual(act.positions[i], exp.positions[i])
            self.assertEqual(len(act.velocities), len(exp.velocities), f"Velocities lengths are not equal")
            for i in range(len(act.velocities)):
                self.assertAlmostEqual(act.velocities[i], exp.velocities[i])

    def test_interpolate_joint_trajectory_points_on_empty(self):
        pts = [JointTrajectoryPoint(positions=(0, 1, 2), time_from_start=rospy.Duration(0)),
               JointTrajectoryPoint(positions=(2, 2, 2), time_from_start=rospy.Duration(1))]
        interpolated_points = interpolate_joint_trajectory_points(pts, max_step_size=100)
        expected_points = \
            [JointTrajectoryPoint(positions=(0, 1, 2), time_from_start=rospy.Duration(0), velocities=(0, 0, 0)),
             JointTrajectoryPoint(positions=(2, 2, 2), time_from_start=rospy.Duration(1), velocities=(0, 0, 0))]
        self.check_trajectories_equal(expected_points, interpolated_points)

    def test_interpolate_joint_trajectory_points_single_interp(self):
        pts = [JointTrajectoryPoint(positions=(1, 1, 1), time_from_start=rospy.Duration(0)),
               JointTrajectoryPoint(positions=(1, 1, 1.4), time_from_start=rospy.Duration(1))]
        interpolated_points = interpolate_joint_trajectory_points(pts, max_step_size=.2)
        expected_points = \
            [JointTrajectoryPoint(positions=(1, 1, 1), time_from_start=rospy.Duration(0), velocities=(0, 0, 0)),
             JointTrajectoryPoint(positions=(1, 1, 1.2), time_from_start=rospy.Duration(0.5), velocities=(0, 0, .4)),
             JointTrajectoryPoint(positions=(1, 1, 1.4), time_from_start=rospy.Duration(1), velocities=(0, 0, 0)),
             ]
        self.check_trajectories_equal(expected_points, interpolated_points)

    def test_interpolate_joint_trajectory_points_on_multiple_interp(self):
        pts = [JointTrajectoryPoint(positions=(1, 1, 1), time_from_start=rospy.Duration(0)),
               JointTrajectoryPoint(positions=(1, 1, 4), time_from_start=rospy.Duration(3)),
               JointTrajectoryPoint(positions=(1, 3, 4), time_from_start=rospy.Duration(5)),
               JointTrajectoryPoint(positions=(2, 3, 4), time_from_start=rospy.Duration(6)),
               ]
        interpolated_points = interpolate_joint_trajectory_points(pts, max_step_size=1)
        expected_points = \
            [JointTrajectoryPoint(positions=(1, 1, 1), time_from_start=rospy.Duration(0), velocities=(0, 0, 0)),
             JointTrajectoryPoint(positions=(1, 1, 2), time_from_start=rospy.Duration(0.5), velocities=(0, 0, 1)),
             JointTrajectoryPoint(positions=(1, 1, 3), time_from_start=rospy.Duration(0.5), velocities=(0, 0, 1)),
             JointTrajectoryPoint(positions=(1, 1, 4), time_from_start=rospy.Duration(0.5), velocities=(0, 0.5, 0.5)),
             JointTrajectoryPoint(positions=(1, 2, 4), time_from_start=rospy.Duration(0.5), velocities=(0, 1, 0)),
             JointTrajectoryPoint(positions=(1, 3, 4), time_from_start=rospy.Duration(0.5), velocities=(0.5, 0.5, 0)),
             JointTrajectoryPoint(positions=(2, 3, 4), time_from_start=rospy.Duration(0.5), velocities=(0, 0, 0)),
             ]
        self.check_trajectories_equal(expected_points, interpolated_points)


def main():
    rospy.init_node('test_ros_trajectory_forwarder')
    client = actionlib.SimpleActionClient('follow_joint_trajectory', FollowJointTrajectoryAction)
    client.wait_for_server()
    print("client connected")

    rospack = rospkg.RosPack()
    arm_robots_path = pathlib.Path(rospack.get_path('arm_robots'))
    yaml_filename = arm_robots_path / "src/arm_robots/tests/test_trajectory1.yaml"
    with yaml_filename.open('r') as f:
        msg_data = yaml.safe_load(f)

    msg = FollowJointTrajectoryGoal()
    fill_message_args(msg, msg_data)

    res = client.send_goal_and_wait(msg)
    print(res)


if __name__ == "__main__":
    main()
