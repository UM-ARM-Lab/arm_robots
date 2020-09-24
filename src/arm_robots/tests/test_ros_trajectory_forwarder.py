#! /usr/bin/env python
import pathlib
import unittest

import rospkg
import yaml

import actionlib
import rospy
from arm_robots.robot import interpolate_joint_trajectory_points
from control_msgs.msg import FollowJointTrajectoryGoal, FollowJointTrajectoryAction
from genpy.message import fill_message_args


class TestRosTrajectoryForwarder(unittest.TestCase):

    def test_interpolate_joint_trajectory_points(self):
        interpolated_points = interpolate_joint_trajectory_points()
        expected_points = [
        ]
        self.assertAlmostEqual(interpolated_points, expected_points)


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
