#! /usr/bin/env python
import pathlib

import rospkg
import yaml

import actionlib
import rospy
from control_msgs.msg import FollowJointTrajectoryGoal, FollowJointTrajectoryAction
from genpy.message import fill_message_args


def main():
    rospy.init_node('test_ros_trajectory_forwarder')
    client = actionlib.SimpleActionClient('follow_joint_trajectory', FollowJointTrajectoryAction)
    client.wait_for_server()
    print("client connected")

    rospack = rospkg.RosPack()
    arm_robots_path = pathlib.Path(rospack.get_path('arm_robots'))
    yaml_filename = arm_robots_path / "src/arm_robots/tests/m_trajectory.yaml"
    with yaml_filename.open('r') as f:
        msg_data = yaml.safe_load(f)

    msg = FollowJointTrajectoryGoal()
    fill_message_args(msg, msg_data)

    res = client.send_goal_and_wait(msg)
    print(res)


if __name__ == "__main__":
    main()
