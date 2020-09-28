#! /usr/bin/env python
import colorama
from colorama import Fore

import actionlib
from control_msgs.msg import FollowJointTrajectoryFeedback
from tf.transformations import quaternion_from_euler

import moveit_commander
import numpy as np

import rospy
from arm_robots.victor import Victor
from geometry_msgs.msg import Pose


def run_mev():
    colorama.init(autoreset=True)

    joint_state_topic = ['joint_states:=/victor/joint_states']
    moveit_commander.roscpp_initialize(joint_state_topic)
    rospy.init_node('basic_motion', anonymous=True)

    robot = Victor(execute_by_default=True)

    rospy.loginfo("If you're using gazebo, make sure you hit play or nothing will happen :)")

    # Plan to joint config
    input("Plan to joint config")
    robot.plan_to_joint_config("right_arm", [0.2, 1, 0, -1, 0.2, 0, 0])

    # Plan to joint config with a stop condition
    input("Plan to joint config, with max force")
    def _stop_condition(action_client: actionlib.SimpleActionClient, feedback: FollowJointTrajectoryFeedback):
        if len(feedback.actual.effort) == 0:
            rospy.loginfo_throttle(5, "no force feedback, this is expected in simulation")
        else:
            max_force = np.max(np.array(feedback.actual.effort))
            if max_force > 0.1:
                rospy.loginfo(f"Force exceeded {max_force}! Canceling all goals.")
                action_client.cancel_all_goals()

    robot.plan_to_joint_config("right_arm", [0.2, 1, 0, -1.5, 0.2, 0, 0], stop_condition=_stop_condition)

    # Plan to pose
    input("Plan to pose 1")
    robot.plan_to_pose("right_arm", "right_tool_placeholder", [0.6, -0.2, 1.0, 4, 1, 0])

    # Or you can use a geometry msgs Pose
    input("Plan to pose 2")
    pose = Pose()
    pose.position.x = 0.7
    pose.position.y = -0.2
    pose.position.z = 1.0
    q = quaternion_from_euler(np.pi, 0, 0)
    pose.orientation.x = q[0]
    pose.orientation.y = q[1]
    pose.orientation.z = q[2]
    pose.orientation.w = q[3]
    robot.plan_to_pose("right_arm", "right_tool_placeholder", pose)

    # Move hand straight works either with jacobian following
    input("Follow jacobian to pose 3")
    robot.follow_jacobian_to_position("right_arm", "right_tool_placeholder", [0.7, -0.6, 0.8])

    # Or with cartesian planning
    input("Cartersian motion back to pose 2")
    robot.plan_to_position_cartesian("right_arm", "right_tool_placeholder", pose.position)


if __name__ == "__main__":
    run_mev()
