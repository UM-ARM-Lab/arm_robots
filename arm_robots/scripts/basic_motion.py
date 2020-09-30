#! /usr/bin/env python
import colorama
import numpy as np

import actionlib
import moveit_commander
import rospy
from arc_utilities.ros_helpers import Listener
from arm_robots.victor import Victor
from control_msgs.msg import FollowJointTrajectoryFeedback
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
from tf.transformations import quaternion_from_euler

effort_thresholds = np.array([
    150,  # victor_left_arm_joint_1
    100,  # victor_left_arm_joint_2
    100,  # victor_left_arm_joint_3
    80,  # victor_left_arm_joint_4
    80,  # victor_left_arm_joint_5
    40,  # victor_left_arm_joint_6
    10,  # victor_left_arm_joint_7
    1,  # victor_left_gripper_fingerA_joint_2
    1,  # victor_left_gripper_fingerA_joint_3
    1,  # victor_left_gripper_fingerA_joint_4
    1,  # victor_left_gripper_fingerB_joint_2
    1,  # victor_left_gripper_fingerB_joint_3
    1,  # victor_left_gripper_fingerB_joint_4
    1,  # victor_left_gripper_fingerB_knuckle
    1,  # victor_left_gripper_fingerC_joint_2
    1,  # victor_left_gripper_fingerC_joint_3
    1,  # victor_left_gripper_fingerC_joint_4
    1,  # victor_left_gripper_fingerC_knuckle
    150,  # victor_right_arm_joint_1
    100,  # victor_right_arm_joint_2
    100,  # victor_right_arm_joint_3
    80,  # victor_right_arm_joint_4
    80,  # victor_right_arm_joint_1
    40,  # victor_right_arm_joint_6
    10,  # victor_right_arm_joint_7
    1,  # victor_right_gripper_fingerA_joint_2
    1,  # victor_right_gripper_fingerA_joint_3
    1,  # victor_right_gripper_fingerA_joint_4
    1,  # victor_right_gripper_fingerB_joint_2
    1,  # victor_right_gripper_fingerB_joint_3
    1,  # victor_right_gripper_fingerB_joint_4
    1,  # victor_right_gripper_fingerB_knuckle
    1,  # victor_right_gripper_fingerC_joint_2
    1,  # victor_right_gripper_fingerC_joint_3
    1,  # victor_right_gripper_fingerC_joint_4
    1,  # victor_right_gripper_fingerC_knuckle
])

debug = False


def myinput(msg):
    global debug
    if not debug:
        input(msg)


def main():
    np.set_printoptions(suppress=True, precision=0, linewidth=200)
    colorama.init(autoreset=True)

    joint_state_topic = ['joint_states:=/victor/joint_states']
    moveit_commander.roscpp_initialize(joint_state_topic)
    rospy.init_node('basic_motion', anonymous=True)

    joint_state_listener = Listener("/victor/joint_states", JointState)

    robot = Victor(execute_by_default=True)

    rospy.loginfo("If you're using gazebo, make sure you hit play or nothing will happen :)")

    # Plan to joint config
    print("press enter when prompted...")
    myinput("Plan to joint config?")
    robot.plan_to_joint_config("right_arm", [0.2, 1, 0, -1, 0.2, 0, 0])

    # Plan to joint config with a stop condition
    myinput("Plan to joint config, with max force?")

    def _stop_condition(action_client: actionlib.SimpleActionClient, feedback: FollowJointTrajectoryFeedback):
        joint_state = joint_state_listener.get()
        effort = np.abs(np.array(joint_state.effort))
        exceeds_threshold = effort > effort_thresholds
        if np.any(exceeds_threshold):
            offending_joints = np.where(exceeds_threshold)[0].tolist()
            offending_joint_names = [joint_state.name[j] for j in offending_joints]
            rospy.loginfo(f"Force on joints {offending_joint_names} exceeded threshold! Canceling all goals.")
            action_client.cancel_all_goals()

    robot.plan_to_joint_config("right_arm", [-0.2, 1, -1, -1.2, 0.2, 0, 0], stop_condition=_stop_condition)

    # Plan to pose
    myinput("Plan to pose 1?")
    robot.plan_to_pose("right_arm", "right_tool_placeholder", [0.6, -0.2, 1.0, 4, 1, 0])

    # Or you can use a geometry msgs Pose
    myinput("Plan to pose 2?")
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

    # # Or with cartesian planning
    myinput("Cartersian motion back to pose 3?")
    robot.plan_to_position_cartesian("right_arm", "right_tool_placeholder", [0.9, -0.4, 0.9], step_size=0.04)

    # Move hand straight works either with jacobian following
    myinput("Follow jacobian to pose 2?")
    robot.follow_jacobian_to_position("right_arm", "right_tool_placeholder", [1.05, 0.15, 1.0])


if __name__ == "__main__":
    main()
