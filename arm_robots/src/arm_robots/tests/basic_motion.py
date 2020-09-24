#! /usr/bin/env python

import moveit_commander

import moveit_msgs.msg
import rospy
from arm_robots.victor import Victor


def run_mev():
    joint_state_topic = ['joint_states:=/victor/joint_states']
    moveit_commander.roscpp_initialize(joint_state_topic)
    rospy.init_node('basic_motion', anonymous=True)

    robot = Victor()

    robot.close_right_gripper()

    # PLAN TO POSE
    # robot.plan_to_pose()
    group_name = "right_arm"
    robot = moveit_commander.RobotCommander()
    move_group = moveit_commander.MoveGroupCommander(group_name)
    print(move_group.get_current_pose())

    # joint_goal = move_group.get_current_joint_values()
    # joint_goal[0] = 0
    # joint_goal[1] = 0.5
    # joint_goal[2] = 0
    # joint_goal[3] = 0.5
    # joint_goal[4] = 0
    # joint_goal[5] = 0.5
    # joint_goal[6] = 0
    # plan = move_group.go(joint_goal, wait=True)
    pose_goal = move_group.get_current_pose()
    pose_goal.pose.position.x = 0.9
    pose_goal.pose.position.y = -0.6
    pose_goal.pose.position.z = 1.0

    move_group.set_pose_target(pose_goal)
    plan = move_group.go(wait=True)

    display_trajectory = moveit_msgs.msg.DisplayTrajectory()
    display_trajectory.trajectory_start = robot.get_current_state()
    display_trajectory.trajectory.append(plan)
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=10)
    display_trajectory_publisher.publish(display_trajectory)

    move_group.stop()


if __name__ == "__main__":
    run_mev()
