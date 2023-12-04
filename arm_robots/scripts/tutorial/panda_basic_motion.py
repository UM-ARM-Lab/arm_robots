#! /usr/bin/env python
import pdb

import rospy
import numpy as np
from arm_robots.panda import Panda
from moveit_msgs.msg import RobotTrajectory
import tf


def execute_plan(panda, group_name, joint_trajectory):
    move_group = panda.get_move_group_commander(group_name)
    plan_msg = RobotTrajectory()
    plan_msg.joint_trajectory = joint_trajectory
    move_group.execute(plan_msg)


if __name__ == '__main__':
    rospy.init_node('panda_motion')
    panda = Panda()
    panda.connect()
    print("Panda connected! Planning now...")
    # pdb.set_trace()
    panda.set_execute(False)  # this will stop the robot from actually executing a path, good for testing
    if not panda.execute:
        print("Real execution disabled.")
    panda_1_action_1_start = [0.25, 0.25, 1.16, np.radians(180), 0, np.radians(-90)]
    panda_2_action_1_start = [0.25, 0.15, 1.16, np.radians(-90), np.radians(-90), 0]
    panda_1_action_1_start_plan = panda.plan_to_pose(panda.panda_1, panda.panda_1_EE, panda_1_action_1_start, frame_id="world")
    panda_2_action_1_start_plan = panda.plan_to_pose(panda.panda_2, panda.panda_2_EE, panda_2_action_1_start, frame_id="world")

    pdb.set_trace()

    execute_plan(panda, panda.panda_1, panda_1_action_1_start_plan.planning_result.plan.joint_trajectory)
    execute_plan(panda, panda.panda_2, panda_2_action_1_start_plan.planning_result.plan.joint_trajectory)
    panda.set_execute(False)

    pdb.set_trace()

    panda_1_action_2_start = [0.25, 0.25, 1.4, np.radians(90), np.radians(-90), 0]
    panda_2_action_2_start = [0.25, 0.15, 1.36, np.radians(-90), np.radians(-90), 0]
    panda_1_action_2_start_plan = panda.plan_to_pose(panda.panda_1, panda.panda_1_EE, panda_1_action_2_start, frame_id="world")
    panda_2_action_2_start_plan = panda.plan_to_pose(panda.panda_2, panda.panda_2_EE, panda_2_action_2_start, frame_id="world")

    pdb.set_trace()

    execute_plan(panda, panda.panda_1, panda_1_action_2_start_plan.planning_result.plan.joint_trajectory)
    execute_plan(panda, panda.panda_2, panda_2_action_2_start_plan.planning_result.plan.joint_trajectory)
    panda.set_execute(False)

    pdb.set_trace()

    # panda_2_action_1_end = [0.25, 0.20, 1.155, np.radians(-90), np.radians(-90), 0]
    # pt = [[np.array(panda_2_action_1_end[:3])]]
    # ori = [tf.transformations.quaternion_from_euler(panda_2_action_1_end[3], panda_2_action_1_end[4], panda_2_action_1_end[5])]
    # panda_2_action_1_end_plan = panda.follow_jacobian_to_position(panda.panda_2, ['panda_2_link_planning_EE'], pt, ori, 0.01)
    #
    # pdb.set_trace()
    #
    # execute_plan(panda, panda.panda_2, panda_2_action_1_end_plan.planning_result.plan.joint_trajectory)
    #
    # pdb.set_trace()

