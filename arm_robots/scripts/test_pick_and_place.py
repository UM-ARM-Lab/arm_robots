#! /usr/bin/env python
import rospy
import tf2_ros as tf
from arm_robots.med import Med
import numpy as np
import pdb
import moveit_commander
from arc_utilities.conversions import convert_to_pose_msg
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from visualization_msgs.msg import Marker



def get_block_pose():

    tf_buffer = tf.BufferCore()
    transformer = tf.TransformListener(tf_buffer)
    rospy.sleep(1.0)

    tf_frames = tf_buffer._getFrameStrings()
    tag_frames = [tf_i for tf_i in tf_frames if 'object_center_' in tf_i]

    tfs = []
    for tag_i in tag_frames:
        tf_i = tf_buffer.lookup_transform_core('med_base', tag_i, rospy.Time(0))
        tfs.append(tf_i)

    poses = []
    for tf_i in tfs:
        frame_position_i = np.array([tf_i.transform.translation.x,
                                     tf_i.transform.translation.y,
                                     tf_i.transform.translation.z
                               ])

        frame_quat_i = np.array([tf_i.transform.rotation.x,
                                 tf_i.transform.rotation.y,
                                 tf_i.transform.rotation.z,
                                 tf_i.transform.rotation.w
                               ])
        block_center_position = frame_position_i
        pose_i = np.concatenate([block_center_position, frame_quat_i])
        poses.append(pose_i)

    return poses

def get_goal_pose(tag_id=3):
    tf_buffer = tf.BufferCore()
    transformer = tf.TransformListener(tf_buffer)
    rospy.sleep(1.0)
    tf_frames = tf_buffer._getFrameStrings()
    tag_frames = [tf_i for tf_i in tf_frames if 'tag_{}'.format(tag_id) in tf_i]

    tfs = []
    for tag_i in tag_frames:
        tf_i = tf_buffer.lookup_transform_core('med_base', tag_i, rospy.Time(0))
        tfs.append(tf_i)

    poses = []
    for tf_i in tfs:
        frame_position_i = np.array([tf_i.transform.translation.x,
                                     tf_i.transform.translation.y,
                                     tf_i.transform.translation.z
                               ])

        frame_quat_i = np.array([tf_i.transform.rotation.x,
                                 tf_i.transform.rotation.y,
                                 tf_i.transform.rotation.z,
                                 tf_i.transform.rotation.w
                               ])
        block_center_position = frame_position_i
        pose_i = np.concatenate([block_center_position, frame_quat_i])
        poses.append(pose_i)

    return poses



# TODO: Transform the pose of the desired grasp_frame to get the med.wrist instead
def tr_pose(pose, frame_origin_name, new_frame_name):
    tf_buffer = tf.BufferCore()
    transformer = tf.TransformListener(tf_buffer)
    rospy.sleep(1.0)




def pickup_block():
    rospy.init_node('med_block_pickup')

    med = Med(manual_execute=True)
    med.connect()
    med.set_grasping_force(60.0)
    med.open_gripper()

    # Add table plane.
    scene = moveit_commander.PlanningSceneInterface(ns="victor")
    scene.add_plane('table_plane', convert_to_pose_msg([0,0,0,0,0,0]))

    # Get tag frame:
    tag_id = 1
    # while True:
    block_poses = get_block_pose()
    block_pose = block_poses[1]
    print('Block Pose: ', block_pose)
    block_xyz = block_pose[:3]
    block_euler = euler_from_quaternion(block_pose[3:])

    block_theta = block_euler[-1]
    print(block_euler)

    pregrasp_h = 0.30
    med.plan_to_joint_config(med.arm_group, [0,0,0,0,0,0,0])
    pregrasp_xyz = block_xyz + np.array([0,0, pregrasp_h])

    plan_frame = med.wrist
    grasp_frame = 'grasp_frame'

    while True:
        _, result, _ = med.plan_to_pose(med.arm_group, plan_frame,  list(pregrasp_xyz)+ [0.0, np.pi, block_theta], frame_id='med_base')
        if result.error_code < 0:
            replan = input('Replan? [y/n]')
            if replan != 'y':
                exit()
        else:
            break

    # Go down and Grasp
    grasp_xyz = block_xyz + np.array([0,0, 0.15])
    while True:
        _, result, _ = med.plan_to_pose(med.arm_group, plan_frame,  list(grasp_xyz)+ [0.0, np.pi, block_theta], frame_id='med_base')
        if result.error_code < 0:
            replan = input('Replan? [y/n]')
            if replan != 'y':
                exit()
        else:
            break

    med.grasp(55.0)

    # pregrasp_h = 0.30
    # med.plan_to_joint_config(med.arm_group, [0,0,0,0,0,0,0])
    # pregrasp_xyz = block_xyz + np.array([0,0, pregrasp_h])
    #
    # while True:
    #     _, result, _ = med.plan_to_pose(med.arm_group, med.wrist,  list(pregrasp_xyz)+ [0.0, np.pi, block_theta], frame_id='med_base')
    #     if result.error_code < 0:
    #         replan = input('Replan? [y/n]')
    #         if replan != 'y':
    #             exit()
    #     else:
    #         break





    med.plan_to_joint_config(med.arm_group, [0,0,0,0,0,0,0])
    exit()




if __name__ == '__main__':
    pickup_block()
