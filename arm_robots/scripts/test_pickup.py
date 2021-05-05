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
        # k = input('')
        # if k == 'c':
        #     break

    # View the block detected pose:
    # publisher = rospy.Publisher('realsense_box_plan', Marker, queue_size=100)
    # rate = rospy.Rate(10)
    #
    # block_size = np.array([0.145, 0.09, 0.051]) # (L x W x H) in meters
    # tag_size = 0.09
    # block_color = np.array([255, 0, 0, 255])/255 # (R, G, B, A)
    #
    # marker = Marker()
    # marker.id = 10
    # marker.header.frame_id = 'med_base'
    # marker.type = Marker.CUBE
    # # size
    # marker.scale.x = block_size[0]
    # marker.scale.y = block_size[1]
    # marker.scale.z = block_size[2]
    # # pose (with respect to the frame 'med_table')
    # marker.pose.position.x = block_pose[0]
    # marker.pose.position.y = block_pose[1]
    # marker.pose.position.z =  block_pose[2]
    # marker.pose.orientation.x = block_pose[3]
    # marker.pose.orientation.y = block_pose[4]
    # marker.pose.orientation.z = block_pose[5]
    # marker.pose.orientation.w = block_pose[6]
    # # color
    # marker.color.r = block_color[0]
    # marker.color.g = block_color[1]
    # marker.color.b = block_color[2]
    # marker.color.a = block_color[3]
    #
    # while not rospy.is_shutdown():
    #     publisher.publish(marker)
    #     rate.sleep()

    # -------------------------

    pregrasp_h = 0.30
    med.plan_to_joint_config(med.arm_group, [0,0,0,0,0,0,0])
    pregrasp_xyz = block_xyz + np.array([0,0, pregrasp_h])

    while True:
        _, result, _ = med.plan_to_pose(med.arm_group, med.wrist,  list(pregrasp_xyz)+ [0.0, np.pi, block_theta], frame_id='med_base')
        if result.error_code < 0:
            replan = input('Replan? [y/n]')
            if replan != 'y':
                exit()
        else:
            break

    # Go down and Grasp
    grasp_xyz = block_xyz + np.array([0,0, 0.15])
    while True:
        _, result, _ = med.plan_to_pose(med.arm_group, med.wrist,  list(grasp_xyz)+ [0.0, np.pi, block_theta], frame_id='med_base')
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
