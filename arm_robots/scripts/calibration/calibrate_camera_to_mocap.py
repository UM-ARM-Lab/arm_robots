#!/usr/bin/env python
from colorama import Fore
import argparse
import logging

import numpy as np
import transformations
from scipy.spatial.transform import Rotation as R
from tqdm import trange

import rospy
from arc_utilities import ros_init
from arc_utilities.tf2wrapper import TF2Wrapper

logger = logging.getLogger(__file__)


def average_transformation_matrices(offsets):
    rots = R.from_matrix([m[:3, :3] for m in offsets])
    mean_rot = rots.mean().as_matrix()
    mean = np.mean(offsets, axis=0)
    mean[:3, :3] = mean_rot
    return mean


def get_transform_stable(tf,
                         camera_tf_name,
                         fiducial_name,
                         close_threshold,
                         n_close_poses_threshold=10,
                         sleep_duration=0.05):
    n_close_poses = 0
    last_transform = tf.get_transform(camera_tf_name, fiducial_name)
    while True:
        transform = tf.get_transform(camera_tf_name, fiducial_name)
        # norm on SE3 matrices is fine if you only care about checking them for closeness
        close = np.linalg.norm(last_transform - transform, ord='fro') < close_threshold
        last_transform = transform
        if close:
            n_close_poses += 1
        else:
            n_close_poses = 0
        if n_close_poses == n_close_poses_threshold:
            break

        rospy.sleep(sleep_duration)

    return transform


@ros_init.with_ros("calibrate_camera_to_mocap")
def main():
    np.set_printoptions(suppress=True, precision=4, linewidth=200)

    parser = argparse.ArgumentParser()
    parser.add_argument('camera_tf_name', help='TF name of the camera in mocap')
    parser.add_argument('camera_link_name', help='TF name of the camera link')

    args = parser.parse_args(rospy.myargv()[1:])

    tf = TF2Wrapper()

    mocap_world_frame = 'mocap_world'

    offsets = []
    fiducial_center_to_marker_corner = np.sqrt(0.118 ** 2 / 2)
    i = 0
    camera2fiducial_last = None
    # input(Fore.CYAN + "Press enter to begin calibration" + Fore.RESET)
    for t in trange(2):
        fiducial_center_to_fiducial_mocap = transformations.compose_matrix(
            translate=[-fiducial_center_to_marker_corner, fiducial_center_to_marker_corner, 0])
        # send TF from fiducial to mocap markers on the fiducial board
        tf.send_transform_matrix(fiducial_center_to_fiducial_mocap, f"fiducial_{i}", f"fiducial_{i}_mocap_markers")

        while True:
            # get the aruco transform once it is stable, and ensure it's different enough from the previous one
            camera2fiducial = get_transform_stable(tf, args.camera_link_name, f"fiducial_{i}", 0.01)
            if camera2fiducial_last is None:
                break
            dist = np.linalg.norm(camera2fiducial_last - camera2fiducial, ord='fro')
            close = dist < 0.1
            if not close:
                break
        camera2fiducial_last = camera2fiducial

        fiducial2camera = transformations.inverse_matrix(camera2fiducial)

        mocap2fiducial_markers = tf.get_transform(mocap_world_frame, f"mocap_calib{i}_calib{i}")
        mocap2fiducial = mocap2fiducial_markers @ transformations.inverse_matrix(fiducial_center_to_fiducial_mocap)

        mocap2camera_sensor_detected = mocap2fiducial @ fiducial2camera
        mocap2camera_markers = tf.get_transform(mocap_world_frame, args.camera_tf_name)
        mocap2camera_sensor_offset = np.linalg.solve(mocap2camera_markers, mocap2camera_sensor_detected)

        offsets.append(mocap2camera_sensor_offset)

    average_offset = average_transformation_matrices(offsets)
    error = sum([np.linalg.norm(average_offset - t, ord='fro') for t in offsets])
    trans = transformations.translation_from_matrix(average_offset)
    rot = transformations.euler_from_matrix(average_offset)
    roll, pitch, yaw = rot
    print(Fore.GREEN)
    print(f"Error: {error:.4f}")
    print(f"Re-run the calibration if the above error is >0.1")
    print('Copy This into the static_transform_publisher')
    print(f'{trans[0]:.5f} {trans[1]:.5f} {trans[2]:.5f} {yaw:.5f} {pitch:.5f} {roll:.5f}')
    print("NOTE: tf2_ros static_transform_publisher uses Yaw, Pitch, Roll so that's what is printed above")
    print(Fore.RESET)


if __name__ == '__main__':
    main()
