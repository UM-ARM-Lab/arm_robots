from rclpy.node import Node
import numpy as np
from arm_utilities.tf2wrapper import TF2Wrapper
import rclpy
from rclpy.executors import MultiThreadedExecutor
from threading import Thread

CAMERA_FRAME = "zivid_optical_frame"


def main():
    rclpy.init()
    node = Node("create_collision_scene")
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    tfwrapper = TF2Wrapper(node)
    calibrated_mat = np.array([[ 0.15080321 , 0.45128095 ,-0.87954754 , 1.4242008 ],
                             [ 0.9875272 , -0.0280358  , 0.15493222 , 0.36401126],
                             [ 0.04525914 ,-0.8919414 , -0.4498801 ,  1.0284033 ],
                             [ 0.      ,    0.    ,      0.        ,  1.        ]])
    tfwrapper.send_transform_matrix(calibrated_mat, "victor_root", CAMERA_FRAME, is_static=True)
    mocap_vec = np.array([-0.6229922806800796, 0.6858175240681051, -0.6757562460396909,
                          0.010570186931784202 ,-0.0035824702370211847 ,0.06951835956903875 ,
                          0.9975182377972135])
    tfwrapper.send_transform(mocap_vec[:3], mocap_vec[3:], "mocap_world", "victor_root", is_static=True)
    executor.spin()


if __name__ == '__main__':
    main()
