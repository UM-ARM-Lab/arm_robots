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
    calibrated_mat = np.array([[1.2519167e-01, 6.2875944e-01, - 7.6745594e-01, 1.5284070e+00],
                      [9.9213248e-01, - 7.8984290e-02, 9.7132102e-02, - 5.0468093e-01],
                      [4.5576331e-04, - 7.7357811e-01, - 6.3370085e-01, 1.1876023e+00],
                      [0.0000000e+00, 0.0000000e+00, 0.0000000e+00, 1.0000000e+00]])
    tfwrapper.send_transform_matrix(calibrated_mat, "victor_root", CAMERA_FRAME, is_static=True)
    executor.spin()


if __name__ == '__main__':
    main()
