from rclpy.node import Node
import numpy as np
from arm_utilities.tf2wrapper import TF2Wrapper
import rclpy
from rclpy.executors import MultiThreadedExecutor
import ast

CAMERA_FRAME = "zivid_optical_frame"


def validate_calibrated_mat(calibrated_mat_param):
    """Validate and convert calibrated_mat parameter to 4x4 numpy array from flat list."""
    if not isinstance(calibrated_mat_param, list) or len(calibrated_mat_param) != 16:
        raise ValueError("calibrated_mat must be a flat list of 16 numbers")
    for val in calibrated_mat_param:
        if not isinstance(val, (int, float)):
            raise ValueError("All values in calibrated_mat must be numbers")
    mat = np.array(calibrated_mat_param, dtype=np.float64).reshape((4, 4))
    return mat

def validate_mocap_vec(mocap_vec_param):
    """Validate and convert mocap_vec parameter to numpy array."""
    if not isinstance(mocap_vec_param, list) or len(mocap_vec_param) != 7:
        raise ValueError("mocap_vec must be a list of 7 numbers")
    
    for val in mocap_vec_param:
        if not isinstance(val, (int, float)):
            raise ValueError("All values in mocap_vec must be numbers")
    
    return np.array(mocap_vec_param, dtype=np.float64)


def main():
    rclpy.init()
    node = Node("create_collision_scene")
    
    # Declare parameters
    node.declare_parameter('calibrated_mat', "[]")
    node.declare_parameter('mocap_vec', "[]")
    
    # Get parameters as strings
    calibrated_mat_param_str = node.get_parameter('calibrated_mat').value
    mocap_vec_param_str = node.get_parameter('mocap_vec').value

    # Evaluate string to list
    try:
        calibrated_mat_param = ast.literal_eval(calibrated_mat_param_str)
        mocap_vec_param = ast.literal_eval(mocap_vec_param_str)
    except Exception as e:
        node.get_logger().error(f"Failed to evaluate parameters: {e}")
        return

    # Validate and convert to numpy arrays
    try:
        calibrated_mat = validate_calibrated_mat(calibrated_mat_param)
        mocap_vec = validate_mocap_vec(mocap_vec_param)
    except ValueError as e:
        node.get_logger().error(f"Parameter validation failed: {e}")
        return

    node.get_logger().info(f"calibrated_mat: \n{calibrated_mat}")
    node.get_logger().info(f"mocap_vec: \n{mocap_vec}")

    executor = MultiThreadedExecutor()
    executor.add_node(node)
    tfwrapper = TF2Wrapper(node)
    tfwrapper.send_transform_matrix(calibrated_mat, "victor_root", CAMERA_FRAME, is_static=True)
    tfwrapper.send_transform(mocap_vec[:3], mocap_vec[3:], "mocap_world", "victor_root", is_static=True)
    
    node.get_logger().info("Transform publishers started")
    executor.spin()


if __name__ == '__main__':
    main()
