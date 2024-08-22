import os

from ament_index_python import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


def load_moveitpy(robot_name: str, kinematics_file: str = None):
    from moveit import MoveItPy

    moveit_config = load_moveit_config(robot_name, kinematics_file).to_dict()
    moveitpy = MoveItPy(node_name=f"{robot_name}_moveitpy", config_dict=moveit_config)
    return moveitpy, moveit_config


def load_moveit_config(robot_name: str, kinematics_file: str = None):
    moveit_package_name = f"{robot_name}_moveit_config"
    builder = MoveItConfigsBuilder(robot_name=robot_name, package_name=moveit_package_name)
    builder = builder.moveit_cpp(
        file_path=os.path.join(
            get_package_share_directory(moveit_package_name),
            "config",
            "moveit_cpp.yaml",
        )
    )
    if kinematics_file:
        builder = builder.robot_description_kinematics(str(kinematics_file))
    return builder.to_moveit_configs()
