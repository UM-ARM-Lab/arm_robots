import copy
import open3d as o3d
import transformations

from arc_utilities import ros_init
import numpy as np
from arc_utilities.tf2wrapper import TF2Wrapper
from arm_robots.basic_3d_pose_interactive_marker import Basic3DPoseInteractiveMarker
from arm_robots.hdt_michigan import Val
from geometry_msgs.msg import Point, Pose
from ros_numpy import numpify
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker


def draw_registration_result(source, target, transformation):
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    source_temp.paint_uniform_color([1, 0.706, 0])
    target_temp.paint_uniform_color([0, 0.651, 0.929])
    source_temp.transform(transformation)
    o3d.visualization.draw_geometries([source_temp, target_temp])


@ros_init.with_ros("calibrate_pc_to_fk")
def main():
    val = Val(raise_on_failure=True)
    val._max_velocity_scale_factor = 1.0
    val.connect()

    frame_a = 'mocap_world'
    frame_b = 'mocap_val_root_val_root'
    radius = 0.005

    def make_left_marker(scale: float):
        marker = Marker(type=Marker.SPHERE)
        marker.scale = Point(2 * radius, 2 * radius, 2 * radius)
        marker.color = ColorRGBA(0.5, 1.0, 0.5, 0.7)

        text = Marker(type=Marker.TEXT_VIEW_FACING)
        text.text = 'left'
        text.scale.z = radius * 3
        text.color = ColorRGBA(1.0, 1.0, 1.0, 1.0)

        return [marker, text]

    def make_right_marker(scale: float):
        marker = Marker(type=Marker.SPHERE)
        marker.scale = Point(2 * radius, 2 * radius, 2 * radius)
        marker.color = ColorRGBA(1.0, 0.5, 0.5, 0.7)

        text = Marker(type=Marker.TEXT_VIEW_FACING)
        text.text = 'right'
        text.color = ColorRGBA(1.0, 1.0, 1.0, 1.0)
        text.scale.z = radius * 3

        return [marker, text]

    left_tool_im = Basic3DPoseInteractiveMarker(name='left',
                                                make_marker=make_left_marker,
                                                frame_id=frame_a)
    right_tool_im = Basic3DPoseInteractiveMarker(name='right',
                                                 make_marker=make_right_marker,
                                                 frame_id=frame_a)

    tf = TF2Wrapper()

    pc_points = []
    fk_points = []
    for i in range(6):
        joint_config_ = val.get_move_group_commander('both_arms').get_random_joint_values()
        joint_config = (np.array(joint_config_) * 0.05).tolist()
        val.plan_to_joint_config('both_arms', joint_config)

        left_fk_transform = tf.get_transform('mocap_val_root_val_root', 'left_tool')
        left_fk_point = left_fk_transform[:3, -1]
        right_fk_transform = tf.get_transform('mocap_val_root_val_root', 'right_tool')
        right_fk_point = right_fk_transform[:3, -1]

        left_tool_im.set_pose(Pose(point=Point(*left_fk_point)))
        right_tool_im.set_pose(Pose(point=Point(*right_fk_point)))

        input("move marker...")
        left_pose_frame_a = left_tool_im.get_pose()
        left_pc_point = numpify(left_pose_frame_a.position)
        right_pose_frame_a = right_tool_im.get_pose()
        right_pc_point = numpify(right_pose_frame_a.position)

        pc_points.append(left_pc_point)
        pc_points.append(right_pc_point)
        fk_points.append(left_fk_point)
        fk_points.append(right_fk_point)

    with open("calib.npz", 'wb') as file:
        np.savez(file, fk_points=fk_points, pc_points=pc_points)

    pc_points_pcd = o3d.geometry.PointCloud()
    pc_points_pcd.points = o3d.utility.Vector3dVector(pc_points)

    fk_points_pcd = o3d.geometry.PointCloud()
    fk_points_pcd.points = o3d.utility.Vector3dVector(fk_points)

    threshold = 0.1
    init = transformations.compose_matrix(angles=np.deg2rad([-0.262, 0.810, 50.48]), translate=[1.197, -0.938, -1.198])
    evaluation = o3d.pipelines.registration.evaluate_registration(fk_points_pcd, pc_points_pcd, threshold, init)
    draw_registration_result(fk_points_pcd, pc_points_pcd, init)
    print(evaluation)

    reg_p2p = o3d.pipelines.registration.registration_icp(fk_points_pcd, pc_points_pcd, threshold, init,
                                                          o3d.pipelines.registration.TransformationEstimationPointToPoint(),
                                                          o3d.pipelines.registration.ICPConvergenceCriteria(
                                                              max_iteration=2000))
    evaluation = o3d.pipelines.registration.evaluate_registration(fk_points_pcd, pc_points_pcd, threshold, reg_p2p.transformation)
    print(reg_p2p.transformation)
    print(evaluation)
    draw_registration_result(fk_points_pcd, pc_points_pcd, reg_p2p.transformation)

    print(np.rad2deg(transformations.euler_from_matrix(reg_p2p.transformation)))
    print(transformations.translation_from_matrix(reg_p2p.transformation) * 1000)

    val.disconnect()


if __name__ == '__main__':
    main()
