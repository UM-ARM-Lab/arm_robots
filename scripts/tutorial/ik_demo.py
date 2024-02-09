import argparse
import pickle

from matplotlib import colors

import rospy
from arc_utilities import ros_init
from arc_utilities.tf2wrapper import TF2Wrapper
from arm_robots.basic_3d_pose_interactive_marker import Basic3DPoseInteractiveMarker
from arm_robots.get_robot import get_moveit_robot
from geometry_msgs.msg import Point
from moveit_msgs.msg import RobotState, PlanningScene
from sensor_msgs.msg import JointState
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker


@ros_init.with_ros("ik_demo")
def main():
    parser = argparse.ArgumentParser("Run from jacobian_follower directory to find the planning scene pkl file")
    parser.add_argument('--robot-namespace', default='hdt_michigan')
    parser.add_argument('--position-only', action='store_true')

    args = parser.parse_args()

    points_pub = rospy.Publisher("points", Marker, queue_size=10)
    joint_state_viz_pub = rospy.Publisher('joint_states_viz', JointState, queue_size=10)
    planning_scene_viz_pub = rospy.Publisher('planning_scene_viz', PlanningScene, queue_size=10)
    tf = TF2Wrapper()

    def plot_points_rviz(positions, label: str, frame_id='robot_root', **kwargs):
        color_msg = ColorRGBA(*colors.to_rgba(kwargs.get("color", "r")))

        scale = kwargs.get('scale', 0.02)

        msg = Marker()
        msg.header.frame_id = frame_id
        msg.header.stamp = rospy.Time.now()
        msg.ns = label
        msg.id = id
        msg.type = Marker.SPHERE_LIST
        msg.action = Marker.ADD
        msg.pose.orientation.w = 1
        msg.scale.x = scale
        msg.scale.y = scale
        msg.scale.z = scale
        msg.color = color_msg
        for p in positions:
            msg.points.append(p)

        points_pub.publish(msg)

    def make_marker(scale: float):
        marker = Marker(type=Marker.SPHERE)
        s = 0.05
        marker.scale = Point(s, s, s)
        marker.color = ColorRGBA(0.5, 1.0, 0.5, 0.7)
        return marker

    tip_names = ["left_tool", "right_tool"]
    im = {n: Basic3DPoseInteractiveMarker(make_marker=make_marker, name=n) for n in tip_names}

    robot = get_moveit_robot(args.robot_namespace)
    j = robot.jacobian_follower

    with open("./scripts/tutorial/test_planning_scene.pkl", 'rb') as f:
        scene_msg = pickle.load(f)

    group_name = "both_arms"

    name = robot.get_joint_names(group_name)

    while not rospy.is_shutdown():
        poses = []
        points = []
        for tip_name in tip_names:
            pose = im[tip_name].get_pose()
            poses.append(pose)
            points.append(pose.position)

        scene_msg.robot_state.joint_state.position = [0] * len(name)
        scene_msg.robot_state.joint_state.name = name

        default_robot_state = RobotState()
        default_robot_state.joint_state.position = [0] * len(name)
        default_robot_state.joint_state.name = name

        robot_state: RobotState
        if args.position_only:
            robot_state = j.compute_collision_free_point_ik(default_robot_state, points, group_name, tip_names,
                                                            scene_msg)
        else:
            robot_state = j.compute_collision_free_pose_ik(default_robot_state, poses, group_name, tip_names, scene_msg)

        planning_scene_viz_pub.publish(scene_msg)
        for tip_name, position, pose in zip(tip_names, points, poses):
            plot_points_rviz([position], label=tip_name + '_target')
            tf.send_transform_from_pose_msg(pose, 'world', tip_name + '_target')
        if robot_state is not None:
            robot.display_robot_state(robot_state, label='')
            joint_state_viz_pub.publish(robot_state.joint_state)
        else:
            print("No Solution!")


if __name__ == '__main__':
    main()
