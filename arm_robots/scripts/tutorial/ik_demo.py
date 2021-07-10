import argparse
import pickle
from typing import Callable, Optional

from matplotlib import colors

import rospy
from arc_utilities import ros_init
from arc_utilities.tf2wrapper import TF2Wrapper
from arm_robots.get_robot import get_moveit_robot
from geometry_msgs.msg import Pose, Point
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from moveit_msgs.msg import RobotState, PlanningScene
from sensor_msgs.msg import JointState
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, Marker, InteractiveMarkerFeedback


def make_box(scale: float):
    marker = Marker(type=Marker.CUBE)
    marker.scale.x = scale * 0.45
    marker.scale.y = scale * 0.45
    marker.scale.z = scale * 0.45
    marker.color.r = 0.5
    marker.color.g = 0.5
    marker.color.b = 0.5
    marker.color.a = 1.0

    return marker


def make_sphere(scale: float):
    marker = Marker(type=Marker.SPHERE)
    marker.scale.x = scale * 0.45
    marker.scale.y = scale * 0.45
    marker.scale.z = scale * 0.45
    marker.color.r = 0.5
    marker.color.g = 0.5
    marker.color.b = 0.5
    marker.color.a = 1.0

    return marker


def make_interactive_marker(name: str,
                            position: Point,
                            make_marker: Callable):
    imarker = InteractiveMarker()
    imarker.header.frame_id = "world"
    imarker.pose.position = position
    imarker.scale = 0.1

    imarker.name = name

    control = InteractiveMarkerControl()
    control.always_visible = True
    control.markers.append(make_marker(imarker.scale))
    imarker.controls.append(control)

    imarker.controls[0].interaction_mode = InteractiveMarkerControl.MOVE_ROTATE_3D

    control = InteractiveMarkerControl()
    control.orientation.w = 1
    control.orientation.x = 1
    control.orientation.y = 0
    control.orientation.z = 0
    control.name = "rotate_x"
    control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
    imarker.controls.append(control)

    control = InteractiveMarkerControl()
    control.orientation.w = 1
    control.orientation.x = 1
    control.orientation.y = 0
    control.orientation.z = 0
    control.name = "move_x"
    control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
    imarker.controls.append(control)

    control = InteractiveMarkerControl()
    control.orientation.w = 1
    control.orientation.x = 0
    control.orientation.y = 1
    control.orientation.z = 0
    control.name = "rotate_z"
    control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
    imarker.controls.append(control)

    control = InteractiveMarkerControl()
    control.orientation.w = 1
    control.orientation.x = 0
    control.orientation.y = 1
    control.orientation.z = 0
    control.name = "move_z"
    control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
    imarker.controls.append(control)

    control = InteractiveMarkerControl()
    control.orientation.w = 1
    control.orientation.x = 0
    control.orientation.y = 0
    control.orientation.z = 1
    control.name = "rotate_y"
    control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
    imarker.controls.append(control)

    control = InteractiveMarkerControl()
    control.orientation.w = 1
    control.orientation.x = 0
    control.orientation.y = 0
    control.orientation.z = 1
    control.name = "move_y"
    control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
    imarker.controls.append(control)
    return imarker


class Basic3DPoseInteractiveMarker:
    def __init__(self,
                 x: float = 0,
                 y: float = 0,
                 z: float = 0,
                 shape: Optional[str] = None,
                 make_marker: Optional[Callable] = None,
                 name: str = 'basic_pose_marker'):
        position = Point(x, y, z)
        self.server = InteractiveMarkerServer(name)

        if make_marker is None:
            if shape is None:
                raise NotImplementedError()
            elif shape == 'sphere':
                make_marker = make_sphere
            elif shape == 'box':
                make_marker = make_box
            else:
                raise NotImplementedError()

        self.marker_name = 'imarker'
        self.imarker = make_interactive_marker(self.marker_name, position, make_marker)
        self.server.insert(self.imarker, self.on_feedback)

        self.server.applyChanges()

    def on_feedback(self, _: InteractiveMarkerFeedback):
        self.server.applyChanges()

    def set_pose(self, pose: Pose):
        self.server.setPose(self.imarker.name, pose)
        self.server.applyChanges()

    def get_pose(self) -> Pose:
        return self.server.get(self.marker_name).pose


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

        robot_state: RobotState
        if args.position_only:
            robot_state = j.compute_collision_free_point_ik(points, group_name, tip_names, scene_msg)
        else:
            robot_state = j.compute_collision_free_pose_ik(poses, group_name, tip_names, scene_msg)

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
