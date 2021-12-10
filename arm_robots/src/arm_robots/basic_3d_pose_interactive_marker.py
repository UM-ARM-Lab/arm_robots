from typing import Callable, Optional

from geometry_msgs.msg import Point, Pose
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from visualization_msgs.msg import Marker, InteractiveMarker, InteractiveMarkerControl, InteractiveMarkerFeedback


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
                            make_marker: Callable,
                            frame_id):
    imarker = InteractiveMarker()
    imarker.header.frame_id = frame_id
    imarker.pose.position = position
    imarker.scale = 0.1

    imarker.name = name

    control = InteractiveMarkerControl()
    control.always_visible = True
    marker = make_marker(imarker.scale)
    if isinstance(marker, list):
        control.markers.extend(marker)
    else:
        control.markers.append(marker)
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
                 frame_id: str = 'world',
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
        self.imarker = make_interactive_marker(self.marker_name, position, make_marker, frame_id)
        self.server.insert(self.imarker, self.on_feedback)

        self.server.applyChanges()

    def on_feedback(self, _: InteractiveMarkerFeedback):
        self.server.applyChanges()

    def set_pose(self, pose: Pose):
        self.server.setPose(self.imarker.name, pose)
        self.server.applyChanges()

    def get_pose(self) -> Pose:
        return self.server.get(self.marker_name).pose
