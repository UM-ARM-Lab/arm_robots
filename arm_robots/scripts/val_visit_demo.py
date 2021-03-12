#! /usr/bin/env python
import colorama
import numpy as np
import ros_numpy
import rospy
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import Marker

from arc_utilities import ros_init
from arc_utilities.tf2wrapper import TF2Wrapper
from arm_robots.hdt_michigan import Val

ask_before_moving = True


def myinput(msg):
    global ask_before_moving
    if ask_before_moving:
        input(msg)


@ros_init.with_ros("val_visit_demo")
def main():
    np.set_printoptions(suppress=True, precision=0, linewidth=200)
    colorama.init(autoreset=True)

    d = Demo()


class Demo:
    def __init__(self):
        # self.pc_sub = rospy.Subscriber("/camera/depth/color/points", PointCloud2, self.point_cloud_callback)
        self.viz_pub = rospy.Publisher("/closest_point", Marker, queue_size=10)

        self.tf2 = TF2Wrapper()

        self.val = Val(raise_on_failure=True)
        self.val.connect()

        self.val.close_right_gripper()

        rospy.logwarn("Planning to home configuration")
        self.val.plan_to_joint_config('both_arms', 'home')
        self.val.store_current_tool_orientations([self.val.left_tool_name, self.val.right_tool_name])
        self.tool_name = self.val.right_tool_name

        self.target_position_robot_frame = ros_numpy.numpify(self.val.get_link_pose(self.tool_name).position)

        while not rospy.is_shutdown():
            try:
                r = 0.35
                for theta in np.linspace(-np.pi, np.pi, 10):
                    x = np.sin(theta) * r
                    z = np.cos(theta) * r + 0.4
                    lp_t = [x - 0.1, 0.8, z]
                    rp_t = [x + 0.1, 0.8, z]

                    self.val.follow_jacobian_to_position('both_arms', [self.val.left_tool_name, self.val.right_tool_name],
                                                         [[lp_t], [rp_t]],
                                                         vel_scaling=1.0)
            except Exception:
                break

        self.val.disconnect()

    def visualize_target_point(self, p):
        closest_point_marker = Marker()
        closest_point_marker.header.frame_id = 'robot_root'
        closest_point_marker.header.stamp = rospy.Time.now()
        closest_point_marker.action = Marker.ADD
        closest_point_marker.id = 0
        closest_point_marker.type = Marker.SPHERE
        closest_point_marker.color.r = 1
        closest_point_marker.color.a = 1
        closest_point_marker.pose.orientation.w = 1
        closest_point_marker.pose.position.x = p[0]
        closest_point_marker.pose.position.y = p[1]
        closest_point_marker.pose.position.z = p[2]
        s = 0.025
        closest_point_marker.scale = Vector3(x=s, y=s, z=s)
        self.viz_pub.publish(closest_point_marker)

    def point_cloud_callback(self, msg: PointCloud2):
        np_pc = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(msg)
        distance_to_realsense = np.linalg.norm(np_pc, axis=1)
        closest_point_idx = np.argmin(distance_to_realsense)
        closest_point = np_pc[closest_point_idx]

        self.tf2.send_transform(closest_point, [0, 0, 0, 1], msg.header.frame_id, 'closest_point')
        transform = self.tf2.get_transform('robot_root', 'closest_point')
        self.target_position_robot_frame = transform[:3, 3]


if __name__ == "__main__":
    main()
