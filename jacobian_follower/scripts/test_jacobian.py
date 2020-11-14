#!/usr/bin/env python
import matplotlib.pyplot as plt
import numpy as np

import ros_numpy
from arc_utilities import ros_init
from arm_robots.victor import Victor
from geometry_msgs.msg import Point


def xy_spiral(position: Point, radius: float, n_steps: int):
    radial_speed = radius / n_steps
    spiral_length = n_steps ** 2 * radial_speed / 2
    position = ros_numpy.numpify(position)
    for d in np.linspace(0, spiral_length, n_steps):
        t = np.sqrt(2 * d / radial_speed)
        dx = radial_speed * t * np.cos(t)
        dy = radial_speed * t * np.sin(t)
        delta_pos = np.array([dx, dy, 0])
        pos_i = position + delta_pos
        yield pos_i


def s():
    xs = []
    ys = []
    for pos in xy_spiral(None, radius=1.0, n_steps=80):
        xs.append(pos[0])
        ys.append(pos[1])

        # v.follow_jacobian_to_position("left_arm", ['left_tool_placeholder'], [[[pos]]], 0.2)
    plt.plot(xs, ys, linewidth=0.3)
    plt.axis("equal")
    plt.show()


def main():
    ros_init.rospy_and_cpp_init("test_jacobain_follower")

    v = Victor()
    v.plan_to_pose("left_arm", "left_tool_placeholder", target_pose=[0.7, 0.3, 0.8, np.pi, 0, 0])
    current_pose = v.get_link_pose("left_arm", "left_tool_placeholder")
    for pos in xy_spiral(current_pose.position, radius=0.5, n_steps=200):
        v.follow_jacobian_to_position("left_arm", ['left_tool_placeholder'], [[pos]], vel_scaling=0.05)
    ros_init.shutdown()


# from arm_robots.victor import Victor
#
#
# def main():
#     victor = Victor()
#
#     group_name = 'both_arms'
#     end_effector = 'left_tool_placeholder'
#     end_position = [0.7, 0.0, 1.0]
#     victor.follow_jacobian_to_position(group_name, end_effector, end_position)


if __name__ == '__main__':
    main()
