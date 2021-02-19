#!/usr/bin/env python
import rospy
import numpy as np
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

def main():
    rospy.init_node("points_and_lines", anonymous=True)
    traj = []
    for i in range(100):  # 给每一个marker添加points
        y = 5 * np.sin(i/100.0 * 2 * np.pi)
        z = 0.0
        traj.append([i-50, y, z])
    rviz_plot_traj(traj, 'val_root')

#traj is a list of position coordinate, each element is a list [x, y, z]
def rviz_traj_plot(traj, frame = 'val_root'):
    #rospy.init_node("points_and_lines", anonymous=True)
    pub = rospy.Publisher("visualization_marker", Marker, queue_size=10)
    rate = rospy.Rate(30)
    f = 0.0
    n = 0
    while not rospy.is_shutdown():
        types = [Marker.POINTS, Marker.LINE_STRIP]
        markers = [Marker() for _ in ['points', 'line_strip']]
        for i, m in enumerate(markers):  # 分别处理不同的marker 
            m.header.frame_id = frame
            m.header.stamp = rospy.Time.now()
            m.ns = 'points_and_lines'
            m.pose.orientation.w = 1.0
            m.action = Marker.ADD
            m.id = i
            m.type = types[i]
            m.color.a = 1.0
            if i == 0: # point
                m.scale.x = 0.015
                m.scale.y = 0.015
                m.color.g = 1.0
            elif i == 1:  # line_strip
                m.scale.x = 0.015
                m.color.b = 1.0
            elif i == 2:
                m.scale.x = 0.1
                m.color.r = 1.0
        for pt in traj:  # 给每一个marker添加points
            p = Point(pt[0], pt[1], pt[2])
            markers[0].points.append(p)
            markers[1].points.append(p)
            #markers[2].points.extend([p, Point(i-50, y, z+1)])
        for m in markers:
            pub.publish(m)
        n += 1
        if n == 20:
            break
        rate.sleep()
        
if __name__=='__main__':
    main()