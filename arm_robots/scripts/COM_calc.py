#!/usr/bin/env python

import sys
import rospy
import argparse
from urdf_parser_py.urdf import URDF
import tf
from val_calib import quat2matrix
import numpy as np
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
# look up tf and urdf info, and do the calculation and publish Center of Mass (COM) on a topic com
# optionally publish marker on com_marker topic, can be shown in rviz

def pub_marker_f(frame, pt, pub_marker):
    marker = Marker()
    marker.header.frame_id = frame
    marker.header.stamp    = rospy.get_rostime()
    marker.ns = "robot"
    marker.id = 0
    marker.type = 2 # sphere
    marker.action = 0
    marker.pose.position = pt
    marker.pose.orientation.x = 0
    marker.pose.orientation.y = 0
    marker.pose.orientation.z = 0
    marker.pose.orientation.w = 1.0
    marker.scale.x = 0.05
    marker.scale.y = 0.05
    marker.scale.z = 0.05
    marker.color.r = 0.0
    marker.color.g = 1.0
    marker.color.b = 0.0
    marker.color.a = 1.0
    marker.lifetime = rospy.Duration(0)
    pub_marker.publish(marker)

def main(vis = True):
    #initialize listener
    r = 30
    rospy.init_node("COM_calc")
    listener = tf.TransformListener()
    rate = rospy.Rate(r)
    print("tf listener set up")
    #initialize publisher
    pub = rospy.Publisher("com", Point, queue_size=10)
    pub_marker = rospy.Publisher("com_marker", Marker, queue_size=10)
    #initialize URDF parser
    robot = URDF.from_parameter_server()
    while not rospy.is_shutdown():        
        num_links = len(robot.links)
        try:
            mass_tot = 0
            numerator = np.zeros((3)) # Sigma mass * position of each link 
            for i in range(num_links):
                if hasattr(robot.links[i], 'inertial') and hasattr(robot.links[i].inertial, 'mass'):
                    name = robot.links[i].name
                    mass = robot.links[i].inertial.mass
                    mass_tot += mass
                    origin = np.array(robot.links[i].inertial.origin.xyz).reshape(3,1)
                    (p, r) = listener.lookupTransform('val_cal', name, rospy.Time(0))
                    T = quat2matrix(p,r)
                    numerator += np.dot(T, np.vstack((origin, [[1]])))[0:3,0] * mass
            print("mass tot:", mass_tot)
            com = numerator/mass_tot
            print("COM", com)
            #publish message on topic com
            pt = Point(com[0], com[1], com[2])
            pub.publish(pt)
            if vis:
                pub_marker_f('val_cal', pt, pub_marker)

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print("tf listener not working")
            continue
        rate.sleep()   
                        
if __name__ == "__main__":
    main()
