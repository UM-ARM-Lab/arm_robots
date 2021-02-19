#!/usr/bin/env python  
import roslib
roslib.load_manifest('arm_robots')

import rospy
import tf

if __name__ == '__main__':
    rospy.init_node('hand_tf_broadcaster')
    br = tf.TransformBroadcaster()
    rate = rospy.Rate(30.0)
    while not rospy.is_shutdown():
        '''
        br.sendTransform((0.0125, 0.0, 0.08),
                         (0.0, 0.0, 0.0, 1.0),
                         rospy.Time.now(),
                         "right_hand",
                         "end_effector_right")
        br.sendTransform((-0.0125, 0.0, 0.08),
                        (0.0, 0.0, 0.0, 1.0),
                        rospy.Time.now(),
                        "left_hand",
                        "end_effector_left")
        
        br.sendTransform((0.0, 0.0, 0.0),
                        (0, 0, 0.7071068, 0.7071068),
                        rospy.Time.now(),
                        "base",
                        "val_cal")
        '''
        
        rate.sleep()