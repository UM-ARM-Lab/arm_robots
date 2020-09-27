#! /usr/bin/env python

"""
This moves Victor's arms to a configuration where impedance validations tends to work well.

"""
from arm_robots.victor import Victor
import rospy
from std_msgs.msg import String

left = [-0.694, 0.14, -0.229, -1.11, -0.512, 1.272, 0.077]

right = [0.724, 0.451, 0.94, -1.425, 0.472, 0.777, -0.809]

if __name__ == "__main__":
    rospy.init_node("move_to_impedance_switch")
    rospy.logerr("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
    rospy.logerr("")
    rospy.logerr("    WARNING: THIS IGNORES OBSTACLES. WATCH CLOSELY")
    rospy.logerr("")
    rospy.logerr("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")

    pub = rospy.Publisher("/polly", String, queue_size=10)
    rospy.sleep(0.4)  # Hardcoded so pub can connect. Less annoying than other checks
    pub.publish("I am ignoring obstacles. Be careful")

    victor = Victor(execute_by_default=True)
    victor.plan_to_joint_config("left_arm", left)
    victor.plan_to_joint_config("right_arm", right)

    rospy.loginfo("Ready to be set to impedance mode")
