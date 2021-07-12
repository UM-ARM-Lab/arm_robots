#! /usr/bin/env python

"""
This moves Victor's arms to a configuration where impedance validations tends to work well.
if you want to actually change the control mode, not just move to the position, set "_actually_switch:=true"
"""
import colorama
from colorama import Fore

import rospy
from arm_robots.victor import Victor

if __name__ == "__main__":
    colorama.init(autoreset=True)
    rospy.init_node("move_to_impedance_switch")
    rospy.logwarn("Make sure you set up a conservative set of obstacles in gazebo")
    rospy.logwarn("View the planning scene in RViz to see what the planner is aware of")
    k = input("Did you set up a conservative set of obstacles in gazebo? [y/N]")
    if k == 'Y' or k == 'y':
        rospy.loginfo(Fore.CYAN + "you better not be lying...")

        victor = Victor(robot_namespace='victor')
        victor.connect()

        actually_switch = rospy.get_param("~actually_switch", False)
        if actually_switch:
            rospy.loginfo("switching to impedance mode")

        victor.move_to_impedance_switch(actually_switch=actually_switch)

        rospy.loginfo("Done")
    else:
        rospy.loginfo("Answered 'no', aborting")
