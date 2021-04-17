#! /usr/bin/env python
import rospy
from arm_robots.med import Med
import numpy as np
import pdb

def main():
    rospy.init_node('med_motion')

    med = Med()
    med.connect()

    # home robot
    med.plan_to_joint_config(med.arm_group, [0,0,0,0,0,0,0])

    # Plan to pose relative to robot base.
    med.plan_to_pose(med.arm_group, med.wrist, [0.6, 0.0, 0.15, 0.0, np.pi, 0.0], frame_id='med_base')

    # Get and print current pose!
    print(med.get_link_pose(med.arm_group, med.wrist))

    # home the arm after you're done
    med.plan_to_joint_config(med.arm_group, [0,0,0,0,0,0,0])

if __name__ == "__main__":
    main()
