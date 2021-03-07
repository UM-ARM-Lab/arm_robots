#! /usr/bin/env python
import rospy
from arm_robots.med import Med
import numpy as np
import pdb

def main():
    rospy.init_node('med_motion')

    med = Med()
    med.connect()

    # Plan to joint config.
    med.plan_to_joint_config(med.arm_group, [0,0,0,0,0,0,0])
    med.plan_to_joint_config(med.arm_group, [0,1.0,0,1.0,0,1.0,0])
    med.plan_to_joint_config(med.arm_group, [0,0,0,0,0,0,0])

    # Plan to position (x,y,z) relative to robot base.
    med.plan_to_position(med.arm_group, 'med_kuka_flange', [0.6, 0.0, 0.6])
    med.plan_to_joint_config(med.arm_group, [0,0,0,0,0,0,0])

    # Plan to pose relative to robot base.
    med.plan_to_pose(med.arm_group, 'med_kuka_flange', [0.6, 0.0, 0.35, 0.0, np.pi/2, 0.0], frame_id='med_base')
    # Get and print current pose!
    print(med.get_link_pose(med.arm_group, 'med_kuka_flange'))
    med.plan_to_position_cartesian(med.arm_group, 'med_kuka_flange', target_position=[0.6, 0.1, 0.35])
    med.plan_to_position_cartesian(med.arm_group, 'med_kuka_flange', target_position=[0.6, -0.1, 0.35])
    med.plan_to_joint_config(med.arm_group, [0,0,0,0,0,0,0])

if __name__ == "__main__":
    main()
