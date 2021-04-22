#! /usr/bin/env python
import rospy
from arm_robots.med import Med
import numpy as np
import pdb

def main():
    rospy.init_node('med_motion')

    med = Med(manual_execute=False)
    med.connect()
    # med.grasp()
    # med.release()
    med.open_gripper()

    # Plan to joint config.
    med.plan_to_joint_config(med.arm_group, [0,0,0,0,0,0,0])
    # med.plan_to_joint_config(med.arm_group, [0,1.0,0,1.0,0,1.0,0])
    # med.plan_to_joint_config(med.arm_group, [0,0,0,0,0,0,0])

    # Plan to position (x,y,z) relative to robot base.
    # med.plan_to_position(med.arm_group, med.wrist, [0.6, 0.0, 0.6])
    # med.plan_to_joint_config(med.arm_group, [0,0,0,0,0,0,0])

    # # Plan to pose relative to robot base.
    med.plan_to_pose(med.arm_group, med.wrist, [0.6, 0.0, 0.35, 0.0, np.pi, 0.0], frame_id='med_base')
    med.grasp()
    med.plan_to_pose(med.arm_group, med.wrist, [0.6, 0.0, 0.45, 0.0, 3*np.pi/4.0, 0.0], frame_id='med_base')
    med.release()
    med.plan_to_joint_config(med.arm_group, [0,0,0,0,0,0,0])
    
    # # Get and print current pose!
    # print(med.get_link_pose(med.arm_group, med.wrist))
    # med.plan_to_position_cartesian(med.arm_group, med.wrist, target_position=[0.6, 0.1, 0.35])
    # med.plan_to_position_cartesian(med.arm_group, med.wrist, target_position=[0.6, -0.1, 0.35])
    # med.plan_to_joint_config(med.arm_group, [0,0,0,0,0,0,0])

if __name__ == "__main__":
    main()
