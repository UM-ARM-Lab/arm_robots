#! /usr/bin/env python
import rospy
from arm_robots.med import Med
import numpy as np
import pdb

def main():
    rospy.init_node('med_motion')

    med = Med(manual_execute=True)
    med.connect()
    med.set_grasping_force(30.0)
    med.open_gripper()
    
    # Pick and place "demo"
    med.plan_to_joint_config(med.arm_group, [0,0,0,0,0,0,0])    

    while True:
        _, result, _ = med.plan_to_pose(med.arm_group, med.wrist, [0.6, 0.12, 0.45, 0.0, np.pi, 0.0], frame_id='med_base')
        if result.error_code < 0:
            replan = input('Replan? [y/n]')
            if replan != 'y':
                exit()
        else:
            break
        
    med.plan_to_position_cartesian(med.arm_group, med.wrist, target_position=[0.6, 0.12, 0.35])
    med.grasp(15.0)
    med.plan_to_position_cartesian(med.arm_group, med.wrist, target_position=[0.6, 0.12, 0.45])
    med.plan_to_position_cartesian(med.arm_group, med.wrist, target_position=[0.6, -0.12, 0.45])
    med.plan_to_position_cartesian(med.arm_group, med.wrist, target_position=[0.6, -0.12, 0.365])
    med.release()
    med.plan_to_position_cartesian(med.arm_group, med.wrist, target_position=[0.6, -0.12, 0.45]) 
    med.plan_to_joint_config(med.arm_group, [0,0,0,0,0,0,0])

    # med.grasp(width=10.0)
    # Plan to joint config.
    # med.plan_to_joint_config(med.arm_group, [0,1.0,0,1.0,0,1.0,0])
    # med.plan_to_joint_config(med.arm_group, [0,0,0,0,0,0,0])

    # Plan to position (x,y,z) relative to robot base.
    # med.plan_to_position(med.arm_group, med.wrist, [0.6, 0.0, 0.6])
    # med.plan_to_joint_config(med.arm_group, [0,0,0,0,0,0,0])

    # # Plan to pose relative to robot base.

    # med.release()
    # med.plan_to_joint_config(med.arm_group, [0,0,0,0,0,0,0])
    
    # # Get and print current pose!
    # print(med.get_link_pose(med.arm_group, med.wrist))
    # med.plan_to_position_cartesian(med.arm_group, med.wrist, target_position=[0.6, 0.1, 0.35])
    # med.plan_to_position_cartesian(med.arm_group, med.wrist, target_position=[0.6, -0.1, 0.35])
    # med.plan_to_joint_config(med.arm_group, [0,0,0,0,0,0,0])

if __name__ == "__main__":
    main()
