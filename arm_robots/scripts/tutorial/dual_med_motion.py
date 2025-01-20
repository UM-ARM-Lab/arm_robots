#! /usr/bin/env python
import rospy
from arm_robots.dual_med import DualMed
import numpy as np
from victor_hardware_interface_msgs.msg import ControlMode
import numpy as np
from trajectory_msgs.msg import JointTrajectoryPoint

DEG2RAD = np.pi / 180.0
if __name__ == '__main__':
    rospy.init_node('combined_med_motion')
    
    combined_med = DualMed(robot_namespace='combined_med', display_goals=False)
    
    control_mode = combined_med.get_control_modes()
    combined_med.set_thanos_arm_control_mode(ControlMode.JOINT_POSITION, vel=0.1)
    combined_med.set_medusa_arm_control_mode(ControlMode.JOINT_POSITION, vel=0.1)
    
    
    joint_names = combined_med.get_arm_joints()
    default_pos = np.array([
        -30.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
    ] * 2) * DEG2RAD
    print(default_pos, joint_names)
    combined_med.goto_config(default_pos, joint_names, control_mode)
    pass