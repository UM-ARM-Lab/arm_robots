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
        7.0,
        28.0,
        23.0,
        -55.0,
        73.0,
        65.0,
        11.0
    ] + [
        -7.0,
         28.0,
        -25.0,
        -55.0,
        -73.0,
         65.0,
        -11.0
    ]) * DEG2RAD
    closer_pos = np.array([0.3006420184241826, 0.5993909486763955, 0.40131466087741896, -0.7900838754565486, 1.165075665734823, 1.0100111804358418, 0.2827273399352438, -0.2850081176576266, 0.5891905157436949, -0.43623992792161886, -0.8066735181942474, -1.1738279847408937, 1.0220741416979744, -0.27070302778948274])
    print(default_pos, joint_names)
    input()
    
    result = combined_med.get_plan_from_goal_config(default_pos, joint_names)
    print(result.plan, result.success)
    input()
    combined_med.follow_plan(result)
    
    result = combined_med.get_plan_from_goal_config(closer_pos, joint_names)
    print(result.plan, result.success)
    input()
    combined_med.follow_plan(result)
    
    # combined_med.goto_config(default_pos, joint_names, control_mode)
    pass