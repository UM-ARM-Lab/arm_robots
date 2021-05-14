#! /usr/bin/env python
import rospy
from arm_robots.med import Med
import numpy as np
from victor_hardware_interface_msgs.msg import ControlMode


def main(sim=True):
    rospy.init_node('med_motion')

    med = Med(display_goals=False)
    med.connect()
    med.set_control_mode(ControlMode.JOINT_POSITION, vel=0.3)

    if not sim:
        med.set_grasping_force(40.0)
        med.open_gripper()

    # Example: Planning to joint configuration.
    med.plan_to_joint_config(med.arm_group, [0, 1.0, 0, -1.0, 0, 1.0, 0])

    # Example: Planning to pose.
    med.plan_to_pose(med.arm_group, med.wrist, [0.6, 0.12, 0.45, 0.0, np.pi, 0.0], frame_id='med_base')

    # Example: Cartesian planning.
    med.plan_to_position_cartesian(med.arm_group, med.wrist, target_position=[0.6, 0.12, 0.35])

    # Example: Grasp object of approx. width 15mm
    if not sim:
        med.grasp(width=15.0)

    med.plan_to_position_cartesian(med.arm_group, med.wrist, target_position=[0.6, 0.12, 0.45])
    med.plan_to_position_cartesian(med.arm_group, med.wrist, target_position=[0.6, -0.12, 0.45])
    med.plan_to_position_cartesian(med.arm_group, med.wrist, target_position=[0.6, -0.12, 0.365])

    # Example: Release grasp.
    if not sim:
        med.release()

    # Example: Getting the robot state.
    print("Current pose at the wrist:")
    print(med.get_link_pose(med.wrist))

    med.plan_to_joint_config(med.arm_group, [0, 0, 0, 0, 0, 0, 0])


if __name__ == "__main__":
    main()
