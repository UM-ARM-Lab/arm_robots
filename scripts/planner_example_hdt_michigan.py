#! /usr/bin/env python

import arm_or_robots.motion_hdt_michigan
import rospy
import time


import IPython
import numpy as np


def run_mehm():
    rospy.init_node("motion_enabled_hdt_michigan")

    print("Using joint position mode")

    test_case = 'single_joint_moving'

    mehm = arm_or_robots.motion_hdt_michigan.MotionEnabledHDTMichigan()

    mehm.move_to_home()

    # mehm.open_gripper("left", blocking=True)
    # mehm.open_gripper("right", blocking=True)

    # mehm.close_gripper("left", blocking=True)
    # mehm.close_gripper("right", blocking=True)

    if test_case == 'ik_interpolation':
        # generate the trajectory
        path = []
        mehm.enable_whole_body()
        for i in range(300):
            left_gripper_tm = np.array([[1, 0, 0, 0.77],
                                        [0, 0, 1, 0.45+i*0.001],
                                        [0,-1, 0, 0.26],
                                        [0, 1, 1,    1]])

            right_gripper_tm = np.array([[-1, 0, 0, 1.20],
                                        [ 0, 0, 1, 0.45+i*0.001],
                                        [ 0, 1, 0, 0.26],
                                        [ 0, 1, 1,    1]])

            maniptm_list = [('left_arm',left_gripper_tm),('right_arm',right_gripper_tm)]

            config = mehm.cbirrt.DoGeneralIK(maniptm=maniptm_list, execute=False)
            config[0] = 0
            config[7] = 0

            if config is not None:
                path.append(config)
            else:
                print("No IK solution.")
                IPython.embed()

        arm_traj = mehm.traj_from_path(path)

        mehm.execute_trajectory(arm_traj)

    elif test_case == 'config_space_planning':
        target_config = np.array([0,0,0,0,0.7,-0.9,0.1,0,0,0,0,-0.7,0.3,1.2,0,0])

        path = []
        for i in range(200):
            path.append(target_config * i / 200.0)

        arm_traj = mehm.traj_from_path(path)

        mehm.execute_trajectory(arm_traj)

        # mehm.plan_to_configuration_whole_body(target_config, execute=True)
    elif test_case == 'single_joint_moving':
        # # target_config = np.array([-0.5,0,0,0,0,0,0,-0.5,0,0,0,0,0,0,0,0.5])
        # initial_config = np.array([0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0.5])

        # path = []
        # for i in range(200):
        #     path.append(initial_config * i / 200.0)

        # arm_traj = mehm.traj_from_path(path)

        # mehm.execute_trajectory(arm_traj)

        # IPython.embed()

        target_config = np.array([1.4,0,0,-0.7,0,0,0,1.4,0,0,-0.7,0,0,0,0,0.7])
        # target_config = np.array([0.6,0,0,0.6,0,0,0,0.6,0,0,0.6,0,0,0,0,-0.3])
        # target_config = np.array([0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0])

        # path = []
        # for i in range(200):
        #     path.append(target_config * i / 200.0)

        path = [np.zeros(16),target_config]

        arm_traj = mehm.traj_from_path(path)

        mehm.execute_trajectory(arm_traj)

    elif test_case == 'single_arm_moving':
        mehm.enable_only_right_arm()

        target_config = np.array([-1.4,0,0,0.7,0,0,0])

        path = [np.zeros(7),target_config]

        arm_traj = mehm.traj_from_path(path)

        mehm.execute_trajectory(arm_traj)

    elif test_case == 'move_hand_straight':
        # mehm.move_hand_straight(manip_name='right_arm',
        #                         moving_direction=[0,1,0],
        #                         moving_distance=0.5,
        #                         step_size=0.005,
        #                         execute=True,
        #                         blocking=True)

        mehm.set_manipulator("left_arm")

        mehm.guarded_move_hand_straight(manip_name='left_arm',
                                        moving_direction=[0,1,0],
                                        moving_distance=0.1,
                                        force_trigger=15.0,
                                        use_whole_body=False)

        mehm.guarded_move_hand_straight(manip_name='left_arm',
                                        moving_direction=[0,1,0],
                                        moving_distance=0.1,
                                        force_trigger=15.0,
                                        use_whole_body=True)

    elif test_case == 'push_with_force':

        mehm.enable_whole_body()

        mehm.guarded_move_hand_straight(manip_name='right_arm',
                                moving_direction=[0,1,0],
                                moving_distance=0.2,
                                force_trigger=15.0,
                                use_whole_body=True)

        IPython.embed()

        mehm.push_with_force(manip_name='right_arm',
                             direction=[0,1,0],
                             desired_force=50.0,
                             planning_distance=0.1,
                             use_whole_body=True)

    IPython.embed()

    mehm.move_to_home()

    time.sleep(1)

    print("plan complete")

    # IPython.embed()


if __name__ == "__main__":
    run_mehm()
