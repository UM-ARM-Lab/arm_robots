#!/usr/bin/env python
import rospy
import math
from sensor_msgs.msg import JointState
from val_basic_motion import *
import numpy as np
from arm_robots.hdt_michigan import Val
from arc_utilities import ros_init

config = {
    'joint56': -1.55,
    'joint57': 0.077,
    'joint41': 0.68,
    'joint42': -0.03,
    'joint43': -0.67,
    'joint44': -0.317735463380813,
    'joint45': -0.1804400086402893,
    'joint46': -0.5654809474945068,
    'joint47': 1.7859916687011719,
    'joint1': 0.5208024382591248,
    'joint2': -0.030105292797088623,
    'joint3': 0.42895248532295227,
    'joint4': -0.08494678139686584,
    'joint5': 6.152984619140625 - 6.28,
    'joint6': 0.6138027906417847,
    'joint7': -1.5069904327392578
}
'''
list of joints: 
name: 
- joint56
- joint57
- joint41
- joint42
- joint43
- joint44
- joint45
- joint46
- joint47
- leftgripper
- leftgripper2
- joint1
- joint2
- joint3
- joint4
- joint5
- joint6
- joint7
- rightgripper
- rightgripper2
'''


def joint_vel_ctl(pub, joint_name, joint_vel, time=1.0):
    print("{} velocity = {}, time = {}".format(joint_name, joint_vel, time))
    # rospy.init_node("send_joint_command")

    r = 100  # control bandwidth
    rate = rospy.Rate(r)
    joint_state = rospy.wait_for_message("/hdt_michigan/joint_states", JointState, timeout=5)
    # find index of the joint_name
    id = -1
    for id_, name_i in enumerate(joint_state.name):
        if (name_i == joint_name):
            id = id_
            break
    print("id is {}".format(id))
    if id == -1:
        print("Error, joint name not found")
        return
    # joint velocity direction 
    dir = 2 * math.pi
    if (joint_vel < 0):
        dir = -2 * math.pi
    # joint cmd generated from current joint state
    joint_cmd = JointState()
    joint_cmd.position = list(joint_state.position)
    joint_cmd.position[id] = dir
    joint_cmd.velocity = list(joint_state.velocity)
    joint_cmd.velocity[id] = abs(joint_vel)
    joint_cmd.name = joint_state.name
    joint_cmd.effort = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                        0.0]

    print(int(r * time))
    for i in range(int(r * time)):
        # print(type(msg))
        # msg.header.stamp= rospy.Time.now()
        # sg.position[-10] = math.pi*2
        # msg.velocity[-10] = 0.1
        '''
        '''
        pub.publish(joint_cmd)
        rate.sleep()


# control bandwidth r
def joint_vel_ctl2(publisher, joint_names, joint_vels, time=1.0, r=100):
    print("{} velocity = {}, time = {}".format(joint_names, joint_vels, time))
    rate = rospy.Rate(r)
    joint_state = rospy.wait_for_message("/hdt_michigan/joint_states", JointState, timeout=5)

    # find index of the joint_name
    joint_id = []
    for name_i in enumerate(joint_names):
        if name_i not in joint_state.name:
            print("Error, joint name not found")
            return
    joint_cmd = JointState()
    joint_cmd.name = joint_names
    # for positive joint velocities, set corresponding position to 2pi, otherwise, set to -2pi
    joint_cmd.position = list(((np.array(joint_vels) > 0) * 2 - 1) * 2 * math.pi)
    joint_cmd.velocity = list(abs(np.array(joint_vels)))

    for i in range(int(r * time)):
        publisher.publish(joint_cmd)
        rate.sleep()

@ros_init.with_ros("send_joint_command")
def main():
    pub = rospy.Publisher("hdt_adroit_coms/joint_cmd", JointState, queue_size=10)
    np.set_printoptions(suppress=True, precision=3, linewidth=200)
    colorama.init(autoreset=True)
    val = Val(raise_on_failure=True)
    val.connect()
    val.plan_to_joint_config('both_arms', config)
    rospy.sleep(1)
    # val = Val(raise_on_failure=True)
    # val.connect()

    ##val.open_left_gripper()
    ##val.close_left_gripper()
    # val.open_right_gripper()
    # val.close_right_gripper()

    # joint_vel_ctl(pub, 'joint7', 0.3, 2.0)
    # joint_vel_ctl(pub, 'joint7', -0.3, 2.0)
    '''
    joint_vel_ctl2(pub, ['joint7', 'joint6'], [-0.3, 0.2], 3.0)
    rospy.sleep(2)
    joint_vel_ctl2(pub, ['joint7', 'joint6'], [0.3, -0.2], 3.0)
    rospy.sleep(2)
    '''
    # val.plan_to_joint_config('both_arms', config)

    val.disconnect()
    ros_init.shutdown()


if __name__ == '__main__':
    main()