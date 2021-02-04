#!/usr/bin/env python
import rospy
import math
from sensor_msgs.msg import JointState

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

def joint_vel_ctl(joint_name, joint_vel, time = 1.0):
    print("{} velocity = {}, time = {}".format(joint_name, joint_vel, time))
    rospy.init_node("send_joint_command")
    pub = rospy.Publisher("hdt_adroit_coms/joint_cmd", JointState, queue_size=10)

    r = 100 # control bandwidth
    rate = rospy.Rate(r)
    joint_state = rospy.wait_for_message("/hdt_michigan/joint_states", JointState, timeout=None)
    #find index of the joint_name
    id = -1
    for id_, name_i in enumerate(joint_state.name):
        if(name_i == joint_name):
            id = id_
            break
    print("id is {}".format(id))
    if id == -1:
        print("Error, joint name not found")
        return
    # joint velocity direction 
    dir = 2 * math.pi
    if(joint_vel < 0):
        dir = -2 * math.pi
    # joint cmd generated from current joint state
    joint_cmd = JointState()
    joint_cmd.position = list(joint_state.position)
    joint_cmd.position[id] = dir
    joint_cmd.velocity = list(joint_state.velocity)
    joint_cmd.velocity[id] = joint_vel
    joint_cmd.name = joint_state.name
    joint_cmd.effort = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]


    print(int(r * time))
    for i in range(int(r * time)):
        #print(type(msg))
        #msg.header.stamp= rospy.Time.now()
        #sg.position[-10] = math.pi*2
        #msg.velocity[-10] = 0.1
        '''
        '''
        pub.publish(joint_cmd)
        rate.sleep()

if __name__ == '__main__':
    joint_vel_ctl('joint7', 0.1, 1.0)



