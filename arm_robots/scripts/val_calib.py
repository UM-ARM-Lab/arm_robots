#!/usr/bin/env python
import sys
import rospy
import math
import tf
import numpy as np
from scipy.spatial.transform import Rotation as R
from arc_utilities import ros_init

from mpl_toolkits import mplot3d
import matplotlib.pyplot as plt

from tempfile import TemporaryFile
import time
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Bool 
# To stop recording data, Ctrl + C in terminal

#REQUIRES: list trans, and rot_q (quaternion of rotation)
#EFFECTS: returns 4 by 4 homogeneous transform of type numpy array 
def quat2matrix(trans, rot_q):
    return np.block([[np.asarray(R.from_quat(rot_q).as_matrix()), np.asarray(trans).reshape(3,1)],[0, 0, 0, 1]])

def write_data(path2file, As):
    f = open(path2file, "w")
    f.write(str(len(As))+"\n")
    for A in As:
        write_matrix2file(f, A)
    f.close()

def write_matrix2file(f, a):
    mat = np.matrix(a)
    for line in mat:
        np.savetxt(f, line, fmt='%.5f')
    f.write("\n")

def calc_pos(listener, hand, As, Bs):
    if hand == "right":
        mocap_hand_topic = '/mocap_RightHand0_RightHand0'
        robot_hand_topic = '/right_hand'
    elif hand == "left":
        mocap_hand_topic = '/mocap_LeftHand0_LeftHand0'
        robot_hand_topic = '/left_hand'    
    else:
        print("hand argument not recognized!")   
    # val_root pose in vicon world frame 
    (trans,rot_q) = listener.lookupTransform('/world_origin', '/mocap_val_root_val_root', rospy.Time(0))
    Twv = quat2matrix(trans, rot_q)
    # end-effector pose in vicon world frame 
    (trans,rot_q) = listener.lookupTransform('/world_origin', mocap_hand_topic, rospy.Time(0))
    Twe = quat2matrix(trans, rot_q)
    # from vicon: end-effector pose in val_root frame 
    Tve_v = np.dot(np.linalg.inv(Twv), Twe)
    #from fk: end-effector pose in vicon world frame 
    (trans,rot_q) = listener.lookupTransform('/val_cal', robot_hand_topic, rospy.Time(0))
    Tve_r = quat2matrix(trans, rot_q)
    As.append(Tve_r)
    Bs.append(Tve_v)
    return [Tve_v[0:3, 3], Tve_r[0:3, 3]]

def plot_traj(traj_right_hand_robot, traj_right_hand_vicon):

    fig = plt.figure()
    ax = plt.axes(projection='3d')

    #change unit from meter to millimeter
    traj_right_hand_robot *= 1000.0
    traj_right_hand_vicon *= 1000.0
    # Data for a three-dimensional line
    xline = traj_right_hand_vicon[0,:] 
    yline = traj_right_hand_vicon[1,:]
    zline = traj_right_hand_vicon[2,:]
    ax.scatter3D(xline, yline, zline, 'blue', label = 'Motion Capture')

    #align first point
    #traj_right_hand_robot -= (traj_right_hand_robot[:,0:1]-traj_right_hand_vicon[:,0:1])

    xline = traj_right_hand_robot[0,:] 
    yline = traj_right_hand_robot[1,:]
    zline = traj_right_hand_robot[2,:]
    ax.scatter3D(xline, yline, zline, 'red', label = 'Forward Kinematics')

    plt.title('End Effector Trajectory measured from \n Forward Kinematics and Motion Capture System')
    plt.xlabel('x(mm)')
    plt.ylabel('y(mm)')
    plt.legend(loc = 'upper right')
    plt.show()

if __name__ == '__main__':
    data_cal = TemporaryFile()
    #Initialize parameter
    r = 10.0 # data collection rate 
    joint_cmds= []
    joint_states = []
    time = []
    As_r = []
    Bs_r = []
    As_l = []
    Bs_l = []

    #initialize listener
    rospy.init_node("val_calib")
    listener = tf.TransformListener()
    rate = rospy.Rate(r)
    print("tf listener set up")
    i = 0
    print("Data Collection start, press Ctrl + C to end")
    while not rospy.is_shutdown():
        try:
            time.append(rospy.get_rostime())
            #record right hand position from fk and vicon
            result_r = calc_pos(listener, "right", As_r, Bs_r)
            #record left hand position from fk and vicon
            result_l = calc_pos(listener, "left", As_l, Bs_l)    
            #record com position
            com = rospy.wait_for_message("/com", PointStamped, timeout=5)
            #record joint state
            joint_state = rospy.wait_for_message("/hdt_michigan/joint_states", JointState, timeout=5)
            #record joint command
            joint_cmd = rospy.wait_for_message("/hdt_adroit_coms/joint_cmd", JointState, timeout=5)

            if i == 0:
                traj_right_hand_vicon = result_r[0].reshape((3,1))#x, y, z trajectory from vicon
                traj_right_hand_robot = result_r[1].reshape((3,1))#x, y, z trajectory from robot fk
                traj_left_hand_vicon = result_l[0].reshape((3,1))#x, y, z trajectory from vicon
                traj_left_hand_robot = result_l[1].reshape((3,1))#x, y, z trajectory from robot fk  
                coms = np.array([com.point.x, com.point.y, com.point.z]).reshape((3,1))
            else:   
                traj_right_hand_vicon = np.append(traj_right_hand_vicon, result_r[0].reshape((3,1)), axis = 1)#x, y, z trajectory from vicon
                traj_right_hand_robot = np.append(traj_right_hand_robot, result_r[1].reshape((3,1)), axis = 1)#x, y, z trajectory from robot fk
                traj_left_hand_vicon  = np.append(traj_left_hand_vicon,  result_l[0].reshape((3,1)), axis = 1) #x, y, z trajectory from vicon
                traj_left_hand_robot  = np.append(traj_left_hand_robot,  result_l[1].reshape((3,1)), axis = 1) #x, y, z trajectory from robot fk  
                coms = np.append(coms, np.array([com.point.x, com.point.y, com.point.z]).reshape((3,1)), axis = 1)

            joint_states.append(joint_state)
            joint_cmds.append(joint_cmd)

            i += 1
            print(i)

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print("tf listener waiting for message")
            continue

        except rospy.ROSInterruptException:
            break

        rate.sleep()

    print("data collection complete")
    print("As_l", len(As_l))
    print("As_r", len(As_r))
    print("Bs_l", len(Bs_l))
    print("Bs_r", len(Bs_r))

    #save A and B matrices to file 
    fra = write_data("r/As.txt", As_r)
    frb = write_data("r/Bs.txt", Bs_r)
    fla = write_data("l/As.txt", As_l)
    flb = write_data("l/Bs.txt", Bs_l)

    #save data
    np.savez('data_cal', traj_right_hand_vicon = traj_right_hand_vicon, traj_right_hand_robot = traj_right_hand_robot, 
                         traj_left_hand_vicon = traj_left_hand_vicon, traj_left_hand_robot = traj_left_hand_robot,
                         joint_states = joint_states, joint_cmds = joint_cmds, coms = coms, time = time,
                         As_l = As_l, As_r = As_r, Bs_l = Bs_l, Bs_r = Bs_r)
    npzfile = np.load('data_cal.npz')
    print(npzfile.files)
    print(npzfile['traj_right_hand_vicon'].shape)
    print(npzfile['traj_right_hand_robot'].shape)
    print(npzfile['traj_left_hand_robot'].shape)
    print(npzfile['traj_left_hand_robot'].shape)
    #print(len(npzfile['joint_states']))
    #print(len(npzfile['joint_cmds']))
    print(npzfile['coms'].shape)   
    #print(len(npzfile['time'])) 
    plot_traj(traj_right_hand_robot, traj_right_hand_vicon)
    plot_traj(traj_left_hand_robot, traj_left_hand_vicon)






        



