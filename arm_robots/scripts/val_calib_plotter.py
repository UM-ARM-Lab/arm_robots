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

def plot_traj(traj_right_hand_robot, traj_right_hand_vicon, calibrated = False):

    fig = plt.figure()
    ax = plt.axes(projection='3d')

    #change unit from meter to millimeter
    traj_right_hand_robot = traj_right_hand_robot * 1000.0
    traj_right_hand_vicon = traj_right_hand_vicon * 1000.0
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
    if calibrated is False:
        plt.title('End Effector Trajectory measured from \n Forward Kinematics and Motion Capture System')
    else:
        plt.title('End Effector Trajectory measured from \n Forward Kinematics and Motion Capture System (Calibrated)')
    plt.xlabel('x(mm)')
    plt.ylabel('y(mm)')
    plt.legend(loc = 'upper right', bbox_to_anchor = (0.0, 0.8, 1., 0.1))
    plt.show()

def calc_calib_path(Bs ,X, Z):
    traj_calibrated = np.zeros((3,len(Bs)))
    for i, B in enumerate(Bs):
        T = Z.dot(B).dot(np.linalg.inv(X))
        traj_calibrated[:,i] = T[0:3,3]
        #print(traj_calibrated)
    return traj_calibrated

def calc_err(As, Bs ,X, Z):
    err = np.zeros(len(Bs))
    for i, (A, B) in enumerate(zip(As, Bs)):
        T = Z.dot(B).dot(np.linalg.inv(X))
        err[i] = np.linalg.norm((T-A)[0:3,3])
    print(np.mean(err))  
    return np.mean(err)

if __name__ == '__main__':
    npzfile = np.load('../data_test_12/data_cal.npz')
    print(npzfile.files)
    traj_right_hand_vicon = npzfile['traj_right_hand_vicon']
    traj_right_hand_robot = npzfile['traj_right_hand_robot']
    traj_left_hand_vicon = npzfile['traj_left_hand_vicon']
    traj_left_hand_robot = npzfile['traj_left_hand_robot']
    Bs_l = npzfile['Bs_l']
    Bs_r = npzfile['Bs_r']
    As_l = npzfile['As_l']
    As_r = npzfile['As_r']
    #print(len(npzfile['joint_states']))
    #print(len(npzfile['joint_cmds']))
    print(npzfile['coms'].shape)   
    #print(len(npzfile['time'])) 
    plot_traj(traj_right_hand_robot, traj_right_hand_vicon)
    plot_traj(traj_left_hand_robot, traj_left_hand_vicon)

    #calculate calibrated path
    Xl = np.array([[0.989503, -0.124218, 0.0738548, -0.0103583],
[0.120056, 0.99105, 0.0583567, -0.0322303],
[-0.0804428, -0.0488774, 0.99556, 0.00325458],
[0, 0, 0, 1]]
) 
    Zl = np.array([[0.996452, 0.0755451, 0.0370885, -0.0302337],
[-0.0761658, 0.996973, 0.0156167, 0.0146246],
[-0.0357965, -0.0183862, 0.99919, 0.0106773],
[0, 0, 0, 1]]
) 
    Xr = np.array([[0.995067, -0.0562877, -0.0816893, -0.0245784],
[0.0754606, 0.964005, 0.254951, -0.0505797],
[0.0643983, -0.259858, 0.963497, -0.0776457],
[0, 0, 0, 1]]
)
    Zr = np.array([[0.999628, 0.00566288, 0.0266616, -0.0437084],
[-0.00760258, 0.997287, 0.0732226, -0.0989311],
[-0.0261746, -0.0733981, 0.996959, 0.0355306],
[0, 0, 0, 1]]
)
    Xl = np.linalg.inv(Xl)
    Xr = np.linalg.inv(Xr)
    print(As_l[0])
    print(Bs_l[0])
    calc_err(As_l, Bs_l, Xl, Zl)
    calc_err(As_r, Bs_r, Xr, Zr)
    calc_err(As_l, Bs_l, np.eye(4), np.eye(4))
    calc_err(As_r, Bs_r, np.eye(4), np.eye(4))
    traj_left_hand_vicon_calib = calc_calib_path(Bs_l, Xl, Zl)
    traj_right_hand_vicon_calib = calc_calib_path(Bs_r, Xr, Zr)
    plot_traj(traj_right_hand_robot, traj_right_hand_vicon_calib, calibrated=True)
    plot_traj(traj_left_hand_robot, traj_left_hand_vicon_calib, calibrated=True)  







        



