#!/usr/bin/env python
import rospy
import math
import tf
import numpy as np
from scipy.spatial.transform import Rotation as R
from arc_utilities import ros_init

from mpl_toolkits import mplot3d
import matplotlib.pyplot as plt

#REQUIRES: list trans, and rot_q (quaternion of rotation)
#EFFECTS: returns 4 by 4 homogeneous transform of type numpy array 
def quat2matrix(trans, rot_q):
    return np.block([[np.asarray(R.from_quat(rot_q).as_matrix()), np.asarray(trans).reshape(3,1)],[0, 0, 0, 1]])

if __name__ == '__main__':
    #Initialize parameter
    n = 100 # total number of data collected
    r = 10.0 # data collection rate 
    traj_v = np.zeros((3,n)) #x, y, z trajectory from vicon
    traj_r = np.zeros((3,n)) #x, y, z trajectory from robot fk
    #initialize listener
    rospy.init_node("val_calib")
    listener = tf.TransformListener()
    rate = rospy.Rate(r)
    print("tf listener set up")
    i = 0
    while not rospy.is_shutdown():
        try:
            print("v")
            # val_root pose in vicon world frame 
            (trans,rot_q) = listener.lookupTransform('/world_origin', '/mocap_val_root_val_root', rospy.Time(0))
            Twv = quat2matrix(trans, rot_q)
            # end-effector pose in vicon world frame 
            (trans,rot_q) = listener.lookupTransform('/world_origin', '/mocap_RightHand0_RightHand0', rospy.Time(0))
            Twe = quat2matrix(trans, rot_q)
            # from vicon: end-effector pose in val_root frame 
            Tve_v = np.dot(np.linalg.inv(Twv), Twe)
            #from fk: end-effector pose in vicon world frame 
            (trans,rot_q) = listener.lookupTransform('/val_root', '/end_effector_right', rospy.Time(0))
            Tve_r = quat2matrix(trans, rot_q)
            
            traj_v[:, i] = Tve_v[0:3, 3]
            traj_r[:, i] = Tve_r[0:3, 3]
            i += 1
            print(i)
            if i == n:
                print("data collection complete")
                break
            '''
            print("Twv", Twv)
            print("Twe", Twe)
            print("Tve", Tve)
            '''
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print("tf listener not working")
            continue
        rate.sleep()

    fig = plt.figure()
    ax = plt.axes(projection='3d')

    # Data for a three-dimensional line
    xline = traj_v[0,:]
    yline = traj_v[1,:]
    zline = traj_v[2,:]
    #ax.plot3D(xline, yline, zline, 'gray')
    ax.scatter3D(xline, yline, zline, 'blue')

    xline = traj_r[0,:]
    yline = traj_r[1,:]
    zline = traj_r[2,:]
    ax.scatter3D(xline, yline, zline, 'red')
    # Data for three-dimensional scattered points
    '''
    zdata = 15 * np.random.random(100)
    xdata = np.sin(zdata) + 0.1 * np.random.randn(100)
    ydata = np.cos(zdata) + 0.1 * np.random.randn(100)
    ax.scatter3D(xdata, ydata, zdata, c=zdata, cmap='Greens')
    '''
    plt.show()






        



