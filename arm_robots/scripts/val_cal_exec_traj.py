#! /usr/bin/env python
import colorama
import numpy as np
import copy
from geometry_msgs.msg import Pose
from tf.transformations import quaternion_from_euler
import tf
import rospy

from arc_utilities import ros_init
from arm_robots.hdt_michigan import Val         
from val_calib import quat2matrix

from geometry_msgs.msg import Pose
import tf_conversions.posemath as pm
import PyKDL
import math, time

#visualization
from rviz_traj_plot import rviz_traj_plot
ask_before_moving = True

def myinput(msg):
    global ask_before_moving
    if ask_before_moving:
        input(msg)
        
home_yuchi = {} # currently not working
home_yuchi['joint41']=0.9472621083259583
home_yuchi['joint42']=-0.3311582207679748
home_yuchi['joint43']=-0.42550089955329895
home_yuchi['joint44']=-0.4929981231689453
home_yuchi['joint45']=-1.592512488365173
home_yuchi['joint46']=-0.00191753450781106
home_yuchi['joint47']=1.521371960639953
home_yuchi['joint1']= 6.28-5.62393712997436
home_yuchi['joint2']=0.1346109211444854
home_yuchi['joint3']=-0.0109299467876553
home_yuchi['joint4']=-0.16183991730213165
home_yuchi['joint5']=-1.6364240646362305
home_yuchi['joint6']=-0.05062291398644447
home_yuchi['joint7']=0.04908888414502144

def myinput(msg):
    global ask_before_moving
    if ask_before_moving:
        input(msg)

def comp_waypt(initial_left_pos, initial_right_pos):
    # The slope is 34.22 deg (0.5973 rad). To do a simle test, set the angle to 0 first.
    # We view the slope surface as the "new" flat ground. The wall is treated like a leaning wall relative to the slope surface. x points forward, y points leftward, and z points upward.
    # Val's base moves in +x direction, and the upper body faces -y and +z direction
    slope_angle = 0.0
    #slope_angle = 0.59728
    x_vec = np.array([1.0, 0.0, 0.0])
    y_vec = np.array([0.0, math.cos(slope_angle), -math.sin(slope_angle)])
    z_vec = np.array([0.0, math.sin(slope_angle), math.cos(slope_angle)])

    # test waypoint
    # whole_body_path = [('left_arm', initial_left_pos - 0.1 * z_vec),
    #                    ('right_arm', initial_right_pos - 0.3 * z_vec)]

    # original waypoint
    whole_body_path = [('left_arm', initial_left_pos - 0.1 * z_vec),
                        ('right_arm', initial_right_pos - 0.3 * z_vec),
                        ('left_arm', initial_left_pos + 0.3 * x_vec + 0.023 * y_vec - 0.3 * z_vec),
                        ('right_arm', initial_right_pos + 0.3 * x_vec + 0.023 * y_vec - 0.4 * z_vec),
                        ('left_arm', initial_left_pos + 0.6 * x_vec + 0.023 * y_vec - 0.3 * z_vec),
                        ('right_arm', initial_right_pos + 0.5 * x_vec + 0.023 * y_vec - 0.4 * z_vec),
                        ('left_arm', initial_left_pos + 0.8 * x_vec + 0.041 * y_vec - 0.4 * z_vec),
                        ('right_arm', initial_right_pos + 0.8 * x_vec - 0.012 * y_vec - 0.2 * z_vec),
                        ('left_arm', initial_left_pos + 0.9 * x_vec - 0.012 * y_vec - 0.1 * z_vec),
                        ('right_arm', initial_right_pos + 0.9 * x_vec - 0.03 * y_vec - 0.1 * z_vec),
    ]
    return whole_body_path

def main():
    #initialize val
    np.set_printoptions(suppress=True, precision=0, linewidth=200)
    colorama.init(autoreset=True)
    ros_init.rospy_and_cpp_init("basic_motion")
    val = Val(raise_on_failure=True)
    val.connect()
    print("press enter if prompted")

    '''
    #Plan to home position, currently fail to plan
    myinput("Plan to home joint config?")
    val.plan_to_joint_config('both_arms', home_yuchi)
    '''

    # Execute Yuchi's test path
    myinput("Start executing arm path?")

    # Get both hand starting position, orientation set to identity
    listener = tf.TransformListener()
    # wait for tf to set up 
    rospy.sleep(2)

    (p_rh,rot_rh) = listener.lookupTransform('val_cal', 'right_hand', rospy.Time(0))
    (p_lh,rot_lh) = listener.lookupTransform('val_cal', 'left_hand', rospy.Time(0))

    #list to numpy array
    right_hand_position = np.array(p_rh)
    left_hand_position = np.array(p_lh)

    right_hand_pose = quat2matrix(p_rh,rot_rh)
    left_hand_pose = quat2matrix(p_lh,rot_lh)
    left_hand_pose[0:3,3] = left_hand_position
    right_hand_pose[0:3,3] = right_hand_position

    print ('Left Hand pose: ', left_hand_pose)
    print ('Right Hand pose ', right_hand_pose)

    #compute whole body path
    whole_body_path = comp_waypt(left_hand_position, right_hand_position)
    whole_body_path_len = len(whole_body_path)
    waypoint_index = 0

    #compute waypoints of each part of whole body path and execute
    while waypoint_index < whole_body_path_len and not rospy.is_shutdown():

        prev_left_hand_position = left_hand_position
        prev_right_hand_position = right_hand_position

        arm_move_height = 0.18 # height of the parabolic path of end-effector from one contact position to another.

        if whole_body_path[waypoint_index][0] == 'left_arm':
            print ("Moving left arm from ", prev_left_hand_position, "to ", whole_body_path[waypoint_index][1])
            total_dist = np.linalg.norm(whole_body_path[waypoint_index][1] - prev_left_hand_position)
        elif whole_body_path[waypoint_index][0] == 'right_arm':
            print ("Moving right arm from ", prev_right_hand_position, "to ", whole_body_path[waypoint_index][1])
            total_dist = np.linalg.norm(whole_body_path[waypoint_index][1] - prev_right_hand_position)
        print("total dist: {}".format(total_dist))
        # Create the parabolic path for moving end-effector and use MoveIt to execute trajectory
        path_ls = [] #list of [x, y, z] waypoint
        path = [] #list of pose messages
        sample = 15
        #transform points from val_cal to val_root for executing path
        R_val_root_val_cal = np.array([  [0.0000000, -1.0000000,  0.0000000],
                                         [1.0000000,  0.0000000,  0.0000000],
                                         [0.0000000,  0.0000000,  1.0000000]])
        T_val_root_val_cal = np.block([ [R_val_root_val_cal, np.array([[0, 0, 0]]).T], 
                                        [0,        0,     0,      1]                 ])
        for i in range(1,sample + 1):
            travel_dist = (i) * total_dist / sample
            #print(travel_dist)

            if whole_body_path[waypoint_index][0] == 'left_arm':
                left_hand_position = prev_left_hand_position + i * (whole_body_path[waypoint_index][1] - prev_left_hand_position) / sample + \
                                        (-4*arm_move_height/(total_dist*total_dist)) * travel_dist * (travel_dist-total_dist) * np.array([0,1,0])
                left_hand_pose[0:3,3] = left_hand_position
                p = pm.fromMatrix(T_val_root_val_cal.dot(left_hand_pose)) #transform points from val_cal to val_root for executing path
                path.append(copy.deepcopy(pm.toMsg(p)))
                path_ls.append(R_val_root_val_cal.dot(left_hand_position.reshape(3,1)).reshape(-1).tolist())

            elif whole_body_path[waypoint_index][0] == 'right_arm':
                right_hand_position = prev_right_hand_position + i * (whole_body_path[waypoint_index][1] - prev_right_hand_position) / sample + \
                                        (-4*arm_move_height/(total_dist*total_dist)) * travel_dist * (travel_dist-total_dist) * np.array([0,1,0])
                right_hand_pose[0:3,3] = right_hand_position
                p = pm.fromMatrix(T_val_root_val_cal.dot(right_hand_pose))
                path.append(copy.deepcopy(pm.toMsg(p)))
                path_ls.append(R_val_root_val_cal.dot(right_hand_position.reshape(3,1)).reshape(-1).tolist())

        myinput("printing path on rviz?")
        rviz_traj_plot(path_ls, "val_root") 

        myinput("Plan path?")
        # execute the trajectory to move the arm
        if whole_body_path[waypoint_index][0] == 'left_arm':
            result = val.plan_cartesian_path(val.left_arm_group, 'left_hand', path, True)
            '''
            for pt in path:
                print('pt', pt)
                print("current left hand pose: ",val.get_link_pose(val.left_arm_group, 'left_hand'))
                val.plan_to_pose(val.left_arm_group, "left_hand", pt)
                '''        
            #for pt in path_ls:
            #    val.plan_to_position_cartesian(val.left_arm_group, 'left_hand', pt, step_size=0.01)

        elif whole_body_path[waypoint_index][0] == 'right_arm':
            result = val.plan_cartesian_path(val.right_arm_group, 'right_hand', path, True)
                
        waypoint_index += 1
        print ("Finish Moving Arm")
        time.sleep(0.5)

    val.disconnect()
    ros_init.shutdown()

if __name__ == "__main__":
    main()
