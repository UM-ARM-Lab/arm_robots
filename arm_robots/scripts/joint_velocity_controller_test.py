#!/usr/bin/env python
import rospy
import math
from sensor_msgs.msg import JointState
from val_basic_motion import *
import numpy as np
from arm_robots.hdt_michigan import Val
from arc_utilities import ros_init

class val_vs():
    def __init__(self):
        # publisher to joint velocity command topic
        self.pub = rospy.Publisher("hdt_adroit_coms/joint_cmd", JointState, queue_size=10)
        self.val = Val(raise_on_failure=True)
        self.val.connect()
        self.right_arm_joints = [
            'joint1',
            'joint2',
            'joint3',
            'joint4',
            'joint5',
            'joint6',
            'joint7',
        ]
        self.left_arm_joints = [
            'joint41',
            'joint42',
            'joint43',
            'joint44',
            'joint45',
            'joint46',
            'joint47',
        ]
        self.home = {
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

    def plan_home(self):
        self.val.plan_to_joint_config('both_arms', self.home)
        rospy.sleep(1)

    def shut_down(self):
        self.val.disconnect()

    def get_jacobian(self, group_name: str):
        """
        returns Jacobian
        requires:
        group_name: "left_arm", "right_arm"
        """
        move_group = self.val.get_move_group_commander(group_name)
        # print("pose ref frame before:", move_group.get_pose_reference_frame())
        # move_group.set_pose_reference_frame('drive_41')
        # print("pose ref frame after:", move_group.get_pose_reference_frame())
        return move_group.get_jacobian_matrix(move_group.get_current_joint_values())

    def get_cur_pos(self, group_name):
        """
        returns end-effector pose
        requires:
        group_name: "left_arm", "right_arm"
        """
        move_group = self.val.get_move_group_commander(group_name)
        # print("eef_link:", move_group.get_end_effector_link())
        return move_group.get_current_pose().pose

    # control bandwidth r
    def joint_vel_ctl(self, joint_names, joint_vels, time=1.0, r=100):
        """
        joint velocity controller
        requires:
        joint_names: list of joint names
        joint_vels: list of joint velocities
        time: execution time in sec
        r: control bandwidth (loop rate)
        """
        print("{} velocity = {}, time = {}".format(joint_names, joint_vels, time))
        rate = rospy.Rate(r)
        joint_state = rospy.wait_for_message("/hdt_michigan/joint_states", JointState, timeout=5)

        # find index of the joint_name
        joint_id = []
        for _, name_i in enumerate(joint_names):
            if name_i not in joint_state.name:
                print("Error, joint name not found")
                return
        joint_cmd = JointState()
        joint_cmd.name = joint_names
        # for positive joint velocities, set corresponding position to 2pi, otherwise, set to -2pi
        joint_cmd.position = list(((np.array(joint_vels) > 0) * 2 - 1) * 2 * math.pi)
        joint_cmd.velocity = list(abs(np.array(joint_vels)))

        for i in range(int(r * time)):
            self.pub.publish(joint_cmd)
            rate.sleep()

    def cartesian_vel_control(self, group_name, twist, time, r=100, v_threshold=0.15, omega_threshold=0.6):
        """
        Cartesian end-effector controller
        requires:
        group_name: 'right_arm' or 'left_arm'
        twist: [vx,vy,vz,wx,wy,wz], in global frame
        time: execution time in sec
        r: control bandwidth (loop rate)
        v_threshold: end-effector max linear velocity magnitude
        omega_threshold: end-effector max angular velocity magnitude
        """
        # set joint_names
        if group_name == 'right_arm':
            joint_names = self.right_arm_joints
        elif group_name == 'left_arm':
            joint_names = self.left_arm_joints

        rate = rospy.Rate(r)
        joint_cmd = JointState()
        joint_cmd.name = joint_names
        move_group = self.val.get_move_group_commander(group_name)

        # velocity security check
        if np.linalg.norm(np.array(twist[:3])) > v_threshold:
            print("linear velocity greater than threshold {} !".format(v_threshold))
            print("Current velocity: {}".format(np.linalg.norm(np.array(twist[:3]))))
            return

        if np.linalg.norm(np.array(twist[3:])) > omega_threshold:
            print("angular velocity greater than threshold {} !".format(omega_threshold))
            print("Current angular velocity: {}".format(np.linalg.norm(np.array(twist[3:]))))
            return

        # control loop
        for i in range(int(r * time)):
            # calculate joint_vels
            J = move_group.get_jacobian_matrix(move_group.get_current_joint_values())  # get current jacobian
            # calculate desired joint velocity (by multiplying jacobian pseudo-inverse), redundant -> min energy path
            joint_vels = list(np.linalg.pinv(np.array(J)).dot(np.array(twist)).reshape(-1))
            joint_cmd.position = list(((np.array(joint_vels) > 0) * 2 - 1) * 2 * math.pi)
            joint_cmd.velocity = list(abs(np.array(joint_vels)))
            self.pub.publish(joint_cmd)
            rate.sleep()

@ros_init.with_ros("send_joint_command")
def main():
    np.set_printoptions(suppress=True, precision=3, linewidth=200)
    colorama.init(autoreset=True)

    vs = val_vs()
    vs.plan_home()
    rospy.sleep(2)
    print(vs.get_jacobian('left_arm'))
    print(vs.get_cur_pos('left_arm'))
    print(vs.get_jacobian('right_arm'))
    print(vs.get_cur_pos('right_arm'))
    t = 1.5
    '''
    vs.cartesian_vel_control('left_arm', [-0.05, 0, 0, 0, 0, 0], t)
    rospy.sleep(1)
    vs.cartesian_vel_control('left_arm', [0.05, 0, 0, 0, 0, 0], t)
    rospy.sleep(1)
    vs.cartesian_vel_control('left_arm', [0, 0.05, 0, 0, 0, 0], t)
    rospy.sleep(1)
    vs.cartesian_vel_control('left_arm', [0, -0.05, 0, 0, 0, 0], t)
    rospy.sleep(1)
    vs.cartesian_vel_control('left_arm', [0, 0, 0.05, 0, 0, 0], t)
    rospy.sleep(1)
    vs.cartesian_vel_control('left_arm', [0, 0, -0.05, 0, 0, 0], t)
    rospy.sleep(2)
    '''

    w = 0.1
    vs.cartesian_vel_control('left_arm', [0, 0, 0, w, 0, 0], t)
    rospy.sleep(1)
    vs.cartesian_vel_control('left_arm', [0, 0, 0, -w, 0, 0], t)
    rospy.sleep(1)
    vs.cartesian_vel_control('left_arm', [0, 0, 0, 0, w, 0], t)
    rospy.sleep(1)
    vs.cartesian_vel_control('left_arm', [0, 0, 0, 0, -w, 0], t)
    rospy.sleep(1)
    vs.cartesian_vel_control('left_arm', [0, 0, 0, 0, 0, w], t)
    rospy.sleep(1)
    vs.cartesian_vel_control('left_arm', [0, 0, 0, 0, 0, -w], t)
    rospy.sleep(2)
    '''
    vs.joint_vel_ctl(['joint7', 'joint6'], [-0.3, 0.2], 3.0)
    rospy.sleep(2)
    vs.joint_vel_ctl(['joint7', 'joint6'], [0.3, -0.2], 3.0)
    rospy.sleep(2)
    '''
    vs.plan_home()
    vs.shut_down()
    ros_init.shutdown()


if __name__ == '__main__':
    main()
