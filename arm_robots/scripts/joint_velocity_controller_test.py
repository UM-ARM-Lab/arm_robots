#!/usr/bin/env python
import rospy
import math
from sensor_msgs.msg import JointState
from val_basic_motion import *
import numpy as np
from arm_robots.hdt_michigan import Val
from arc_utilities import ros_init
from apriltag_ros.msg import *
from tf.transformations import quaternion_matrix

class TagDetectionListener:
    def __init__(self, topic, vs):
        self.topic = topic
        self.tag_sub = rospy.Subscriber(topic, AprilTagDetectionArray, self.callback)
        self.vs = vs

    def callback(self, data):
        self.vs.tag_detections = data
        #print(data)
        '''
    for detection in data.detections:
        if detection.id[0] == 1:  # id 1 is the tag on the robot
            flag = True
            tca_position[0] += detection.pose.pose.pose.position.x
            tca_position[1] += detection.pose.pose.pose.position.y
            tca_position[2] += detection.pose.pose.pose.position.z
            tca_quaternion[0] += detection.pose.pose.pose.orientation.x
            tca_quaternion[1] += detection.pose.pose.pose.orientation.y
            tca_quaternion[2] += detection.pose.pose.pose.orientation.z
            tca_quaternion[3] += detection.pose.pose.pose.orientation.w
    rospy.sleep(0.1)
    '''
        # for detection in data.detections:
        # print(detection.id[0])
        # print(detection.pose.pose.pose.position)


class val_vs():
    def __init__(self):
        # publisher to joint velocity command topic
        self.pub = rospy.Publisher("hdt_adroit_coms/joint_cmd", JointState, queue_size=10)
        self.val = Val(raise_on_failure=True)
        self.val.connect()

        # initialize Apriltag detections
        self.tag_detections = np.array([])
        tag_detection_topic = "/tag_detections"
        self.tag_detection_listener = TagDetectionListener(tag_detection_topic, self)


        # set joint names
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

        '''
    [0.25829190015792847, 0.05810129642486572, -0.5179260969161987, 0.29261577129364014, 1.4885820150375366, 0.9461115002632141, -3.719825267791748, 
    -0.06845598667860031, -0.06845598667860031, 0.5206106305122375, -0.030105292797088623, 0.429719477891922, -0.08379626274108887, -0.12751606106758118, 0.6141862869262695, -1.5073739290237427, 0.17161935567855835, 0.17161935567855835]
    
        '''
        self.home = {
            'joint56': -1.55,
            'joint57': 0.077,
            'joint41':  0.25829190015792847,
            'joint42':0.05810129642486572,
            'joint43':-0.5179260969161987,
            'joint44': 0.29261577129364014,
            'joint45': 1.4885820150375366,
            'joint46': 0.9461115002632141,
            'joint47': -3.719825267791748,
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

    def cartesian_vel_control(self, group_name,
                                    twist,
                                    time,
                                    r=100,
                                    v_threshold=0.15,
                                    omega_threshold=0.6,
                                    joint_vel_threshold = 1.5,
                                    cond_threshold = 1000):
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

        # cartesian velocity safety check
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

            # joint velocity safety check
            if max(joint_vels) > joint_vel_threshold:
                print("Highest joint velocity is {:.2f}, greater than {:.2f}, stopping!".format(max(joint_vels), joint_vel_threshold))
                break

            # condition number of JJ', used for safety check
            cond = np.linalg.cond(J.dot(J.T))
            cond2 = np.linalg.cond(J[:3, :].dot(J[:3,:].T))
            # print(J.dot(J.T))
            print("Conditional number of JJ' {:.2f}".format(cond))
            print("Conditional number of JJ' cart {:.2f}".format(cond2))
            if cond > cond_threshold:
                print("Large conditional number! {:.2f}".format(cond))
                break

            joint_cmd.position = list(((np.array(joint_vels) > 0) * 2 - 1) * 2 * math.pi)
            joint_cmd.velocity = list(abs(np.array(joint_vels)))
            self.pub.publish(joint_cmd)
            rate.sleep()

    def single_tag_calib(self, z_bias=-0.02):
        # take average of n tca position and orientation (poses of robot apriltag in camera frame)
        n = 5  # number of samples
        # pose of apriltag in camera frame
        tca_position = np.zeros((3))
        tca_quaternion = np.zeros((4))  # (x, y, z, w)
        for i in range(n):
            data = self.tag_detections
            flag = False
            for detection in data.detections:
                if detection.id[0] == 1:  # id 1 is the tag on the robot
                    flag = True
                    tca_position[0] += detection.pose.pose.pose.position.x
                    tca_position[1] += detection.pose.pose.pose.position.y
                    tca_position[2] += detection.pose.pose.pose.position.z
                    tca_quaternion[0] += detection.pose.pose.pose.orientation.x
                    tca_quaternion[1] += detection.pose.pose.pose.orientation.y
                    tca_quaternion[2] += detection.pose.pose.pose.orientation.z
                    tca_quaternion[3] += detection.pose.pose.pose.orientation.w
            rospy.sleep(0.1)
            if flag == False:
                print("Tag on the robot not detected, tag blocked by obstacles.")
        tca_position /= n
        tca_quaternion /= n
        # convert pose to 4x4 homogeneous form
        tca = quaternion_matrix(tca_quaternion)
        tca[:3, 3] = tca_position.T
        # tra, pose of apriltag in robot frame, calculated from drawing
        tra = np.array([[0, 1, 0, -0.1416],
                        [-1, 0, 0, 0],
                        [0, 0, 1, 0.037],
                        [0, 0, 0, 1]])
        tcr = tca.dot(np.linalg.inv(tra))
        tcr[2, 3] += z_bias
        self.extrinsic = tcr
        self.extrinsicCalibrated = True
        print("Entrinsic Parameter:\n", tcr)


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
    t = 1.0
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
    t2 = 1.5
    w = 0.25
    vs.cartesian_vel_control('left_arm', [0, 0, 0, w, 0, 0], t2)
    rospy.sleep(1)
    vs.cartesian_vel_control('left_arm', [0, 0, 0, -w, 0, 0], t2)
    rospy.sleep(1)
    vs.cartesian_vel_control('left_arm', [0, 0, 0, 0, w, 0], t2)
    rospy.sleep(1)
    vs.cartesian_vel_control('left_arm', [0, 0, 0, 0, -w, 0], t2)
    rospy.sleep(1)
    vs.cartesian_vel_control('left_arm', [0, 0, 0, 0, 0, w], t2)
    rospy.sleep(1)
    vs.cartesian_vel_control('left_arm', [0, 0, 0, 0, 0, -w], t2)
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
