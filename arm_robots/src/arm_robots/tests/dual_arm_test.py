#! /usr/bin/env python
import argparse
import time
from math import pi

import rospy
from arm_robots.victor import Victor
from std_msgs.msg import String
from tf.transformations import compose_matrix
from victor_hardware_interface import victor_utils as vu
from victor_hardware_interface_msgs.msg import ControlMode

talkative = True

left1 = [1.596, 0.525, -0.756, -1.161, -0.321, 1.097, 0.236]
left2 = [1.109, 0.679, -1.656, -1.143, -0.763, 1.369, 0.398]

right1 = [1.363, -1.179, -1.003, -1.199, -0.929, 1.401, -1.064]
right2 = [1.857, -1.005, -0.524, -1.339, 0.061, 1.183, -1.15]

left_hand_start = compose_matrix(angles=[-pi / 2, -pi / 4, -pi],
                                 translate=[.63, .33, .72])

right_hand_start = compose_matrix(angles=[pi / 2, pi / 4, pi],
                                  translate=[.63, -.33, .72])

speech_pub = None


def speak(msg):
    global speech_pub
    global talkative
    if talkative:
        speech_pub.publish(msg)


def dual_arm_motion(mev):
    speak("Both arms should move")
    mev.plan_to_configuration_both_arms(left_config=left2, right_config=right2, execute=True)
    mev.plan_to_configuration_both_arms(left_config=left1, right_config=right1, execute=True)

    speak("Just left")
    mev.set_manipulator("left_arm")
    mev.plan_to_configuration(left2, execute=True)

    speak("Just right")
    mev.set_manipulator("right_arm")
    mev.plan_to_configuration(right2, execute=True)

    speak("both")
    mev.plan_to_configuration_both_arms(left_config=left1, right_config=right1, execute=True)


def dual_arm_position_mode(mev):
    print("Verifying dual arm control position mode")
    speak("Joint position")
    vu.set_control_mode(ControlMode.JOINT_POSITION, "left_arm")
    vu.set_control_mode(ControlMode.JOINT_POSITION, "right_arm")
    dual_arm_motion(mev)


def dual_arm_impedance_mode(mev):
    print("Verifying dual arm control impedance stiff mode")
    speak("Stiff impedance")
    dual_arm_impedance_stiffness(mev, vu.Stiffness.STIFF)

    print("Verifying dual arm control impedance medium mode")
    speak("medium impedance")
    dual_arm_impedance_stiffness(mev, vu.Stiffness.MEDIUM)


def dual_arm_impedance_stiffness(mev, stiffness):
    vu.set_control_mode(ControlMode.JOINT_IMPEDANCE, "left_arm", stiffness)
    vu.set_control_mode(ControlMode.JOINT_IMPEDANCE, "right_arm", stiffness)
    dual_arm_motion(mev)


def setup(mev):
    vu.set_control_mode(ControlMode.JOINT_POSITION, "left_arm")
    vu.set_control_mode(ControlMode.JOINT_POSITION, "right_arm")
    mev.plan_to_configuration_both_arms(left_config=left1, right_config=right1, execute=True)


def run_dual_arm_tests():
    global speech_pub
    rospy.init_node("motion_enabled_victor")
    speech_pub = rospy.Publisher("/polly", String, queue_size=1)
    mev = Victor()

    # Setup
    print("Moving to setup position")
    setup(mev)

    # tests
    dual_arm_position_mode(mev)
    dual_arm_impedance_mode(mev)

    speak("finished")

    print("plan complete")
    time.sleep(.5)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--no-talk', action='store_true')

    args = parser.parse_args()
    if args.no_talk:
        print("staying slient")
        talkative = False
    run_dual_arm_tests()
