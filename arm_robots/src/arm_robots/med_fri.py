#! /usr/bin/env python
import rospy
from drake import lcmt_iiwa_status
from victor_hardware_interface_msgs.msg import MotionStatus
from sensor_msgs.msg import JointState
import lcm

class MedFRILCM:
    def __init__(self):
        self.lcm = lcm.LCM()
        self.medusa_sub = self.lcm.subscribe('IIWA_STATUS_MEDUSA', lambda channel, data: self.medusa_msg_handler(channel, data))
        self.thanos_sub = self.lcm.subscribe('IIWA_STATUS_THANOS', lambda channel, data: self.thanos_msg_handler(channel, data))
        
        self.thanos_status_pub = rospy.Publisher('thanos_med/motion_status', MotionStatus, queue_size=1)
        self.thanos_joints_pub = rospy.Publisher('thanos_med/joint_states', JointState, queue_size=1)
        
        self.medusa_status_pub = rospy.Publisher('medusa_med/motion_status', MotionStatus, queue_size=1)        
        self.medusa_joints_pub = rospy.Publisher('medusa_med/joint_states', JointState, queue_size=1)
        
    def thanos_msg_handler(self, channel, data):
        thanos_fri_msg = lcmt_iiwa_status.decode(data)
        
        thanos_status = MotionStatus()
        thanos_status.header.stamp = rospy.Time.now()
        thanos_status.commanded_joint_position  = thanos_fri_msg.joint_position_commanded
        
        thanos_status.measured_joint_position   = thanos_fri_msg.joint_position_measured
        thanos_status.measured_joint_velocity   = thanos_fri_msg.joint_velocity_estimated
        thanos_status.measured_joint_torque     = thanos_fri_msg.joint_torque_measured
        thanos_status.estimated_external_torque = thanos_fri_msg.joint_torque_external
        self.thanos_status_pub.publish(self.thanos_status)
        
        thanos_joint_state = JointState()
        thanos_joint_state.header.stamp = rospy.Time.now()
        thanos_joint_state.name = ['iiwa_joint_1', 'iiwa_joint_2', 'iiwa_joint_3', 'iiwa_joint_4', 'iiwa_joint_5', 'iiwa_joint_6', 'iiwa_joint_7']
        thanos_joint_state.position = thanos_fri_msg.joint_position_measured
        thanos_joint_state.velocity = thanos_fri_msg.joint_velocity_estimated
        thanos_joint_state.effort = thanos_fri_msg.joint_torque_measured
        self.thanos_joints_pub.publish(thanos_joint_state)
        
    def medusa_msg_handler(self, channel, data):
        medusa_fri_msg = lcmt_iiwa_status.decode(data)
        
        medusa_status = MotionStatus()
        medusa_status.header.stamp = rospy.Time.now()
        medusa_status.commanded_joint_position  = medusa_fri_msg.joint_position_commanded
        
        medusa_status.measured_joint_position   = medusa_fri_msg.joint_position_measured
        medusa_status.measured_joint_velocity   = medusa_fri_msg.joint_velocity_estimated
        medusa_status.measured_joint_torque     = medusa_fri_msg.joint_torque_measured
        medusa_status.estimated_external_torque = medusa_fri_msg.joint_torque_external
        self.medusa_status_pub.publish(medusa_status)
        
        medusa_joint_state = JointState()
        medusa_joint_state.header.stamp = rospy.Time.now()
        medusa_joint_state.name = ['iiwa_joint_1', 'iiwa_joint_2', 'iiwa_joint_3', 'iiwa_joint_4', 'iiwa_joint_5', 'iiwa_joint_6', 'iiwa_joint_7']
        medusa_joint_state.position = medusa_fri_msg.joint_position_measured
        medusa_joint_state.velocity = medusa_fri_msg.joint_velocity_estimated
        medusa_joint_state.effort = medusa_fri_msg.joint_torque_measured
        self.medusa_joints_pub.publish(medusa_joint_state)
        
    def handle(self):
        self.lcm.handle()
if __name__ == '__main__':
    rospy.init_node('med_fri')
    
    fri_lcm = MedFRILCM()
    
    while not rospy.is_shutdown():
        fri_lcm.handle()