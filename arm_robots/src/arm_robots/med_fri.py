#! /usr/bin/env python
import rospy
from drake import lcmt_iiwa_status
from victor_hardware_interface_msgs.msg import MotionStatus
from sensor_msgs.msg import JointState
import lcm

class DualMedFRILCM:
    def __init__(self):
        self.lcm = lcm.LCM()
        self.medusa_sub = self.lcm.subscribe('IIWA_STATUS_MEDUSA', lambda channel, data: self.medusa_msg_handler(channel, data))
        self.thanos_sub = self.lcm.subscribe('IIWA_STATUS_THANOS', lambda channel, data: self.thanos_msg_handler(channel, data))
        
        self.thanos_status_pub = rospy.Publisher('/med/motion_status', MotionStatus, queue_size=1)
        self.thanos_joints_pub = rospy.Publisher('/med/joint_states', JointState, queue_size=1)
        self.joint_states_pub = rospy.Publisher('joint_states', JointState, queue_size=1)
        
        self.medusa_status_pub = rospy.Publisher('/medusa_med/motion_status', MotionStatus, queue_size=1)        
        self.medusa_joints_pub = rospy.Publisher('/medusa_med/joint_states', JointState, queue_size=1)
        
        self.thanos_joint_state = JointState()
        self.medusa_joint_state = JointState()
        
    def thanos_msg_handler(self, channel, data):
        thanos_fri_msg = lcmt_iiwa_status.decode(data)
        
        thanos_status = MotionStatus()
        thanos_status.header.stamp = rospy.Time.now()
        thanos_status.commanded_joint_position  = thanos_fri_msg.joint_position_commanded
        thanos_status.measured_joint_position   = thanos_fri_msg.joint_position_measured
        thanos_status.measured_joint_velocity   = thanos_fri_msg.joint_velocity_estimated
        thanos_status.measured_joint_torque     = thanos_fri_msg.joint_torque_measured
        thanos_status.estimated_external_torque = thanos_fri_msg.joint_torque_external
        self.thanos_status_pub.publish(thanos_status)
        
        thanos_joint_state = JointState()
        thanos_joint_state.header.stamp = rospy.Time.now()
        thanos_joint_state.name = ['thanos_kuka_joint_1', 
                                   'thanos_kuka_joint_2', 
                                   'thanos_kuka_joint_3', 
                                   'thanos_kuka_joint_4', 
                                   'thanos_kuka_joint_5', 
                                   'thanos_kuka_joint_6', 
                                   'thanos_kuka_joint_7']
        thanos_joint_state.position = list(thanos_fri_msg.joint_position_measured)
        
        thanos_joint_state.velocity = list(thanos_fri_msg.joint_velocity_estimated)
        thanos_joint_state.effort = list(thanos_fri_msg.joint_torque_measured)
        self.thanos_joints_pub.publish(thanos_joint_state)
        
        self.thanos_joint_state = thanos_joint_state
        
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
        medusa_joint_state.name = ['medusa_kuka_joint_1',
                                    'medusa_kuka_joint_2',
                                    'medusa_kuka_joint_3',
                                    'medusa_kuka_joint_4',
                                    'medusa_kuka_joint_5',
                                    'medusa_kuka_joint_6',
                                    'medusa_kuka_joint_7']
        
        medusa_joint_state.position = list(medusa_fri_msg.joint_position_measured)
        medusa_joint_state.velocity = list(medusa_fri_msg.joint_velocity_estimated)
        medusa_joint_state.effort = list(medusa_fri_msg.joint_torque_measured)
        self.medusa_joints_pub.publish(medusa_joint_state)
        
        self.medusa_joint_state = medusa_joint_state
    def handle(self):
        self.lcm.handle()
    def publish(self):
        msg = JointState()
        msg.name = self.thanos_joint_state.name + self.medusa_joint_state.name
        msg.position = self.thanos_joint_state.position + self.medusa_joint_state.position
        msg.velocity = self.thanos_joint_state.velocity + self.medusa_joint_state.velocity
        msg.effort = self.thanos_joint_state.effort + self.medusa_joint_state.effort
        
        msg.header.stamp = rospy.Time.now()
        self.joint_states_pub.publish(msg)
        
if __name__ == '__main__':
    rospy.init_node('joint_state_publisher')
    
    fri_lcm = DualMedFRILCM()
    
    while not rospy.is_shutdown():
        fri_lcm.handle()
        fri_lcm.publish()