#! /usr/bin/env python
import rospy
import rospkg
from manipulation.station import MakeHardwareStation, MakeHardwareStationInterface, load_scenario
import os
from pydrake.all import (
    LcmSubscriberSystem,
    IiwaDriver,
    ApplyLcmBusConfig,
    DiagramBuilder,
    DrakeLcmInterface,
    Subscriber,
    DrakeLcm
)
from drake import lcmt_iiwa_status
'''
use drake to get lcm
'''
DUAL_SCENARIO_STRING = '''
Demo:
  directives:
      # Add iiwa_medusa
      - add_model:
          name: iiwa_medusa
          file: package://med_description/urdf/med_fri.urdf
          default_joint_positions:
              iiwa_joint_1: [0]
              iiwa_joint_2: [0]
              iiwa_joint_3: [0]
              iiwa_joint_4: [0]
              iiwa_joint_5: [0]
              iiwa_joint_6: [0]
              iiwa_joint_7: [0]
      # Add iiwa_thanos
      - add_model:
          name: iiwa_thanos
          file: package://med_description/urdf/med_fri.urdf
          default_joint_positions:
              iiwa_joint_1: [0]
              iiwa_joint_2: [0]
              iiwa_joint_3: [0]
              iiwa_joint_4: [0]
              iiwa_joint_5: [0]
              iiwa_joint_6: [0]
              iiwa_joint_7: [0]
      - add_frame:
          name: iiwa_medusa_origin
          X_PF:
              base_frame: world
              rotation: !Rpy { deg: [0.0, 0.0, 0.0]}
              translation: [0, 1.2192, 0.0]
      - add_weld:
          parent: iiwa_medusa_origin
          child: iiwa_medusa::base
      - add_weld:
          parent: world
          child: iiwa_thanos::base

  lcm_buses:
    medusa_lcm:
      channel_suffix: _MEDUSA
    thanos_lcm:
      channel_suffix: _THANOS
  model_drivers:
    iiwa_medusa: !IiwaDriver
      control_mode: position_only
      lcm_bus: medusa_lcm
    iiwa_thanos: !IiwaDriver
      control_mode: position_only
      lcm_bus: thanos_lcm
'''

import lcm

if __name__ == '__main__':
    rospy.init_node('med_fri')
    rospack = rospkg.RosPack()
    scenario = load_scenario(data=DUAL_SCENARIO_STRING, scenario_name='Demo')
    
    # builder = DiagramBuilder()
    # # real_station = MakeHardwareStationInterface(scenario, package_xmls=[os.path.join(rospack.get_path('med_description'), 'package.xml')])
    
    def msg_handler(channel, data):
        msg = lcmt_iiwa_status.decode(data)
        print(msg.joint_position_commanded)
    
    lcm = lcm.LCM()
    sub = lcm.subscribe('IIWA_STATUS_MEDUSA', lambda channel, data: msg_handler(channel, data))
    
    while not rospy.is_shutdown():
        lcm.handle()
    pass