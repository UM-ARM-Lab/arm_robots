import rospy
import rospkg
import os
import numpy as np
from manipulation.station import MakeHardwareStationInterface, load_scenario
from arm_robots.robot_utils import PlanningResult
from pydrake.all import (
    Simulator,
    DiagramBuilder,
    TrajectorySource,
    DiagramBuilder,
    PiecewisePolynomial,
)
import numpy as np
import multiprocessing as mp

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

def get_hardware_diagram():
    scenario = load_scenario(data=DUAL_SCENARIO_STRING, scenario_name='Demo')
    real_station = MakeHardwareStationInterface(scenario, package_xmls=[os.path.join(rospkg.RosPack().get_path('med_description') + 'package.xml')])
    return real_station
    
def dualarm_follow_traj(planning_result: PlanningResult):
    root_builder = DiagramBuilder()
    hardware_diagram = get_hardware_diagram(root_builder)
    hardware_block = root_builder.AddNamedSystem("real_station", hardware_diagram)
    
    
    endtime = planning_result.planning_time
    traj_pts = planning_result.plan.joint_trajectory.points # list of JointTrajectoryPoint
    ts = []
    positions = []
    velocities = []
    for traj_pt in traj_pts:
        pos = traj_pt.positions
        vel = traj_pt.velocities
        t   = traj_pt.time_from_start.to_sec()
        positions.append(pos)
        velocities.append(vel)
        ts.append(t)
    ts = np.array(ts) # (n,)
    positions = np.array(positions) # (n, 14)
    velocities = np.array(velocities) # (n, 14)
    
    pos_thanos = positions[:,:7] # (n, 7)
    vel_thanos = velocities[:,:7] # (n, 7)
    
    pos_medusa = positions[:,7:] # (n, 7)
    vel_medusa = velocities[:,7:] # (n, 7)
    
    traj_thanos = PiecewisePolynomial.CubicHermite(ts, pos_thanos.T, vel_thanos.T)
    traj_medusa = PiecewisePolynomial.CubicHermite(ts, pos_medusa.T, vel_medusa.T)
    
    traj_medusa_block = root_builder.AddSystem(TrajectorySource(traj_medusa))
    traj_thanos_block = root_builder.AddSystem(TrajectorySource(traj_thanos))
    
    root_builder.Connect(traj_thanos_block.get_output_port(), hardware_block.GetInputPort("iiwa_thanos.position"))
    root_builder.Connect(traj_medusa_block.get_output_port(), hardware_block.GetInputPort("iiwa_medusa.position"))

    root_diagram = root_builder.Build()
    
    # run simulation
    simulator = Simulator(root_diagram)
    simulator.set_target_realtime_rate(1.0)
    simulator.AdvanceTo(endtime + 0.1)
    
def dualarm_follow_traj_mp(planning_result: PlanningResult):
    def fn(plan):
        dualarm_follow_traj(plan)
    proc = mp.Process(target=fn, args=(planning_result,))
    proc.start()
    proc.join()