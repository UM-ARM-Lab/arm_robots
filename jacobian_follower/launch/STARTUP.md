# Repos

## 18.04: ~/catkin_ws

```
  Localname                S SCM Version (Spec)         UID  (Spec)  URI  (Spec) [http(s)://...]
 ---------                - --- --------------         -----------  ---------------------------
 sdf_tools                  git melodic  (-)           4b7a0ab808d0 git@github.com:UM-ARM-Lab/sdf_tools.git
 link_bot                   git val_rework  (-)        1ac208037d68 git@github.com:UM-ARM-Lab/link_bot.git
 lightweight_vicon_bridge   git master  (-)            6ed19db45b9b git@github.com:UM-ARM-Lab/lightweight_vicon_bridge.git
 kuka_iiwa_interface        git peter_3d_rope  (-)     cf293a99eb9b git@github.com:UM-ARM-Lab/kuka_iiwa_interface
 hdt_robot                  git peter_3d_rope  (-)     e8c5f3199bd6 git@github.com:UM-ARM-Lab/hdt_robot.git
 cdcpd_ros                  git PeterRopeTracking  (-) ce51fba24773 git@github.com:UM-ARM-Lab/cdcpd_ros.git
 cdcpd                      git PeterRopeTracking  (-) 34a143e8c10f git@github.com:UM-ARM-Lab/cdcpd.git
 arc_utilities              git no_tests  (-)          5292c2750f24 git@github.com:UM-ARM-Lab/arc_utilities.git
```

## 16.04 virtual: ~/catkin_ws
```
 or_urdf                     git master  (-)            2ec13fcd536e git@github.com:UM-ARM-Lab/or_urdf.git
 or_ros_plugin_initializer   git master  (-)            eaa0f06cb532 git@github.com:UM-ARM-Lab/or_ros_plugin_initializer.git
 or_plugin                   git master  (-)            5b5739f5d1e6 git@github.com:personalrobotics/or_plugin.git
 openrave_catkin             git master  (-)            601c5bcb84c0 git@github.com:UM-ARM-Lab/openrave_catkin.git
 kuka_iiwa_interface         git master  (-)            e35cf3a9b9c4 git@github.com:UM-ARM-Lab/kuka_iiwa_interface.git
 hdt_robot                   git master  (-)            f5527feead4e git@github.com:UM-ARM-Lab/hdt_robot.git
 comps                       git master  (-)            205f7eccd597 git@github.com:UM-ARM-Lab/comps.git
 arm_or_robots               git ValTrajForwarding  (-) cd655095b97d git@github.com:UM-ARM-Lab/arm_or_robots.git
 arc_utilities               git master  (-)            63dcf2e063f0 git@github.com:UM-ARM-Lab/arc_utilities.git
```

# Nodes/etc open for Gazebo Val -- untested
```
roscore
roslaunch link_bot_gazebo val.launch pause:=true world_name:=val_table_rope --screen
rviz
roslaunch physical_robot_3d_rope_shim robot_shim.launch use_val:=true --screen
rosrun physical_robot_3d_rope_shim test_move.py
```

# Nodes/etc open for Fake Val
```
roscore
rcnova && roslaunch hdt_michigan_control joint_control_fake_robot.launch --screen
roslaunch physical_robot_3d_rope_shim vicon.launch live:=false use_val:=true
rcnova && rosrun arm_or_robots ros_trajectory_forwarder.py _world_frame:="robot_root" _robot:="val"
rviz
roslaunch physical_robot_3d_rope_shim robot_shim.launch use_val:=true --screen
rosrun physical_robot_3d_rope_shim test_move.py
```

# Nodes/etc open for Real Val -- untested
```
roscore
rcnova && roslaunch hdt_michigan_control joint_control_filter_robot.launch --screen
roslaunch physical_robot_3d_rope_shim vicon.launch live:=true use_val:=true
rcnova && rosrun arm_or_robots ros_trajectory_forwarder.py _world_frame:="robot_root" _robot:="val"
rviz
roslaunch physical_robot_3d_rope_shim robot_shim.launch use_val:=true --screen
rosrun physical_robot_3d_rope_shim test_move.py
```