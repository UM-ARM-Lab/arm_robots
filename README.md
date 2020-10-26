See `scripts/basic_motion.py` for example usage. In order to test this, you'll need simulated or real victor

## Setup / Instal/

See the `arm_robots.rosinstall` for what pacakges you need to clean. You can also use the rosinstall directly with `wstool merge arm_robots/arm_robots.rosinstall`, followed by `wstool update`.

To get the dependencies, you can try `rosdep install -r --from-paths arm_robots --ignore-src` but that may still show errors.

## Simulation

    roslaunch arm_robots victor.launch  use_gazebo:=true


## Real Victor

    roslaunch arm_robots victor.launch


## [Here's what it should look like](https://drive.google.com/file/d/1-R0NOalh0yAsff310mjcobYLGQLQ7oJZ/view?usp=sharing)

![basic motion demo](https://raw.githubusercontent.com/UM-ARM-Lab/arm_robots/master/arm_robots/basic_motion_demo1.gif?token=AA6TGEVDB4GGCNMKH3TJE4S7T4XCK)
