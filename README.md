See `scripts/basic_motion.py` for example usage. In order to test this, you'll need simulated or real victor

## Setup / Install

See the `arm_robots.rosinstall` for what source pacakges you need to clone. You can also use the rosinstall directly. If you alredy have a `.rosinstall` file, use `wstool merge arm_robots/arm_robots.rosinstall`, followed by `wstool update`. If you don't have a `.rosinstall` file, just download this one, put it in the `src` directory of your catkin ws, and run `wstool update`. This will clone and/or pull all the packages you need.

To get the dependencies, you can try `rosdep install -r --from-paths arm_robots`.

Apt dependencies:
`sudo apt install ros-noetic-pybind11-catkin ros-noetic-joint-trajectory-controller`

## Simulation

    roslaunch arm_robots victor.launch sim:=true


## Real Victor

    roslaunch arm_robots victor.launch


## Here's what it should look like

https://drive.google.com/file/d/1-R0NOalh0yAsff310mjcobYLGQLQ7oJZ/view?usp=sharing
