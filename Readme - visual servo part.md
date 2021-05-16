## Readme for visual servo part
joint velocity controller and Cartesian velocity controller are implemented and tested on this branch
**Author:** [Haoran Mike Cheng](https://www.linkedin.com/in/hrcheng/)
**Email:** hrcheng@umich.edu
**video1:** [link](https://drive.google.com/file/d/1DtUT4oGw0dRihGC9xFf87BB2URdZNaLy/view?usp=sharing)
**video2:** [link](https://drive.google.com/file/d/1IeT5DUZyk38ivkdZwN-TUh9okAyj9j8U/view?usp=sharing)
**References:** [Project Report](https://drive.google.com/file/d/1RTxMGtyoRdckZbu1weEqGqMV1YU7K-99/view?usp=sharing)
To run the Cartesian velocity controller 
On val, under home folder, run `roslaunch catkin_ws/src/visual_servo_val.launch` 
On armoire, run  
`export ROS_MASTER_URI=http://10.10.10.111:11311`
`roslaunch arm_robots val_visual_servo.launch`
then run  arm_robots/scripts/joint_velocity_controller_test.py 
which generates the motion in video1 and video2