#! /usr/bin/env python
from rosgraph.names import ns_join
from typing import List, Tuple

from arm_robots.robot import MoveitEnabledRobot
from trajectory_msgs.msg import JointTrajectoryPoint


class Panda(MoveitEnabledRobot):

    def __init__(self, robot_namespace: str = 'combined_panda', force_trigger: float = -0.0, **kwargs):
        MoveitEnabledRobot.__init__(self,
                                    robot_namespace=robot_namespace,
                                    robot_description=ns_join(robot_namespace, 'robot_description'),
                                    arms_controller_name='effort_joint_trajectory_controller',
                                    force_trigger=force_trigger,
                                    **kwargs)
        self.nebula_arm = "nebula_arm"
        self.rocket_arm = "rocket_arm"
        self.nebula_wrist = "panda_1_link8"
        self.rocket_wrist = "panda_2_link8"
    def send_joint_command(self, joint_names: List[str], trajectory_point: JointTrajectoryPoint) -> Tuple[bool, str]:
        # TODO: Fill in to send set point to controller.
        pass

    # TODO: Add gripper helpers.

    # TODO: Add control mode setter/getter.
