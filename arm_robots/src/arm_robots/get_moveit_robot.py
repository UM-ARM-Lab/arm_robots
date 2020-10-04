from typing import Optional

import rospy
from arm_robots.hdt_michigan import Val
from arm_robots.victor import Victor


def get_moveit_robot(robot_namespace: Optional[str] = None):
    if robot_namespace is None:
        robot_namespace = rospy.get_namespace().strip("/")
    if robot_namespace == 'victor':
        return Victor(robot_namespace)
    elif robot_namespace in ['val', 'hdt_michigan']:
        return Val(robot_namespace)
    else:
        raise NotImplementedError(f"robot with namespace {robot_namespace} not implemented")