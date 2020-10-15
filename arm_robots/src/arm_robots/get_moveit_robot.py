from typing import Optional

import rospy
from arm_robots.hdt_michigan import Val
from arm_robots.victor import Victor


def get_moveit_robot(robot_namespace: Optional[str] = None):
    default = 'victor'
    if robot_namespace is None:
        if rospy.has_param("robot_namespace"):
            robot_namespace = rospy.get_param("robot_namespace")
        elif rospy.get_namespace() != "/":
            robot_namespace = rospy.get_namespace().strip("/")
        else:
            rospy.logwarn(f"using default robot_namespace {default}")
            robot_namespace = default

    if robot_namespace == 'victor':
        return Victor(robot_namespace)
    elif robot_namespace in ['val', 'hdt_michigan']:
        return Val(robot_namespace)
    else:
        raise NotImplementedError(f"robot with namespace {robot_namespace} not implemented")
