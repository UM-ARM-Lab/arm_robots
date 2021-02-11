from typing import Optional

import rospy
from arm_robots.hdt_michigan import Val, BaseVal
from arm_robots.victor import BaseVictor, Victor


def get_moveit_robot(robot_name: Optional[str] = None, **kwargs):
    """
    Get the right robot. It considers first the robot_name argument,
    then checks the ros parameter server,
    then the ROS_NAMESPACE variable.
    Args:
        robot_name: robot name
        **kwargs:

    Returns:

    """
    default = 'victor'
    if robot_name is None:
        if rospy.has_param("robot_name"):
            robot_name = rospy.get_param("robot_name")
        elif rospy.get_namespace() != "/":
            robot_name = rospy.get_namespace().strip("/")
        else:
            rospy.logwarn(f"using default robot_name{default}")
            robot_name = default

    if robot_name == 'victor':
        return Victor('victor', **kwargs)
    elif robot_name in ['val', 'hdt_michigan']:
        return Val('hdt_michigan', **kwargs)
    else:
        raise NotImplementedError(f"robot with name {robot_name} not implemented")


def get_base_robot(robot_name: Optional[str] = None):
    """
    Get the right robot. It considers first the robot_name argument,
    then checks the ros parameter server,
    then the ROS_NAMESPACE variable.
    Args:
        robot_name: robot name

    Returns:

    """
    default = 'victor'
    if robot_name is None:
        if rospy.has_param("robot_name"):
            robot_name = rospy.get_param("robot_name")
        elif rospy.get_namespace() != "/":
            robot_name = rospy.get_namespace().strip("/")
        else:
            rospy.logwarn(f"using default robot_name{default}")
            robot_name = default

    if robot_name == 'victor':
        return BaseVictor(robot_name)
    elif robot_name in ['val', 'hdt_michigan']:
        return BaseVal(robot_name)
    else:
        raise NotImplementedError(f"robot with name {robot_name} not implemented")
