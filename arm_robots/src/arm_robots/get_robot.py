from typing import Optional, Type

import rospy
from arm_robots.hdt_michigan import Val, BaseVal
from arm_robots.victor import BaseVictor, Victor


def get_moveit_robot(robot_name: Optional[str] = None, **kwargs):
    get_robot(Val, Victor, robot_name, **kwargs)


def get_base_robot(robot_name: Optional[str] = None, **kwargs):
    get_robot(BaseVal, BaseVictor, robot_name, **kwargs)


def get_robot(val_type: Type, victor_type: Type, robot_name: Optional[str] = None, **kwargs):
    """
    Get the right robot. It considers first the robot_name argument,
    then checks the ros parameter server,
    then the ROS_NAMESPACE variable.
    Args:
        victor_type: the class to instantiate for victor
        val_type:  the class to instantiate for val
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
        return victor_type(robot_name, **kwargs)
    elif robot_name in ['val', 'hdt_michigan']:
        return val_type(robot_name, **kwargs)
    else:
        raise NotImplementedError(f"robot with name {robot_name} not implemented")
