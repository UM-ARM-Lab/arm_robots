from typing import Optional, Type

import rospy
from arm_robots.robot import MoveitEnabledRobot
from arm_robots.base_robot import BaseRobot

def get_base_robot(robot_name: Optional[str] = None, **kwargs) -> BaseRobot:
    return get_robot(base=True, robot_name=robot_name, **kwargs)

def get_moveit_robot(robot_name: Optional[str] = None, **kwargs) -> MoveitEnabledRobot:
    return get_robot(base=False, robot_name=robot_name, **kwargs)

def get_robot(base: bool, robot_name: Optional[str] = None, **kwargs):
    """
    Get the right robot. It considers first the robot_name argument,
    then checks the ros parameter server,
    then the ROS_NAMESPACE variable.
    this nonsense is all to avoid a bit of code duplication
    Args:
        base:  true for the base versions of the class
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
        from arm_robots.victor import Victor, BaseVictor
        return BaseVictor(robot_name, **kwargs) if base else Victor(robot_name, **kwargs)
    elif robot_name in ['val', 'hdt_michigan']:
        from arm_robots.hdt_michigan import Val, BaseVal
        return BaseVal('hdt_michigan', **kwargs) if base else Val('hdt_michigan', **kwargs)
    elif robot_name == 'med':
        from arm_robots.med import Med, BaseMed
        return BaseMed(robot_name, **kwargs) if base else Med(robot_name, **kwargs)
    else:
        raise NotImplementedError(f"robot with name {robot_name} not implemented")
