#! /usr/bin/env python
import colorama
import numpy as np

from arc_utilities.ros_init import rospy_and_cpp_init
from arm_robots.hdt_michigan import Val
from geometry_msgs.msg import Pose
from tf.transformations import quaternion_from_euler

ask_before_moving = True


def myinput(msg):
    global ask_before_moving
    if ask_before_moving:
        input(msg)


def main():
    np.set_printoptions(suppress=True, precision=0, linewidth=200)
    colorama.init(autoreset=True)

    rospy_and_cpp_init("basic_motion")

    val = Val(raise_on_failure=True)
    val.connect()

    # rospy.sleep(1)
    # val.open_left_gripper()
    # rospy.sleep(2)
    # val.close_left_gripper()
    # rospy.sleep(2)
    # val.open_right_gripper()
    # rospy.sleep(2)
    # val.close_right_gripper()
    # rospy.sleep(2)

    print("press enter if prompted")

    # Plan to joint config
    # myinput("Plan to home joint config?")
    val.plan_to_joint_config('both_arms', 'home')

    # s = rospy.ServiceProxy("hdt_michigan/plan_kinematic_path", GetMotionPlan)
    # req = GetMotionPlanRequest()
    # right_tool_goal = Constraints()
    # right_tool_goal.name = 'right_tool_goal'
    # right_tool_goal_position = PositionConstraint(link_name='right_tool', target_point_offset=Vector3(x=0.3, y=0.6, z=0.5))
    # right_tool_goal_position.header.frame_id = 'robot_root'
    # right_tool_goal_position.weight = 1.0
    # right_tool_goal.position_constraints = [right_tool_goal_position]
    # req.motion_plan_request.goal_constraints = [right_tool_goal]
    # req.motion_plan_request.start_state.
    # req.motion_plan_request.group_name = 'both_arms'
    # s(req)

    # Plan to pose
    # myinput("Plan to pose 1?")
    # val.plan_to_pose(val.right_arm_group, val.right_tool_name, [0.3, 0.8, 0.5, -np.pi / 2, np.pi / 2, 0])
    #
    # # # Or you can use a geometry msgs Pose
    # myinput("Plan to pose 2?")
    pose = Pose()
    pose.position.x = 0.2
    pose.position.y = 0.5
    pose.position.z = 0.1
    q = quaternion_from_euler(-np.pi, 0, 0)
    pose.orientation.x = q[0]
    pose.orientation.y = q[1]
    pose.orientation.z = q[2]
    pose.orientation.w = q[3]
    val.plan_to_pose(val.right_arm_group, val.right_tool_name, pose)

    # # # Or with cartesian planning
    # myinput("Cartesian motion back to pose 3?")
    # val.plan_to_position_cartesian(val.right_arm_group, val.right_tool_name, [0.9, -0.4, 0.9], step_size=0.01)
    # val.plan_to_position_cartesian(val.right_arm_group, val.right_tool_name, [0.7, -0.4, 0.9], step_size=0.01)
    #
    # Move hand straight works either with jacobian following
    # myinput("Follow jacobian to pose 2?")
    val.store_current_tool_orientations([val.right_tool_name])

    j = [-0.0208909590972981, 0.30571615955042386, 2.2035334724890276e-05, -1.1273044151494105e-06, 8.6424966716514e-05,
         -5.286047277053285e-05, -7.785048456954158e-05, -3.908594273838162e-05, -5.34043457358635e-05,
         6.017180012007534, -2.528550200283428, -1.4580031505310607, -0.47424552142509757, 2.929741212951332,
         1.5706280249196416, 3.9696152804865275]
    p = [0, 0, 0]
    for i in range(10):
        val.plan_to_joint_config('both_arms', j)
        val.follow_jacobian_to_position('both_arms', [val.right_tool_name], [[p]], vel_scaling=1.0)
        val.plan_to_joint_config('both_arms', 'home')
        val.plan_to_pose(val.right_arm_group, val.right_tool_name, pose)


if __name__ == "__main__":
    main()
