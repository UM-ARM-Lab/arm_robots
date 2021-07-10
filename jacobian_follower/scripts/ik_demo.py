import logging
import pathlib
import pickle
from math import pi

import numpy as np
import tensorflow as tf

import ros_numpy
import rospy
from arc_utilities import ros_init
from geometry_msgs.msg import Pose, Quaternion, Point
from link_bot_pycommon.get_scenario import get_scenario
from link_bot_pycommon.ros_pycommon import silence_urdfpy_warnings
from moonshine.tf_profiler_helper import TFProfilerHelper
from moveit_msgs.msg import RobotState
from tensorflow_kinematics import hdt_ik
from tensorflow_kinematics.hdt_ik import HdtIK, target
from tf.transformations import quaternion_from_euler


@ros_init.with_ros("ik_demo")
def main():
    tf.get_logger().setLevel(logging.ERROR)
    silence_urdfpy_warnings()
    # tensorflow_kinematics_ik_demo()
    bio_ik_demo()


def bio_ik_demo():
    scenario = get_scenario("dual_arm_rope_sim_val_with_robot_feasibility_checking")
    j = scenario.robot.jacobian_follower

    with pathlib.Path("/media/shared/pretransfer_initial_configs/car/initial_config_0.pkl").open("rb") as f:
        scene_msg = pickle.load(f)['env']['scene_msg']

    group_name = "both_arms"

    gen = tf.random.Generator.from_seed(0)

    name = scenario.robot.get_joint_names(group_name)
    zero_robot_state = RobotState()
    zero_robot_state.joint_state.position = [0] * len(name)
    zero_robot_state.joint_state.name = name
    scenario.robot.display_robot_state(zero_robot_state, label='zero')

    n = 5
    positions = np.stack(np.meshgrid(np.linspace(-0.4, 0.0, n), np.linspace(0.7, 0.7, n), np.linspace(0.0, 0.2, n)),
                         axis=-1)
    positions = positions.reshape([-1, 3])
    # positions = [np.array([-0.25, 0.5, 0.25])] * n
    tip_names = ["left_tool", "right_tool"]
    for position in positions:
        left_position = position
        right_position = position + np.array([0.2, 0, 0])
        left_point = ros_numpy.msgify(Point, position)
        right_point = ros_numpy.msgify(Point, position + np.array([0.2, 0, 0]))
        orientation_list = Quaternion(*quaternion_from_euler(-np.pi / 2 - 0.2, 0, 0))

        left_target_pose = Pose(left_point, orientation_list)
        right_target_pose = Pose(right_point, orientation_list)
        scene_msg.robot_state.joint_state.position = [0] * len(name)
        scene_msg.robot_state.joint_state.name = name

        poses = [left_target_pose, right_target_pose]
        # robot_state: RobotState = j.compute_collision_free_pose_ik(poses, group_name, tip_names, scene_msg)

        points = [left_point, right_point]
        robot_state: RobotState = j.compute_collision_free_point_ik(points, group_name, tip_names, scene_msg)

        scenario.plot_points_rviz([left_position], label='left_target')
        scenario.plot_points_rviz([right_position], label='right_target')
        scenario.tf.send_transform_from_pose_msg(left_target_pose, 'world', 'left_target')
        scenario.tf.send_transform_from_pose_msg(right_target_pose, 'world', 'right_target')
        scenario.planning_scene_viz_pub.publish(scene_msg)
        if robot_state is not None:
            scenario.robot.display_robot_state(robot_state, label='')
            scenario.joint_state_viz_pub.publish(robot_state.joint_state)
        else:
            print("No Solution!")
        rospy.sleep(0.1)


def tensorflow_kinematics_ik_demo():
    urdf_filename = pathlib.Path("/hdt_robot/hdt_michigan_description/urdf/hdt_michigan.urdf")
    scenario = get_scenario("dual_arm_rope_sim_val_with_robot_feasibility_checking")
    ik_solver = HdtIK(urdf_filename, scenario)

    hdt_ik.logger.setLevel(logging.INFO)

    batch_size = 32
    viz = False
    profile = False

    gen = tf.random.Generator.from_seed(0)
    target_noise = gen.uniform([2, batch_size, 7],
                               [-1, -1, -1, 0, 0, 0, 0],
                               [1, 1, 1, 0, 0, 0, 0],
                               dtype=tf.float32) * 0.1
    left_target_pose = tf.tile(target(-0.2, 0.55, 0.2, -pi / 2, 0, 0), [batch_size, 1]) + target_noise[0]
    right_target_pose = tf.tile(target(0.2, 0.55, 0.22, -pi / 2 + 0.5, -pi, 0), [batch_size, 1]) + target_noise[1]
    # right_target_pose = tf.tile(target(0.0, 0.0, 0.0, 0, -pi, 0), [batch_size, 1])
    o = tf.constant([[[-0.25, 0.2, 0.2]]], tf.float32)
    env_points = tf.random.uniform([batch_size, 100, 3], -0.1, 0.1, dtype=tf.float32) + o

    # gen = tf.random.Generator.from_seed(0)
    # initial_noise = gen.uniform([batch_size, ik_solver.get_num_joints()], -1, 1, dtype=tf.float32) * 0.1
    # initial_value = tf.zeros([batch_size, ik_solver.get_num_joints()], dtype=tf.float32) + initial_noise

    logdir = "ik_demo_logdir"
    if profile:
        h = TFProfilerHelper(profile_arg=(1, 10), train_logdir=logdir)
    else:
        h = None

    # from moonshine.simple_profiler import SimpleProfiler
    # total_p = SimpleProfiler()
    #
    # def _solve():
    #     ik_solver.solve(env_points=env_points,
    #                     left_target_pose=left_target_pose,
    #                     right_target_pose=right_target_pose,
    #                     viz=viz,
    #                     profiler_helper=h)
    #
    # total_p.profile(5, _solve, skip_first_n=1)
    # print()
    # print(total_p)

    with pathlib.Path("/media/shared/pretransfer_initial_configs/car/initial_config_0.pkl").open("rb") as f:
        scene_msg = pickle.load(f)['env']['scene_msg']
    scene_msg_batched = [scene_msg] * batch_size

    scenario.planning_scene_viz_pub.publish(scene_msg)

    q, converged = ik_solver.solve(env_points=env_points,
                                   scene_msg=scene_msg_batched,
                                   left_target_pose=left_target_pose,
                                   right_target_pose=right_target_pose,
                                   viz=viz)
    print(ik_solver.get_percentage_solved())
    # from merrrt_visualization.rviz_animation_controller import RvizSimpleStepper
    # stepper = RvizSimpleStepper()
    # for b in range(batch_size):
    #     ik_solver.plot_robot_and_targets(q, left_target_pose, right_target_pose, b=b)
    #     stepper.step()


if __name__ == '__main__':
    main()
