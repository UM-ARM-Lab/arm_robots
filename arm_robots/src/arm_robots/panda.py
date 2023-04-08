#! /usr/bin/env python
import pdb

import numpy as np

import actionlib
import rospy
from arc_utilities.listener import Listener
from arm_robots.robot_utils import PlanningResult, PlanningAndExecutionResult
from franka_gripper.msg import GraspAction, MoveAction, HomingAction, StopAction, GraspGoal, MoveGoal, HomingGoal, \
    StopGoal
from franka_msgs.msg import FrankaState, ErrorRecoveryActionGoal
from geometry_msgs.msg import PoseStamped, Point
from rosgraph.names import ns_join
from typing import List, Tuple, Union, Optional, Callable

from arm_robots.robot import MoveitEnabledRobot, RobotPlanningError
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint
from franka_msgs.srv import SetJointImpedance, SetLoad, SetCartesianImpedance
from controller_manager_msgs.srv import LoadController, SwitchController
from moveit_msgs.srv import GetPositionIK, GetPositionFK, GetPositionFKRequest
from moveit_msgs.msg import PositionIKRequest, RobotState, MoveItErrorCodes, RobotTrajectory, DisplayTrajectory

DEFAULT_JOINT_IMPEDANCE = [3000.0, 3000.0, 3000.0, 2500.0, 2500.0, 2000.0, 2000.0]
SOFT_JOINT_IMPEDANCE = [100.0, 100.0, 100.0, 100.0, 50.0, 50.0, 10.0]
DEFAULT_CARTESIAN_IMPEDANCE = [3000.0, 3000.0, 3000.0, 300.0, 300.0, 300.0]
POSITION_JOINT_TRAJECTORY_CONTROLLER_NAME = 'position_joint_trajectory_controller'


class Panda(MoveitEnabledRobot):

    def __init__(self, robot_namespace: str = 'combined_panda', force_trigger: float = -0.0,
                 arms_controller_name='position_joint_trajectory_controller', panda_name: str = "panda_1",
                 has_gripper: bool = False, **kwargs):
        MoveitEnabledRobot.__init__(self,
                                    robot_namespace=robot_namespace,
                                    robot_description=ns_join(robot_namespace, 'robot_description'),
                                    arms_controller_name=arms_controller_name,
                                    force_trigger=force_trigger,
                                    **kwargs)
        self.panda_name = panda_name
        self.has_gripper = has_gripper

        # Panda HW Services - for setting internal controller parameters.
        self.joint_impedance_srv = rospy.ServiceProxy(self.ns('%s/set_joint_impedance' % self.panda_name),
                                                      SetJointImpedance)
        self.cartesian_impedance_srv = rospy.ServiceProxy(self.ns('%s/set_cartesian_impedance' % self.panda_name),
                                                          SetCartesianImpedance)
        self.set_load_srv = rospy.ServiceProxy(self.ns('%s/set_load' % self.panda_name), SetLoad)

        # Controller Manager Services - for loading/unloading/switching controllers.
        self.load_controller_srv = rospy.ServiceProxy(self.ns('controller_manager/load_controller'), LoadController)
        self.switch_controller_srv = rospy.ServiceProxy(self.ns('controller_manager/switch_controller'),
                                                        SwitchController)

        # IK Service.
        self.ik_srv = rospy.ServiceProxy(self.ns('compute_ik'), GetPositionIK)
        self.fk_srv = rospy.ServiceProxy(self.ns('compute_fk'), GetPositionFK)

        # Default position joint trajectory controller.
        self.active_controller_name = POSITION_JOINT_TRAJECTORY_CONTROLLER_NAME

        # Franka state listener.
        self.franka_state_listener = Listener(self.ns('franka_state_controller/franka_states'), FrankaState)

        # Error recovery publisher.
        self.error_recovery_pub = rospy.Publisher(self.ns('%s/error_recovery/goal' % self.panda_name),
                                                  ErrorRecoveryActionGoal, queue_size=10)

        if self.has_gripper:
            self.gripper = PandaGripper(self.robot_namespace, self.panda_name)
        else:
            self.gripper = None

    def send_joint_command(self, joint_names: List[str], trajectory_point: JointTrajectoryPoint) -> Tuple[bool, str]:
        # TODO: Fill in to send set point to controller.
        pass

    # TODO: Add gripper helpers.

    def switch_controller(self, start_controllers=None, stop_controllers=None) -> bool:
        if stop_controllers is None:
            stop_controllers = []
        if start_controllers is None:
            start_controllers = []

        try:
            switch_controller_resp = self.switch_controller_srv(start_controllers=start_controllers,
                                                                stop_controllers=stop_controllers,
                                                                strictness=2)
        except rospy.ServiceException as e:
            raise Exception("Service call failed: %s" % e)

        return switch_controller_resp.ok

    def load_controller(self, controller_name: str) -> bool:
        try:
            load_controller_resp = self.load_controller_srv(controller_name)
        except rospy.ServiceException as e:
            raise Exception("Service call failed: %s" % e)

        return load_controller_resp.ok

    def set_joint_impedance(self, joint_impedance: List[float]) -> bool:
        # Start by switching off the current controller, if active.
        if self.active_controller_name is not None:
            self.switch_controller(stop_controllers=[self.active_controller_name])

        # Change joint impedance.
        try:
            set_joint_imped_resp = self.joint_impedance_srv(joint_impedance)
        except rospy.ServiceException as e:
            raise Exception("Failed to set impedance: %s" % e)

        if not set_joint_imped_resp.success:
            raise Exception("Failed to set impedance: %s" % set_joint_imped_resp.error)

        # Switch to joint position controller.
        self.switch_controller(start_controllers=[POSITION_JOINT_TRAJECTORY_CONTROLLER_NAME])
        self.active_controller_name = POSITION_JOINT_TRAJECTORY_CONTROLLER_NAME

        return set_joint_imped_resp.success

    def set_cartesian_impedance(self, cartesian_impedance: List[float]) -> bool:
        # Start by switching off the current controller, if active.
        if self.active_controller_name is not None:
            self.switch_controller(stop_controllers=[self.active_controller_name])

        # Change joint impedance.
        try:
            set_cartesian_imp_res = self.cartesian_impedance_srv(cartesian_impedance)
        except rospy.ServiceException as e:
            raise Exception("Failed to set impedance: %s" % e)

        if not set_cartesian_imp_res.success:
            raise Exception("Failed to set impedance: %s" % set_cartesian_imp_res.error)

        # Switch to joint position controller.
        self.switch_controller(start_controllers=[POSITION_JOINT_TRAJECTORY_CONTROLLER_NAME])
        self.active_controller_name = POSITION_JOINT_TRAJECTORY_CONTROLLER_NAME

        return set_cartesian_imp_res.success

    def set_load(self, mass: float, load_com: List[float], load_inertia: List[float]) -> bool:
        """
        See also: setLoad() in libfranka Robot class.

        Parameters
        ----------
        mass - load mass in kg
        load_com - Translation from flange to center of mass in meters
        load_inertia - Inertia matrix in kgxm^2, *column major*.

        Returns
        -------
        bool - whether service completed successfully.
        """
        # Start by switching off the current controller, if active.
        if self.active_controller_name is not None:
            self.switch_controller(stop_controllers=[self.active_controller_name])

        if len(load_com) != 3 or len(load_inertia) != 9:
            raise Exception("Invalid load information provided.")

        try:
            set_load_resp = self.set_load_srv(mass, load_com, load_inertia)
        except rospy.ServiceException as e:
            raise Exception("Failed to set load info: %s" % e)

        # Switch to joint position controller.
        self.switch_controller(start_controllers=[POSITION_JOINT_TRAJECTORY_CONTROLLER_NAME])
        self.active_controller_name = POSITION_JOINT_TRAJECTORY_CONTROLLER_NAME

        return set_load_resp.success

    def get_ik(self, group_name: str, pose: PoseStamped, frame: str = "panda_link8"):
        ik_request = PositionIKRequest()
        ik_request.group_name = group_name
        ik_request.robot_state = self.get_state(group_name=group_name)
        ik_request.ik_link_name = frame
        ik_request.pose_stamped = pose
        ik_request.timeout.secs = 10
        ik_request.avoid_collisions = True

        try:
            ik_response = self.ik_srv(ik_request=ik_request)
        except rospy.ServiceException as e:
            raise Exception("Failed to get ik response: %s" % e)

        if ik_response.error_code.val != MoveItErrorCodes.SUCCESS:
            rospy.loginfo("IK call failed with code: %d" % ik_response.error_code.val)
            return None

        return ik_response.solution.joint_state.position[:7]

    def plan_to_position_cartesian(self,
                                   group_name: str,
                                   ee_link_name: str,
                                   target_position: Union[Point, List, np.array],
                                   step_size: float = 0.02,
                                   stop_condition: Optional[Callable] = None,
                                   velocity_scaling_factor=0.05,
                                   acceleration_scaling_factor=0.05
                                   ):
        move_group = self.get_move_group_commander(group_name)
        move_group.set_end_effector_link(ee_link_name)

        # by starting with the current pose, we will be preserving the orientation
        waypoint_pose = move_group.get_current_pose().pose
        if isinstance(target_position, Point):
            waypoint_pose.position = target_position
        else:
            waypoint_pose.position.x = target_position[0]
            waypoint_pose.position.y = target_position[1]
            waypoint_pose.position.z = target_position[2]
        waypoints = [waypoint_pose]

        plan, fraction = move_group.compute_cartesian_path(waypoints=waypoints, eef_step=step_size, jump_threshold=0.0)
        planning_result = PlanningResult(success=(fraction == 1.0), plan=plan)
        if self.raise_on_failure and not planning_result.success:
            raise RobotPlanningError(f"Cartesian path is only {fraction * 100}% complete")

        # Retime plan.
        retimed_plan = move_group.retime_trajectory(move_group.get_current_state(), plan,
                                                    velocity_scaling_factor=velocity_scaling_factor,
                                                    acceleration_scaling_factor=acceleration_scaling_factor)
        planning_result.plan = retimed_plan

        # path_pub = rospy.Publisher("/retimed_path", DisplayTrajectory, queue_size=5)
        # display_traj = DisplayTrajectory()
        # display_traj.trajectory = [retimed_plan]
        # display_traj.trajectory_start = move_group.get_current_state()
        # for _ in range(10):
        #     path_pub.publish(display_traj)
        #     rospy.sleep(0.1)

        execution_result = self.follow_arms_joint_trajectory(retimed_plan.joint_trajectory, stop_condition)
        return PlanningAndExecutionResult(planning_result, execution_result)

    def get_franka_state(self):
        franka_state: FrankaState = self.franka_state_listener.get()
        return franka_state

    def is_panda_up(self):
        panda_up = True
        franka_state: FrankaState = self.franka_state_listener.get()

        panda_up = panda_up and (franka_state.robot_mode != FrankaState.ROBOT_MODE_REFLEX)
        panda_up = panda_up and (franka_state.robot_mode != FrankaState.ROBOT_MODE_USER_STOPPED)

        return panda_up

    def clear_error(self):
        for _ in range(100):
            self.error_recovery_pub.publish(ErrorRecoveryActionGoal())
            rospy.sleep(0.01)


class PandaGripper:
    def __init__(self, robot_ns, arm_id):
        self.gripper_ns = ns_join(robot_ns, f'{arm_id}/franka_gripper')
        self.grasp_client = actionlib.SimpleActionClient(ns_join(self.gripper_ns, 'grasp'), GraspAction)
        self.move_client = actionlib.SimpleActionClient(ns_join(self.gripper_ns, 'move'), MoveAction)
        self.homing_client = actionlib.SimpleActionClient(ns_join(self.gripper_ns, 'homing'), HomingAction)
        self.stop_client = actionlib.SimpleActionClient(ns_join(self.gripper_ns, 'stop'), StopAction)
        self.grasp_client.wait_for_server()
        self.move_client.wait_for_server()
        self.homing_client.wait_for_server()
        self.stop_client.wait_for_server()
        self.gripper_width = None
        rospy.Subscriber(ns_join(self.gripper_ns, 'joint_states'), JointState, self.gripper_cb)
        self.MIN_FORCE = 0.05
        self.MAX_FORCE = 50  # documentation says up to 70N is possible as continuous force
        self.MIN_WIDTH = 0.0
        self.MAX_WIDTH = 0.08
        self.DEFAULT_EPSILON = 0.005
        self.DEFAULT_SPEED = 0.02
        self.DEFAULT_FORCE = 10

    def gripper_cb(self, data):
        self.gripper_width = data.position[0] + data.position[1]

    def grasp(self, width, speed=None, epsilon_outer=None, epsilon_inner=None, force=None, wait_for_result=False):
        if width > self.gripper_width:
            self.move(self.MAX_WIDTH, wait_for_result=True)
        goal = GraspGoal()
        goal.width = width
        goal.epsilon.outer = self.DEFAULT_EPSILON if not epsilon_outer else epsilon_outer
        goal.epsilon.inner = self.DEFAULT_EPSILON if not epsilon_inner else epsilon_inner
        goal.speed = self.DEFAULT_SPEED if not speed else speed
        goal.force = self.DEFAULT_FORCE if not force else force
        self.grasp_client.send_goal(goal)
        if wait_for_result:
            result = self.grasp_client.wait_for_result(rospy.Duration(10))
            return result
        return True

    def move(self, width, speed=None, wait_for_result=False):
        goal = MoveGoal()
        goal.width = width
        goal.speed = self.DEFAULT_SPEED if not speed else speed
        self.move_client.send_goal(goal)
        if wait_for_result:
            result = self.move_client.wait_for_result(rospy.Duration(10))
            return result
        return True

    def homing(self, wait_for_result=True):
        goal = HomingGoal()
        self.homing_client.send_goal(goal)
        if wait_for_result:
            result = self.homing_client.wait_for_result(rospy.Duration(10))
            return result
        return True

    def stop(self, wait_for_result=False):
        goal = StopGoal()
        self.stop_client.send_goal(goal)
        if wait_for_result:
            result = self.stop_client.wait_for_result(rospy.Duration(10))
            return result
        return True

    def open(self, wait_for_result=False):
        self.move(self.MAX_WIDTH, wait_for_result=wait_for_result)

    def close(self, wait_for_result=False):
        self.move(self.MIN_WIDTH, wait_for_result=wait_for_result)
