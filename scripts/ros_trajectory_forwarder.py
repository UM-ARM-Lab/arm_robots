#! /usr/bin/env python

"""
ros_trajectory_follower.py

Executes a ROS JointTrajectory as a message or action on Victor.

Author: Andrew Price
Date: 2018-06-06
"""

import sys
import rospy
import actionlib
from std_srvs.srv import SetBool, SetBoolResponse
from trajectory_msgs.msg import JointTrajectory
import control_msgs.msg
import arm_or_robots.motion_victor

import openravepy as rave


class TrajectoryForwarder(object):
    def __init__(self, world_frame, plan_to_start):
        self._action_name = "follow_joint_trajectory"
        self.plan_to_start = plan_to_start

        self.srv = rospy.Service("set_plan_to_start_config", SetBool, self.handle_set_planning)

        self._victor = arm_or_robots.motion_victor.MotionEnabledVictor(world_frame=world_frame, viewer=False)
        self._victor.or_model.env.GetCollisionChecker().SetCollisionOptions(rave.CollisionOptions.ActiveDOFs)

        self._chain_joint_names = dict()
        self._chain_joint_names["all_arms"] = set()

        # self._victor.or_model.robot.GetManipulators()
        self._manipulator_names = ["left_arm", "right_arm"]
        joints = self._victor.or_model.robot.GetJoints()
        for manip_name in self._manipulator_names:
            manip = self._victor.or_model.robot.GetManipulator(manip_name)
            self._chain_joint_names[manip_name] = set()
            indices = manip.GetArmIndices()
            for idx in indices:
                jnt = joints[idx]
                self._chain_joint_names[manip_name].add(jnt.GetName())
            self._chain_joint_names["all_arms"] |= self._chain_joint_names[manip_name]

        self.sub = rospy.Subscriber("joint_trajectory", JointTrajectory, self.joint_trajectory_callback)

        self._as = actionlib.SimpleActionServer(self._action_name, control_msgs.msg.FollowJointTrajectoryAction,
                                                execute_cb=self.execute_cb, auto_start=False)
        self._as.start()

    def handle_set_planning(self, req):
        self.plan_to_start = req.data
        rospy.loginfo("Set self.plan_to_start to " + str(self.plan_to_start))
        return SetBoolResponse(success=True)

    def execute_cb(self, goal):
        res = control_msgs.msg.FollowJointTrajectoryResult()
        if self.execute_trajectory(goal.trajectory):
            res.error_code = control_msgs.msg.FollowJointTrajectoryResult.SUCCESSFUL
            self._as.set_succeeded(res)
        else:
            rospy.logerr("Trajectory forwarding failed.")
            res.error_code = control_msgs.msg.FollowJointTrajectoryResult.INVALID_JOINTS
            self._as.set_aborted(res)

    def joint_trajectory_callback(self, traj_msg):
        self.execute_trajectory(traj_msg)

    def execute_trajectory(self, traj_msg):
        names = set(traj_msg.joint_names)
        if names == self._chain_joint_names["left_arm"]:
            self._victor.enable_only_left_arm()
        elif names == self._chain_joint_names["right_arm"]:
            self._victor.enable_only_right_arm()
        elif names == self._chain_joint_names["all_arms"]:
            self._victor.enable_both_arms()
        else:
            rospy.logerr("Names mismatch: the list of joint names does not match either arm or the combination")
            return False

        if self.plan_to_start:
            self._victor.plan_to_configuration(traj_msg.points[0].positions, execute=True)
        else:
            start_matches_current = True
            q = self._victor.or_model.robot.GetDOFValues()
            for i, joint_name in enumerate(traj_msg.joint_names):
                idx = self._victor.or_model.robot.GetJoint(joint_name).GetDOFIndex()
                if abs(traj_msg.points[0].positions[i] - q[idx]) > 0.05:
                    rospy.logerr("Joint " + joint_name + " does not match trajectory start. (" + str(traj_msg.points[0].positions[i]) + " vs " + str(q[idx]) + ")")
                    start_matches_current = False
            if not start_matches_current:
                return False

        or_traj = self._victor.traj_from_path_msg(traj_msg)
        self._victor.execute_trajectory(or_traj, blocking=True)

        return True


def main(argv):
    rospy.init_node('victor_ros_trajectory_forwarder', argv=argv)

    fwd = TrajectoryForwarder(rospy.get_param('world_frame', None),
                              rospy.get_param('plan_to_start_config', False))

    rospy.spin()


if __name__ == "__main__":
    main(sys.argv)
