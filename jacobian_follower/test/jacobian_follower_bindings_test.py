#! /usr/bin/env python3
# Note: python3 needs to be hardcoded for rostests to work properly: https://github.com/ros/ros_comm/issues/1830

from arm_robots.victor import Victor
import unittest
import rostest
import rospy
from geometry_msgs.msg import Pose




class TestJacobianFollowerBindings(unittest.TestCase):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

        self.victor = Victor()
        self.victor.connect()

    def _assert_pose_equal(self, p1: Pose, p2: Pose, places=4):
        self.assertAlmostEqual(p1.position.x, p2.position.x, places=places)
        self.assertAlmostEqual(p1.position.y, p2.position.y, places=places)
        self.assertAlmostEqual(p1.position.z, p2.position.z, places=places)
        self.assertAlmostEqual(p1.orientation.x, p2.orientation.x, places=places)
        self.assertAlmostEqual(p1.orientation.y, p2.orientation.y, places=places)
        self.assertAlmostEqual(p1.orientation.z, p2.orientation.z, places=places)
        self.assertAlmostEqual(p1.orientation.w, p2.orientation.w, places=places)

    def test_fk_ik_bindings_work_and_are_consistent(self):
        test_joint_angles = [0.2, 0.3, 0.5, 0.9, 0.1, 0.2]
        fk = self.victor.jacobian_follower.fk(test_joint_angles, "right_arm")
        iks = self.victor.jacobian_follower.compute_IK_solutions(fk, "right_arm")

        for ik in iks:
            this_fk = self.victor.jacobian_follower.fk(ik, "right_arm")
            self._assert_pose_equal(fk, this_fk)


if __name__ == "__main__":
    rospy.init_node("test_node")
    rostest.rosrun("jacobian_follower", 'jacobian_follower_bindings_test_node', TestJacobianFollowerBindings)
