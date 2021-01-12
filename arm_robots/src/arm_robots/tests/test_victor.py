from unittest import TestCase

import arm_robots.config.victor_config
from arm_robots.victor import delegate_to_arms
from arm_robots import victor


class Test(TestCase):
    def test_delegate_positions_to_arms(self):
        values = [1, 2, 3, 4, 5, 6, 7]
        inferred_positions, abort, msg = delegate_to_arms(values, arm_robots.config.victor_config.RIGHT_ARM_JOINT_NAMES)
        self.assertFalse(abort)
        self.assertEqual(inferred_positions['left_arm'], None)
        self.assertEqual(inferred_positions['left_gripper'], None)
        self.assertEqual(inferred_positions['right_gripper'], None)
        self.assertEqual(inferred_positions['right_arm'], values)

        values = [1, 2, 3, 4, 5, 6, 7]
        inferred_positions, abort, msg = delegate_to_arms(values + list(reversed(values)),
                                                          arm_robots.config.victor_config.RIGHT_ARM_JOINT_NAMES + arm_robots.config.victor_config.LEFT_ARM_JOINT_NAMES)
        self.assertFalse(abort)
        self.assertEqual(inferred_positions['left_gripper'], None)
        self.assertEqual(inferred_positions['right_gripper'], None)
        self.assertEqual(inferred_positions['left_arm'], list(reversed(values)))
        self.assertEqual(inferred_positions['right_arm'], values)

        inferred_positions, abort, msg = delegate_to_arms(values[0:6],
                                                          arm_robots.config.victor_config.RIGHT_ARM_JOINT_NAMES[0:6])
        self.assertTrue(abort)
        self.assertEqual(inferred_positions['left_gripper'], None)
        self.assertEqual(inferred_positions['right_gripper'], None)
        self.assertEqual(inferred_positions['left_arm'], None)
        self.assertEqual(inferred_positions['right_arm'], None)

        inferred_positions, abort, msg = delegate_to_arms(values,
                                                          list(reversed(
                                                              arm_robots.config.victor_config.RIGHT_ARM_JOINT_NAMES)))
        self.assertFalse(abort)
        self.assertEqual(inferred_positions['left_gripper'], None)
        self.assertEqual(inferred_positions['right_gripper'], None)
        self.assertEqual(inferred_positions['left_arm'], None)
        self.assertEqual(inferred_positions['right_arm'], list(reversed(values)))
