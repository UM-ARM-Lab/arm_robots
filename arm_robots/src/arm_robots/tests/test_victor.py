from unittest import TestCase
from arm_robots.victor import delegate_positions_to_arms
from arm_robots import victor


class Test(TestCase):
    def test_delegate_positions_to_arms(self):
        values = [1, 2, 3, 4, 5, 6, 7]
        inferred_positions, abort, msg = delegate_positions_to_arms(values, victor.right_arm_joints)
        self.assertFalse(abort)
        self.assertEqual(inferred_positions['left_arm'], None)
        self.assertEqual(inferred_positions['left_gripper'], None)
        self.assertEqual(inferred_positions['right_gripper'], None)
        self.assertEqual(inferred_positions['right_arm'], values)

        values = [1, 2, 3, 4, 5, 6, 7]
        inferred_positions, abort, msg = delegate_positions_to_arms(values + list(reversed(values)),
                                                                    victor.right_arm_joints + victor.left_arm_joints)
        self.assertFalse(abort)
        self.assertEqual(inferred_positions['left_gripper'], None)
        self.assertEqual(inferred_positions['right_gripper'], None)
        self.assertEqual(inferred_positions['left_arm'], list(reversed(values)))
        self.assertEqual(inferred_positions['right_arm'], values)

        inferred_positions, abort, msg = delegate_positions_to_arms(values[0:6],
                                                                    victor.right_arm_joints[0:6])
        self.assertTrue(abort)
        self.assertEqual(inferred_positions['left_gripper'], None)
        self.assertEqual(inferred_positions['right_gripper'], None)
        self.assertEqual(inferred_positions['left_arm'], None)
        self.assertEqual(inferred_positions['right_arm'], None)

        inferred_positions, abort, msg = delegate_positions_to_arms(values,
                                                                    list(reversed(victor.right_arm_joints)))
        self.assertFalse(abort)
        self.assertEqual(inferred_positions['left_gripper'], None)
        self.assertEqual(inferred_positions['right_gripper'], None)
        self.assertEqual(inferred_positions['left_arm'], None)
        self.assertEqual(inferred_positions['right_arm'], list(reversed(values)))
