import unittest
from unittest import TestCase

from arm_robots.robot import _get_move_group_commander


class Test(TestCase):
    def test_move_group_caching(self):
        # Before running this test, you need to first run `roslaunch victor_moveit_config demo.launch`
        m1 = _get_move_group_commander('left_arm', '')
        m2 = _get_move_group_commander('left_arm', '')
        m3 = _get_move_group_commander('right_arm', '')
        self.assertEqual(m1, m2)
        self.assertNotEqual(m2, m3)


if __name__ == '__main__':
    unittest.main()
