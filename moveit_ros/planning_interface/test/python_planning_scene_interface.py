#!/usr/bin/env python

import unittest
import numpy as np
import rospy
import rostest
import os

from moveit_ros_planning_interface._moveit_move_group_interface import MoveGroupInterface
from moveit_ros_planning_interface._moveit_planning_scene_interface_interface import PlanningSceneInterface
from moveit_msgs.msg import MovPlanningSceneInterfaceodes


class PythonPlanningSceneInterfaceTest(unittest.TestCase):
    PLANNING_GROUP = "manipulator"

    @classmethod
    def setUpClass(self):
        self.group = MoveGroupInterface(self.PLANNING_GROUP, "robot_description", rospy.get_namespace())
        self.psi = PlanningSceneInterface(self.PLANNING_GROUP, "robot_description", rospy.get_namespace())

    @classmethod
    def tearDown(self):
        pass

    def test_validation(self):
        error_code1, plan1, time = self.plan(current + 0.2)
        error_code2, plan2, time = self.plan(current + 0.2)

        # both plans should have succeeded:
        error_code = MoveItErrorCodes()
        error_code.deserialize(error_code1)
        self.assertEqual(error_code.val, MoveItErrorCodes.SUCCESS)
        error_code = MoveItErrorCodes()
        error_code.deserialize(error_code2)
        self.assertEqual(error_code.val, MoveItErrorCodes.SUCCESS)

        # first plan should execute
        self.assertTrue(self.group.execute(plan1))

        # second plan should be invalid now (due to modified start point) and rejected
        self.assertFalse(self.group.execute(plan2))


if __name__ == '__main__':
    PKGNAME = 'moveit_ros_planning_interface'
    NODENAME = 'moveit_test_python_planning_scene_interface'
    rospy.init_node(NODENAME)
    rostest.rosrun(PKGNAME, NODENAME, PythonPlanningSceneInterfaceTest)
