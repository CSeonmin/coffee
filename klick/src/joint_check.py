#!/usr/bin/env python

import sys
import rospy
import moveit_commander
from urdf_parser_py.urdf import URDF

def print_joint_limits():
    robot = URDF.from_parameter_server()
    print("Active joints and their limits:")
    for joint in robot.joints:
        if joint.limit:  # Check if the joint has limits
            print(f"{joint.name}: min_position = {joint.limit.lower}, max_position = {joint.limit.upper}")

if __name__ == "__main__":
    rospy.init_node('joint_limits_checker')
    moveit_commander.roscpp_initialize(sys.argv)
    robot = moveit_commander.RobotCommander()
    arm_group = moveit_commander.MoveGroupCommander("arm_group")

    print_joint_limits()
