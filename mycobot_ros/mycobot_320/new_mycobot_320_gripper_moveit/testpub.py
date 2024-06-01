#!/usr/bin/env python
import sys
import rospy
import moveit_commander
from moveit_msgs.msg import MoveItErrorCodes
from geometry_msgs.msg import PoseStamped, Pose
import time

def main():
    rospy.init_node('move_and_grip', anonymous=True)
    
    moveit_commander.roscpp_initialize(sys.argv)
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()

    arm_group = moveit_commander.MoveGroupCommander("arm_group")
    gripper_group = moveit_commander.MoveGroupCommander("gripper_group")

    # Define the "home" position directly
    home_position = [0, 0, 0, 0, 0, 0]
    arm_group.set_joint_value_target(home_position)
    arm_group.go(wait=True)
    rospy.loginfo("Robot is in the home position.")

    # Move to a specified position
    pose_target = Pose()
    pose_target.position.x = 0.20
    pose_target.position.y = -0.1
    pose_target.position.z = 0.3
    pose_target.orientation.x = 0
    pose_target.orientation.y = 0
    pose_target.orientation.z = 0
    pose_target.orientation.w = 1  # No rotation quaternion

    arm_group.set_pose_target(pose_target)
    
    plan = arm_group.go(wait=True)
    arm_group.stop()
    arm_group.clear_pose_targets()

    if plan:
        rospy.loginfo("Movement successful, proceeding with gripper operation.")
        
        # Gripper operation
        gripper_group.set_joint_value_target([0.01])  # Gripper close
        gripper_group.go(wait=True)

        time.sleep(1)  # Holding time

        gripper_group.set_joint_value_target([0.04])  # Gripper open
        gripper_group.go(wait=True)
        
        rospy.loginfo("Gripper operation complete. Returning to home.")
    else:
        rospy.logerr("Movement failed")

    # Return to the home position
    arm_group.set_joint_value_target(home_position)
    arm_group.go(wait=True)
    rospy.loginfo("Robot returned to the home position.")

    moveit_commander.roscpp_shutdown()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
