#!/usr/bin/env python
import sys
import rospy
import moveit_commander
from moveit_commander import MoveGroupCommander
from geometry_msgs.msg import Pose
import numpy as np
import time

def calculate_joint_movement(plan):
    joint_movements = np.zeros(len(plan.joint_trajectory.joint_names))
    for i in range(1, len(plan.joint_trajectory.points)):
        for j in range(len(plan.joint_trajectory.joint_names)):
            joint_movements[j] += abs(plan.joint_trajectory.points[i].positions[j] - plan.joint_trajectory.points[i-1].positions[j])
    return sum(joint_movements)

def move_to_pose():
    rospy.init_node('move_to_pose_node', anonymous=True)
    moveit_commander.roscpp_initialize(sys.argv)
    robot = moveit_commander.RobotCommander()
    arm_group = MoveGroupCommander("arm_group")
    
    current_joint_values = arm_group.get_current_joint_values()
    print("Current joint values:", current_joint_values)
    
    pose_goal = Pose()
    pose_goal.position.x = 0.025
    pose_goal.position.y = 0.423
    pose_goal.position.z = 0.259
    pose_goal.orientation.x = -0.696
    pose_goal.orientation.y = -0.123
    pose_goal.orientation.z = 0.062
    pose_goal.orientation.w = 0.704

# Current pose: position: 
#   x: 0.025454695327555246
#   y: 0.4243141114696377
#   z: 0.25874222553106935
# orientation: 
#   x: -0.6963440508753133
#   y: -0.12284219232696306
#   z: 0.06170147217450943
#   w: 0.7044201068441781

    arm_group.set_pose_target(pose_goal)

    best_plan = None
    min_joint_movement = float('inf')
    successful_plans = 0

    planning_times = []
    for i in range(10):
        print(f"Planning attempt {i+1}/10")
        start_time = time.time()
        plan = arm_group.plan()
        end_time = time.time()
        planning_time = end_time - start_time
        planning_times.append(planning_time)
        
        if isinstance(plan, tuple):
            plan = plan[1]  

        if plan and hasattr(plan, 'joint_trajectory') and plan.joint_trajectory.points:
            successful_plans += 1
            joint_movement = calculate_joint_movement(plan)
            if joint_movement < min_joint_movement:
                min_joint_movement = joint_movement
                best_plan = plan
            print(f"Plan {i+1} successful with joint movement: {joint_movement} in {planning_time:.4f} seconds")
        else:
            print(f"Plan {i+1} failed in {planning_time:.4f} seconds")

    print(f"Total successful plans: {successful_plans}/10")
    print(f"Average planning time: {np.mean(planning_times):.4f} seconds")
    
    if best_plan:
        start_execution_time = time.time()
        arm_group.execute(best_plan, wait=True)
        end_execution_time = time.time()
        execution_time = end_execution_time - start_execution_time
        print(f"Movement successful in {execution_time:.4f} seconds!")
    else:
        print("Movement failed!")
    
    arm_group.stop()
    arm_group.clear_pose_targets()

if __name__ == '__main__':
    try:
        move_to_pose()
    except rospy.ROSInterruptException:
        pass
