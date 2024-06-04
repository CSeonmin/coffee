#!/usr/bin/env python
import sys
import rospy
import moveit_commander
import time
import math
from std_msgs.msg import Int32
from klick.srv import GripperControl, GripperControlRequest, GripperControlResponse
import numpy as np

gripper_service = None
gripper_pub = rospy.Publisher("state_check", Int32, queue_size=10)

def degrees_to_radians(degrees):
    return [angle * (math.pi / 180.0) for angle in degrees]

def calculate_joint_movement(plan):
    joint_movements = np.zeros(len(plan.joint_trajectory.joint_names))
    for i in range(1, len(plan.joint_trajectory.points)):
        for j in range(len(plan.joint_trajectory.joint_names)):
            joint_movements[j] += abs(plan.joint_trajectory.points[i].positions[j] - plan.joint_trajectory.points[i-1].positions[j])
    return sum(joint_movements)

def gripper_move(val):
    global gripper_service
    gripper_pub.publish(0)
    try:
        print("Waiting for gripper_service...")
        rospy.wait_for_service('gripper_service', timeout=10)
        print("gripper_service available.")
        gripper_service = rospy.ServiceProxy('gripper_service', GripperControl)
        request = GripperControlRequest(val=val)
        response = gripper_service(request)
        if response.success:
            print(f"Gripper moved to {val}")
        else:
            print("Gripper service call failed")
    except rospy.ServiceException as e:
        print(f"Gripper service call failed: {e}")
    except rospy.ROSException as e:
        print(f"Service call timeout: {e}")

def move_to_jointpose(joint_goal):
    moveit_commander.roscpp_initialize(sys.argv)
    robot = moveit_commander.RobotCommander()
    arm_group = moveit_commander.MoveGroupCommander("arm_group")
    arm_group.set_joint_value_target(joint_goal)

    best_plan = None
    min_joint_movement = float('inf')

    for i in range(10):
        plan = arm_group.plan()
        
        if isinstance(plan, tuple):
            plan = plan[1]

        if plan and hasattr(plan, 'joint_trajectory') and plan.joint_trajectory.points:
            joint_movement = calculate_joint_movement(plan)
            if joint_movement < min_joint_movement:
                min_joint_movement = joint_movement
                best_plan = plan

    if best_plan:
        arm_group.execute(best_plan, wait=True)
        arm_group.stop()
        arm_group.clear_pose_targets()
        print("Movement successful with minimum joint movement!")
        return True
    else:
        print("Movement failed!")
        return False

def main():
    rospy.init_node('move_joint_pose_node', anonymous=True)

    # 그리퍼 열기
    print("Attempting to open gripper")
    gripper_move(95)
    print("Gripper opened")
    # time.sleep(1)

    # 첫 번째 자세로 이동
    joint_pose_1 = [-130.69028960428412, 129.8265039520659, -79.09464411518309, -51.73039505214806, -37.3508166246381, -8.084426685044441]
    joint_pose_1_rad = degrees_to_radians(joint_pose_1)
    print("Moving to joint pose 1:", joint_pose_1_rad)

    if move_to_jointpose(joint_pose_1_rad):
        print("Moved to joint pose 1")
        time.sleep(2)

    # 두 번째 자세로 이동
    joint_pose_2 = [-122.98358718698353, 114.98233181631265, -58.938883212922256, -56.89225238501183, -45.0578082828973, -8.281959675778296]
    joint_pose_2_rad = degrees_to_radians(joint_pose_2)
    print("Moving to joint pose 2:", joint_pose_2_rad)
    
    if move_to_jointpose(joint_pose_2_rad):
        print("Moved to joint pose 2")
        time.sleep(2)


    # 그리퍼 닫기
    print("Attempting to close gripper")
    gripper_move(50)
    print("Gripper closed")
    time.sleep(1)

    # 세 번째 자세로 이동
    joint_pose_3 = [-121.14437634240409, 94.73974686706585, -51.643650178855445, -43.92511994902927, -46.898591864086995, -8.314659038418842]
    joint_pose_3_rad = degrees_to_radians(joint_pose_3)
    print("Moving to joint pose 3:", joint_pose_3_rad)

    if move_to_jointpose(joint_pose_3_rad):
        print("Moved to joint pose 3")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
