#!/usr/bin/env python
import sys
import rospy
import moveit_commander
from moveit_commander import MoveGroupCommander
from geometry_msgs.msg import Pose, Point, Quaternion
from klick.srv import OrderPose, OrderPoseResponse
import numpy as np
import time

def calculate_joint_movement(plan):
    joint_movements = np.zeros(len(plan.joint_trajectory.joint_names))
    for i in range(1, len(plan.joint_trajectory.points)):
        for j in range(len(plan.joint_trajectory.joint_names)):
            joint_movements[j] += abs(plan.joint_trajectory.points[i].positions[j] - plan.joint_trajectory.points[i-1].positions[j])
    return sum(joint_movements)

def move_to_pose(pose_goal):
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
    
    response = OrderPoseResponse()
    if best_plan:
        start_execution_time = time.time()
        arm_group.execute(best_plan, wait=True)
        end_execution_time = time.time()
        execution_time = end_execution_time - start_execution_time
        print(f"Movement successful in {execution_time:.4f} seconds!")
        response.success = True
        response.execution_time = execution_time
    else:
        print("Movement failed!")
        response.success = False
    
    arm_group.stop()
    arm_group.clear_pose_targets()

    return response

def handle_order_pose(req):
    poses = {
        "ice": Pose(
            position=Point(x=-0.07133465424571968, y=-0.12338453622334564, z=0.4645782610823642),
            orientation=Quaternion(x=-0.014189142046421153, y=0.7287104368040898, z=-0.6839181996009684, w=0.03218173076995012)
        ),
        "hot": Pose(
            position=Point(x=0.22035660942142493, y=-0.3028769976684234, z=0.4035574471275266),
            orientation=Quaternion(x=-0.3091989142011673, y=0.6999820350874303, z=-0.5805675200605409, w=0.2781412171225842)
        ),
        "water": Pose(
            position=Point(x=-0.2886202891996795, y=-0.12064164920784562, z=0.45554086190730114),
            orientation=Quaternion(x=-0.47485739015680895, y=-0.5239393540064138, z=0.4853561264685334, w=0.514225089660421)
        ),
        "milk": Pose(
            position=Point(x=-0.3139641285683432, y=0.09473113599624029, z=0.43188768136114786),
            orientation=Quaternion(x=-0.44497694415549754, y=-0.5495425767030933, z=0.5390032967889011, w=0.4576832109781353)
        ),
        "syrup": Pose(  
            position=Point(x=0.20894596170957094, y=-0.07586467445221927, z=0.5229588090617533),
            orientation=Quaternion(x=0.011175563455213717, y=0.7070885106429307, z=0.03192579636282336, w=0.7063155728318756)
        )
    }

    if req.temperature not in ["ice", "hot"]:
        return OrderPoseResponse(success=False, execution_time=0.0)
    if req.drink_type not in ["americano", "latte"]:
        return OrderPoseResponse(success=False, execution_time=0.0)

    temperature_pose = poses[req.temperature]
    if req.drink_type == "americano":
        drink_pose = poses["water"]
    else:
        drink_pose = poses["milk"]

    total_execution_time = 0.0

    # 첫 번째 위치로 이동 (ice or hot)
    response1 = move_to_pose(temperature_pose)
    if not response1.success:
        return OrderPoseResponse(success=False, execution_time=response1.execution_time)
    total_execution_time += response1.execution_time

    rospy.sleep(8)  # 5초 대기

    # 두 번째 위치로 이동 (water or milk)
    response2 = move_to_pose(drink_pose)
    if not response2.success:
        return OrderPoseResponse(success=False, execution_time=total_execution_time + response2.execution_time)
    total_execution_time += response2.execution_time

    # 시럽이 있는 경우 시럽 위치로 이동
    if req.syrup:
        rospy.sleep(8)  # 5초 대기
        response3 = move_to_pose(poses["syrup"])
        if not response3.success:
            return OrderPoseResponse(success=False, execution_time=total_execution_time + response3.execution_time)
        total_execution_time += response3.execution_time

    return OrderPoseResponse(success=True, execution_time=total_execution_time)

if __name__ == "__main__":
    rospy.init_node('order_pose_server')
    moveit_commander.roscpp_initialize(sys.argv)
    robot = moveit_commander.RobotCommander()
    global arm_group
    arm_group = MoveGroupCommander("arm_group")

    s = rospy.Service('order_pose', OrderPose, handle_order_pose)
    print("Ready to receive order poses.")
    rospy.spin()
