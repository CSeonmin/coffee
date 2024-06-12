#!/usr/bin/env python
import sys
import rospy
import moveit_commander
from moveit_commander import MoveGroupCommander
from geometry_msgs.msg import Pose, Point, Quaternion
from std_msgs.msg import Int32, String
from klick.srv import OrderPose, OrderPoseResponse, GripperControl, GripperControlRequest, GripperControlResponse, CheckAgv, CheckAgvRequest
import numpy as np
import time, math
from queue import Queue, Empty

gripper_service = None
gripper_pub = rospy.Publisher("state_check", Int32, queue_size=10)
takeout_queue = Queue()
agv_move_pub = rospy.Publisher('agv_move_command', Int32, queue_size=10)
order_complete_pub = rospy.Publisher("order_complete", String, queue_size=10)

joints = {
    "americano_cup": [-82, 125, -71, -62, -95, -1],
    "latte_cup": [-112, 125, -71, -62, -65, -1],
    "syrup": [-50, 98, -35, -15, -30, -17],
    "cup_place": [-60, 107.5, -60, -55, -8, -3],
    "shot_cup": [-72, 95, -20, -90, -48, 0],
    "shot_pour_pose": [-65, 48, 5, -80, -17, 0],
    "shot_pour": [-65, 48, 5, -80, -17, -93],
    "water_dispenser": [-68, 35, -30, -10, -112, 0],
    "milk_dispenser": [-98, 35, -30, -10, -82, 0],
    "takeout_place": [-2, 100, -45, -75, -60, -1.05],
    "serving_place_first": [-51, -128, 65, 75, 21, -3],
    "serving_place_second": [-31, -128, 65, 75, 1, -3],
    "init_pose": [-85, 0, 0, 0, -95, 0]
}

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

def move_to_joint_pose(joint_goal):
    joint_goal_radians = degrees_to_radians(joint_goal)
    arm_group.set_joint_value_target(joint_goal_radians)
    
    print(f"Target joint positions (radians): {joint_goal_radians}")
    
    # Optional: Set different planner
    arm_group.set_planner_id("RRTConnect")
    arm_group.set_num_planning_attempts(10)
    arm_group.set_planning_time(10)

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

def init_move():
    response = move_to_joint_pose(joints["init_pose"])
    if not response.success:
        return response
    time.sleep(5)
    gripper_move(60)
    time.sleep(1.5)

def handle_water_or_milk(order_number, drink_count, menu):
    print(f"주문번호: {order_number}, {drink_count}번째 잔, {menu} 음료를 준비 중입니다.")
    if menu == "아메리카노":
        dispenser_joint_goal = joints["water_dispenser"]
        cup_type_joint_goal = joints["americano_cup"]
        push_dispenser_joint_goal = [-68, 50, -10, -55, -115.5, -2]
    else:
        dispenser_joint_goal = joints["milk_dispenser"]
        cup_type_joint_goal = joints["latte_cup"]
        push_dispenser_joint_goal = [-98, 50, -10, -55, -85, 0]

    response = move_to_joint_pose(dispenser_joint_goal)
    if not response.success:
        return response
    time.sleep(5)
    print("move to dispenser")

    response = move_to_joint_pose(push_dispenser_joint_goal)
    if not response.success:
        return response
    time.sleep(5)

    print("push the dispenser")
    
    response = move_to_joint_pose(dispenser_joint_goal)
    if not response.success:
        return response
    time.sleep(5)
    print("Water or milk pour complete.")

    gripper_move(100)
    time.sleep(1.5)
    
    response = move_to_joint_pose(cup_type_joint_goal)
    if not response.success:
        return response
    time.sleep(5)

    gripper_move(65)
    time.sleep(1.5)

    print("Hold the cup")

    return response

def handle_shot(order_number, drink_count, temperature):
    print(f"주문번호: {order_number}, {drink_count}번째 잔, {temperature} 음료를 제조 중입니다.")
    
    response = move_to_joint_pose(joints["cup_place"])
    if not response.success:
        return response
    time.sleep(5)

    gripper_move(100)
    time.sleep(1.5)

    print("Put down the cup")

    response = move_to_joint_pose(joints["shot_cup"])
    if not response.success:
        return response
    time.sleep(5)

    gripper_move(50)
    time.sleep(1.5)
    
    print("Hold the shot cup")

    response = move_to_joint_pose(joints["shot_pour_pose"])
    if not response.success:
        return response
    time.sleep(5)
    print("move to cup for pouring the shot")
    
    response = move_to_joint_pose(joints["shot_pour"])
    if not response.success:
        return response
    time.sleep(5)
    print("shot poured")

    response = move_to_joint_pose(joints["shot_pour_pose"])
    if not response.success:
        return response
    time.sleep(5)
    print("sucess to shot pouring")

    response = move_to_joint_pose(joints["shot_cup"])
    if not response.success:
        return response
    time.sleep(5)

    gripper_move(100)
    time.sleep(2)

    print("return the shot cup")

    return OrderPoseResponse(success=True, execution_time=0.0)

def handle_syrup(order_number, drink_count, syrup_option):
    print(f"주문번호: {order_number}, {drink_count}번째 잔에 시럽을 추가 중입니다.")

    if syrup_option == "시럽(O)":
        gripper_move(13)
        time.sleep(2.5)
        response = move_to_joint_pose(joints["syrup"])
        if not response.success:
            return response
        time.sleep(5)
        print("squeeze syrup")
    
    gripper_move(100)
    time.sleep(2.5)
    
    response = move_to_joint_pose(joints["cup_place"])
    if not response.success:
        return response
    time.sleep(5)

    gripper_move(65)
    time.sleep(1.5)

    print("syrup added")

    return OrderPoseResponse(success=True, execution_time=0.0)

def handle_takeout_or_serving(order_number, takeout_serving, drink_count):

    if takeout_serving == "테이크아웃":
        print(f"주문번호: {order_number}, {drink_count}번째 잔을 테이크아웃 위치로 이동 중입니다.")
        response = move_to_joint_pose(joints["takeout_place"])
        if not response.success:
            return response
        time.sleep(5)

        gripper_move(100)
        time.sleep(1.5)

        print("put down the cup for takeout")

    elif takeout_serving == "배달":
        if drink_count == 1:
            print(f"주문번호: {order_number}, 첫 번째 잔을 배달 위치 1로 이동 중입니다.")
            response = move_to_joint_pose(joints["serving_place_first"])
            if not response.success:
                return response
            time.sleep(5)

            gripper_move(100)
            time.sleep(1.5)

            print("put down the first cup for serving on AGV")

        elif drink_count == 2:
            print(f"주문번호: {order_number}, 두 번째 잔을 배달 위치 2로 이동 중입니다.")
            response = move_to_joint_pose(joints["serving_place_second"])
            if not response.success:
                return response
            time.sleep(5)

            gripper_move(100)
            time.sleep(1.5)
            
            print("put down the second cup for serving on AGV")

    response = move_to_joint_pose(joints["init_pose"])
    if not response.success:
        return response

    return OrderPoseResponse(success=True, execution_time=0.0)

def check_agv(command):
    rospy.wait_for_service('check_agv')
    try:
        check_service = rospy.ServiceProxy('check_agv', CheckAgv)
        request = CheckAgvRequest(command=command)
        response = check_service(request)
        return response.success
    except rospy.ServiceException as e:
        print(f"Service call failed: {e}")
        return False

def send_agv_move_command():
    rospy.loginfo("Sending AGV move command")
    agv_move_pub.publish(1)

def handle_order_pose(request):
    order_number = request.order_number
    order_type = request.order_type
    menu = request.drink_type
    temperature = request.temperature
    syrup = request.syrup
    count = request.count

    if order_type == "테이크아웃":
        takeout_queue.put(request)

    total_execution_time = 0.0

    if order_type == "배달":
        if not check_agv(0):
            print("Delivery robot is not in position. Sending command to move.")
            send_agv_move_command()
            start_time = time.time()
            while not check_agv(0):
                print("Waiting for delivery robot to get in position...")
                if time.time() - start_time > 15:
                    print("Delivery robot did not return in 15 seconds. Processing next takeout order.")
                    takeout_order = takeout_queue.get() if not takeout_queue.empty() else None
                    if takeout_order:
                        handle_order_pose(takeout_order)
                    return OrderPoseResponse(success=False, execution_time=total_execution_time)
                time.sleep(1)
        print("Delivery robot is in position. Starting order preparation.")

    for i in range(count):
        drink_count = i + 1
        print(f"주문번호: {order_number}, {drink_count}번째 잔을 제조 시작합니다. 메뉴: {menu}, 옵션: {temperature}, 시럽: {'있음' if syrup else '없음'}")

        response = handle_water_or_milk(order_number, drink_count, menu)
        if not response.success:
            return response
        total_execution_time += response.execution_time

        response = handle_shot(order_number, drink_count, temperature)
        if not response.success:
            return response
        total_execution_time += response.execution_time

        if syrup == "시럽(O)":
            response = handle_syrup(order_number, drink_count, syrup)
            if not response.success:
                return response
            total_execution_time += response.execution_time

        response = handle_takeout_or_serving(order_number, order_type, drink_count)
        if not response.success:
            return response
        total_execution_time += response.execution_time

    if order_type == "테이크아웃":
        try:
            takeout_queue.get_nowait()
        except Empty:
            pass

    order_complete_msg = f"주문번호 {order_number} 완료"
    order_complete_pub.publish(order_complete_msg)
    print(order_complete_msg)

    return OrderPoseResponse(success=True, execution_time=total_execution_time)



if __name__ == "__main__":
    rospy.init_node('order_pose_server')
    moveit_commander.roscpp_initialize(sys.argv)
    robot = moveit_commander.RobotCommander()
    global arm_group
    arm_group = MoveGroupCommander("arm_group")

    s1 = rospy.Service('order_pose', OrderPose, handle_order_pose)
    s2 = rospy.Service('gripper_control', GripperControl, gripper_move)
    print("Ready to receive order poses.")
    rospy.spin()
