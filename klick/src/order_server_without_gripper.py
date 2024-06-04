#!/usr/bin/env python
import sys
import rospy
import moveit_commander
from moveit_commander import MoveGroupCommander
from geometry_msgs.msg import Pose, Point, Quaternion
from std_msgs.msg import Int32
from klick.srv import OrderPose, OrderPoseResponse, GripperControl, GripperControlRequest, GripperControlResponse, CheckAgv, CheckAgvRequest
import numpy as np
import time, math
from queue import Queue, Empty

# gripper_service = None
# gripper_pub = rospy.Publisher("state_check", Int32, queue_size=10)
takeout_queue = Queue()
agv_move_pub = rospy.Publisher('agv_move_command', Int32, queue_size=10)

poses = {
    "ice_cup": Pose( # ice cup front
        position=Point(x=0.0220, y=0.4062, z=0.2577),
        orientation=Quaternion(x=-0.6918, y=-0.1272, z=0.0191, w=0.7106)
    ),
    "hot_cup": Pose( # hot cup front
        position=Point(x=-0.0933, y=0.3609, z=0.2903),
        orientation=Quaternion(x=-0.6842, y=-0.0497, z=-0.0071, w=0.7275)
    ),
    "syrup": Pose( # syrup over
        position=Point(x=-0.2872, y=0.2568, z=0.4577),
        orientation=Quaternion(x=-0.5851, y=-0.3651, z=0.3544, w=0.6315)
    ),
    "cup_place": Pose( # cup place over
        position=Point(x=-0.2524, y=0.2529, z=0.2611),
        orientation=Quaternion(x=-0.5226, y=-0.4767, z=0.4451, w=0.5491)
    ),
    "shot_cup": Pose( # shot cup front
        position=Point(x=-0.1931, y=0.3240, z=0.3754),
        orientation=Quaternion(x=-0.5875, y=-0.3055, z=0.2524, w=0.7055)
    ),
    "shot_pour": Pose( # shot pour
        position=Point(x=-0.2094, y=0.2831, z=0.3311),
        orientation=Quaternion(x=-0.6142, y=-0.3508, z=0.3390, w=0.6203)
    ),
    "water_cup_place": Pose( # water cup place over
        position=Point(x=-0.3675, y=0.1166, z=0.2992),
        orientation=Quaternion(x=-0.4743, y=-0.5253, z=0.4884, w=0.5104)
    ),
    "milk_cup_place": Pose( # milk cup place over
        position=Point(x=-0.3605, y=0.0120, z=0.3271),
        orientation=Quaternion(x=-0.6438, y=-0.2711, z=0.2818, w=0.6578)
    ),
    "water_dispenser": Pose( # water dispenser front
        position=Point(x=-0.2812, y=0.1152, z=0.4670),
        orientation=Quaternion(x=-0.4927, y=-0.5096, z=0.5035, w=0.4940)
    ),
    "milk_dispenser": Pose( # milk dospenser front
        position=Point(x=-0.2812, y=0.2152, z=0.4670),
        orientation=Quaternion(x=-0.4927, y=-0.5096, z=0.5035, w=0.4940)
    ),
    "takeout_place": Pose( # takeout place over
        position=Point(x=0.0, y=0.0, z=0.0),
        orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=0.0)
    ),
    "serving_place_first": Pose( # serving place 1
        position=Point(x=0.0, y=0.0, z=0.0),
        orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=0.0)
    ),
    "serving_place_second": Pose( # serving place 2
        position=Point(x=0.0, y=0.0, z=0.0),
        orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=0.0)
    ),
    "init_pose": Pose( # basic pose
        position=Point(x=0.0, y=0.0, z=0.0),
        orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=0.0)
    )
}

def degrees_to_radians(degrees):
    return [angle * (math.pi / 180.0) for angle in degrees]

def calculate_joint_movement(plan):
    joint_movements = np.zeros(len(plan.joint_trajectory.joint_names))
    for i in range(1, len(plan.joint_trajectory.points)):
        for j in range(len(plan.joint_trajectory.joint_names)):
            joint_movements[j] += abs(plan.joint_trajectory.points[i].positions[j] - plan.joint_trajectory.points[i-1].positions[j])
    return sum(joint_movements)

# def gripper_move(val):
#     global gripper_service
#     gripper_pub.publish(0)
#     try:
#         print("Waiting for gripper_service...")
#         rospy.wait_for_service('gripper_service', timeout=10)
#         print("gripper_service available.")
#         gripper_service = rospy.ServiceProxy('gripper_service', GripperControl)
#         request = GripperControlRequest(val=val)
#         response = gripper_service(request)
#         if response.success:
#             print(f"Gripper moved to {val}")
#         else:
#             print("Gripper service call failed")
#     except rospy.ServiceException as e:
#         print(f"Gripper service call failed: {e}")
#     except rospy.ROSException as e:
#         print(f"Service call timeout: {e}")

def move_to_pose(pose_goal):
    arm_group.set_pose_target(pose_goal)
    
    # Optional: Set different planner
    arm_group.set_planner_id("RRTstar")
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


# def parse_order_details(order_details):
#     details = order_details.split(", ")
#     order_info = {}
#     for detail in details:
#         key, value = detail.split(": ")
#         order_info[key] = value
#     return order_info

def handle_cup_and_shot(order_number, drink_count, temperature, poses):
    cup_pose = poses["ice_cup"] if temperature == "아이스" else poses["hot_cup"]
    cup_place_pose = poses["cup_place"]
    shot_cup_pose = poses["shot_cup"]
    shot_pour_pose = poses["shot_pour"]
    print(f"주문번호: {order_number}, {drink_count}번째 잔, {temperature} 음료를 제조 중입니다.")

    print("Attempting to open gripper")
    # gripper_move(95)
    print("Gripper opened")

    # Move to cup (ice or hot)
    response = move_to_pose(cup_pose) # front of cup
    if not response.success:
        return response
    
    cup_grab_pose = Pose( 
        position=Point(x=cup_pose.position.x - 0.0136, y=cup_pose.position.y, z=cup_pose.position.z),
        orientation=cup_pose.orientation
    )
    response = move_to_pose(cup_grab_pose) # grab cup
    if not response.success:
        return response
    
    # gripper_move(65)

    cup_lift_pose = Pose( 
        position=Point(x=cup_pose.position.x - 0.0136, y=cup_pose.position.y, z=cup_pose.position.z + 0.0668),
        orientation=cup_pose.orientation
    )
    response = move_to_pose(cup_lift_pose) # lift cup
    if not response.success:
        return response
    
    # Move to cup place
    response = move_to_pose(cup_place_pose) # over cup place
    if not response.success:
        return response
    
    cup_put_pose = Pose( 
        position=Point(x=cup_place_pose.position.x, y=cup_place_pose.position.y, z=cup_place_pose.position.z - 0.04),
        orientation=cup_place_pose.orientation
    )
    response = move_to_pose(cup_put_pose) # put down the cup on cup place
    if not response.success:
        return response

    # gripper_move(90)

    cup_out_pose = Pose( 
        position=Point(x=cup_place_pose.position.x + 0.07, y=cup_place_pose.position.y, z=cup_place_pose.position.z - 0.04),
        orientation=cup_place_pose.orientation
    )
    response = move_to_pose(cup_out_pose) # let go of the cup place
    if not response.success:
        return response
    
    # gripper_move(65)

    # Move to shot cup
    response = move_to_pose(shot_cup_pose) # front of shot cup
    if not response.success:
        return response
    
    grab_shot_pose = Pose( 
        position=Point(x=shot_cup_pose.position.x -0.0447, y=shot_cup_pose.position.y, z=shot_cup_pose.position.z),
        orientation=shot_cup_pose.orientation
    )
    response = move_to_pose(grab_shot_pose) # grab shot cup
    if not response.success:
        return response

    # gripper_move(35)

    lift_shot_pose = Pose( 
        position=Point(x=shot_cup_pose.position.x -0.0447, y=shot_cup_pose.position.y, z=shot_cup_pose.position.z + 0.0867),
        orientation=shot_cup_pose.orientation
    )
    response = move_to_pose(lift_shot_pose) # lift shot cup
    if not response.success:
        return response
    
    # Move to shot pour
    response = move_to_pose(shot_pour_pose) # pour shot cup
    if not response.success:
        return response
    
    tip_shot_pose = Pose( 
        position=shot_pour_pose.position,
        orientation=Quaternion(x=-0.0109, y=-0.7073, z=-0.3612, w=0.6076)
    )
    response = move_to_pose(tip_shot_pose) # tip shot cup
    if not response.success:
        return response
    time.sleep(1)

    # Move back to shot cup
    response = move_to_pose(shot_pour_pose)
    if not response.success:
        return response
    
    response = move_to_pose(lift_shot_pose) 
    if not response.success:
        return response
    
    response = move_to_pose(grab_shot_pose) # put down shot cup
    if not response.success:
        return response
    
    # gripper_move(65)

    response = move_to_pose(shot_cup_pose) # front of shot cup
    if not response.success:
        return response
    
    midlle_shot_syrup_pose = Pose( 
        position=Point(x=shot_cup_pose.position.x, y=shot_cup_pose.position.y, z=shot_cup_pose.position.z + 0.09),
        orientation=shot_cup_pose.orientation
    )
    response = move_to_pose(midlle_shot_syrup_pose) 
    if not response.success:
        return response

    return OrderPoseResponse(success=True, execution_time=response.execution_time)

def handle_syrup(order_number, drink_count, poses):
    print(f"주문번호: {order_number}, {drink_count}번째 잔에 시럽을 추가 중입니다.")
    syrup_pose = poses["syrup"]

    # gripper_move(50)

    response = move_to_pose(syrup_pose) # over the syrup bottle
    if not response.success:
        return response
    
    press_syrup_pose = Pose( 
        position=Point(x=syrup_pose.position.x, y=syrup_pose.position.y, z=syrup_pose.position.z - 0.1013),
        orientation=syrup_pose.orientation
    )
    response = move_to_pose(press_syrup_pose) # press syrup bottle
    if not response.success:
        return response
    time.sleep(1)

    response = move_to_pose(syrup_pose) # over the syrup bottle
    if not response.success:
        return response

    return OrderPoseResponse(success=True, execution_time=0.0)

def handle_water_or_milk(order_number, drink_count, menu, poses):
    print(f"주문번호: {order_number}, {drink_count}번째 잔, {menu} 음료를 준비 중입니다.")
    cup_place_pose = poses["cup_place"]
    if menu == "아메리카노":
        dispenser_pose = poses["water_dispenser"]
        cup_type_place = poses["water_cup_place"]
    else:
        dispenser_pose = poses["milk_dispenser"]
        cup_type_place = poses["milk_cup_place"]

    # Move to cup place
    response = move_to_pose(cup_place_pose) # over cup place
    if not response.success:
        return response
    
    # gripper_move(90)

    front_cup_pose = Pose( 
        position=Point(x=cup_place_pose.position.x + 0.07, y=cup_place_pose.position.y, z=cup_place_pose.position.z - 0.04),
        orientation=cup_place_pose.orientation
    )
    response = move_to_pose(front_cup_pose) # front of cup place
    if not response.success:
        return response
    
    grab_cup_pose = Pose( 
        position=Point(x=cup_place_pose.position.x, y=cup_place_pose.position.y, z=cup_place_pose.position.z - 0.04),
        orientation=cup_place_pose.orientation
    )
    response = move_to_pose(grab_cup_pose) # grab the cup
    if not response.success:
        return response
    
    # gripper_move(65)
    
    response = move_to_pose(cup_place_pose) # over cup place
    if not response.success:
        return response
    
    # Move to specific cup place (water or milk)
    response = move_to_pose(cup_type_place) # over the cup place (below the dispenser)
    if not response.success:
        return response
    
    put_cup_pose = Pose( 
        position=Point(x=cup_type_place.position.x, y=cup_type_place.position.y, z=cup_type_place.position.z - 0.0355),
        orientation=cup_type_place.orientation
    )
    response = move_to_pose(put_cup_pose) # put down the cup
    if not response.success:
        return response

    # gripper_move(90)

    out_cup_pose = Pose( 
        position=Point(x=cup_type_place.position.x + 0.6417, y=cup_type_place.position.y, z=cup_type_place.position.z - 0.0355),
        orientation=cup_type_place.orientation
    )
    response = move_to_pose(out_cup_pose) # let go of the cup place
    if not response.success:
        return response

    raise_cup_pose = Pose( 
        position=Point(x=cup_type_place.position.x + 0.6417, y=cup_type_place.position.y, z=cup_type_place.position.z + 0.1323),
        orientation=cup_type_place.orientation
    )
    response = move_to_pose(raise_cup_pose) # raise the cup
    if not response.success:
        return response

    # Move to dispenser
    response = move_to_pose(dispenser_pose) # front of dispenser
    if not response.success:
        return response
    
    push_dispenser_pose = Pose( 
        position=Point(x=dispenser_pose.position.x - 0.0502, y=dispenser_pose.position.y, z=dispenser_pose.position.z),
        orientation=dispenser_pose.orientation
    )
    response = move_to_pose(push_dispenser_pose) # push the dispenser
    if not response.success:
        return response
    time.sleep(8)
    
    response = move_to_pose(dispenser_pose) # front of dispenser
    if not response.success:
        return response
    
    response = move_to_pose(raise_cup_pose) 
    if not response.success:
        return response
    
    response = move_to_pose(out_cup_pose) 
    if not response.success:
        return response
    
    response = move_to_pose(put_cup_pose) 
    if not response.success:
        return response

    # gripper_move(65)

    response = move_to_pose(cup_type_place) # over the cup place (below the dispenser)
    if not response.success:
        return response

    return OrderPoseResponse(success=True, execution_time=response.execution_time)

def handle_takeout_or_serving(order_number, takeout_serving, drink_count, poses):
    takeout_pose = poses["takeout_place"]
    serving_first_pose = poses["serving_place_first"]
    serving_second_pose = poses["serving_place_second"]
    basic_pose = poses["init_pose"]

    if takeout_serving == "테이크아웃":
        print(f"주문번호: {order_number}, {drink_count}번째 잔을 테이크아웃 위치로 이동 중입니다.")
        response = move_to_pose(takeout_pose) # over the takeout place
        if not response.success:
            return response 
        
        put_takeout_pose = Pose( 
            position=Point(x=takeout_pose.position.x, y=takeout_pose.position.y, z=takeout_pose.position.z - 0.0355),
            orientation=takeout_pose.orientation
        )
        response = move_to_pose(put_takeout_pose) # put down the cup on takeout place
        if not response.success:
            return response
        
        # gripper_move(90)

        out_takeout_pose = Pose( 
            position=Point(x=takeout_pose.position.x + 0.07, y=takeout_pose.position.y, z=takeout_pose.position.z - 0.0355),
            orientation=takeout_pose.orientation
        )
        response = move_to_pose(out_takeout_pose)
        if not response.success:
            return response
        
        raise_takeout_pose = Pose( 
            position=Point(x=takeout_pose.position.x + 0.07, y=takeout_pose.position.y, z=takeout_pose.position.z + 0.05),
            orientation=takeout_pose.orientation
        )
        response = move_to_pose(raise_takeout_pose)
        if not response.success:
            return response

    elif takeout_serving == "배달":
        if drink_count == 1:
            print(f"주문번호: {order_number}, 첫 번째 잔을 배달 위치 1로 이동 중입니다.")
            response = move_to_pose(serving_first_pose)
            if not response.success:
                return response
            
            put_serving_pose1 = Pose( 
                position=Point(x=serving_first_pose.position.x, y=serving_first_pose.position.y, z=serving_first_pose.position.z - 0.0355),
                orientation=serving_first_pose.orientation
            )
            response = move_to_pose(put_serving_pose1) # put down the cup on takeout place
            if not response.success:
                return response
            
            # gripper_move(90)

            out_serving_pose1 = Pose( 
                position=Point(x=serving_first_pose.position.x + 0.07, y=serving_first_pose.position.y, z=serving_first_pose.position.z - 0.0355),
                orientation=serving_first_pose.orientation
            )
            response = move_to_pose(out_serving_pose1)
            if not response.success:
                return response
            
            raise_serving_pose1 = Pose( 
                position=Point(x=serving_first_pose.position.x + 0.07, y=serving_first_pose.position.y, z=serving_first_pose.position.z + 0.05),
                orientation=serving_first_pose.orientation
            )
            response = move_to_pose(raise_serving_pose1)
            if not response.success:
                return response

        elif drink_count == 2:
            print(f"주문번호: {order_number}, 두 번째 잔을 배달 위치 2로 이동 중입니다.")
            response = move_to_pose(serving_second_pose)
            if not response.success:
                return response
            
            put_serving_pose2 = Pose( 
                position=Point(x=serving_second_pose.position.x, y=serving_second_pose.position.y, z=serving_second_pose.position.z - 0.0355),
                orientation=serving_second_pose.orientation
            )
            response = move_to_pose(put_serving_pose2) # put down the cup on takeout place
            if not response.success:
                return response
            
            # gripper_move(90)

            out_serving_pose2 = Pose( 
                position=Point(x=serving_second_pose.position.x + 0.07, y=serving_second_pose.position.y, z=serving_second_pose.position.z - 0.0355),
                orientation=serving_second_pose.orientation
            )
            response = move_to_pose(out_serving_pose2)
            if not response.success:
                return response
            
            raise_serving_pose2 = Pose( 
                position=Point(x=serving_second_pose.position.x + 0.07, y=serving_second_pose.position.y, z=serving_second_pose.position.z + 0.05),
                orientation=serving_second_pose.orientation
            )
            response = move_to_pose(raise_serving_pose2)
            if not response.success:
                return response

    response = move_to_pose(basic_pose)
    if not response.success:
        return response

    return OrderPoseResponse(success=True, execution_time=response.execution_time)

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

        # Handle cup and shot sequence
        response = handle_cup_and_shot(order_number, drink_count, temperature, poses)
        if not response.success:
            return response
        total_execution_time += response.execution_time

        # Handle syrup if present
        if syrup:
            response = handle_syrup(order_number, drink_count, poses)
            if not response.success:
                return response
            total_execution_time += response.execution_time

        # Handle water or milk sequence
        response = handle_water_or_milk(order_number, drink_count, menu, poses)
        if not response.success:
            return response
        total_execution_time += response.execution_time

        # Handle takeout or serving
        response = handle_takeout_or_serving(order_number, order_type, drink_count, poses)
        if not response.success:
            return response
        total_execution_time += response.execution_time

    if order_type == "테이크아웃":
        try:
            takeout_queue.get_nowait()
        except Empty:
            pass
        
    return OrderPoseResponse(success=True, execution_time=total_execution_time)



if __name__ == "__main__":
    rospy.init_node('order_pose_server')
    moveit_commander.roscpp_initialize(sys.argv)
    robot = moveit_commander.RobotCommander()
    global arm_group
    arm_group = MoveGroupCommander("arm_group")

    s1 = rospy.Service('order_pose', OrderPose, handle_order_pose)
    # s2 = rospy.Service('gripper_control', GripperControl, gripper_move)
    print("Ready to receive order poses.")
    rospy.spin()
