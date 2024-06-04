#!/usr/bin/env python
# -*- coding:utf-8 -*-
import math
import time
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Int32
from pymycobot import MyCobotSocket
from pymycobot.mycobot import MyCobot
from klick.srv import GripperControl, GripperControlRequest, GripperControlResponse

mc = None
state = True

def callback(data):
    global state, mc
    if state:
        data_list = []
        for index, value in enumerate(data.position[:6]):
            radians_to_angles = round(math.degrees(value), 2)
            data_list.append(radians_to_angles)
        rospy.loginfo("Data list: %s", data_list)
        mc.send_angles(data_list, 25)
        joint_positions = mc.get_angles()
        rospy.loginfo("Joint positions: %s", joint_positions)
        
def state_callback(data):
    global state
    state = data.data 
    rospy.loginfo("Move state changed: %s", "Start" if state else "Stop")
    
def gripper_callback(request):
    global state, mc
    mc.set_gripper_mode(0)
    mc.init_eletric_gripper()
    mc.set_eletric_gripper(0)
    mc.set_gripper_value(request.val, 20)
    time.sleep(5)
    # while True:
    #     gripper_value = mc.get_gripper_value()
    #     if gripper_value == request.val:
    #         break
    rospy.loginfo("gripper value is %d", request.val)
    state = True
    return GripperControlResponse(success=True)

def listener():
    global mc
    # ip = "192.168.168.72"
    # port = 9000
    # print(ip, port)
    # mc = MyCobotSocket(ip, port)
    port = rospy.get_param("~port", "/dev/ttyACM0")
    baud = rospy.get_param("~baud", 115200)
    print(port, baud)
    mc = MyCobot(port, baud)
    rospy.init_node("control_slider", anonymous=True)
    rospy.Subscriber("joint_states", JointState, callback)
    rospy.Subscriber("state_check", Int32 , state_callback)
    rospy.Service("gripper_service", GripperControl, gripper_callback)  
    time.sleep(0.02)
    mc.set_fresh_mode(1)
    time.sleep(0.03)
    print("spin ...")
    rospy.spin()

if __name__ == "__main__":
    listener()
