#!/usr/bin/env python
import rospy
from klick.srv import GripperControl, GripperControlResponse

def handle_gripper_control(req):
    val = req.val
    rospy.loginfo("Gripper moving to: %d", val)
    
    mc.set_gripper_mode(0)
    mc.set_eletric_gripper(0)
    mc.set_gripper_value(val, 10) 
    
    while True:
        gripper_value = mc.get_gripper_value()
        if gripper_value == val:
            break
    rospy.loginfo("Gripper moving to: %d", val)
    success = True  
    return GripperControlResponse(success=success)

def gripper_control_server():
    rospy.init_node('gripper_control_server')
    s = rospy.Service('gripper_service', GripperControl, handle_gripper_control)
    print("Ready to control gripper.")
    rospy.spin()

if __name__ == "__main__":
    gripper_control_server()
