#!/usr/bin/env python
import sys
import rospy
from klick.srv import OrderPose, OrderPoseRequest

def order_pose_client(order_number, drink_type, temperature, syrup):
    rospy.wait_for_service('order_pose')
    try:
        order_pose = rospy.ServiceProxy('order_pose', OrderPose)
        
        request = OrderPoseRequest(order_number=order_number, drink_type=drink_type, temperature=temperature, syrup=syrup)
        response = order_pose(request)
        
        if response.success:
            print(f"Movement successful! Total execution time: {response.execution_time:.4f} seconds")
        else:
            print("Movement failed!")

    except rospy.ServiceException as e:
        print(f"Service call failed: {e}")

if __name__ == "__main__":
    rospy.init_node('order_pose_client')
    if len(sys.argv) != 5:
        print("Usage: order_pose_client.py order_number [americano|latte] [ice|hot] [syrup|no_syrup]")
        sys.exit(1)
    order_number = int(sys.argv[1])
    drink_type = sys.argv[2]
    temperature = sys.argv[3]
    syrup = sys.argv[4].lower() == 'syrup'
    order_pose_client(order_number, drink_type, temperature, syrup)
