import sys
import rospy
import moveit_commander
from geometry_msgs.msg import PoseStamped
import math

def print_current_pose():
    rospy.init_node('print_current_pose_node', anonymous=True)
    moveit_commander.roscpp_initialize(sys.argv)
    robot = moveit_commander.RobotCommander()
    arm_group = moveit_commander.MoveGroupCommander("arm_group")

    current_joint_values = arm_group.get_current_joint_values()
    print("Current joint values:", current_joint_values)

    current_joint_values_degrees = [math.degrees(joint) for joint in current_joint_values]
    print("Current joint values (degrees):", current_joint_values_degrees)
    
    current_pose = arm_group.get_current_pose()
    print("Current pose:", current_pose.pose)

    moveit_commander.roscpp_shutdown()
    
if __name__ == "__main__":
    try:
        print_current_pose()
    except rospy.ROSInterruptException:
        pass