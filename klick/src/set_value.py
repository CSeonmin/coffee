import rospy
import sys
import math
import moveit_commander
from moveit_msgs.msg import PlanningScene, AllowedCollisionEntry, AllowedCollisionMatrix

def degrees_to_radians(degrees):
    return [angle * (math.pi / 180.0) for angle in degrees]

def move_to_jointpose():
    rospy.init_node('move_joint_pose_node', anonymous=True)
    print("move_to_jointpose start")
    
    moveit_commander.roscpp_initialize(sys.argv)
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    
    arm_group = moveit_commander.MoveGroupCommander("arm_group")
    angles_in_degrees =    [-104.11810708169162, 44.63338479059295, 67.01128181217638, -109.47459733033766, -14.80005673328852, -8.484014421577813]






    radian_pose = degrees_to_radians(angles_in_degrees)
    print("Target joint pose in radians:", radian_pose)

    arm_group.set_joint_value_target(radian_pose)
    
    arm_group.go(wait=True)
    
    arm_group.stop()
    arm_group.clear_pose_targets()
    

if __name__ == '__main__':
    try:
        move_to_jointpose()
    except rospy.ROSInterruptException:
        pass