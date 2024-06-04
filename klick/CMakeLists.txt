cmake_minimum_required(VERSION 3.0.2)
project(klick)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  moveit_commander
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs
  message_generation
)

add_service_files(
  FILES
  OrderPose.srv
  GripperControl.srv
  CheckAgv.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
#  INCLUDE_DIRS include
  LIBRARIES klick
  CATKIN_DEPENDS rospy std_msgs moveit_commander geometry_msgs tf2_ros tf2_geometry_msgs
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  src/makecoffee.py
  src/get_pose.py
  src/set_pose.py
  src/set_value.py
  src/order_pose_server.py
  src/order_pose_client.py
  src/move_pose_gripper.py
  src/order_coffee.py
  src/slider.py
  src/order_server_without_gripper.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
