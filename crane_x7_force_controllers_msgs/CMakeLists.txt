cmake_minimum_required(VERSION 2.8.3)
project(crane_x7_force_controllers_msgs)

find_package(catkin REQUIRED COMPONENTS
  roscpp 
  rospy
  actionlib_msgs
  std_msgs
  message_generation
)

add_message_files(
  FILES
  GoalPosition.msg
)

add_service_files(
  FILES
  SetJointStiffness.srv
  SetEndEffectorStiffness.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  # LIBRARIES ${PROJECT_NAME}
  # CATKIN_DEPENDS 
  # message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
