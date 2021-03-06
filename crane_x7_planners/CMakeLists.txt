cmake_minimum_required(VERSION 3.0.2)
project(crane_x7_planners)
set(CMAKE_CXX_STANDARD 11)

find_package(Eigen3 REQUIRED)
find_package(pinocchio REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  std_msgs
  crane_x7_force_controllers_msgs
)

catkin_package(
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS 
  roscpp 
  roslib
)

add_executable(
  crane_x7_differential_ik
  src/crane_x7_differential_ik.cpp
)
target_include_directories(
  crane_x7_differential_ik
  PUBLIC
  include/${PROJECT_NAME}
  ${EIGEN3_INCLUDE_DIR}
  ${PINOCCHIO_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${CRANE_X7_FORCE_CONTROLLERS_MSGS_INCLUDE_DIR}
)
target_link_libraries(
  crane_x7_differential_ik
  PUBLIC
  ${PINOCCHIO_LIBRARIES}
  ${catkin_LIBRARIES}
)

install(TARGETS crane_x7_differential_ik
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp"
  PATTERN ".svn" EXCLUDE
)

foreach(dir launch urdf)
  install(DIRECTORY ${dir}/ 
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir}
  )
endforeach(dir)