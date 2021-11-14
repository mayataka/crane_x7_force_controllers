cmake_minimum_required(VERSION 2.8.3)
project(crane_x7_admittance_controller)
set(CMAKE_CXX_STANDARD 11)

find_package(Eigen3 REQUIRED)
find_package(pinocchio REQUIRED)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  controller_interface
  hardware_interface
  crane_x7_force_controllers_msgs
)

catkin_package(
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS 
  roscpp 
  roslib
  controller_interface 
)

add_library(
  ${PROJECT_NAME} 
  src/crane_x7_admittance_controller.cpp
)
target_include_directories(
  ${PROJECT_NAME} 
  PRIVATE
  include/${PROJECT_NAME}
  ${EIGEN3_INCLUDE_DIR}
  ${PINOCCHIO_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${CRANE_X7_FORCE_CONTROLLERS_MSGS_INCLUDE_DIR}
)
target_link_libraries(
  ${PROJECT_NAME} 
  PUBLIC
  ${PINOCCHIO_LIBRARIES}
  ${catkin_LIBRARIES}
)
if(catkin_EXPORTED_LIBRARIES)
  add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_LIBRARIES})
endif()

foreach(dir config urdf)
  install(DIRECTORY ${dir}/ 
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir}
  )
endforeach(dir)