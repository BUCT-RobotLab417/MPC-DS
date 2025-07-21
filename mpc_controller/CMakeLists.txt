cmake_minimum_required(VERSION 3.0.2)
project(mpc_controller)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(Eigen3 REQUIRED)
find_package(PCL 1.8 REQUIRED)
find_package(casadi REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  nav_msgs
)

catkin_package(
 INCLUDE_DIRS include
#  LIBRARIES mpc_controller
 CATKIN_DEPENDS roscpp rospy std_msgs nav_msgs
#  DEPENDS system_lib
)

include_directories(
  SYSTEM 
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS} 
  ${PCL_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/include
)

add_library(mpc_controller
  src/mpc_controller.cpp
)

target_link_libraries(mpc_controller
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${EIGEN3_LIBRARIES}
  casadi
)

add_executable(mpc_node
  src/mpc_node.cpp
)

target_link_libraries(mpc_node
  mpc_controller
  ${catkin_LIBRARIES}
)

