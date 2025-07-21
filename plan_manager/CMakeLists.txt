cmake_minimum_required(VERSION 3.0.2)

project(plan_manager)

# SET(CMAKE_BUILD_TYPE "Debug")
# SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g -ggdb")
# SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")
find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  roscpp
  std_msgs
  visualization_msgs
  plan_env
  cv_bridge
  front_end
  back_end
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES plan_manager
 CATKIN_DEPENDS nav_msgs roscpp std_msgs visualization_msgs plan_env front_end back_end
)

include_directories(
  SYSTEM
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS} 
  ${PROJECT_SOURCE_DIR}/include
)

add_library(plan_manager 
    src/plan_manager.cpp
)

target_link_libraries(plan_manager
    ${catkin_LIBRARIES} 
)

add_library(replan_fsm 
    src/replan_fsm.cpp
)

target_link_libraries(replan_fsm
    ${catkin_LIBRARIES} 
    plan_manager
)


add_executable(manager_node src/manager_node.cpp )

target_link_libraries(manager_node
    plan_manager
    replan_fsm
    ${catkin_LIBRARIES} 
)
