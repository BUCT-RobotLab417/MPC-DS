cmake_minimum_required(VERSION 3.0.2)
project(front_end)

find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  plan_env
  roscpp
  std_msgs
  visualization_msgs
  cv_bridge
)
find_package(Eigen3 REQUIRED)
find_package(ompl REQUIRED)


catkin_package(
 INCLUDE_DIRS include
 LIBRARIES front_end
 CATKIN_DEPENDS plan_env roscpp std_msgs visualization_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS} 
  ${PCL_INCLUDE_DIRS}
  ${OMPL_INCLUDE_DIRS}

)

add_library(front_end src/theta_astar.cpp)

target_link_libraries( front_end
    ${catkin_LIBRARIES} 
    ${OMPL_LIBRARIES}
)

add_executable(front_end_node src/front_end_node.cpp )

target_link_libraries(front_end_node
    front_end
    ${catkin_LIBRARIES} 
    ${OMPL_LIBRARIES}
)

