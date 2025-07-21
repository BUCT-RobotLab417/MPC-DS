cmake_minimum_required(VERSION 3.0.2)
project(back_end)

find_package(NLopt REQUIRED)
set(NLopt_INCLUDE_DIRS ${NLOPT_INCLUDE_DIR})

find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)



find_package(catkin REQUIRED COMPONENTS
  front_end
  plan_env
  roscpp
  std_msgs
  visualization_msgs
  cv_bridge
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES back_end
  CATKIN_DEPENDS roscpp std_msgs plan_env visualization_msgs front_end
)


include_directories(
  SYSTEM 
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS} 
  ${NLOPT_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/include
)

set(CMAKE_CXX_FLAGS "-std=c++14 ${CMAKE_CXX_FLAGS} -O3 -Wall")

add_library(back_end 
    src/bspline_opt.cpp
)

target_link_libraries(back_end
    ${catkin_LIBRARIES} 
    ${NLOPT_LIBRARIES}
)

add_executable(back_end_node src/back_end_node.cpp )

target_link_libraries(back_end_node
    back_end
    ${catkin_LIBRARIES} 
    ${NLOPT_LIBRARIES}
)
