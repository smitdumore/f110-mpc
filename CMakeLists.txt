cmake_minimum_required(VERSION 3.0.2)
project(f110-mpc)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  visualization_msgs
  tf2_geometry_msgs
  tf2_ros
  roslib
)

set(CMAKE_PREFIX_PATH /usr/local/lib/cmake)
find_package(Eigen3 REQUIRED)
find_package(OsqpEigen REQUIRED)


catkin_package(
  #INCLUDE_DIRS include
  #LIBRARIES f110-mpc
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(mpc_node 
  #src/mpc.cpp
  src/input.cpp
  src/state.cpp
  src/constraints.cpp  
  src/project.cpp
  src/node.cpp
  src/occupancy_grid.cpp
  src/transforms.cpp
  src/trajectory.cpp
  src/model.cpp
  src/visualizer.cpp
)

add_dependencies(mpc_node ${f110_mpc_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(mpc_node 
  ${catkin_LIBRARIES}
  OsqpEigen::OsqpEigen osqp::osqp
  stdc++fs
)
