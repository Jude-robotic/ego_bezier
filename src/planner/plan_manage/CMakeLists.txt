cmake_minimum_required(VERSION 2.8.3)
project(ego_planner)

set(CMAKE_BUILD_TYPE "Release")
ADD_COMPILE_OPTIONS(-std=c++11 )
ADD_COMPILE_OPTIONS(-std=c++14 )
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  quadrotor_msgs
  plan_env
  path_searching
  bezier_opt
  traj_utils
  message_generation
  cv_bridge
)

# Generate messages in the 'msg' folder
add_message_files(
  FILES
  Bezier.msg
  DataDisp.msg
)

# Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES ego_planner
 CATKIN_DEPENDS plan_env path_searching bezier_opt traj_utils message_runtime
)

include_directories(
  include
  SYSTEM
  ${catkin_INCLUDE_DIRS} ${PROJECT_SOURCE_DIR}/include
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)

add_executable(ego_planner_node
  src/ego_planner_node.cpp 
  src/ego_replan_fsm.cpp
  src/planner_manager.cpp
  src/local_coordinate_system.cpp
)
target_link_libraries(ego_planner_node 
  ${catkin_LIBRARIES}
)
add_dependencies(ego_planner_node ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_executable(traj_server src/traj_server.cpp)
target_link_libraries(traj_server ${catkin_LIBRARIES})
add_dependencies(traj_server ${${PROJECT_NAME}_EXPORTED_TARGETS})

# Trajectory State Publisher
add_executable(trajectory_state_publisher src/trajectory_state_publisher.cpp)
target_link_libraries(trajectory_state_publisher ${catkin_LIBRARIES})
add_dependencies(trajectory_state_publisher ${${PROJECT_NAME}_EXPORTED_TARGETS})

# Local Map Visualizer
add_executable(local_map_visualizer src/local_map_visualizer.cpp)
target_link_libraries(local_map_visualizer ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_dependencies(local_map_visualizer ${${PROJECT_NAME}_EXPORTED_TARGETS})

# Enhanced Trajectory Visualizer
add_executable(trajectory_visualizer src/trajectory_visualizer.cpp)
target_link_libraries(trajectory_visualizer ${catkin_LIBRARIES} bezier_opt)
add_dependencies(trajectory_visualizer ${${PROJECT_NAME}_EXPORTED_TARGETS})

# Metrics Recorder Node
add_executable(metrics_recorder_node
  src/metrics_recorder_node.cpp
  src/planner_metrics.cpp
)
target_link_libraries(metrics_recorder_node ${catkin_LIBRARIES})
add_dependencies(metrics_recorder_node ${${PROJECT_NAME}_EXPORTED_TARGETS})

# Swarm Trajectory Generator Node (多机协同)
add_executable(swarm_traj_generator_node
  src/swarm_traj_generator_node.cpp
  src/swarm_trajectory_generator.cpp
)
target_link_libraries(swarm_traj_generator_node ${catkin_LIBRARIES})
add_dependencies(swarm_traj_generator_node ${${PROJECT_NAME}_EXPORTED_TARGETS})
