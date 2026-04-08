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
  std_srvs
  geometry_msgs
  nav_msgs
  visualization_msgs
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
  SwarmAgentState.msg
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

# Swarm Master Coordinator Node
add_executable(swarm_master_node
  src/swarm_master_node.cpp
  src/swarm_master_coordinator.cpp
)
target_link_libraries(swarm_master_node ${catkin_LIBRARIES} bezier_opt plan_env)
add_dependencies(swarm_master_node ${${PROJECT_NAME}_EXPORTED_TARGETS})

# Swarm agent state reporter (simulation/real-data compatible)
add_executable(swarm_agent_state_reporter_node
  src/swarm_agent_state_reporter_node.cpp
)
target_link_libraries(swarm_agent_state_reporter_node ${catkin_LIBRARIES})
add_dependencies(swarm_agent_state_reporter_node ${${PROJECT_NAME}_EXPORTED_TARGETS})

# Follower guidance injection + local refine node
add_executable(swarm_follower_guidance_optimizer_node
  src/swarm_follower_guidance_optimizer_node.cpp
)
target_link_libraries(swarm_follower_guidance_optimizer_node ${catkin_LIBRARIES} bezier_opt plan_env)
add_dependencies(swarm_follower_guidance_optimizer_node ${${PROJECT_NAME}_EXPORTED_TARGETS})

# Swarm preflight self-check node
add_executable(swarm_preflight_check_node
  src/swarm_preflight_check_node.cpp
)
target_link_libraries(swarm_preflight_check_node ${catkin_LIBRARIES})
add_dependencies(swarm_preflight_check_node ${${PROJECT_NAME}_EXPORTED_TARGETS})

# Swarm takeoff/arm gate node
add_executable(swarm_takeoff_gate_node
  src/swarm_takeoff_gate_node.cpp
)
target_link_libraries(swarm_takeoff_gate_node ${catkin_LIBRARIES})
add_dependencies(swarm_takeoff_gate_node ${${PROJECT_NAME}_EXPORTED_TARGETS})

# Swarm communication visualization node (master <-> followers)
add_executable(swarm_comm_visualizer_node
  src/swarm_comm_visualizer_node.cpp
)
target_link_libraries(swarm_comm_visualizer_node ${catkin_LIBRARIES})
add_dependencies(swarm_comm_visualizer_node ${${PROJECT_NAME}_EXPORTED_TARGETS})

# Swarm guidance trajectory visualization node (samples guidance bezier for RViz)
add_executable(swarm_guidance_traj_visualizer_node
  src/swarm_guidance_traj_visualizer_node.cpp
)
target_link_libraries(swarm_guidance_traj_visualizer_node ${catkin_LIBRARIES})
add_dependencies(swarm_guidance_traj_visualizer_node ${${PROJECT_NAME}_EXPORTED_TARGETS})

# Swarm payload visualization node (actual/planned payload center and ropes)
add_executable(swarm_payload_visualizer_node
  src/swarm_payload_visualizer_node.cpp
)
target_link_libraries(swarm_payload_visualizer_node ${catkin_LIBRARIES} bezier_opt)
add_dependencies(swarm_payload_visualizer_node ${${PROJECT_NAME}_EXPORTED_TARGETS})
