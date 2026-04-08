cmake_minimum_required(VERSION 2.8.3)
project(bezier_opt)

set(CMAKE_BUILD_TYPE "Release")
ADD_COMPILE_OPTIONS(-std=c++11 )
ADD_COMPILE_OPTIONS(-std=c++14 )
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  visualization_msgs
  plan_env
  cv_bridge
  path_searching
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES bezier_opt
 CATKIN_DEPENDS plan_env
)

include_directories( 
    SYSTEM 
    include 
    ${catkin_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIRS} 
    ${PCL_INCLUDE_DIRS}
)

add_library(bezier_opt 
    src/piecewise_bezier.cpp 
    src/bezier_optimizer.cpp 
    src/payload_geometry.cpp
    src/cooperative_payload_optimizer.cpp
    src/gradient_descent_optimizer.cpp
)
target_link_libraries(bezier_opt
    ${catkin_LIBRARIES} 
)

add_executable(test_bezier src/test_bezier.cpp)
target_link_libraries(test_bezier bezier_opt ${catkin_LIBRARIES})
