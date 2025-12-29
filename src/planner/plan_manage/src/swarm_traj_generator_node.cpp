/**
 * @file swarm_traj_generator_node.cpp
 * @brief 集群轨迹生成器节点 - 多机协同系统的核心节点
 * 
 * 功能：
 * 1. 订阅主机(uav_0)的轨迹
 * 2. 调用集群算法生成从机轨迹
 * 3. 发布每个从机的独立轨迹到对应topic
 */

#include <ros/ros.h>
#include <plan_manage/swarm_trajectory_generator.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "swarm_trajectory_generator");
  ros::NodeHandle nh("~");

  ego_planner::SwarmTrajectoryGenerator swarm_generator(nh);
  
  swarm_generator.init();

  ROS_INFO("\033[1;32m=====> Swarm Trajectory Generator Started.\033[0m");

  ros::spin();

  return 0;
}
