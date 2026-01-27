/**
 * @file swarm_master_publisher_node.cpp
 * @brief 集群主机轨迹发布节点入口
 */

#include <ros/ros.h>
#include <plan_manage/swarm_master_publisher.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "swarm_master_publisher");
  ros::NodeHandle nh;
  
  ego_planner::SwarmMasterPublisher publisher;
  publisher.init(nh);
  
  ROS_INFO("[SwarmMaster] Publisher node started");
  
  ros::spin();
  
  return 0;
}
