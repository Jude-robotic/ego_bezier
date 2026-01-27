/**
 * @file swarm_follower_fsm_node.cpp
 * @brief 集群从机FSM节点入口
 */

#include <ros/ros.h>
#include <plan_manage/swarm_follower_fsm.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "swarm_follower_fsm");
  // 使用相对命名空间以正确获取group namespace下的参数
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  
  ego_planner::SwarmFollowerFSM fsm;
  fsm.init(nh);  // 传入非私有NodeHandle
  
  ROS_INFO("[Follower] FSM node started");
  
  ros::spin();
  
  return 0;
}
