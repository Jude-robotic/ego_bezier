#include <plan_manage/swarm_master_coordinator.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "swarm_master_node");
  ros::NodeHandle nh;

  ego_planner::SwarmMasterCoordinator coordinator;
  coordinator.init(nh);

  ros::spin();
  return 0;
}
