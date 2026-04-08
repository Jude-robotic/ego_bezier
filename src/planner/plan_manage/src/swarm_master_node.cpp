#include <plan_manage/swarm_master_coordinator.h>
#include <plan_env/grid_map.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "swarm_master_node");
  // 使用私有命名空间 (~) 即 /swarm_master_node，与 launch 中
  // ns="swarm_master_node" 加载的参数路径保持一致
  ros::NodeHandle nh("~");

  // 创建 GridMap 并接入主机传感器数据
  // 话题 remap 由 swarm_longhall.launch 完成：
  //   /grid_map/odom  → /visual_slam/odom
  //   /grid_map/cloud → /pcl_render_node/cloud
  // 参数由 swarm_master_grid_map.yaml 提供（grid_map/* 命名空间）
  GridMap::Ptr grid_map = std::make_shared<GridMap>();
  grid_map->initMap(nh);

  ego_planner::SwarmMasterCoordinator coordinator;
  coordinator.init(nh, grid_map);

  ros::spin();
  return 0;
}
