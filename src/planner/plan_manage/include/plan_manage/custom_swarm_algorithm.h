/**
 * @file custom_swarm_algorithm.h
 * @brief 自定义集群算法模板
 * @author 您的名字
 * 
 * 这个文件提供了一个模板，用于实现自定义的集群算法。
 * 请复制这个文件并实现您自己的集群算法。
 * 
 * 使用步骤：
 * 1. 复制此文件并重命名
 * 2. 实现 computeFollowerTrajectories() 方法
 * 3. 在 swarm_traj_generator_node.cpp 中实例化并设置您的算法
 * 4. 重新编译并运行
 */

#ifndef _CUSTOM_SWARM_ALGORITHM_H_
#define _CUSTOM_SWARM_ALGORITHM_H_

#include <plan_manage/swarm_trajectory_generator.h>

namespace ego_planner
{

/**
 * @brief 自定义集群算法类（模板）
 * 
 * 继承自 SwarmAlgorithmBase，实现您自己的集群算法逻辑。
 */
class CustomSwarmAlgorithm : public SwarmAlgorithmBase
{
private:
  // ========== 在这里定义您的算法参数 ==========
  
  // 示例参数：
  double my_parameter_1_;
  double my_parameter_2_;
  int my_parameter_3_;
  
  // 您可以添加任何需要的成员变量
  
  // ========== 在这里定义辅助函数 ==========
  
  /**
   * @brief 示例辅助函数
   */
  Eigen::Vector3d computeFormationOffset(int follower_index, int num_followers)
  {
    // 实现您的偏移计算逻辑
    Eigen::Vector3d offset;
    // ...
    return offset;
  }

public:
  /**
   * @brief 构造函数 - 初始化算法参数
   */
  CustomSwarmAlgorithm(double param1 = 1.0, double param2 = 2.0, int param3 = 3)
      : my_parameter_1_(param1), my_parameter_2_(param2), my_parameter_3_(param3)
  {
    ROS_INFO("[CustomSwarmAlgorithm] Initialized with params: %.2f, %.2f, %d",
             param1, param2, param3);
  }

  virtual ~CustomSwarmAlgorithm() {}

  /**
   * @brief 计算从机轨迹的核心算法（需要实现）
   * 
   * @param leader_trajectory 主机位置轨迹点序列
   * @param leader_velocities 主机速度轨迹点序列
   * @param leader_accelerations 主机加速度轨迹点序列
   * @param num_followers 从机数量
   * @param time_stamps 轨迹点的时间戳
   * @return 每个从机的轨迹点序列
   * 
   * 返回格式说明：
   * - 返回值是一个二维vector
   * - follower_trajectories[i] 是第i个从机的轨迹
   * - follower_trajectories[i][j] 是第i个从机在第j个时间点的位置
   * - 确保返回的从机数量与 num_followers 一致
   * - 确保每个从机的轨迹点数量与主机一致
   */
  std::vector<std::vector<Eigen::Vector3d>> computeFollowerTrajectories(
      const std::vector<Eigen::Vector3d> &leader_trajectory,
      const std::vector<Eigen::Vector3d> &leader_velocities,
      const std::vector<Eigen::Vector3d> &leader_accelerations,
      int num_followers,
      const std::vector<double> &time_stamps) override
  {
    // ========== 开始实现您的集群算法 ==========
    
    std::vector<std::vector<Eigen::Vector3d>> follower_trajectories;
    follower_trajectories.resize(num_followers);

    // 基本检查
    if (leader_trajectory.empty())
    {
      ROS_WARN("[CustomSwarmAlgorithm] Leader trajectory is empty!");
      return follower_trajectories;
    }

    int traj_length = leader_trajectory.size();
    ROS_INFO("[CustomSwarmAlgorithm] Computing trajectories for %d followers, %d points",
             num_followers, traj_length);

    // ========== 在这里实现您的算法逻辑 ==========
    
    /* 算法设计建议：
     * 
     * 1. 编队形状选择：
     *    - 圆形编队：从机在主机周围等角度分布
     *    - V字编队：从机排列成V字形
     *    - 队列编队：从机排成一排跟随
     *    - 自适应编队：根据环境动态调整
     * 
     * 2. 考虑因素：
     *    - 主机速度方向：让编队朝向与速度一致
     *    - 障碍物规避：从机避开障碍物
     *    - 保持距离：维持与主机的安全距离
     *    - 平滑轨迹：避免突然转向
     * 
     * 3. 参考信息：
     *    - leader_trajectory[j]：主机在时刻j的位置
     *    - leader_velocities[j]：主机在时刻j的速度
     *    - leader_accelerations[j]：主机在时刻j的加速度
     *    - time_stamps[j]：时刻j的时间戳
     */

    // 示例实现：简单的圆形编队
    for (int i = 0; i < num_followers; i++)
    {
      follower_trajectories[i].resize(traj_length);
      
      // 为每个从机计算在编队中的角度
      double angle = 2.0 * M_PI * i / num_followers;
      
      for (int j = 0; j < traj_length; j++)
      {
        Eigen::Vector3d leader_pos = leader_trajectory[j];
        Eigen::Vector3d leader_vel = leader_velocities[j];
        
        // TODO: 在这里实现您的位置计算逻辑
        // 下面是一个简单示例（圆形编队）
        
        double radius = my_parameter_1_;  // 使用您定义的参数
        Eigen::Vector3d offset;
        offset(0) = radius * cos(angle);
        offset(1) = radius * sin(angle);
        offset(2) = my_parameter_2_;  // 高度偏移
        
        follower_trajectories[i][j] = leader_pos + offset;
      }
    }

    // ========== 算法实现结束 ==========

    ROS_INFO("[CustomSwarmAlgorithm] Successfully generated %d follower trajectories",
             num_followers);

    return follower_trajectories;
  }

  // ========== 参数设置函数（可选） ==========
  
  void setParameter1(double value) { my_parameter_1_ = value; }
  void setParameter2(double value) { my_parameter_2_ = value; }
  void setParameter3(int value) { my_parameter_3_ = value; }
};

} // namespace ego_planner

#endif // _CUSTOM_SWARM_ALGORITHM_H_


/* ============================================================================
 * 
 * 使用示例：
 * 
 * 在 swarm_traj_generator_node.cpp 中：
 * 
 * #include <plan_manage/custom_swarm_algorithm.h>
 * 
 * int main(int argc, char **argv)
 * {
 *   ros::init(argc, argv, "swarm_trajectory_generator");
 *   ros::NodeHandle nh("~");
 * 
 *   ego_planner::SwarmTrajectoryGenerator swarm_generator(nh);
 *   
 *   // 创建您的自定义算法实例
 *   ego_planner::CustomSwarmAlgorithm* custom_algo = 
 *       new ego_planner::CustomSwarmAlgorithm(3.0, 0.5, 10);
 *   
 *   // 设置算法
 *   swarm_generator.setSwarmAlgorithm(custom_algo);
 *   
 *   swarm_generator.init();
 * 
 *   ros::spin();
 *   return 0;
 * }
 * 
 * ============================================================================
 */
