/**
 * @file swarm_trajectory_generator.h
 * @brief 集群轨迹生成器 - 为从机生成环绕跟随轨迹
 * @details 该模块接收主机轨迹，调用集群算法为每个从机生成独立轨迹
 */

#ifndef _SWARM_TRAJECTORY_GENERATOR_H_
#define _SWARM_TRAJECTORY_GENERATOR_H_

#include <ros/ros.h>
#include <Eigen/Eigen>
#include <vector>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <ego_planner/Bezier.h>

namespace ego_planner
{

/**
 * @brief 集群算法接口类 - 这是需要后续补充的核心算法
 * 
 * 该类定义了集群算法的标准接口。用户需要继承该类并实现computeFollowerTrajectories方法。
 * 
 * 算法输入：
 * - 主机轨迹点序列
 * - 从机数量
 * - 时间戳信息
 * 
 * 算法输出：
 * - 每个从机的轨迹点序列（环绕跟随效果）
 */
class SwarmAlgorithmBase
{
public:
  SwarmAlgorithmBase() {}
  virtual ~SwarmAlgorithmBase() {}

  /**
   * @brief 计算从机轨迹的核心算法接口（需要用户实现）
   * 
   * @param leader_trajectory 主机轨迹点序列 (每个点包含位置、速度、加速度)
   * @param num_followers 从机数量
   * @param time_stamps 轨迹点的时间戳
   * @return std::vector<std::vector<Eigen::Vector3d>> 每个从机的轨迹点序列
   * 
   * @note 返回格式: follower_trajectories[i][j] 表示第i个从机在第j个时间点的位置
   * @note 请确保返回的轨迹数量与num_followers一致
   * 
   * TODO: 用户需要在这里实现具体的集群算法逻辑
   * 建议算法：
   * 1. 圆形编队：从机围绕主机形成圆形阵型
   * 2. V字编队：从机排列成V字形
   * 3. 自适应编队：根据障碍物动态调整编队形状
   */
  virtual std::vector<std::vector<Eigen::Vector3d>> computeFollowerTrajectories(
      const std::vector<Eigen::Vector3d> &leader_trajectory,
      const std::vector<Eigen::Vector3d> &leader_velocities,
      const std::vector<Eigen::Vector3d> &leader_accelerations,
      int num_followers,
      const std::vector<double> &time_stamps) = 0;
};

/**
 * @brief 默认集群算法实现 - 简单的环绕跟随
 * 
 * 该算法让从机在主机周围形成固定半径的圆形编队，按等角度分布
 * 这是一个基础实现，用于验证系统闭环。后续可以替换为更复杂的算法。
 */
class SimpleCircleFormationAlgorithm : public SwarmAlgorithmBase
{
private:
  double formation_radius_;  // 编队半径(米)
  double altitude_offset_;   // 从机相对主机的高度偏移(米)

public:
  SimpleCircleFormationAlgorithm(double radius = 3.0, double altitude_offset = 0.5)
      : formation_radius_(radius), altitude_offset_(altitude_offset) {}

  virtual ~SimpleCircleFormationAlgorithm() {}

  std::vector<std::vector<Eigen::Vector3d>> computeFollowerTrajectories(
      const std::vector<Eigen::Vector3d> &leader_trajectory,
      const std::vector<Eigen::Vector3d> &leader_velocities,
      const std::vector<Eigen::Vector3d> &leader_accelerations,
      int num_followers,
      const std::vector<double> &time_stamps) override;

  void setFormationRadius(double radius) { formation_radius_ = radius; }
  void setAltitudeOffset(double offset) { altitude_offset_ = offset; }
};

/**
 * @brief 集群轨迹生成器 - 主控制类
 * 
 * 该类负责：
 * 1. 订阅主机轨迹
 * 2. 调用集群算法生成从机轨迹
 * 3. 发布每个从机的独立轨迹到对应的topic
 */
class SwarmTrajectoryGenerator
{
private:
  ros::NodeHandle nh_;

  // 从机数量
  int num_followers_;

  // 集群算法实例
  SwarmAlgorithmBase *swarm_algorithm_;

  // ROS订阅器和发布器
  ros::Subscriber leader_traj_sub_;          // 订阅主机轨迹
  ros::Subscriber leader_bezier_sub_;        // 订阅主机Bezier轨迹
  std::vector<ros::Publisher> follower_traj_pubs_;   // 发布从机轨迹 (Path格式)
  std::vector<ros::Publisher> follower_bezier_pubs_; // 发布从机Bezier轨迹

  // 主机最新轨迹数据
  std::vector<Eigen::Vector3d> leader_positions_;
  std::vector<Eigen::Vector3d> leader_velocities_;
  std::vector<Eigen::Vector3d> leader_accelerations_;
  std::vector<double> leader_time_stamps_;
  bool has_leader_trajectory_;

  // TF frame names
  std::string world_frame_;
  std::vector<std::string> follower_frames_;

  // Callbacks
  void leaderTrajectoryCallback(const nav_msgs::Path::ConstPtr &msg);
  void leaderBezierCallback(const ego_planner::Bezier::ConstPtr &msg);

  // Helper functions
  void publishFollowerTrajectories(
      const std::vector<std::vector<Eigen::Vector3d>> &follower_trajectories);
  
  nav_msgs::Path createPathMsg(
      const std::vector<Eigen::Vector3d> &positions,
      const std::vector<double> &time_stamps,
      const std::string &frame_id);

public:
  SwarmTrajectoryGenerator(ros::NodeHandle &nh);
  ~SwarmTrajectoryGenerator();

  void init();

  /**
   * @brief 设置自定义集群算法
   * @param algorithm 用户自定义的集群算法实例（需继承SwarmAlgorithmBase）
   */
  void setSwarmAlgorithm(SwarmAlgorithmBase *algorithm);

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

} // namespace ego_planner

#endif // _SWARM_TRAJECTORY_GENERATOR_H_
