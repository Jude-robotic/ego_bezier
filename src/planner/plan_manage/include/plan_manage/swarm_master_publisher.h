/**
 * @file swarm_master_publisher.h
 * @brief 集群主机轨迹发布器
 * 
 * 主要功能：
 * 1. 订阅主机的Bezier轨迹
 * 2. 为每个从机生成编队引导轨迹
 * 3. 以相对坐标系发布给从机
 * 4. 考虑从机动力学约束
 * 5. 提供丢包预测信息
 */

#ifndef _SWARM_MASTER_PUBLISHER_H_
#define _SWARM_MASTER_PUBLISHER_H_

#include <ros/ros.h>
#include <Eigen/Eigen>
#include <vector>
#include <map>
#include <mutex>

#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <ego_planner/Bezier.h>
#include <ego_planner/SwarmBezierTrajectory.h>
#include <ego_planner/SwarmState.h>

namespace ego_planner
{

/**
 * @brief 编队算法基类
 */
class FormationAlgorithm
{
public:
  virtual ~FormationAlgorithm() {}
  
  /**
   * @brief 计算从机相对于主机的编队偏移
   * @param follower_id 从机ID (1~N)
   * @param total_followers 从机总数
   * @param leader_velocity 主机当前速度（用于动态调整编队）
   * @return 编队偏移向量（相对于主机的局部坐标系）
   */
  virtual Eigen::Vector3d computeFormationOffset(
      int follower_id,
      int total_followers,
      const Eigen::Vector3d& leader_velocity) = 0;
};

/**
 * @brief 圆形编队算法
 */
class CircleFormation : public FormationAlgorithm
{
public:
  CircleFormation(double radius = 3.0, double altitude_offset = 0.5)
      : radius_(radius), altitude_offset_(altitude_offset) {}
  
  Eigen::Vector3d computeFormationOffset(
      int follower_id,
      int total_followers,
      const Eigen::Vector3d& leader_velocity) override
  {
    // 均匀分布在主机周围的圆上
    double angle = 2.0 * M_PI * (follower_id - 1) / total_followers;
    return Eigen::Vector3d(
        radius_ * cos(angle),
        radius_ * sin(angle),
        altitude_offset_);
  }
  
  void setRadius(double r) { radius_ = r; }
  void setAltitudeOffset(double h) { altitude_offset_ = h; }

private:
  double radius_;
  double altitude_offset_;
};

/**
 * @brief V字编队算法
 */
class VFormation : public FormationAlgorithm
{
public:
  VFormation(double spacing = 2.0, double angle = M_PI / 6)
      : spacing_(spacing), half_angle_(angle) {}
  
  Eigen::Vector3d computeFormationOffset(
      int follower_id,
      int total_followers,
      const Eigen::Vector3d& leader_velocity) override
  {
    // 根据主机速度方向调整V字方向
    Eigen::Vector3d forward = leader_velocity.normalized();
    if (forward.norm() < 0.1) forward = Eigen::Vector3d(1, 0, 0);
    
    // 左右交替排列
    int side = (follower_id % 2 == 0) ? 1 : -1;
    int row = (follower_id + 1) / 2;
    
    Eigen::Vector3d right(-forward.y(), forward.x(), 0);
    right.normalize();
    
    return -forward * row * spacing_ * cos(half_angle_) 
           + right * side * row * spacing_ * sin(half_angle_);
  }

private:
  double spacing_;
  double half_angle_;
};

/**
 * @brief 集群主机轨迹发布器
 */
class SwarmMasterPublisher
{
public:
  SwarmMasterPublisher() : formation_algo_(nullptr), has_leader_odom_(false), has_bezier_traj_(false) {}
  ~SwarmMasterPublisher() { if (formation_algo_) delete formation_algo_; }
  
  void init(ros::NodeHandle& nh);
  
  /**
   * @brief 设置编队算法
   */
  void setFormationAlgorithm(FormationAlgorithm* algo) { formation_algo_ = algo; }

private:
  ros::NodeHandle nh_;
  
  // ============== 参数 ==============
  int num_followers_;                   // 从机数量
  double publish_rate_;                 // 发布频率 (Hz)
  double prediction_horizon_;           // 预测时间范围 (秒)
  int prediction_steps_;                // 预测步数
  double max_vel_, max_acc_, max_jerk_; // 动力学约束
  std::string world_frame_;
  
  // ============== 状态 ==============
  Eigen::Vector3d leader_pos_;
  Eigen::Vector3d leader_vel_;
  Eigen::Vector3d leader_acc_;
  Eigen::Quaterniond leader_orient_;
  ros::Time leader_state_time_;
  bool has_leader_odom_;
  
  // 当前Bezier轨迹
  ego_planner::Bezier current_bezier_traj_;
  bool has_bezier_traj_;
  int64_t current_traj_id_;
  uint32_t seq_counter_;
  
  // 从机状态（用于机间避障信息）
  std::map<int, ego_planner::SwarmState> follower_states_;
  std::mutex state_mutex_;
  
  // 编队算法
  FormationAlgorithm* formation_algo_;
  
  // ============== ROS接口 ==============
  ros::Subscriber leader_odom_sub_;
  ros::Subscriber leader_bezier_sub_;
  std::vector<ros::Subscriber> follower_state_subs_;
  std::vector<ros::Publisher> swarm_traj_pubs_;
  ros::Publisher leader_state_pub_;
  ros::Publisher follower_viz_pub_;  // 从机可视化发布器
  ros::Timer publish_timer_;
  
  // ============== 回调函数 ==============
  void leaderOdomCallback(const nav_msgs::OdometryConstPtr& msg);
  void leaderBezierCallback(const ego_planner::BezierConstPtr& msg);
  void followerStateCallback(const ego_planner::SwarmStateConstPtr& msg);
  void publishTimerCallback(const ros::TimerEvent& e);
  
  // ============== 辅助函数 ==============
  /**
   * @brief 将Bezier控制点转换为相对坐标系
   */
  std::vector<Eigen::Vector3d> transformToRelativeFrame(
      const std::vector<Eigen::Vector3d>& world_points);
  
  /**
   * @brief 为从机生成引导轨迹控制点
   * @param follower_id 从机ID
   * @param base_control_points 主机轨迹控制点（相对坐标系）
   * @return 从机引导轨迹控制点（相对坐标系）
   */
  std::vector<Eigen::Vector3d> generateFollowerControlPoints(
      int follower_id,
      const std::vector<Eigen::Vector3d>& base_control_points);
  
  /**
   * @brief 生成预测位置（用于丢包时继续飞行）
   */
  std::vector<Eigen::Vector3d> generatePredictions(
      const std::vector<Eigen::Vector3d>& control_points,
      double segment_duration,
      int order);
  
  /**
   * @brief 检查轨迹是否满足动力学约束
   */
  bool checkDynamicFeasibility(
      const std::vector<Eigen::Vector3d>& control_points,
      double segment_duration);
  
  /**
   * @brief 收集其他从机的位置信息（用于机间避障）
   */
  void collectOtherFollowersInfo(
      int target_follower_id,
      std::vector<uint8_t>& other_ids,
      std::vector<Eigen::Vector3d>& other_positions,
      std::vector<Eigen::Vector3d>& other_velocities);

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

} // namespace ego_planner

#endif // _SWARM_MASTER_PUBLISHER_H_
