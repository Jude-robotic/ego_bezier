/**
 * @file swarm_follower_fsm.h
 * @brief 集群从机有限状态机
 * 
 * 主要功能：
 * 1. 订阅主机发布的引导轨迹
 * 2. 将引导轨迹作为EGO-Bezier优化的初值
 * 3. 结合自身传感器数据进行局部避障
 * 4. 实现机间避障
 * 5. 处理通信丢包（基于预测继续飞行）
 */

#ifndef _SWARM_FOLLOWER_FSM_H_
#define _SWARM_FOLLOWER_FSM_H_

#include <ros/ros.h>
#include <Eigen/Eigen>
#include <vector>
#include <deque>
#include <mutex>

#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>

#include <bezier_opt/bezier_optimizer.h>
#include <bezier_opt/piecewise_bezier.h>
#include <plan_env/grid_map.h>
#include <ego_planner/Bezier.h>
#include <ego_planner/SwarmBezierTrajectory.h>
#include <ego_planner/SwarmState.h>
#include <traj_utils/planning_visualization.h>

namespace ego_planner
{

/**
 * @brief 从机FSM状态
 */
enum class FollowerFSMState
{
  INIT,              // 初始化
  WAIT_GUIDANCE,     // 等待引导轨迹
  OPTIMIZE_TRAJ,     // 优化轨迹
  EXEC_TRAJ,         // 执行轨迹
  LOST_CONNECTION,   // 丢失与主机连接
  EMERGENCY_STOP,    // 紧急停止
  LOCAL_AVOIDANCE    // 局部避障
};

/**
 * @brief 集群从机有限状态机
 */
class SwarmFollowerFSM
{
public:
  SwarmFollowerFSM() {}
  ~SwarmFollowerFSM() {}
  
  void init(ros::NodeHandle& nh);

private:
  ros::NodeHandle nh_;
  
  // ============== 身份标识 ==============
  int uav_id_;                          // 从机ID
  std::string uav_namespace_;           // ROS命名空间
  
  // ============== FSM状态 ==============
  FollowerFSMState state_;
  ros::Time state_enter_time_;
  int consecutive_state_calls_;
  
  // ============== 参数 ==============
  double max_vel_, max_acc_, max_jerk_;
  double safety_distance_;              // 安全距离
  double inter_robot_clearance_;        // 机间最小距离
  double connection_timeout_;           // 连接超时时间(秒)
  double initial_wait_time_;            // 初始等待引导轨迹的时间(秒)
  std::string odom_topic_;              // odom topic (可配置)
  double replan_threshold_;             // 重规划阈值
  double planning_horizon_;
  bool use_local_avoidance_;            // 是否使用局部避障
  bool use_inter_robot_avoidance_;      // 是否使用机间避障
  std::string world_frame_;
  
  // ============== 优化器参数 ==============
  double lambda_smooth_;
  double lambda_collision_;
  double lambda_feasibility_;
  double lambda_fitness_;
  double lambda_inter_robot_;           // 机间避障权重
  
  // ============== 状态变量 ==============
  // 自身里程计
  Eigen::Vector3d odom_pos_, odom_vel_, odom_acc_;
  Eigen::Quaterniond odom_orient_;
  ros::Time odom_time_;
  bool has_odom_;
  
  // 主机状态
  Eigen::Vector3d leader_pos_, leader_vel_;
  Eigen::Quaterniond leader_orient_;
  ros::Time leader_state_time_;
  
  // 引导轨迹
  ego_planner::SwarmBezierTrajectory current_guidance_;
  bool has_guidance_;
  uint32_t last_guidance_seq_;
  ros::Time last_guidance_time_;
  int missed_guidance_count_;           // 丢包计数
  
  // 其他从机信息（用于机间避障）
  struct OtherRobotInfo
  {
    int id;
    Eigen::Vector3d position;
    Eigen::Vector3d velocity;
    ros::Time update_time;
  };
  std::vector<OtherRobotInfo> other_robots_;
  std::mutex other_robots_mutex_;
  
  // 当前执行的轨迹
  PiecewiseBezier current_traj_;
  Eigen::MatrixXd current_ctrl_pts_;
  double current_traj_duration_;
  ros::Time traj_start_time_;
  int64_t current_traj_id_;
  bool has_traj_;
  
  // 预测轨迹（用于丢包时继续飞行）
  std::vector<Eigen::Vector3d> predicted_positions_;
  std::vector<double> prediction_timestamps_;
  
  // ============== 核心模块 ==============
  GridMap::Ptr grid_map_;
  BezierOptimizer::Ptr optimizer_;
  PlanningVisualization::Ptr visualization_;
  
  // ============== ROS接口 ==============
  ros::Subscriber odom_sub_;
  ros::Subscriber guidance_sub_;
  ros::Subscriber depth_sub_;
  ros::Subscriber cloud_sub_;
  std::vector<ros::Subscriber> other_state_subs_;
  
  ros::Publisher bezier_pub_;
  ros::Publisher state_pub_;
  ros::Publisher cmd_pub_;
  ros::Publisher traj_path_pub_;
  
  ros::Timer fsm_timer_;
  ros::Timer safety_timer_;
  ros::Timer state_publish_timer_;
  
  // ============== 回调函数 ==============
  void odomCallback(const nav_msgs::OdometryConstPtr& msg);
  void guidanceCallback(const ego_planner::SwarmBezierTrajectoryConstPtr& msg);
  void depthCallback(const sensor_msgs::ImageConstPtr& msg);
  void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg);
  void otherStateCallback(const ego_planner::SwarmStateConstPtr& msg);
  
  void fsmTimerCallback(const ros::TimerEvent& e);
  void safetyTimerCallback(const ros::TimerEvent& e);
  void statePublishCallback(const ros::TimerEvent& e);
  
  // ============== FSM状态处理 ==============
  void processInit();
  void processWaitGuidance();
  void processOptimizeTraj();
  void processExecTraj();
  void processLostConnection();
  void processEmergencyStop();
  void processLocalAvoidance();
  
  void changeState(FollowerFSMState new_state, const std::string& reason);
  std::string stateToString(FollowerFSMState s);
  
  // ============== 核心功能 ==============
  /**
   * @brief 将引导轨迹控制点转换到世界坐标系
   */
  Eigen::MatrixXd transformGuidanceToWorld(
      const ego_planner::SwarmBezierTrajectory& guidance);
  
  /**
   * @brief 优化轨迹（以引导轨迹为初值）
   * @return true 如果优化成功
   */
  bool optimizeTrajectory(const Eigen::MatrixXd& init_ctrl_pts, double segment_duration);
  
  /**
   * @brief 使用预测轨迹继续飞行（丢包处理）
   */
  bool usePredictedTrajectory();
  
  /**
   * @brief 检查是否需要局部避障
   */
  bool needLocalAvoidance();
  
  /**
   * @brief 执行局部避障规划
   */
  bool planLocalAvoidance();
  
  /**
   * @brief 检查是否与主机失去连接
   */
  bool isConnectionLost();
  
  /**
   * @brief 检查当前轨迹是否安全
   */
  bool checkTrajectoryCollision();
  
  /**
   * @brief 紧急停止
   */
  void emergencyStop();
  
  /**
   * @brief 发布优化后的Bezier轨迹
   */
  void publishBezierTrajectory();
  
  /**
   * @brief 发布自身状态
   */
  void publishSelfState();
  
  /**
   * @brief 可视化
   */
  void visualizeTrajectory();
  void visualizeGuidance();
  void visualizeOtherRobots();

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

} // namespace ego_planner

#endif // _SWARM_FOLLOWER_FSM_H_
