/**
 * @file swarm_follower_fsm.cpp
 * @brief 集群从机有限状态机实现
 */

#include <plan_manage/swarm_follower_fsm.h>
#include <quadrotor_msgs/PositionCommand.h>

namespace ego_planner
{

void SwarmFollowerFSM::init(ros::NodeHandle& nh)
{
  nh_ = nh;
  
  // ============== 读取参数 ==============
  nh_.param("follower/uav_id", uav_id_, 1);
  nh_.param("follower/max_vel", max_vel_, 1.5);
  nh_.param("follower/max_acc", max_acc_, 2.0);
  nh_.param("follower/max_jerk", max_jerk_, 4.0);
  nh_.param("follower/safety_distance", safety_distance_, 0.5);
  nh_.param("follower/inter_robot_clearance", inter_robot_clearance_, 1.0);
  nh_.param("follower/connection_timeout", connection_timeout_, 1.0);
  nh_.param("follower/initial_wait_time", initial_wait_time_, 10.0);  // 初始等待引导的时间
  nh_.param("follower/replan_threshold", replan_threshold_, 1.0);
  nh_.param("follower/planning_horizon", planning_horizon_, 5.0);
  nh_.param("follower/use_local_avoidance", use_local_avoidance_, true);
  nh_.param("follower/use_inter_robot_avoidance", use_inter_robot_avoidance_, true);
  nh_.param("follower/world_frame", world_frame_, std::string("world"));
  
  nh_.param("optimization/lambda_smooth", lambda_smooth_, 1.0);
  nh_.param("optimization/lambda_collision", lambda_collision_, 10.0);
  nh_.param("optimization/lambda_feasibility", lambda_feasibility_, 100.0);
  nh_.param("optimization/lambda_fitness", lambda_fitness_, 1.0);
  nh_.param("optimization/lambda_inter_robot", lambda_inter_robot_, 10.0);
  
  uav_namespace_ = "/uav_" + std::to_string(uav_id_);
  
  // 读取odom topic参数 (支持launch文件配置)
  std::string odom_topic;
  nh_.param("follower/odom_topic", odom_topic, uav_namespace_ + "/odom");
  odom_topic_ = odom_topic;
  
  // ============== 初始化模块 ==============
  visualization_.reset(new PlanningVisualization(nh_));
  optimizer_.reset(new BezierOptimizer);
  optimizer_->setParam(nh_);
  grid_map_.reset(new GridMap);
  grid_map_->initMap(nh_);
  optimizer_->setEnvironment(grid_map_);
  
  // ============== 初始化状态 ==============
  state_ = FollowerFSMState::INIT;
  state_enter_time_ = ros::Time::now();
  consecutive_state_calls_ = 0;
  has_odom_ = false;
  has_guidance_ = false;
  has_traj_ = false;
  last_guidance_seq_ = 0;
  missed_guidance_count_ = 0;
  
  // ============== 设置ROS订阅器 ==============
  // 使用参数配置的odom topic
  odom_sub_ = nh_.subscribe(odom_topic_, 1, 
      &SwarmFollowerFSM::odomCallback, this);
  guidance_sub_ = nh_.subscribe(uav_namespace_ + "/swarm_guidance_traj", 1,
      &SwarmFollowerFSM::guidanceCallback, this);
  depth_sub_ = nh_.subscribe("/pcl_render_node/depth", 1,
      &SwarmFollowerFSM::depthCallback, this);
  cloud_sub_ = nh_.subscribe("/pcl_render_node/cloud", 1,
      &SwarmFollowerFSM::cloudCallback, this);
  
  // 订阅其他从机状态
  for (int i = 1; i <= 10; ++i)  // 最多支持10个从机
  {
    if (i != uav_id_)
    {
      std::string topic = "/uav_" + std::to_string(i) + "/swarm_state";
      other_state_subs_.push_back(
          nh_.subscribe<ego_planner::SwarmState>(topic, 1,
              boost::bind(&SwarmFollowerFSM::otherStateCallback, this, _1)));
    }
  }
  
  // ============== 设置ROS发布器 ==============
  bezier_pub_ = nh_.advertise<ego_planner::Bezier>(uav_namespace_ + "/planning/bezier", 10);
  state_pub_ = nh_.advertise<ego_planner::SwarmState>(uav_namespace_ + "/swarm_state", 10);
  cmd_pub_ = nh_.advertise<quadrotor_msgs::PositionCommand>(uav_namespace_ + "/position_cmd", 50);
  traj_path_pub_ = nh_.advertise<nav_msgs::Path>(uav_namespace_ + "/planning/trajectory", 10);
  
  // ============== 启动定时器 ==============
  fsm_timer_ = nh_.createTimer(ros::Duration(0.01),
      &SwarmFollowerFSM::fsmTimerCallback, this);
  safety_timer_ = nh_.createTimer(ros::Duration(0.05),
      &SwarmFollowerFSM::safetyTimerCallback, this);
  state_publish_timer_ = nh_.createTimer(ros::Duration(0.1),
      &SwarmFollowerFSM::statePublishCallback, this);
  
  ROS_INFO("[Follower-%d] FSM initialized, waiting for guidance trajectory", uav_id_);
}

void SwarmFollowerFSM::odomCallback(const nav_msgs::OdometryConstPtr& msg)
{
  odom_pos_ = Eigen::Vector3d(
      msg->pose.pose.position.x,
      msg->pose.pose.position.y,
      msg->pose.pose.position.z);
  
  odom_vel_ = Eigen::Vector3d(
      msg->twist.twist.linear.x,
      msg->twist.twist.linear.y,
      msg->twist.twist.linear.z);
  
  odom_orient_ = Eigen::Quaterniond(
      msg->pose.pose.orientation.w,
      msg->pose.pose.orientation.x,
      msg->pose.pose.orientation.y,
      msg->pose.pose.orientation.z);
  
  odom_time_ = msg->header.stamp;
  has_odom_ = true;
}

void SwarmFollowerFSM::guidanceCallback(const ego_planner::SwarmBezierTrajectoryConstPtr& msg)
{
  current_guidance_ = *msg;
  has_guidance_ = true;
  
  // 检测丢包
  if (msg->seq_num > last_guidance_seq_ + 1 && last_guidance_seq_ > 0)
  {
    missed_guidance_count_ += msg->seq_num - last_guidance_seq_ - 1;
    ROS_WARN("[Follower-%d] Detected %d missed guidance messages", 
             uav_id_, msg->seq_num - last_guidance_seq_ - 1);
  }
  
  last_guidance_seq_ = msg->seq_num;
  last_guidance_time_ = ros::Time::now();
  missed_guidance_count_ = 0;
  
  ROS_DEBUG("[Follower-%d] Received guidance traj (id=%ld, seq=%d)", 
            uav_id_, msg->traj_id, msg->seq_num);
}

void SwarmFollowerFSM::depthCallback(const sensor_msgs::ImageConstPtr& msg)
{
  // 更新深度信息到grid_map
  // TODO: 实现深度图到栅格地图的转换
}

void SwarmFollowerFSM::cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
  // 更新点云信息到grid_map
  // TODO: 实现点云到栅格地图的转换
}

void SwarmFollowerFSM::otherStateCallback(const ego_planner::SwarmStateConstPtr& msg)
{
  std::lock_guard<std::mutex> lock(other_robots_mutex_);
  
  for (auto& robot : other_robots_)
  {
    if (robot.id == msg->uav_id)
    {
      robot.position = Eigen::Vector3d(msg->position.x, msg->position.y, msg->position.z);
      robot.velocity = Eigen::Vector3d(msg->velocity.x, msg->velocity.y, msg->velocity.z);
      robot.update_time = ros::Time::now();
      return;
    }
  }
  
  // 新的从机
  OtherRobotInfo robot;
  robot.id = msg->uav_id;
  robot.position = Eigen::Vector3d(msg->position.x, msg->position.y, msg->position.z);
  robot.velocity = Eigen::Vector3d(msg->velocity.x, msg->velocity.y, msg->velocity.z);
  robot.update_time = ros::Time::now();
  other_robots_.push_back(robot);
}

void SwarmFollowerFSM::fsmTimerCallback(const ros::TimerEvent& e)
{
  if (state_ == FollowerFSMState::INIT)
    processInit();
  else if (state_ == FollowerFSMState::WAIT_GUIDANCE)
    processWaitGuidance();
  else if (state_ == FollowerFSMState::OPTIMIZE_TRAJ)
    processOptimizeTraj();
  else if (state_ == FollowerFSMState::EXEC_TRAJ)
    processExecTraj();
  else if (state_ == FollowerFSMState::LOST_CONNECTION)
    processLostConnection();
  else if (state_ == FollowerFSMState::EMERGENCY_STOP)
    processEmergencyStop();
  else if (state_ == FollowerFSMState::LOCAL_AVOIDANCE)
    processLocalAvoidance();
  
  consecutive_state_calls_++;
}

void SwarmFollowerFSM::safetyTimerCallback(const ros::TimerEvent& e)
{
  if (!has_odom_) return;
  
  // 检查是否与主机失去连接
  if (isConnectionLost() && state_ != FollowerFSMState::LOST_CONNECTION)
  {
    changeState(FollowerFSMState::LOST_CONNECTION, "Connection lost");
  }
  
  // 检查当前轨迹是否与障碍物碰撞
  if (has_traj_ && checkTrajectoryCollision())
  {
    ROS_WARN("[Follower-%d] Trajectory collision detected!", uav_id_);
    changeState(FollowerFSMState::LOCAL_AVOIDANCE, "Collision detected");
  }
}

void SwarmFollowerFSM::statePublishCallback(const ros::TimerEvent& e)
{
  publishSelfState();
}

void SwarmFollowerFSM::processInit()
{
  if (!has_odom_)
  {
    ROS_WARN_THROTTLE(2.0, "[Follower-%d] Waiting for odometry...", uav_id_);
    return;
  }
  
  changeState(FollowerFSMState::WAIT_GUIDANCE, "Odometry ready");
}

void SwarmFollowerFSM::processWaitGuidance()
{
  if (!has_guidance_)
  {
    ROS_DEBUG_THROTTLE(2.0, "[Follower-%d] Waiting for guidance trajectory...", uav_id_);
    return;
  }
  
  changeState(FollowerFSMState::OPTIMIZE_TRAJ, "Guidance received");
}

void SwarmFollowerFSM::processOptimizeTraj()
{
  if (!has_guidance_) return;
  
  // 将引导轨迹控制点转换到世界坐标系
  Eigen::MatrixXd guidance_ctrl_pts = transformGuidanceToWorld(current_guidance_);
  
  double segment_duration = current_guidance_.segment_durations[0];
  
  // 优化轨迹
  if (optimizeTrajectory(guidance_ctrl_pts, segment_duration))
  {
    current_traj_id_ = current_guidance_.traj_id;
    has_traj_ = true;
    
    publishBezierTrajectory();
    visualizeTrajectory();
    
    changeState(FollowerFSMState::EXEC_TRAJ, "Trajectory optimized");
  }
  else
  {
    ROS_WARN("[Follower-%d] Trajectory optimization failed", uav_id_);
    changeState(FollowerFSMState::EMERGENCY_STOP, "Optimization failed");
  }
}

void SwarmFollowerFSM::processExecTraj()
{
  // 检查是否需要新的引导
  if (has_guidance_ && (current_guidance_.traj_id != current_traj_id_ || 
                        ros::Time::now() - last_guidance_time_ < ros::Duration(0.5)))
  {
    changeState(FollowerFSMState::OPTIMIZE_TRAJ, "New guidance available");
  }
}

void SwarmFollowerFSM::processLostConnection()
{
  // 如果有预测轨迹，继续执行
  if (!predicted_positions_.empty())
  {
    if (!usePredictedTrajectory())
    {
      ROS_ERROR("[Follower-%d] Failed to use predicted trajectory", uav_id_);
      changeState(FollowerFSMState::EMERGENCY_STOP, "Prediction failed");
    }
  }
  else
  {
    ROS_ERROR("[Follower-%d] No predicted trajectory, emergency stop", uav_id_);
    changeState(FollowerFSMState::EMERGENCY_STOP, "No prediction");
  }
  
  // 如果重新收到引导，返回正常状态
  if (has_guidance_ && ros::Time::now() - last_guidance_time_ < ros::Duration(0.5))
  {
    changeState(FollowerFSMState::OPTIMIZE_TRAJ, "Connection restored");
  }
}

void SwarmFollowerFSM::processEmergencyStop()
{
  emergencyStop();
}

void SwarmFollowerFSM::processLocalAvoidance()
{
  if (planLocalAvoidance())
  {
    has_traj_ = true;
    publishBezierTrajectory();
    changeState(FollowerFSMState::EXEC_TRAJ, "Local avoidance planned");
  }
  else
  {
    ROS_ERROR("[Follower-%d] Local avoidance failed", uav_id_);
    changeState(FollowerFSMState::EMERGENCY_STOP, "Avoidance failed");
  }
}

void SwarmFollowerFSM::changeState(FollowerFSMState new_state, const std::string& reason)
{
  if (state_ != new_state)
  {
    ROS_INFO("[Follower-%d] State: %s -> %s (%s)",
             uav_id_, stateToString(state_).c_str(), stateToString(new_state).c_str(), reason.c_str());
    state_ = new_state;
    state_enter_time_ = ros::Time::now();
    consecutive_state_calls_ = 0;
  }
}

std::string SwarmFollowerFSM::stateToString(FollowerFSMState s)
{
  switch (s)
  {
    case FollowerFSMState::INIT: return "INIT";
    case FollowerFSMState::WAIT_GUIDANCE: return "WAIT_GUIDANCE";
    case FollowerFSMState::OPTIMIZE_TRAJ: return "OPTIMIZE_TRAJ";
    case FollowerFSMState::EXEC_TRAJ: return "EXEC_TRAJ";
    case FollowerFSMState::LOST_CONNECTION: return "LOST_CONNECTION";
    case FollowerFSMState::EMERGENCY_STOP: return "EMERGENCY_STOP";
    case FollowerFSMState::LOCAL_AVOIDANCE: return "LOCAL_AVOIDANCE";
    default: return "UNKNOWN";
  }
}

Eigen::MatrixXd SwarmFollowerFSM::transformGuidanceToWorld(
    const ego_planner::SwarmBezierTrajectory& guidance)
{
  Eigen::MatrixXd ctrl_pts(3, guidance.control_points.size());
  
  // 获取主机坐标系变换矩阵
  Eigen::Quaterniond q(guidance.leader_orientation.w, guidance.leader_orientation.x,
                       guidance.leader_orientation.y, guidance.leader_orientation.z);
  Eigen::Matrix3d R = q.toRotationMatrix();
  Eigen::Vector3d leader_pos(guidance.leader_position.x, guidance.leader_position.y,
                             guidance.leader_position.z);
  
  // 转换所有控制点
  for (size_t i = 0; i < guidance.control_points.size(); ++i)
  {
    Eigen::Vector3d relative_pt(guidance.control_points[i].x,
                                guidance.control_points[i].y,
                                guidance.control_points[i].z);
    Eigen::Vector3d world_pt = leader_pos + R * relative_pt;
    ctrl_pts.col(i) = world_pt;
  }
  
  return ctrl_pts;
}

bool SwarmFollowerFSM::optimizeTrajectory(const Eigen::MatrixXd& init_ctrl_pts, double segment_duration)
{
  optimizer_->setControlPoints(init_ctrl_pts);
  optimizer_->setSegmentDuration(segment_duration);
  optimizer_->setGuidePath(std::vector<Eigen::Vector3d>());
  
  // 设置其他从机为约束
  if (use_inter_robot_avoidance_)
  {
    std::lock_guard<std::mutex> lock(other_robots_mutex_);
    // TODO: 将其他从机信息添加到优化器约束中
  }
  
  Eigen::MatrixXd optimal_ctrl_pts;
  bool success = optimizer_->BezierOptimizeTrajRebound(optimal_ctrl_pts, segment_duration);
  
  if (success)
  {
    current_ctrl_pts_ = optimal_ctrl_pts;
    current_traj_duration_ = optimal_ctrl_pts.cols() / 3 * segment_duration;
    traj_start_time_ = ros::Time::now();
  }
  
  return success;
}

bool SwarmFollowerFSM::usePredictedTrajectory()
{
  if (predicted_positions_.empty()) return false;
  
  // 使用预测位置作为控制点
  Eigen::MatrixXd pred_ctrl_pts(3, predicted_positions_.size());
  for (size_t i = 0; i < predicted_positions_.size(); ++i)
  {
    pred_ctrl_pts.col(i) = predicted_positions_[i];
  }
  
  double segment_duration = current_guidance_.expected_update_interval;
  return optimizeTrajectory(pred_ctrl_pts, segment_duration);
}

bool SwarmFollowerFSM::needLocalAvoidance()
{
  // TODO: 实现局部避障判断
  return false;
}

bool SwarmFollowerFSM::planLocalAvoidance()
{
  // TODO: 实现局部避障规划
  return false;
}

bool SwarmFollowerFSM::isConnectionLost()
{
  // 如果从未收到引导且在初始等待时间内，不算失联
  if (!has_guidance_) {
    double elapsed = (ros::Time::now() - state_enter_time_).toSec();
    if (elapsed < initial_wait_time_) {
      return false;  // 还在初始等待期
    }
    return true;  // 超过初始等待期仍未收到引导
  }
  return (ros::Time::now() - last_guidance_time_).toSec() > connection_timeout_;
}

bool SwarmFollowerFSM::checkTrajectoryCollision()
{
  // TODO: 实现轨迹碰撞检查
  return false;
}

void SwarmFollowerFSM::emergencyStop()
{
  // 发送停止命令
  quadrotor_msgs::PositionCommand stop_cmd;
  stop_cmd.header.stamp = ros::Time::now();
  stop_cmd.header.frame_id = world_frame_;
  stop_cmd.position.x = odom_pos_.x();
  stop_cmd.position.y = odom_pos_.y();
  stop_cmd.position.z = odom_pos_.z();
  stop_cmd.velocity.x = 0;
  stop_cmd.velocity.y = 0;
  stop_cmd.velocity.z = 0;
  stop_cmd.acceleration.x = 0;
  stop_cmd.acceleration.y = 0;
  stop_cmd.acceleration.z = 0;
  
  cmd_pub_.publish(stop_cmd);
}

void SwarmFollowerFSM::publishBezierTrajectory()
{
  ego_planner::Bezier traj_msg;
  traj_msg.order = 3;  // Cubic Bezier
  traj_msg.traj_id = current_traj_id_;
  traj_msg.start_time = traj_start_time_;
  
  // 转换控制点
  for (int i = 0; i < current_ctrl_pts_.cols(); ++i)
  {
    geometry_msgs::Point pt;
    pt.x = current_ctrl_pts_(0, i);
    pt.y = current_ctrl_pts_(1, i);
    pt.z = current_ctrl_pts_(2, i);
    traj_msg.pos_pts.push_back(pt);
  }
  
  // 段时间
  traj_msg.segment_durations = current_guidance_.segment_durations;
  
  bezier_pub_.publish(traj_msg);
}

void SwarmFollowerFSM::publishSelfState()
{
  ego_planner::SwarmState state_msg;
  state_msg.header.stamp = ros::Time::now();
  state_msg.header.frame_id = world_frame_;
  state_msg.uav_id = uav_id_;
  state_msg.uav_type = ego_planner::SwarmState::UAV_TYPE_FOLLOWER;
  
  state_msg.position.x = odom_pos_.x();
  state_msg.position.y = odom_pos_.y();
  state_msg.position.z = odom_pos_.z();
  state_msg.velocity.x = odom_vel_.x();
  state_msg.velocity.y = odom_vel_.y();
  state_msg.velocity.z = odom_vel_.z();
  state_msg.orientation.w = odom_orient_.w();
  state_msg.orientation.x = odom_orient_.x();
  state_msg.orientation.y = odom_orient_.y();
  state_msg.orientation.z = odom_orient_.z();
  
  state_msg.has_valid_odom = has_odom_;
  state_msg.is_executing_traj = has_traj_;
  state_msg.current_traj_id = current_traj_id_;
  state_msg.leader_connection_ok = !isConnectionLost();
  state_msg.safety_radius = inter_robot_clearance_;
  
  switch (state_)
  {
    case FollowerFSMState::INIT:
    case FollowerFSMState::WAIT_GUIDANCE:
      state_msg.current_mode = ego_planner::SwarmState::MODE_IDLE;
      break;
    case FollowerFSMState::OPTIMIZE_TRAJ:
    case FollowerFSMState::EXEC_TRAJ:
      state_msg.current_mode = ego_planner::SwarmState::MODE_FOLLOWING;
      break;
    case FollowerFSMState::LOCAL_AVOIDANCE:
      state_msg.current_mode = ego_planner::SwarmState::MODE_LOCAL_AVOIDANCE;
      break;
    case FollowerFSMState::LOST_CONNECTION:
      state_msg.current_mode = ego_planner::SwarmState::MODE_LOST_CONNECTION;
      break;
    case FollowerFSMState::EMERGENCY_STOP:
      state_msg.current_mode = ego_planner::SwarmState::MODE_EMERGENCY_STOP;
      break;
  }
  
  state_pub_.publish(state_msg);
}

void SwarmFollowerFSM::visualizeTrajectory()
{
  // TODO: 实现轨迹可视化
}

void SwarmFollowerFSM::visualizeGuidance()
{
  // TODO: 实现引导轨迹可视化
}

void SwarmFollowerFSM::visualizeOtherRobots()
{
  // TODO: 实现其他机器人可视化
}

} // namespace ego_planner
