/**
 * @file swarm_master_publisher.cpp
 * @brief 集群主机轨迹发布器实现
 */

#include <array>
#include <plan_manage/swarm_master_publisher.h>

namespace ego_planner
{

void SwarmMasterPublisher::init(ros::NodeHandle& nh)
{
  nh_ = nh;
  
  // ============== 读取参数 ==============
  nh_.param("swarm/num_followers", num_followers_, 4);
  nh_.param("swarm/publish_rate", publish_rate_, 10.0);  // 10Hz发布频率
  nh_.param("swarm/prediction_horizon", prediction_horizon_, 2.0);  // 预测2秒
  nh_.param("swarm/prediction_steps", prediction_steps_, 10);
  nh_.param("swarm/max_vel", max_vel_, 1.5);
  nh_.param("swarm/max_acc", max_acc_, 2.0);
  nh_.param("swarm/max_jerk", max_jerk_, 4.0);
  nh_.param("swarm/world_frame", world_frame_, std::string("world"));
  
  double formation_radius, altitude_offset;
  nh_.param("swarm/formation_radius", formation_radius, 3.0);
  nh_.param("swarm/altitude_offset", altitude_offset, 0.5);
  
  // ============== 初始化状态 ==============
  has_leader_odom_ = false;
  has_bezier_traj_ = false;
  current_traj_id_ = 0;
  seq_counter_ = 0;
  
  // 默认编队算法
  if (!formation_algo_)
  {
    formation_algo_ = new CircleFormation(formation_radius, altitude_offset);
  }
  
  // ============== 设置ROS订阅器 ==============
  leader_odom_sub_ = nh_.subscribe("/uav_0/odom", 1, 
      &SwarmMasterPublisher::leaderOdomCallback, this);
  leader_bezier_sub_ = nh_.subscribe("/planning/bezier", 1, 
      &SwarmMasterPublisher::leaderBezierCallback, this);
  
  // 订阅所有从机状态
  for (int i = 1; i <= num_followers_; ++i)
  {
    std::string topic = "/uav_" + std::to_string(i) + "/swarm_state";
    follower_state_subs_.push_back(
        nh_.subscribe<ego_planner::SwarmState>(topic, 1,
            boost::bind(&SwarmMasterPublisher::followerStateCallback, this, _1)));
  }
  
  // ============== 设置ROS发布器 ==============
  // 为每个从机创建独立的轨迹发布器
  for (int i = 1; i <= num_followers_; ++i)
  {
    std::string topic = "/uav_" + std::to_string(i) + "/swarm_guidance_traj";
    swarm_traj_pubs_.push_back(
        nh_.advertise<ego_planner::SwarmBezierTrajectory>(topic, 10));
  }
  
  // 发布主机状态
  leader_state_pub_ = nh_.advertise<ego_planner::SwarmState>("/uav_0/swarm_state", 10);
  
  // 发布从机可视化（用于RViz显示虚拟从机位置）
  follower_viz_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/swarm/follower_visualization", 10);
  
  // 发布从机可视化（用于RViz显示虚拟从机位置）
  follower_viz_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/swarm/follower_visualization", 10);
  
  // ============== 启动发布定时器 ==============
  publish_timer_ = nh_.createTimer(ros::Duration(1.0 / publish_rate_),
      &SwarmMasterPublisher::publishTimerCallback, this);
  
  ROS_INFO("[SwarmMaster] Initialized with %d followers, publish rate: %.1f Hz",
           num_followers_, publish_rate_);
}

void SwarmMasterPublisher::leaderOdomCallback(const nav_msgs::OdometryConstPtr& msg)
{
  leader_pos_ = Eigen::Vector3d(
      msg->pose.pose.position.x,
      msg->pose.pose.position.y,
      msg->pose.pose.position.z);
  
  leader_vel_ = Eigen::Vector3d(
      msg->twist.twist.linear.x,
      msg->twist.twist.linear.y,
      msg->twist.twist.linear.z);
  
  leader_orient_ = Eigen::Quaterniond(
      msg->pose.pose.orientation.w,
      msg->pose.pose.orientation.x,
      msg->pose.pose.orientation.y,
      msg->pose.pose.orientation.z);
  
  leader_state_time_ = msg->header.stamp;
  has_leader_odom_ = true;
}

void SwarmMasterPublisher::leaderBezierCallback(const ego_planner::BezierConstPtr& msg)
{
  current_bezier_traj_ = *msg;
  has_bezier_traj_ = true;
  current_traj_id_ = msg->traj_id;
  
  ROS_DEBUG("[SwarmMaster] Received new Bezier trajectory, id=%ld, %zu control points",
           current_traj_id_, msg->pos_pts.size());
}

void SwarmMasterPublisher::followerStateCallback(const ego_planner::SwarmStateConstPtr& msg)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  follower_states_[msg->uav_id] = *msg;
}

void SwarmMasterPublisher::publishTimerCallback(const ros::TimerEvent& e)
{
  if (!has_leader_odom_)
  {
    ROS_WARN_THROTTLE(2.0, "[SwarmMaster] Waiting for leader odometry...");
    return;
  }
  
  if (!has_bezier_traj_)
  {
    ROS_DEBUG_THROTTLE(2.0, "[SwarmMaster] No Bezier trajectory available yet");
    return;
  }
  
  if (!formation_algo_)
  {
    ROS_ERROR_THROTTLE(2.0, "[SwarmMaster] Formation algorithm not initialized!");
    return;
  }
  
  if (!formation_algo_)
  {
    ROS_ERROR_THROTTLE(2.0, "[SwarmMaster] Formation algorithm not initialized!");
    return;
  }
  
  // 提取主机轨迹控制点
  std::vector<Eigen::Vector3d> leader_ctrl_pts;
  for (const auto& pt : current_bezier_traj_.pos_pts)
  {
    leader_ctrl_pts.push_back(Eigen::Vector3d(pt.x, pt.y, pt.z));
  }
  
  // 转换到相对坐标系
  std::vector<Eigen::Vector3d> relative_ctrl_pts = transformToRelativeFrame(leader_ctrl_pts);
  
  // 为每个从机生成并发布引导轨迹
  for (int follower_id = 1; follower_id <= num_followers_; ++follower_id)
  {
    ego_planner::SwarmBezierTrajectory swarm_traj;
    
    // ============== 填充消息头 ==============
    swarm_traj.header.stamp = ros::Time::now();
    swarm_traj.header.frame_id = "leader_relative";
    swarm_traj.traj_id = current_traj_id_;
    swarm_traj.seq_num = seq_counter_++;
    swarm_traj.follower_id = follower_id;
    swarm_traj.total_followers = num_followers_;
    
    // ============== 主机状态 ==============
    swarm_traj.leader_position.x = leader_pos_.x();
    swarm_traj.leader_position.y = leader_pos_.y();
    swarm_traj.leader_position.z = leader_pos_.z();
    swarm_traj.leader_velocity.x = leader_vel_.x();
    swarm_traj.leader_velocity.y = leader_vel_.y();
    swarm_traj.leader_velocity.z = leader_vel_.z();
    swarm_traj.leader_orientation.w = leader_orient_.w();
    swarm_traj.leader_orientation.x = leader_orient_.x();
    swarm_traj.leader_orientation.y = leader_orient_.y();
    swarm_traj.leader_orientation.z = leader_orient_.z();
    swarm_traj.leader_state_timestamp = leader_state_time_;
    
    // ============== Bezier轨迹参数 ==============
    swarm_traj.order = current_bezier_traj_.order;
    swarm_traj.segment_durations = current_bezier_traj_.segment_durations;
    swarm_traj.traj_start_time = current_bezier_traj_.start_time;
    
    // 生成从机引导轨迹控制点
    std::vector<Eigen::Vector3d> follower_ctrl_pts = 
        generateFollowerControlPoints(follower_id, relative_ctrl_pts);
    
    for (const auto& pt : follower_ctrl_pts)
    {
      geometry_msgs::Point p;
      p.x = pt.x(); p.y = pt.y(); p.z = pt.z();
      swarm_traj.control_points.push_back(p);
    }
    
    // ============== 编队信息 ==============
    Eigen::Vector3d offset = formation_algo_->computeFormationOffset(
        follower_id, num_followers_, leader_vel_);
    swarm_traj.formation_offset.x = offset.x();
    swarm_traj.formation_offset.y = offset.y();
    swarm_traj.formation_offset.z = offset.z();
    swarm_traj.formation_radius = offset.head<2>().norm();
    swarm_traj.formation_angle = atan2(offset.y(), offset.x());
    
    // ============== 动力学约束 ==============
    swarm_traj.max_vel = max_vel_;
    swarm_traj.max_acc = max_acc_;
    swarm_traj.max_jerk = max_jerk_;
    
    // ============== 预测信息 ==============
    swarm_traj.expected_update_interval = 1.0 / publish_rate_;
    
    if (!current_bezier_traj_.segment_durations.empty())
    {
      std::vector<Eigen::Vector3d> predictions = generatePredictions(
          follower_ctrl_pts, current_bezier_traj_.segment_durations[0], 
          current_bezier_traj_.order);
      
      for (const auto& pred : predictions)
      {
        geometry_msgs::Point p;
        p.x = pred.x(); p.y = pred.y(); p.z = pred.z();
        swarm_traj.predicted_positions.push_back(p);
      }
      
      double dt = prediction_horizon_ / prediction_steps_;
      for (int i = 0; i < prediction_steps_; ++i)
      {
        swarm_traj.prediction_timestamps.push_back(i * dt);
      }
    }
    
    // ============== 其他从机信息（机间避障） ==============
    std::vector<uint8_t> other_ids;
    std::vector<Eigen::Vector3d> other_positions, other_velocities;
    collectOtherFollowersInfo(follower_id, other_ids, other_positions, other_velocities);
    
    swarm_traj.other_follower_ids = other_ids;
    for (size_t i = 0; i < other_positions.size(); ++i)
    {
      geometry_msgs::Point p;
      p.x = other_positions[i].x();
      p.y = other_positions[i].y();
      p.z = other_positions[i].z();
      swarm_traj.other_follower_positions.push_back(p);
      
      geometry_msgs::Vector3 v;
      v.x = other_velocities[i].x();
      v.y = other_velocities[i].y();
      v.z = other_velocities[i].z();
      swarm_traj.other_follower_velocities.push_back(v);
    }
    
    swarm_traj.status = ego_planner::SwarmBezierTrajectory::STATUS_NORMAL;
    
    // 发布
    swarm_traj_pubs_[follower_id - 1].publish(swarm_traj);
  }
  
  // 发布主机状态
  ego_planner::SwarmState leader_state;
  leader_state.header.stamp = ros::Time::now();
  leader_state.header.frame_id = world_frame_;
  leader_state.uav_id = 0;
  leader_state.uav_type = ego_planner::SwarmState::UAV_TYPE_LEADER;
  leader_state.position.x = leader_pos_.x();
  leader_state.position.y = leader_pos_.y();
  leader_state.position.z = leader_pos_.z();
  leader_state.velocity.x = leader_vel_.x();
  leader_state.velocity.y = leader_vel_.y();
  leader_state.velocity.z = leader_vel_.z();
  leader_state.has_valid_odom = true;
  leader_state.current_mode = ego_planner::SwarmState::MODE_FOLLOWING;
  leader_state_pub_.publish(leader_state);
  
  // ============== 发布从机可视化 ==============
  visualization_msgs::MarkerArray follower_markers;
  
  // 定义不同从机的颜色
  std::vector<std::array<float, 3>> colors = {
    {0.0f, 1.0f, 0.0f},   // 绿色 - uav_1
    {0.0f, 0.5f, 1.0f},   // 蓝色 - uav_2
    {1.0f, 0.5f, 0.0f},   // 橙色 - uav_3
    {1.0f, 0.0f, 1.0f}    // 紫色 - uav_4
  };
  
  for (int i = 1; i <= num_followers_; ++i)
  {
    // 计算从机的虚拟位置（主机位置 + 编队偏移）
    Eigen::Vector3d offset = formation_algo_->computeFormationOffset(i, num_followers_, leader_vel_);
    
    // 根据主机朝向旋转偏移
    Eigen::Matrix3d R;
    if (leader_vel_.norm() > 0.1) {
      Eigen::Vector3d x_axis = leader_vel_.normalized();
      Eigen::Vector3d z_axis(0, 0, 1);
      Eigen::Vector3d y_axis = z_axis.cross(x_axis).normalized();
      z_axis = x_axis.cross(y_axis);
      R.col(0) = x_axis;
      R.col(1) = y_axis;
      R.col(2) = z_axis;
    } else {
      R = leader_orient_.toRotationMatrix();
    }
    
    Eigen::Vector3d follower_pos = leader_pos_ + R * offset;
    
    // 创建无人机模型Marker（简化为球体）
    visualization_msgs::Marker drone_marker;
    drone_marker.header.stamp = ros::Time::now();
    drone_marker.header.frame_id = world_frame_;
    drone_marker.ns = "follower_drones";
    drone_marker.id = i;
    drone_marker.type = visualization_msgs::Marker::MESH_RESOURCE;
    drone_marker.mesh_resource = "package://odom_visualization/meshes/hummingbird.mesh";
    drone_marker.action = visualization_msgs::Marker::ADD;
    drone_marker.pose.position.x = follower_pos.x();
    drone_marker.pose.position.y = follower_pos.y();
    drone_marker.pose.position.z = follower_pos.z();
    drone_marker.pose.orientation.w = leader_orient_.w();
    drone_marker.pose.orientation.x = leader_orient_.x();
    drone_marker.pose.orientation.y = leader_orient_.y();
    drone_marker.pose.orientation.z = leader_orient_.z();
    drone_marker.scale.x = 1.0;
    drone_marker.scale.y = 1.0;
    drone_marker.scale.z = 1.0;
    
    int color_idx = (i - 1) % colors.size();
    drone_marker.color.r = colors[color_idx][0];
    drone_marker.color.g = colors[color_idx][1];
    drone_marker.color.b = colors[color_idx][2];
    drone_marker.color.a = 1.0;
    drone_marker.lifetime = ros::Duration(0.2);
    
    follower_markers.markers.push_back(drone_marker);
    
    // 添加ID标签
    visualization_msgs::Marker text_marker;
    text_marker.header = drone_marker.header;
    text_marker.ns = "follower_labels";
    text_marker.id = i + 100;
    text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    text_marker.action = visualization_msgs::Marker::ADD;
    text_marker.pose.position.x = follower_pos.x();
    text_marker.pose.position.y = follower_pos.y();
    text_marker.pose.position.z = follower_pos.z() + 0.5;
    text_marker.scale.z = 0.3;
    text_marker.color.r = 1.0;
    text_marker.color.g = 1.0;
    text_marker.color.b = 1.0;
    text_marker.color.a = 1.0;
    text_marker.text = "UAV_" + std::to_string(i);
    text_marker.lifetime = ros::Duration(0.2);
    
    follower_markers.markers.push_back(text_marker);
  }
  
  follower_viz_pub_.publish(follower_markers);
}

std::vector<Eigen::Vector3d> SwarmMasterPublisher::transformToRelativeFrame(
    const std::vector<Eigen::Vector3d>& world_points)
{
  std::vector<Eigen::Vector3d> relative_points;
  relative_points.reserve(world_points.size());
  
  // 计算主机朝向（基于速度方向，如果速度太小则使用姿态）
  Eigen::Matrix3d R;
  if (leader_vel_.norm() > 0.1)
  {
    Eigen::Vector3d x_axis = leader_vel_.normalized();
    Eigen::Vector3d z_axis(0, 0, 1);
    Eigen::Vector3d y_axis = z_axis.cross(x_axis).normalized();
    z_axis = x_axis.cross(y_axis);
    R.col(0) = x_axis;
    R.col(1) = y_axis;
    R.col(2) = z_axis;
  }
  else
  {
    R = leader_orient_.toRotationMatrix();
  }
  
  // 将世界坐标转换为主机相对坐标
  for (const auto& pt : world_points)
  {
    Eigen::Vector3d relative = R.transpose() * (pt - leader_pos_);
    relative_points.push_back(relative);
  }
  
  return relative_points;
}

std::vector<Eigen::Vector3d> SwarmMasterPublisher::generateFollowerControlPoints(
    int follower_id,
    const std::vector<Eigen::Vector3d>& base_control_points)
{
  std::vector<Eigen::Vector3d> follower_pts;
  follower_pts.reserve(base_control_points.size());
  
  // 获取编队偏移
  Eigen::Vector3d formation_offset = formation_algo_->computeFormationOffset(
      follower_id, num_followers_, leader_vel_);
  
  // 对每个控制点添加编队偏移
  for (const auto& base_pt : base_control_points)
  {
    // 在相对坐标系中，偏移量是固定的
    Eigen::Vector3d follower_pt = base_pt + formation_offset;
    follower_pts.push_back(follower_pt);
  }
  
  return follower_pts;
}

std::vector<Eigen::Vector3d> SwarmMasterPublisher::generatePredictions(
    const std::vector<Eigen::Vector3d>& control_points,
    double segment_duration,
    int order)
{
  std::vector<Eigen::Vector3d> predictions;
  
  if (control_points.empty() || segment_duration <= 0)
    return predictions;
  
  // 简单的线性插值预测
  double dt = prediction_horizon_ / prediction_steps_;
  int num_segments = (control_points.size() - 1) / order;
  double total_duration = num_segments * segment_duration;
  
  for (int i = 0; i < prediction_steps_; ++i)
  {
    double t = i * dt;
    if (t >= total_duration)
    {
      predictions.push_back(control_points.back());
      continue;
    }
    
    // 找到对应的segment
    int seg_idx = static_cast<int>(t / segment_duration);
    seg_idx = std::min(seg_idx, num_segments - 1);
    double local_t = (t - seg_idx * segment_duration) / segment_duration;
    
    // De Casteljau算法计算Bezier曲线点
    int base_idx = seg_idx * order;
    std::vector<Eigen::Vector3d> pts;
    for (int j = 0; j <= order; ++j)
    {
      if (base_idx + j < control_points.size())
        pts.push_back(control_points[base_idx + j]);
    }
    
    // 递归计算
    while (pts.size() > 1)
    {
      std::vector<Eigen::Vector3d> new_pts;
      for (size_t j = 0; j < pts.size() - 1; ++j)
      {
        new_pts.push_back((1 - local_t) * pts[j] + local_t * pts[j + 1]);
      }
      pts = new_pts;
    }
    
    predictions.push_back(pts[0]);
  }
  
  return predictions;
}

bool SwarmMasterPublisher::checkDynamicFeasibility(
    const std::vector<Eigen::Vector3d>& control_points,
    double segment_duration)
{
  if (control_points.size() < 4 || segment_duration <= 0)
    return false;
  
  int order = 3;  // Cubic Bezier
  int num_segments = (control_points.size() - 1) / order;
  
  for (int seg = 0; seg < num_segments; ++seg)
  {
    int base = seg * order;
    
    // 检查速度：v = 3*(P1 - P0)/T
    for (int i = 0; i < order; ++i)
    {
      Eigen::Vector3d vel = order * (control_points[base + i + 1] - control_points[base + i]) 
                            / segment_duration;
      if (vel.norm() > max_vel_ * 1.2)  // 留20%余量
        return false;
    }
    
    // 检查加速度：a = 6*(P2 - 2*P1 + P0)/T^2
    for (int i = 0; i < order - 1; ++i)
    {
      Eigen::Vector3d acc = order * (order - 1) * 
          (control_points[base + i + 2] - 2 * control_points[base + i + 1] + control_points[base + i])
          / (segment_duration * segment_duration);
      if (acc.norm() > max_acc_ * 1.2)
        return false;
    }
  }
  
  return true;
}

void SwarmMasterPublisher::collectOtherFollowersInfo(
    int target_follower_id,
    std::vector<uint8_t>& other_ids,
    std::vector<Eigen::Vector3d>& other_positions,
    std::vector<Eigen::Vector3d>& other_velocities)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  
  other_ids.clear();
  other_positions.clear();
  other_velocities.clear();
  
  for (const auto& pair : follower_states_)
  {
    if (pair.first != target_follower_id)
    {
      const auto& state = pair.second;
      other_ids.push_back(state.uav_id);
      
      // 转换到相对坐标系
      Eigen::Vector3d world_pos(state.position.x, state.position.y, state.position.z);
      Eigen::Vector3d relative_pos = world_pos - leader_pos_;
      other_positions.push_back(relative_pos);
      
      other_velocities.push_back(
          Eigen::Vector3d(state.velocity.x, state.velocity.y, state.velocity.z));
    }
  }
}

} // namespace ego_planner
