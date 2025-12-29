/**
 * @file swarm_trajectory_generator.cpp
 * @brief 集群轨迹生成器实现
 */

#include <plan_manage/swarm_trajectory_generator.h>
#include <tf/tf.h>

namespace ego_planner
{

// ==================== SimpleCircleFormationAlgorithm 实现 ====================

std::vector<std::vector<Eigen::Vector3d>> SimpleCircleFormationAlgorithm::computeFollowerTrajectories(
    const std::vector<Eigen::Vector3d> &leader_trajectory,
    const std::vector<Eigen::Vector3d> &leader_velocities,
    const std::vector<Eigen::Vector3d> &leader_accelerations,
    int num_followers,
    const std::vector<double> &time_stamps)
{
  std::vector<std::vector<Eigen::Vector3d>> follower_trajectories;
  follower_trajectories.resize(num_followers);

  if (leader_trajectory.empty())
  {
    ROS_WARN("[SwarmAlgorithm] Leader trajectory is empty!");
    return follower_trajectories;
  }

  // 为每个从机计算轨迹
  for (int i = 0; i < num_followers; i++)
  {
    double angle = 2.0 * M_PI * i / num_followers; // 等角度分布
    
    follower_trajectories[i].resize(leader_trajectory.size());

    for (size_t j = 0; j < leader_trajectory.size(); j++)
    {
      Eigen::Vector3d leader_pos = leader_trajectory[j];
      
      // 计算从机在主机周围的偏移位置
      // 基本策略：在水平面上环绕，保持固定半径
      Eigen::Vector3d offset;
      offset(0) = formation_radius_ * cos(angle); // X偏移
      offset(1) = formation_radius_ * sin(angle); // Y偏移
      offset(2) = altitude_offset_;               // Z偏移（高度）
      
      // 如果主机有速度，可以让编队动态调整（这里先用简单的固定偏移）
      // TODO: 后续可以根据主机速度方向调整编队朝向
      
      follower_trajectories[i][j] = leader_pos + offset;
    }
  }

  ROS_INFO("[SwarmAlgorithm] Generated trajectories for %d followers, each with %lu points",
           num_followers, leader_trajectory.size());

  return follower_trajectories;
}

// ==================== SwarmTrajectoryGenerator 实现 ====================

SwarmTrajectoryGenerator::SwarmTrajectoryGenerator(ros::NodeHandle &nh)
    : nh_(nh), has_leader_trajectory_(false), swarm_algorithm_(nullptr)
{
}

SwarmTrajectoryGenerator::~SwarmTrajectoryGenerator()
{
  if (swarm_algorithm_ != nullptr)
  {
    delete swarm_algorithm_;
  }
}

void SwarmTrajectoryGenerator::init()
{
  // 读取参数
  nh_.param("swarm/num_followers", num_followers_, 4);
  nh_.param("swarm/world_frame", world_frame_, std::string("world"));

  double formation_radius, altitude_offset;
  nh_.param("swarm/formation_radius", formation_radius, 3.0);
  nh_.param("swarm/altitude_offset", altitude_offset, 0.5);

  ROS_INFO("[SwarmTrajGen] Initializing with %d followers", num_followers_);
  ROS_INFO("[SwarmTrajGen] Formation radius: %.2f m, Altitude offset: %.2f m", 
           formation_radius, altitude_offset);

  // 创建默认集群算法（简单圆形编队）
  if (swarm_algorithm_ == nullptr)
  {
    swarm_algorithm_ = new SimpleCircleFormationAlgorithm(formation_radius, altitude_offset);
    ROS_INFO("[SwarmTrajGen] Using default SimpleCircleFormationAlgorithm");
  }

  // 初始化从机frame名称
  follower_frames_.resize(num_followers_);
  for (int i = 0; i < num_followers_; i++)
  {
    follower_frames_[i] = "uav_" + std::to_string(i + 1);
  }

  // ============ 订阅主机轨迹 ============
  // 订阅主机的Path消息
  leader_traj_sub_ = nh_.subscribe("/uav_0/planning/trajectory", 10,
                                   &SwarmTrajectoryGenerator::leaderTrajectoryCallback, this);

  // 订阅主机的Bezier轨迹消息
  leader_bezier_sub_ = nh_.subscribe("/uav_0/planning/bezier_trajectory", 10,
                                     &SwarmTrajectoryGenerator::leaderBezierCallback, this);

  // ============ 为每个从机创建发布器 ============
  follower_traj_pubs_.resize(num_followers_);
  follower_bezier_pubs_.resize(num_followers_);

  for (int i = 0; i < num_followers_; i++)
  {
    std::string follower_ns = "/uav_" + std::to_string(i + 1);
    
    // 创建Path发布器
    follower_traj_pubs_[i] = nh_.advertise<nav_msgs::Path>(
        follower_ns + "/planning/trajectory", 10);
    
    // 创建Bezier轨迹发布器
    follower_bezier_pubs_[i] = nh_.advertise<ego_planner::Bezier>(
        follower_ns + "/planning/bezier_trajectory", 10);

    ROS_INFO("[SwarmTrajGen] Created publishers for %s", follower_ns.c_str());
  }

  ROS_INFO("[SwarmTrajGen] ============================================");
  ROS_INFO("[SwarmTrajGen] Swarm Trajectory Generator Initialized!");
  ROS_INFO("[SwarmTrajGen] ============================================");
  ROS_INFO("[SwarmTrajGen] Subscribing to:");
  ROS_INFO("[SwarmTrajGen]   - /uav_0/planning/trajectory");
  ROS_INFO("[SwarmTrajGen]   - /uav_0/planning/bezier_trajectory");
  ROS_INFO("[SwarmTrajGen] Publishing to:");
  for (int i = 0; i < num_followers_; i++)
  {
    ROS_INFO("[SwarmTrajGen]   - /uav_%d/planning/trajectory", i + 1);
    ROS_INFO("[SwarmTrajGen]   - /uav_%d/planning/bezier_trajectory", i + 1);
  }
  ROS_INFO("[SwarmTrajGen] ============================================");
  ROS_INFO("[SwarmTrajGen] ");
  ROS_INFO("[SwarmTrajGen] *** 集群算法接口位置 ***");
  ROS_INFO("[SwarmTrajGen] 请在以下位置实现自定义集群算法：");
  ROS_INFO("[SwarmTrajGen] 1. 继承 SwarmAlgorithmBase 类");
  ROS_INFO("[SwarmTrajGen] 2. 实现 computeFollowerTrajectories() 方法");
  ROS_INFO("[SwarmTrajGen] 3. 调用 setSwarmAlgorithm() 设置算法");
  ROS_INFO("[SwarmTrajGen] ");
  ROS_INFO("[SwarmTrajGen] 当前使用默认算法: SimpleCircleFormationAlgorithm");
  ROS_INFO("[SwarmTrajGen] ============================================");
}

void SwarmTrajectoryGenerator::setSwarmAlgorithm(SwarmAlgorithmBase *algorithm)
{
  if (swarm_algorithm_ != nullptr)
  {
    delete swarm_algorithm_;
  }
  swarm_algorithm_ = algorithm;
  ROS_INFO("[SwarmTrajGen] Swarm algorithm updated");
}

void SwarmTrajectoryGenerator::leaderTrajectoryCallback(const nav_msgs::Path::ConstPtr &msg)
{
  if (msg->poses.empty())
  {
    ROS_WARN("[SwarmTrajGen] Received empty leader trajectory");
    return;
  }

  // 提取主机轨迹数据
  leader_positions_.clear();
  leader_time_stamps_.clear();

  for (size_t i = 0; i < msg->poses.size(); i++)
  {
    Eigen::Vector3d pos;
    pos(0) = msg->poses[i].pose.position.x;
    pos(1) = msg->poses[i].pose.position.y;
    pos(2) = msg->poses[i].pose.position.z;
    leader_positions_.push_back(pos);

    // 时间戳（简化处理，使用索引作为时间）
    leader_time_stamps_.push_back(i * 0.1); // 假设10Hz
  }

  // 速度和加速度暂时用零填充（后续可以从轨迹计算）
  leader_velocities_.resize(leader_positions_.size(), Eigen::Vector3d::Zero());
  leader_accelerations_.resize(leader_positions_.size(), Eigen::Vector3d::Zero());

  has_leader_trajectory_ = true;

  // 调用集群算法生成从机轨迹
  if (swarm_algorithm_ != nullptr)
  {
    auto follower_trajectories = swarm_algorithm_->computeFollowerTrajectories(
        leader_positions_, leader_velocities_, leader_accelerations_,
        num_followers_, leader_time_stamps_);

    // 发布从机轨迹
    publishFollowerTrajectories(follower_trajectories);
  }
  else
  {
    ROS_WARN_THROTTLE(1.0, "[SwarmTrajGen] Swarm algorithm not set!");
  }
}

void SwarmTrajectoryGenerator::leaderBezierCallback(const ego_planner::Bezier::ConstPtr &msg)
{
  // TODO: 处理Bezier轨迹消息
  // 这里可以从Bezier曲线中提取轨迹点，然后生成从机轨迹
  // 目前先留空，使用Path消息作为主要接口
  ROS_DEBUG("[SwarmTrajGen] Received leader Bezier trajectory");
}

void SwarmTrajectoryGenerator::publishFollowerTrajectories(
    const std::vector<std::vector<Eigen::Vector3d>> &follower_trajectories)
{
  if (follower_trajectories.size() != static_cast<size_t>(num_followers_))
  {
    ROS_ERROR("[SwarmTrajGen] Follower trajectory count mismatch! Expected %d, got %lu",
              num_followers_, follower_trajectories.size());
    return;
  }

  // 为每个从机发布轨迹
  for (int i = 0; i < num_followers_; i++)
  {
    if (follower_trajectories[i].empty())
    {
      ROS_WARN("[SwarmTrajGen] Empty trajectory for follower %d", i + 1);
      continue;
    }

    // 创建Path消息
    nav_msgs::Path path_msg = createPathMsg(
        follower_trajectories[i],
        leader_time_stamps_,
        world_frame_);

    // 发布
    follower_traj_pubs_[i].publish(path_msg);
  }

  ROS_DEBUG("[SwarmTrajGen] Published trajectories for %d followers", num_followers_);
}

nav_msgs::Path SwarmTrajectoryGenerator::createPathMsg(
    const std::vector<Eigen::Vector3d> &positions,
    const std::vector<double> &time_stamps,
    const std::string &frame_id)
{
  nav_msgs::Path path;
  path.header.frame_id = frame_id;
  path.header.stamp = ros::Time::now();

  for (size_t i = 0; i < positions.size(); i++)
  {
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = frame_id;
    pose.header.stamp = ros::Time::now() + ros::Duration(time_stamps[i]);
    
    pose.pose.position.x = positions[i](0);
    pose.pose.position.y = positions[i](1);
    pose.pose.position.z = positions[i](2);
    
    pose.pose.orientation.w = 1.0;
    pose.pose.orientation.x = 0.0;
    pose.pose.orientation.y = 0.0;
    pose.pose.orientation.z = 0.0;

    path.poses.push_back(pose);
  }

  return path;
}

} // namespace ego_planner
