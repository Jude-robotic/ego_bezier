#include <plan_manage/swarm_master_coordinator.h>

#include <boost/bind.hpp>
#include <cmath>

namespace ego_planner
{

void SwarmMasterCoordinator::init(ros::NodeHandle &nh)
{
  nh_ = nh;

  nh_.param("swarm_master/leader_bezier_topic", leader_bezier_topic_, std::string("/planning/bezier"));
  nh_.param("swarm_master/state_topic_template", state_topic_template_, std::string("/swarm/agent_{id}/state"));
  nh_.param("swarm_master/guidance_topic_template", guidance_topic_template_,
            std::string("/swarm/agent_{id}/guidance_bezier"));

  nh_.param("swarm_master/formation/type", formation_type_, std::string("V"));
  nh_.param("swarm_master/formation/spacing", formation_spacing_, 1.6);
  nh_.param("swarm_master/formation/angle_deg", formation_angle_deg_, 35.0);
  nh_.param("swarm_master/formation/z_offset", formation_z_offset_, 0.0);

  nh_.param("swarm_master/plan_rate", plan_rate_, 20.0);
  nh_.param("swarm_master/safe_radius", safe_radius_, 1.2);
  nh_.param("swarm_master/collision_weight", collision_weight_, 80.0);
  nh_.param("swarm_master/collision_alpha", collision_alpha_, 8.0);
  nh_.param("swarm_master/penalty_step", penalty_step_, 0.002);
  nh_.param("swarm_master/penalty_iters", penalty_iters_, 8);

  nh_.param("swarm_master/start_time_offset", start_time_offset_, 0.08);
  nh_.param("swarm_master/state_timeout", state_timeout_, 0.2);
  nh_.param("swarm_master/guidance_c0_blend_dist", guidance_c0_blend_dist_, 0.3);

  if (!nh_.getParam("swarm_master/agent_ids", agent_ids_) || agent_ids_.empty())
  {
    ROS_WARN("[SwarmMaster] swarm_master/agent_ids not found, fallback to [1,2].");
    agent_ids_.push_back(1);
    agent_ids_.push_back(2);
  }

  leader_bezier_sub_ = nh_.subscribe(leader_bezier_topic_, 10, &SwarmMasterCoordinator::leaderBezierCallback, this);

  for (const int id : agent_ids_)
  {
    const std::string state_topic = formatTopic(state_topic_template_, id);
    const std::string guidance_topic = formatTopic(guidance_topic_template_, id);

    agent_state_subs_.push_back(
        nh_.subscribe<ego_planner::SwarmAgentState>(state_topic, 10,
                                                    boost::bind(&SwarmMasterCoordinator::agentStateCallback, this, _1, id)));

    guidance_pubs_[id] = nh_.advertise<ego_planner::Bezier>(guidance_topic, 10);
    agent_states_[id] = AgentState();

    ROS_INFO("[SwarmMaster] agent=%d, state_topic=%s, guidance_topic=%s", id, state_topic.c_str(),
             guidance_topic.c_str());
  }

  plan_timer_ = nh_.createTimer(ros::Duration(1.0 / std::max(1.0, plan_rate_)),
                                &SwarmMasterCoordinator::planTimerCallback, this);

  ROS_INFO("[SwarmMaster] initialized. leader_topic=%s, agents=%zu", leader_bezier_topic_.c_str(), agent_ids_.size());
}

void SwarmMasterCoordinator::leaderBezierCallback(const ego_planner::BezierConstPtr &msg)
{
  latest_leader_bezier_ = *msg;
  have_leader_traj_ = true;
}

void SwarmMasterCoordinator::agentStateCallback(const ego_planner::SwarmAgentStateConstPtr &msg, int agent_id)
{
  AgentState st;
  st.pos = Eigen::Vector3d(msg->position.x, msg->position.y, msg->position.z);
  st.vel = Eigen::Vector3d(msg->velocity.x, msg->velocity.y, msg->velocity.z);
  st.stamp = msg->header.stamp;
  st.valid = msg->is_valid;
  agent_states_[agent_id] = st;
}

void SwarmMasterCoordinator::planTimerCallback(const ros::TimerEvent & /*e*/)
{
  runOnce();
}

bool SwarmMasterCoordinator::checkReady() const
{
  if (!have_leader_traj_)
    return false;

  const ros::Time now = ros::Time::now();
  for (const int id : agent_ids_)
  {
    auto it = agent_states_.find(id);
    if (it == agent_states_.end() || !it->second.valid)
      return false;

    const double age = (now - it->second.stamp).toSec();
    if (age > state_timeout_)
      return false;
  }

  return true;
}

Eigen::MatrixXd SwarmMasterCoordinator::leaderBezierToCtrlPts(const ego_planner::Bezier &msg) const
{
  Eigen::MatrixXd ctrl_pts(3, msg.pos_pts.size());
  for (size_t i = 0; i < msg.pos_pts.size(); ++i)
  {
    ctrl_pts(0, i) = msg.pos_pts[i].x;
    ctrl_pts(1, i) = msg.pos_pts[i].y;
    ctrl_pts(2, i) = msg.pos_pts[i].z;
  }
  return ctrl_pts;
}

Eigen::Vector3d SwarmMasterCoordinator::computeForwardDir(const Eigen::MatrixXd &ctrl_pts, int cp_idx) const
{
  const int n = ctrl_pts.cols();
  if (n < 2)
    return Eigen::Vector3d(1.0, 0.0, 0.0);

  Eigen::Vector3d tangent = Eigen::Vector3d::Zero();
  if (cp_idx <= 0)
    tangent = ctrl_pts.col(1) - ctrl_pts.col(0);
  else if (cp_idx >= n - 1)
    tangent = ctrl_pts.col(n - 1) - ctrl_pts.col(n - 2);
  else
    tangent = ctrl_pts.col(cp_idx + 1) - ctrl_pts.col(cp_idx - 1);

  tangent(2) = 0.0;
  if (tangent.norm() < 1e-6)
    return Eigen::Vector3d(1.0, 0.0, 0.0);

  return tangent.normalized();
}

void SwarmMasterCoordinator::generateFormationGuidance(
    const Eigen::MatrixXd &leader_ctrl_pts,
    std::unordered_map<int, Eigen::MatrixXd> &agent_ctrl_pts_map) const
{
  const double angle_rad = formation_angle_deg_ * M_PI / 180.0;

  for (size_t k = 0; k < agent_ids_.size(); ++k)
  {
    const int agent_id = agent_ids_[k];

    const int level = static_cast<int>(k / 2) + 1;
    const int side = (k % 2 == 0) ? 1 : -1;

    const double back_dist = level * formation_spacing_ * std::cos(angle_rad);
    const double lateral_dist = side * level * formation_spacing_ * std::sin(angle_rad);

    Eigen::MatrixXd follower_ctrl = leader_ctrl_pts;

    for (int c = 0; c < follower_ctrl.cols(); ++c)
    {
      const Eigen::Vector3d forward = computeForwardDir(leader_ctrl_pts, c);
      Eigen::Vector3d left(-forward.y(), forward.x(), 0.0);
      if (left.norm() < 1e-6)
        left = Eigen::Vector3d(0.0, 1.0, 0.0);
      else
        left.normalize();

      Eigen::Vector3d offset = -forward * back_dist + left * lateral_dist;
      offset.z() += formation_z_offset_;
      follower_ctrl.col(c) += offset;
    }

    agent_ctrl_pts_map[agent_id] = follower_ctrl;
  }
}

void SwarmMasterCoordinator::applySwarmCollisionPenalty(
    std::unordered_map<int, Eigen::MatrixXd> &agent_ctrl_pts_map) const
{
  if (agent_ctrl_pts_map.size() < 2)
    return;

  std::vector<int> ids;
  ids.reserve(agent_ctrl_pts_map.size());
  for (const auto &kv : agent_ctrl_pts_map)
    ids.push_back(kv.first);

  const int cp_num = agent_ctrl_pts_map.begin()->second.cols();

  for (int iter = 0; iter < std::max(1, penalty_iters_); ++iter)
  {
    for (int c = 0; c < cp_num; ++c)
    {
      for (size_t i = 0; i < ids.size(); ++i)
      {
        for (size_t j = i + 1; j < ids.size(); ++j)
        {
          const Eigen::Vector3d pi = agent_ctrl_pts_map[ids[i]].col(c);
          const Eigen::Vector3d pj = agent_ctrl_pts_map[ids[j]].col(c);
          const Eigen::Vector3d diff = pi - pj;
          const double d = diff.norm();
          if (d < 1e-6 || d >= safe_radius_)
            continue;

          const double err = safe_radius_ - d;
          const double gain = collision_weight_ * std::exp(collision_alpha_ * err) * collision_alpha_;
          const Eigen::Vector3d grad = gain * (diff / d);

          agent_ctrl_pts_map[ids[i]].col(c) += penalty_step_ * grad;
          agent_ctrl_pts_map[ids[j]].col(c) -= penalty_step_ * grad;
        }
      }
    }
  }
}

ego_planner::Bezier SwarmMasterCoordinator::buildAgentBezierMsg(int agent_id, const Eigen::MatrixXd &ctrl_pts,
                                                                 const ros::Time &start_time, int64_t traj_id) const
{
  ego_planner::Bezier out;
  out.order = latest_leader_bezier_.order > 0 ? latest_leader_bezier_.order : 3;
  out.traj_id = traj_id;
  out.start_time = start_time;
  out.segment_durations = latest_leader_bezier_.segment_durations;

  out.pos_pts.reserve(ctrl_pts.cols());
  for (int i = 0; i < ctrl_pts.cols(); ++i)
  {
    geometry_msgs::Point p;
    p.x = ctrl_pts(0, i);
    p.y = ctrl_pts(1, i);
    p.z = ctrl_pts(2, i);
    out.pos_pts.push_back(p);
  }

  (void)agent_id;
  return out;
}

std::string SwarmMasterCoordinator::formatTopic(const std::string &topic_template, int agent_id) const
{
  std::string topic = topic_template;
  const std::string key = "{id}";
  const auto pos = topic.find(key);
  if (pos != std::string::npos)
  {
    topic.replace(pos, key.size(), std::to_string(agent_id));
  }
  return topic;
}

void SwarmMasterCoordinator::runOnce()
{
  if (!checkReady())
    return;

  // 关键修复：只在 leader 轨迹发生变化时下发一次引导，避免高频重复发布同一轨迹
  // 反复刷新 start_time 会导致 follower 端 t_cur 长期 < 0，表现为悬停不跟随。
  const bool same_leader_traj =
      (latest_leader_bezier_.traj_id == last_published_leader_traj_id_) &&
      (std::fabs((latest_leader_bezier_.start_time - last_published_leader_start_time_).toSec()) < 1e-4);
  if (same_leader_traj)
    return;

  Eigen::MatrixXd leader_ctrl = leaderBezierToCtrlPts(latest_leader_bezier_);
  if (leader_ctrl.cols() < 4)
    return;

  std::unordered_map<int, Eigen::MatrixXd> agent_ctrl_pts_map;
  generateFormationGuidance(leader_ctrl, agent_ctrl_pts_map);
  applySwarmCollisionPenalty(agent_ctrl_pts_map);

  // === Bug-1 修复：guidance C0/C1 连续性保证 ===
  // 根因：generateFormationGuidance 按新轨迹起点航向计算编队偏置。
  // 当 leader 转弯重规划时，新旧航向可相差数十度，导致 guidance P0 相对
  // 从机当前位置跳变幅度 ≈ formation_spacing (≈1.6 m)，引发从机剧烈抖动。
  //
  // 修复策略：
  //   C0 — 将 guidance P0 强制设为从机实际位置（从机始终从自身当前点出发）
  //   C1 — 将 P1 向"当前速度方向隐含点"混合：
  //        jump < blend_dist  时保留理想编队 P1（稳态精度优先）
  //        jump >= blend_dist 时完全采用速度方向（转弯边界平滑优先）
  {
    const double ts_seg = latest_leader_bezier_.segment_durations.empty()
                              ? 0.1
                              : latest_leader_bezier_.segment_durations[0];
    const int order = latest_leader_bezier_.order > 0 ? latest_leader_bezier_.order : 3;

    for (auto &kv : agent_ctrl_pts_map)
    {
      const int agent_id     = kv.first;
      Eigen::MatrixXd &ctrl  = kv.second;
      if (ctrl.cols() < 2) continue;

      auto state_it = agent_states_.find(agent_id);
      if (state_it == agent_states_.end() || !state_it->second.valid) continue;
      const AgentState &state = state_it->second;

      const Eigen::Vector3d ideal_p0 = ctrl.col(0);
      const double jump              = (ideal_p0 - state.pos).norm();

      // C0：强制起点 = 从机实际位置
      ctrl.col(0) = state.pos;

      // C1：在理想编队 P1 与速度方向 P1 之间线性混合
      //     vel_p1 由 Bezier 微分关系导出：vel(0) = (order/ts)*(P1-P0)
      //     => P1 = P0 + (ts/order)*vel
      const Eigen::Vector3d ideal_p1 = ctrl.col(1);
      const Eigen::Vector3d vel_p1   =
          state.pos + (ts_seg / static_cast<double>(order)) * state.vel;
      const double blend = std::min(1.0, jump / std::max(guidance_c0_blend_dist_, 1e-3));
      ctrl.col(1) = (1.0 - blend) * ideal_p1 + blend * vel_p1;

      if (jump > 0.05)
        ROS_WARN_THROTTLE(0.5,
                          "[SwarmMaster] agent=%d C0_jump=%.3f m blend=%.2f corrected.",
                          agent_id, jump, blend);
    }
  }

  // 直接使用主机轨迹的原始绝对时间戳，保持主从机全局时钟同步。
  // 移除了原先的 start_time_offset(0.08s) 人工延迟：
  //   当主机高频重规划时(>12Hz)，该延迟会导致从机 traj_server 中
  //   t_cur 永远 < 0，不发布任何 pos_cmd，表现为"起飞后悬停不跟随"。
  const ros::Time start_time = latest_leader_bezier_.start_time;

  for (const auto &kv : agent_ctrl_pts_map)
  {
    const int agent_id = kv.first;
    auto pub_it = guidance_pubs_.find(agent_id);
    if (pub_it == guidance_pubs_.end())
      continue;

    const ego_planner::Bezier msg = buildAgentBezierMsg(agent_id, kv.second, start_time, ++out_traj_id_);
    pub_it->second.publish(msg);
  }

  last_published_leader_traj_id_ = latest_leader_bezier_.traj_id;
  last_published_leader_start_time_ = latest_leader_bezier_.start_time;
}

} // namespace ego_planner
