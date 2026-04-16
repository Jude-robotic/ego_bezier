#include <plan_manage/swarm_master_coordinator.h>
#include <bezier_opt/bezier_optimizer.h>
#include <bezier_opt/cooperative_payload_optimizer.h>
#include <bezier_opt/payload_geometry.h>

#include <algorithm>
#include <boost/bind.hpp>
#include <cmath>
#include <limits>
#include <set>
#include <xmlrpcpp/XmlRpcValue.h>

namespace ego_planner
{

namespace
{
constexpr double kFixedObsEps = 1e-6;

double clamp01(const double v)
{
  return std::max(0.0, std::min(1.0, v));
}

Eigen::Vector3d normalizeOrFallback(const Eigen::Vector3d &vec, const Eigen::Vector3d &fallback)
{
  const double n = vec.norm();
  if (n < kFixedObsEps)
    return fallback;
  return vec / n;
}

double blendRamp(const double value, const double start, const double end)
{
  if (end <= start + kFixedObsEps)
    return value >= end ? 1.0 : 0.0;
  return clamp01((value - start) / (end - start));
}
}  // namespace

void SwarmMasterCoordinator::init(ros::NodeHandle &nh, GridMap::Ptr grid_map)
{
  nh_ = nh;

  // === GridMap 初始化 ===
  // 优先使用外部传入的 GridMap（与主机规划栈共享同一地图实例）；
  // 若为空，则自行创建并从 NodeHandle 读取点云/深度话题初始化。
  if (grid_map)
  {
    grid_map_ = grid_map;
    ROS_INFO("[SwarmMaster] Using externally provided GridMap.");
  }
  else
  {
    grid_map_ = std::make_shared<GridMap>();
    grid_map_->initMap(nh_);
    ROS_INFO("[SwarmMaster] GridMap self-initialized from NodeHandle.");
    ROS_WARN("[SwarmMaster] GridMap self-initialized. If no sensor topics are "
             "configured for swarm_master_node, follower collision check will "
             "be automatically disabled (safe fallback to raw V-formation).");
  }

  nh_.param("swarm_master/leader_bezier_topic", leader_bezier_topic_, std::string("/planning/bezier"));
  nh_.param("swarm_master/leader_state_topic", leader_state_topic_, std::string("/visual_slam/odom"));
  nh_.param("swarm_master/leader_corrected_topic", leader_corrected_topic_,
            std::string("/swarm/leader/corrected_bezier"));
  nh_.param("swarm_master/state_topic_template", state_topic_template_, std::string("/swarm/agent_{id}/state"));
  nh_.param("swarm_master/guidance_topic_template", guidance_topic_template_,
            std::string("/swarm/agent_{id}/guidance_bezier"));
  nh_.param("swarm_master/leader_state_timeout", leader_state_timeout_, 0.2);
  nh_.param("swarm_master/leader_state_timeout_grace", leader_state_timeout_grace_, 0.0);

  nh_.param("swarm_master/formation/type", formation_type_, std::string("V"));
  nh_.param("swarm_master/formation/spacing", formation_spacing_, 1.6);
  nh_.param("swarm_master/formation/angle_deg", formation_angle_deg_, 35.0);
  nh_.param("swarm_master/formation/z_offset", formation_z_offset_, 0.0);

  nh_.param("swarm_master/plan_rate", plan_rate_, 20.0);
  nh_.param("swarm_master/safe_radius", safe_radius_, 1.2);
  nh_.param("swarm_master/collision_weight", collision_weight_, 80.0);
  nh_.param("swarm_master/collision_gain_cap", collision_gain_cap_, 200.0);
  nh_.param("swarm_master/penalty_step", penalty_step_, 0.005);
  nh_.param("swarm_master/penalty_iters", penalty_iters_, 3);

  nh_.param("swarm_master/state_timeout", state_timeout_, 0.2);
  nh_.param("swarm_master/state_timeout_grace", state_timeout_grace_, 0.0);
  leader_state_timeout_grace_ = std::max(0.0, leader_state_timeout_grace_);
  state_timeout_grace_ = std::max(0.0, state_timeout_grace_);
  nh_.param("swarm_master/guidance_c0_blend_dist", guidance_c0_blend_dist_, 0.6);
  nh_.param("swarm_master/guidance_c2_blend_weight", guidance_c2_blend_weight_, 0.6);
  nh_.param("swarm_master/guidance_c0_deadzone", guidance_c0_deadzone_, 0.08);
  nh_.param("swarm_master/guidance_log_jump_threshold", guidance_log_jump_threshold_, 0.05);
  nh_.param("swarm_master/guidance_state_prediction_gain", guidance_state_prediction_gain_, 1.0);
  nh_.param("swarm_master/guidance_state_prediction_max_dt", guidance_state_prediction_max_dt_, 0.20);
  nh_.param("swarm_master/guidance_same_traj_c0_max_step", guidance_same_traj_c0_max_step_, 0.0);
  guidance_c0_deadzone_ = std::max(0.0, guidance_c0_deadzone_);
  guidance_log_jump_threshold_ = std::max(0.0, guidance_log_jump_threshold_);
  guidance_state_prediction_gain_ = std::max(0.0, guidance_state_prediction_gain_);
  guidance_state_prediction_max_dt_ = std::max(0.0, guidance_state_prediction_max_dt_);
  guidance_same_traj_c0_max_step_ = std::max(0.0, guidance_same_traj_c0_max_step_);

  // ── 从机避障优化参数（Prompt 4-5 新增）──
  nh_.param("swarm_master/follower_collision_check",       follower_collision_check_,        true);
  nh_.param("swarm_master/follower_optimization_enabled",  follower_optimization_enabled_,   true);
  nh_.param("swarm_master/formation_weight",               formation_weight_,                20.0);
  nh_.param("swarm_master/follower_clearance",             follower_clearance_,              0.4);
  nh_.param("swarm_master/follower_bezier_sample_density", follower_bezier_sample_density_,  4);
  nh_.param("swarm_master/opt_timeout_ms",                 opt_timeout_ms_,                  15.0);
  nh_.param("swarm_master/follower_max_vel",            follower_max_vel_,                 2.0);
  // ── 窄通道编队自适应参数 ───────────────────────────────────────────────
  nh_.param("swarm_master/narrow_passage_threshold", narrow_passage_threshold_, 3.0);
  nh_.param("swarm_master/transition_ctrl_pts",      transition_ctrl_pts_,       6);
  nh_.param("swarm_master/z_formation_max",          z_formation_max_,           2.5);
  nh_.param("swarm_master/z_formation_min",          z_formation_min_,           0.5);
  nh_.param("swarm_master/trailing_safety_margin", trailing_safety_margin_, 3.0);
  nh_.param("swarm_master/narrow_passage_max_age",  narrow_passage_max_age_,  10.0);
  nh_.param("swarm_master/ring_detect_radius",      ring_detect_radius_,      2.0);
  nh_.param("swarm_master/ring_detect_angles",      ring_detect_angles_,      12);
  nh_.param("swarm_master/ring_center_clear_dist",  ring_center_clear_dist_,  1.0);
  nh_.param("swarm_master/ring_surround_ratio",     ring_surround_ratio_,     0.5);
  nh_.param("swarm_master/slit_spread_factor",      slit_spread_factor_,      2.5);
  nh_.param("swarm_master/ring_longitudinal_spacing", ring_longitudinal_spacing_, 1.0);
  nh_.param("swarm_master/ring_compact_lateral_scale", ring_compact_lateral_scale_, 0.35);
  nh_.param("swarm_master/ring_scan_steps",           ring_scan_steps_,           6);
  // ── 障碍物识别去抖动参数 ──────────────────────────────────────────────
  nh_.param("swarm_master/obs_confirm_frames",   obs_confirm_frames_,   4);
  nh_.param("swarm_master/obs_release_duration", obs_release_duration_, 0.5);
  nh_.param("swarm_master/use_fixed_obstacle_schedule", use_fixed_obstacle_schedule_, false);
  nh_.param("swarm_master/disable_online_classification", disable_online_classification_, false);
  nh_.param("swarm_master/refresh_guidance_on_same_leader_traj",
            refresh_guidance_on_same_leader_traj_, false);
  nh_.param("swarm_master/allow_same_traj_refresh_without_fixed_schedule",
            allow_same_traj_refresh_without_fixed_schedule_, false);
  nh_.param("swarm_master/nonfixed_same_traj_refresh_interval",
            nonfixed_same_traj_refresh_interval_, 0.0);
  nh_.param("swarm_master/fixed_obstacle_release_require_payload_clear",
            fixed_obstacle_release_require_payload_clear_, true);
  nh_.param("swarm_master/startup_sync_enabled", startup_sync_enabled_, false);
  nh_.param("swarm_master/startup_sync_release_topic", startup_sync_release_topic_,
            std::string("/swarm/startup_sync/release"));
  nh_.param("swarm_master/startup_sync_same_traj_refresh_rate_hz",
            startup_sync_same_traj_refresh_rate_hz_, 0.0);
  nh_.param("swarm_master/fixed_schedule_same_traj_refresh_min_interval",
            fixed_schedule_same_traj_refresh_min_interval_, 0.0);
  nh_.param("swarm_master/fixed_schedule_same_traj_refresh_min_blend_delta",
            fixed_schedule_same_traj_refresh_min_blend_delta_, 0.0);
    nh_.param("swarm_master/fixed_schedule_same_traj_force_refresh_interval",
        fixed_schedule_same_traj_force_refresh_interval_, 0.0);
  fixed_schedule_same_traj_refresh_min_interval_ =
      std::max(0.0, fixed_schedule_same_traj_refresh_min_interval_);
  fixed_schedule_same_traj_refresh_min_blend_delta_ =
      std::max(0.0, fixed_schedule_same_traj_refresh_min_blend_delta_);
    fixed_schedule_same_traj_force_refresh_interval_ =
      std::max(0.0, fixed_schedule_same_traj_force_refresh_interval_);
  nonfixed_same_traj_refresh_interval_ =
      std::max(0.0, nonfixed_same_traj_refresh_interval_);
  startup_sync_same_traj_refresh_rate_hz_ =
      std::max(0.0, startup_sync_same_traj_refresh_rate_hz_);
  startup_sync_released_ = !startup_sync_enabled_;
  nh_.param("swarm_master/payload/rope_length", payload_rope_length_, 1.0);
  nh_.param("swarm_master/payload/radius", payload_radius_, 0.2);
  nh_.param("swarm_master/payload/extra_margin", payload_extra_margin_, 0.05);
  nh_.param("swarm_master/payload/template_rope_margin", payload_template_rope_margin_, 0.05);
  nh_.param("swarm_master/payload/triangle_area_min", triangle_area_min_, 0.05);
  nh_.param("swarm_master/payload/inter_uav_sep_min", inter_uav_sep_min_, 0.8);
  nh_.param("swarm_master/payload/samples_per_seg", payload_samples_per_seg_, 3);
  nh_.param("swarm_master/cooperative_opt/w_smooth", cooperative_w_smooth_, 10.0);
  nh_.param("swarm_master/cooperative_opt/w_feas", cooperative_w_feas_, 1.0);
  nh_.param("swarm_master/cooperative_opt/w_uav_obs", cooperative_w_uav_obs_, 100.0);
  nh_.param("swarm_master/cooperative_opt/w_leader_ref", cooperative_w_leader_ref_, 50.0);
  nh_.param("swarm_master/cooperative_opt/w_mode_shape", cooperative_w_mode_shape_, 30.0);
  nh_.param("swarm_master/cooperative_opt/w_payload_feas", cooperative_w_payload_feas_, 120.0);
  nh_.param("swarm_master/cooperative_opt/w_payload_obs", cooperative_w_payload_obs_, 160.0);
  nh_.param("swarm_master/cooperative_opt/w_sep", cooperative_w_sep_, 40.0);
  nh_.param("swarm_master/cooperative_opt/payload_invalid_penalty",
            cooperative_payload_invalid_penalty_, 2000.0);
  nh_.param("swarm_master/cooperative_opt/max_iterations", cooperative_max_iterations_, 60);
  nh_.param("swarm_master/cooperative_opt/finite_diff_eps", cooperative_fd_eps_, 1e-3);
  nh_.param("swarm_master/cooperative_opt/obstacle_search_radius",
            cooperative_obstacle_search_radius_, 1.5);
  nh_.param("swarm_master/online_payload_opt_enabled", online_payload_opt_enabled_, true);
  nh_.param("swarm_master/online_payload_window_segs", online_payload_window_segs_, 3);
  online_payload_window_segs_ = std::max(2, online_payload_window_segs_);
  ROS_INFO("[SwarmMaster] payload online opt: enabled=%d window_segs=%d",
           static_cast<int>(online_payload_opt_enabled_), online_payload_window_segs_);
  ROS_INFO("[SwarmMaster] same leader traj refresh: enabled=%d",
           static_cast<int>(refresh_guidance_on_same_leader_traj_));
  ROS_INFO("[SwarmMaster] nonfixed same-traj refresh gate: allow_without_fixed=%d interval=%.3fs startup_sync_rate=%.2fHz",
           static_cast<int>(allow_same_traj_refresh_without_fixed_schedule_),
           nonfixed_same_traj_refresh_interval_, startup_sync_same_traj_refresh_rate_hz_);
  ROS_INFO("[SwarmMaster] startup-sync gate: enabled=%d released=%d topic=%s",
           static_cast<int>(startup_sync_enabled_), static_cast<int>(startup_sync_released_),
           startup_sync_release_topic_.c_str());
  ROS_INFO("[SwarmMaster] fixed schedule release: require_payload_clear=%d",
           static_cast<int>(fixed_obstacle_release_require_payload_clear_));
  ROS_INFO("[SwarmMaster] fixed schedule refresh gate: min_interval=%.2fs min_blend_delta=%.2f force_interval=%.2fs",
           fixed_schedule_same_traj_refresh_min_interval_,
           fixed_schedule_same_traj_refresh_min_blend_delta_,
           fixed_schedule_same_traj_force_refresh_interval_);
  ROS_INFO("[SwarmMaster] payload template guard: rope_margin=%.2f",
           payload_template_rope_margin_);
  ROS_INFO("[SwarmMaster] ready gate timeout: leader=%.2fs (+%.2fs grace), agent=%.2fs (+%.2fs grace)",
           leader_state_timeout_, leader_state_timeout_grace_,
           state_timeout_, state_timeout_grace_);
  ROS_INFO("[SwarmMaster] obs debounce: confirm_frames=%d release_duration=%.2fs",
           obs_confirm_frames_, obs_release_duration_);
  ROS_INFO("[SwarmMaster] ring detect: radius=%.1f angles=%d center_clear=%.1f surround_ratio=%.1f",
           ring_detect_radius_, ring_detect_angles_, ring_center_clear_dist_, ring_surround_ratio_);
  ROS_INFO("[SwarmMaster] narrow passage: threshold=%.2f m, transition_pts=%d, "
           "z=[%.2f, %.2f]",
           narrow_passage_threshold_, transition_ctrl_pts_, z_formation_min_, z_formation_max_);
  ROS_INFO("[SwarmMaster] follower opt: collision_check=%d opt_enabled=%d "
           "formation_weight=%.1f clearance=%.2f timeout_ms=%.1f",
           (int)follower_collision_check_, (int)follower_optimization_enabled_,
           formation_weight_, follower_clearance_, opt_timeout_ms_);

  if (!nh_.getParam("swarm_master/agent_ids", agent_ids_) || agent_ids_.empty())
  {
    ROS_WARN("[SwarmMaster] swarm_master/agent_ids not found, fallback to [1,2].");
    agent_ids_.push_back(1);
    agent_ids_.push_back(2);
  }

  leader_bezier_sub_ = nh_.subscribe(leader_bezier_topic_, 10, &SwarmMasterCoordinator::leaderBezierCallback, this);
  leader_state_sub_ = nh_.subscribe(leader_state_topic_, 20, &SwarmMasterCoordinator::leaderStateCallback, this);
  if (startup_sync_enabled_)
  {
    startup_sync_release_sub_ = nh_.subscribe(startup_sync_release_topic_, 10,
                                              &SwarmMasterCoordinator::startupSyncReleaseCallback, this);
  }
  leader_corrected_pub_ = nh_.advertise<ego_planner::Bezier>(leader_corrected_topic_, 10);
  loadFixedObstacleSchedule();

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

  obs_state_marker_pub_ = nh_.advertise<visualization_msgs::Marker>(
      "/swarm/master/obs_state_marker", 1, true);


  ROS_INFO("[SwarmMaster] initialized. leader_topic=%s corrected_topic=%s agents=%zu fixed_schedule=%d",
           leader_bezier_topic_.c_str(), leader_corrected_topic_.c_str(), agent_ids_.size(),
           static_cast<int>(use_fixed_obstacle_schedule_));
}

void SwarmMasterCoordinator::leaderBezierCallback(const ego_planner::BezierConstPtr &msg)
{
  latest_leader_bezier_ = *msg;
  have_leader_traj_ = true;
}

void SwarmMasterCoordinator::leaderStateCallback(const nav_msgs::OdometryConstPtr &msg)
{
  leader_state_.pos = Eigen::Vector3d(msg->pose.pose.position.x,
                                      msg->pose.pose.position.y,
                                      msg->pose.pose.position.z);
  leader_state_.vel = Eigen::Vector3d(msg->twist.twist.linear.x,
                                      msg->twist.twist.linear.y,
                                      msg->twist.twist.linear.z);
  leader_state_.stamp = msg->header.stamp;
  leader_state_.valid = true;
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

void SwarmMasterCoordinator::startupSyncReleaseCallback(const std_msgs::BoolConstPtr &msg)
{
  if (!startup_sync_enabled_)
  {
    return;
  }

  if (msg->data && !startup_sync_released_)
  {
    ROS_INFO("[SwarmMaster] startup-sync release received.");
  }
  if (!msg->data && startup_sync_released_)
  {
    ROS_WARN("[SwarmMaster] startup-sync release reset to false.");
  }

  startup_sync_released_ = msg->data;
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
  const double leader_timeout = leader_state_timeout_ + leader_state_timeout_grace_;
  const double agent_timeout = state_timeout_ + state_timeout_grace_;
  if (use_fixed_obstacle_schedule_)
  {
    if (!leader_state_.valid)
      return false;
    const double age = (now - leader_state_.stamp).toSec();
    if (age > leader_timeout)
      return false;
  }

  for (const int id : agent_ids_)
  {
    auto it = agent_states_.find(id);
    if (it == agent_states_.end() || !it->second.valid)
      return false;

    const double age = (now - it->second.stamp).toSec();
    if (age > agent_timeout)
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

Eigen::MatrixXd SwarmMasterCoordinator::computeSmoothedHeadings(const Eigen::MatrixXd &ctrl_pts) const
{
    const int N = ctrl_pts.cols();           // 总控制点数 = 3*num_seg + 1
    const int order = latest_leader_bezier_.order > 0 ? latest_leader_bezier_.order : 3;
    const int num_seg = (N - 1) / order;     // Bezier 段数
    Eigen::MatrixXd headings(3, N);
    const Eigen::Vector3d fallback(1.0, 0.0, 0.0);

    if (N < 2) {
      headings.col(0) = fallback;
      return headings;
    }

    // --- Step 1: 对每个控制点求其所在 Bezier 段的曲线真实切线 B'(t) ---
    // 对 3 阶 Bezier: B'(t)=3(1-t)^2(P1-P0)+6(1-t)t(P2-P1)+3t^2(P3-P2)
    for (int c = 0; c < N; ++c)
    {
      int seg = std::min(c / order, num_seg - 1);   // 所属段号
      double u = static_cast<double>(c - seg * order) / static_cast<double>(order);  // 段内局部参数 [0,1]
      // 段端点处 u=1 等价于下一段 u=0，保持在当前段
      u = std::min(u, 1.0);

      const int base = seg * order;  // 段首控制点索引
      const Eigen::Vector3d &P0 = ctrl_pts.col(base);
      const Eigen::Vector3d &P1 = ctrl_pts.col(base + 1);
      const Eigen::Vector3d &P2 = ctrl_pts.col(base + 2);
      const Eigen::Vector3d &P3 = ctrl_pts.col(base + 3);

      // 3 阶 Bezier 导数
      Eigen::Vector3d tangent = 3.0 * (1.0 - u) * (1.0 - u) * (P1 - P0)
                              + 6.0 * (1.0 - u) * u       * (P2 - P1)
                              + 3.0 * u * u               * (P3 - P2);
      tangent(2) = 0.0;  // 只取 XY 平面航向
      headings.col(c) = tangent;
    }

    // --- Step 2: 滑动平均平滑（窗口=5）消除段间接缝微小抖动 ---
    const int half_win = 2;  // 窗口半宽
    Eigen::MatrixXd smoothed(3, N);
    for (int c = 0; c < N; ++c)
    {
      Eigen::Vector3d sum = Eigen::Vector3d::Zero();
      int cnt = 0;
      for (int w = c - half_win; w <= c + half_win; ++w)
      {
        if (w >= 0 && w < N) {
          sum += headings.col(w);
          ++cnt;
        }
      }
      smoothed.col(c) = sum / static_cast<double>(cnt);
    }

    // --- Step 3: 归一化 ---
    for (int c = 0; c < N; ++c)
    {
      if (smoothed.col(c).norm() < 1e-6)
        smoothed.col(c) = fallback;
      else
        smoothed.col(c).normalize();
    }

    return smoothed;
}

std::string SwarmMasterCoordinator::obstacleTypeName(const ObstacleType type) const
{
  switch (type)
  {
    case ObstacleType::DOOR_FRAME:
      return "DOOR_FRAME";
    case ObstacleType::RING:
      return "RING";
    case ObstacleType::Z_SLIT:
      return "Z_SLIT";
    case ObstacleType::NONE:
    default:
      return "NONE";
  }
}

void SwarmMasterCoordinator::buildDefaultFixedObstacleSchedule()
{
  fixed_obstacle_schedule_.clear();

  auto add_spec = [&](FixedObstacleSpec spec) {
    spec.id = static_cast<int>(fixed_obstacle_schedule_.size());
    spec.forward_axis = normalizeOrFallback(spec.forward_axis, Eigen::Vector3d::UnitX());
    Eigen::Vector3d local_y = spec.world_from_local.col(1);
    local_y = normalizeOrFallback(local_y, Eigen::Vector3d::UnitY());
    Eigen::Vector3d local_z = spec.world_from_local.col(2);
    local_z = normalizeOrFallback(local_z, Eigen::Vector3d::UnitZ());
    spec.world_from_local.col(0) = spec.forward_axis;
    spec.world_from_local.col(1) = local_y;
    spec.world_from_local.col(2) = local_z;
    fixed_obstacle_schedule_.push_back(spec);
  };

  FixedObstacleSpec ring1;
  ring1.name = "ring1";
  ring1.type = ObstacleType::RING;
  ring1.center = Eigen::Vector3d(-7.0, 1.0, 1.5);
  ring1.forward_axis = Eigen::Vector3d::UnitX();
  ring1.world_from_local = Eigen::Matrix3d::Identity();
  ring1.primary_axis = ring1.world_from_local.col(0);
  ring1.auxiliary_axis = ring1.world_from_local.col(2);
  ring1.physical_half_extent = 0.6;
  ring1.activation_window_enter = -1.0;
  ring1.activation_window_exit = 0.6;
  ring1.blend_in = 2.4;
  ring1.blend_out = 1.2;
  ring1.release_margin = 0.8;
  ring1.opt_window_margin = 2;
  ring1.major_r = 1.8;
  ring1.minor_r = 0.4;
  ring1.mode_template.back_offset = 1.0;
  ring1.mode_template.primary_span = 1.15;
  ring1.mode_template.aux_span = 0.28;
  ring1.mode_template.leader_bias_local = Eigen::Vector3d(0.0, -0.25, 0.12);
  add_spec(ring1);

  FixedObstacleSpec slit;
  slit.name = "slit";
  slit.type = ObstacleType::Z_SLIT;
  slit.center = Eigen::Vector3d(-2.0, 0.0, 1.3);
  slit.forward_axis = Eigen::Vector3d::UnitX();
  slit.world_from_local = Eigen::Matrix3d::Identity();
  slit.primary_axis = slit.world_from_local.col(1);
  slit.auxiliary_axis = slit.world_from_local.col(2);
  slit.physical_half_extent = 0.2;
  slit.activation_window_enter = -0.9;
  slit.activation_window_exit = 0.2;
  slit.blend_in = 2.2;
  slit.blend_out = 1.2;
  slit.release_margin = 0.8;
  slit.opt_window_margin = 2;
  slit.z_gap_low = 0.2;
  slit.z_gap_high = 2.0;
  slit.thickness = 0.4;
  slit.mode_template.back_offset = 1.0;
  slit.mode_template.primary_span = 1.4;
  slit.mode_template.aux_span = 0.22;
  slit.mode_template.leader_bias_local = Eigen::Vector3d(0.0, 0.0, 0.12);
  add_spec(slit);

  FixedObstacleSpec door2;
  door2.name = "door2";
  door2.type = ObstacleType::DOOR_FRAME;
  door2.center = Eigen::Vector3d(6.0, -0.5, 1.5);
  door2.forward_axis = Eigen::Vector3d::UnitX();
  door2.world_from_local = Eigen::Matrix3d::Identity();
  door2.primary_axis = door2.world_from_local.col(2);
  door2.auxiliary_axis = door2.world_from_local.col(1);
  door2.physical_half_extent = 0.2;
  door2.activation_window_enter = -1.0;
  door2.activation_window_exit = 0.2;
  door2.blend_in = 2.4;
  door2.blend_out = 1.2;
  door2.release_margin = 0.8;
  door2.opt_window_margin = 2;
  door2.gap_center_y = 0.0;
  door2.gap_width = 1.3;
  door2.thickness = 0.4;
  door2.mode_template.back_offset = 1.05;
  door2.mode_template.primary_span = 1.15;
  door2.mode_template.aux_span = 0.18;
  door2.mode_template.leader_bias_local = Eigen::Vector3d(0.0, -0.05, 0.0);
  add_spec(door2);

  FixedObstacleSpec ring2 = ring1;
  ring2.name = "ring2";
  ring2.center = Eigen::Vector3d(14.0, 1.5, 1.5);
  ring2.major_r = 1.4;
  ring2.minor_r = 0.35;
  ring2.physical_half_extent = 0.5;
  ring2.activation_window_enter = -0.5;
  ring2.activation_window_exit = 0.5;
  ring2.blend_in = 1.2;
  ring2.opt_window_margin = 1;
  const double tilt = 15.0 * M_PI / 180.0;
  ring2.forward_axis = Eigen::Vector3d(std::cos(tilt), 0.0, std::sin(tilt));
  ring2.world_from_local.col(0) = ring2.forward_axis;
  ring2.world_from_local.col(1) = Eigen::Vector3d::UnitY();
  ring2.world_from_local.col(2) = ring2.forward_axis.cross(Eigen::Vector3d::UnitY());
  ring2.world_from_local.col(2) = normalizeOrFallback(ring2.world_from_local.col(2), Eigen::Vector3d::UnitZ());
  ring2.primary_axis = ring2.world_from_local.col(0);
  ring2.auxiliary_axis = ring2.world_from_local.col(2);
  ring2.mode_template.back_offset = 0.9;
  ring2.mode_template.primary_span = 0.95;
  ring2.mode_template.aux_span = 0.24;
  ring2.mode_template.leader_bias_local = Eigen::Vector3d(0.0, 0.0, 0.16);
  add_spec(ring2);
}

void SwarmMasterCoordinator::loadFixedObstacleSchedule()
{
  buildDefaultFixedObstacleSchedule();

  XmlRpc::XmlRpcValue schedule_param;
  if (nh_.getParam("swarm_master/fixed_obstacle_schedule", schedule_param) &&
      schedule_param.getType() == XmlRpc::XmlRpcValue::TypeArray)
  {
    auto read_double = [](const XmlRpc::XmlRpcValue &v, const char *key, double fallback) {
      if (!v.hasMember(key))
        return fallback;
      if (v[key].getType() == XmlRpc::XmlRpcValue::TypeDouble)
        return static_cast<double>(v[key]);
      if (v[key].getType() == XmlRpc::XmlRpcValue::TypeInt)
        return static_cast<double>(v[key]);
      return fallback;
    };
    auto read_int = [](const XmlRpc::XmlRpcValue &v, const char *key, int fallback) {
      if (!v.hasMember(key) || v[key].getType() != XmlRpc::XmlRpcValue::TypeInt)
        return fallback;
      return static_cast<int>(v[key]);
    };
    auto read_bool = [](const XmlRpc::XmlRpcValue &v, const char *key, bool fallback) {
      if (!v.hasMember(key) || v[key].getType() != XmlRpc::XmlRpcValue::TypeBoolean)
        return fallback;
      return static_cast<bool>(v[key]);
    };
    auto read_string = [](const XmlRpc::XmlRpcValue &v, const char *key, const std::string &fallback) {
      if (!v.hasMember(key) || v[key].getType() != XmlRpc::XmlRpcValue::TypeString)
        return fallback;
      return static_cast<std::string>(v[key]);
    };
    auto read_vec3 = [&](const XmlRpc::XmlRpcValue &v, const char *key, const Eigen::Vector3d &fallback) {
      if (!v.hasMember(key) || v[key].getType() != XmlRpc::XmlRpcValue::TypeArray || v[key].size() != 3)
        return fallback;
      Eigen::Vector3d out = fallback;
      for (int i = 0; i < 3; ++i)
      {
        if (v[key][i].getType() == XmlRpc::XmlRpcValue::TypeDouble)
          out(i) = static_cast<double>(v[key][i]);
        else if (v[key][i].getType() == XmlRpc::XmlRpcValue::TypeInt)
          out(i) = static_cast<int>(v[key][i]);
      }
      return out;
    };

    std::vector<FixedObstacleSpec> loaded;
    for (int i = 0; i < schedule_param.size(); ++i)
    {
      if (schedule_param[i].getType() != XmlRpc::XmlRpcValue::TypeStruct)
        continue;

      FixedObstacleSpec spec;
      if (i < static_cast<int>(fixed_obstacle_schedule_.size()))
        spec = fixed_obstacle_schedule_[static_cast<size_t>(i)];

      spec.id = read_int(schedule_param[i], "id", i);
      spec.name = read_string(schedule_param[i], "name", spec.name);
      const std::string type_str = read_string(schedule_param[i], "type", obstacleTypeName(spec.type));
      if (type_str == "DOOR_FRAME")
        spec.type = ObstacleType::DOOR_FRAME;
      else if (type_str == "RING")
        spec.type = ObstacleType::RING;
      else if (type_str == "Z_SLIT")
        spec.type = ObstacleType::Z_SLIT;
      else
        spec.type = ObstacleType::NONE;

      spec.center = read_vec3(schedule_param[i], "center", spec.center);
      spec.forward_axis = normalizeOrFallback(read_vec3(schedule_param[i], "forward_axis", spec.forward_axis),
                                              spec.forward_axis);
      spec.primary_axis = normalizeOrFallback(read_vec3(schedule_param[i], "primary_axis", spec.primary_axis),
                                              spec.primary_axis);
      spec.auxiliary_axis = normalizeOrFallback(read_vec3(schedule_param[i], "auxiliary_axis", spec.auxiliary_axis),
                                                spec.auxiliary_axis);
      spec.world_from_local.col(0) = spec.forward_axis;
      spec.world_from_local.col(1) = normalizeOrFallback(read_vec3(schedule_param[i], "local_y_axis",
                                                                  spec.world_from_local.col(1)),
                                                         spec.world_from_local.col(1));
      spec.world_from_local.col(2) = normalizeOrFallback(read_vec3(schedule_param[i], "local_z_axis",
                                                                  spec.world_from_local.col(2)),
                                                         spec.world_from_local.col(2));
      spec.physical_half_extent = read_double(schedule_param[i], "physical_half_extent", spec.physical_half_extent);
      spec.activation_window_enter = read_double(schedule_param[i], "activation_window_enter", spec.activation_window_enter);
      spec.activation_window_exit = read_double(schedule_param[i], "activation_window_exit", spec.activation_window_exit);
      spec.blend_in = read_double(schedule_param[i], "blend_in", spec.blend_in);
      spec.blend_out = read_double(schedule_param[i], "blend_out", spec.blend_out);
      spec.release_margin = read_double(schedule_param[i], "release_margin", spec.release_margin);
      spec.opt_window_margin = read_int(schedule_param[i], "opt_window_margin", spec.opt_window_margin);
      spec.gap_center_y = read_double(schedule_param[i], "gap_center_y", spec.gap_center_y);
      spec.gap_width = read_double(schedule_param[i], "gap_width", spec.gap_width);
      spec.thickness = read_double(schedule_param[i], "thickness", spec.thickness);
      spec.z_gap_low = read_double(schedule_param[i], "z_gap_low", spec.z_gap_low);
      spec.z_gap_high = read_double(schedule_param[i], "z_gap_high", spec.z_gap_high);
      spec.major_r = read_double(schedule_param[i], "major_r", spec.major_r);
      spec.minor_r = read_double(schedule_param[i], "minor_r", spec.minor_r);
      spec.mode_template.back_offset =
          read_double(schedule_param[i], "template_back_offset", spec.mode_template.back_offset);
      spec.mode_template.primary_span =
          read_double(schedule_param[i], "template_primary_span", spec.mode_template.primary_span);
      spec.mode_template.aux_span =
          read_double(schedule_param[i], "template_aux_span", spec.mode_template.aux_span);
      spec.mode_template.leader_bias_local =
          read_vec3(schedule_param[i], "template_leader_bias_local", spec.mode_template.leader_bias_local);

      if (schedule_param[i].hasMember("leader_traction") &&
          schedule_param[i]["leader_traction"].getType() == XmlRpc::XmlRpcValue::TypeStruct)
      {
        const XmlRpc::XmlRpcValue &traction = schedule_param[i]["leader_traction"];
        spec.leader_traction.enabled =
            read_bool(traction, "enabled", spec.leader_traction.enabled);
        spec.leader_traction.window_enter =
            read_double(traction, "window_enter", spec.leader_traction.window_enter);
        spec.leader_traction.window_exit =
            read_double(traction, "window_exit", spec.leader_traction.window_exit);
        spec.leader_traction.blend_in =
            read_double(traction, "blend_in", spec.leader_traction.blend_in);
        spec.leader_traction.blend_out =
            read_double(traction, "blend_out", spec.leader_traction.blend_out);
        spec.leader_traction.clearance_margin =
            read_double(traction, "clearance_margin", spec.leader_traction.clearance_margin);
        spec.leader_traction.bias_local =
            read_vec3(traction, "bias_local", spec.leader_traction.bias_local);
        spec.leader_traction.debug =
            read_bool(traction, "debug", spec.leader_traction.debug);
      }
      spec.leader_traction.blend_in = std::max(0.0, spec.leader_traction.blend_in);
      spec.leader_traction.blend_out = std::max(0.0, spec.leader_traction.blend_out);
      spec.leader_traction.clearance_margin =
          std::max(0.0, spec.leader_traction.clearance_margin);
      loaded.push_back(spec);
    }

    if (!loaded.empty())
      fixed_obstacle_schedule_ = loaded;
  }

  for (auto &spec : fixed_obstacle_schedule_)
    enforcePayloadFeasibleTemplate(spec);

  fixed_obstacle_runtimes_.assign(fixed_obstacle_schedule_.size(), FixedObstacleRuntime());
  current_fixed_obstacle_idx_ = 0;
  ROS_INFO("[SwarmMaster] fixed obstacle schedule loaded: %zu obstacle(s)", fixed_obstacle_schedule_.size());

  if (debug_fixed_schedule_manifest_)
  {
    const double target_radius = payload_rope_length_ - payload_template_rope_margin_;
    for (const auto &spec : fixed_obstacle_schedule_)
    {
      ROS_INFO("[SwarmMaster][ScheduleManifest] id=%d name=%s type=%s center=(%.2f, %.2f, %.2f) "
               "window=[%.2f, %.2f] blend=[%.2f, %.2f] release=%.2f template=(back=%.2f primary=%.2f aux=%.2f) "
               "payload_R=%.3f target<=%.3f traction=(enabled=%d enter=%.2f exit=%.2f blend_in=%.2f blend_out=%.2f margin=%.2f)",
               spec.id, spec.name.c_str(), obstacleTypeName(spec.type).c_str(),
               spec.center.x(), spec.center.y(), spec.center.z(),
               spec.activation_window_enter, spec.activation_window_exit,
               spec.blend_in, spec.blend_out, spec.release_margin,
               spec.mode_template.back_offset, spec.mode_template.primary_span,
               spec.mode_template.aux_span, computeTemplateCircumradius(spec), target_radius,
               static_cast<int>(spec.leader_traction.enabled),
               spec.leader_traction.window_enter, spec.leader_traction.window_exit,
               spec.leader_traction.blend_in, spec.leader_traction.blend_out,
               spec.leader_traction.clearance_margin);
    }
  }
}

double SwarmMasterCoordinator::computeTemplateCircumradius(const FixedObstacleSpec &spec) const
{
  const double back_offset = spec.mode_template.back_offset;
  const double aux_abs_min = triangle_area_min_ / std::max(back_offset, 0.2);
  const double aux_span = std::max(std::abs(spec.mode_template.aux_span), aux_abs_min);

  const Eigen::Vector3d follower1 =
      -back_offset * spec.forward_axis +
      0.5 * spec.mode_template.primary_span * spec.primary_axis +
      aux_span * spec.auxiliary_axis +
      Eigen::Vector3d(0.0, 0.0, formation_z_offset_);
  const Eigen::Vector3d follower2 =
      -back_offset * spec.forward_axis -
      0.5 * spec.mode_template.primary_span * spec.primary_axis -
      aux_span * spec.auxiliary_axis +
      Eigen::Vector3d(0.0, 0.0, formation_z_offset_);

  const payload_geometry::CircumradiusGrad circum =
      payload_geometry::computeCircumradiusAndGrad(Eigen::Vector3d::Zero(), follower1, follower2);
  if (!circum.valid)
    return std::numeric_limits<double>::infinity();
  return circum.circumradius;
}

void SwarmMasterCoordinator::enforcePayloadFeasibleTemplate(FixedObstacleSpec &spec) const
{
  if (spec.type == ObstacleType::NONE)
    return;

  const double target_radius = payload_rope_length_ - payload_template_rope_margin_;
  if (target_radius <= 1e-6)
  {
    ROS_WARN_THROTTLE(1.0,
                      "[SwarmMaster] payload/template_rope_margin=%.3f is too large for rope_length=%.3f, skip template guard.",
                      payload_template_rope_margin_, payload_rope_length_);
    return;
  }

  const double template_radius = computeTemplateCircumradius(spec);
  if (!std::isfinite(template_radius) || template_radius <= target_radius + 1e-6)
    return;

  const double scale = std::max(1e-3, std::min(1.0, target_radius / template_radius));
  spec.mode_template.back_offset *= scale;
  spec.mode_template.primary_span *= scale;
  spec.mode_template.aux_span *= scale;

  const double guarded_radius = computeTemplateCircumradius(spec);
  ROS_WARN("[SwarmMaster] obstacle=%s template scaled by %.3f for payload feasibility "
           "(R: %.3f -> %.3f, target<=%.3f, rope=%.3f, margin=%.3f).",
           spec.name.c_str(), scale, template_radius, guarded_radius, target_radius,
           payload_rope_length_, payload_template_rope_margin_);
}

Eigen::Vector3d SwarmMasterCoordinator::obstacleLocal(const int obstacle_id, const Eigen::Vector3d &world_pt) const
{
  if (obstacle_id < 0 || obstacle_id >= static_cast<int>(fixed_obstacle_schedule_.size()))
    return world_pt;

  const FixedObstacleSpec &spec = fixed_obstacle_schedule_[static_cast<size_t>(obstacle_id)];
  return spec.world_from_local.transpose() * (world_pt - spec.center);
}

double SwarmMasterCoordinator::obstacleAlong(const int obstacle_id, const Eigen::Vector3d &world_pt) const
{
  return obstacleLocal(obstacle_id, world_pt).x();
}

Eigen::Vector3d SwarmMasterCoordinator::computeObstacleLeaderTractionTargetWorld(
    const FixedObstacleSpec &spec,
    const Eigen::Vector3d &reference_world) const
{
  Eigen::Vector3d local_target = spec.world_from_local.transpose() * (reference_world - spec.center);

  switch (spec.type)
  {
    case ObstacleType::DOOR_FRAME:
    {
      const double half_gap = 0.5 * std::max(0.0, spec.gap_width);
      local_target.y() = std::max(spec.gap_center_y - half_gap,
                                  std::min(spec.gap_center_y + half_gap, local_target.y()));
      local_target += spec.mode_template.leader_bias_local;
      break;
    }
    case ObstacleType::Z_SLIT:
      local_target.z() = std::max(spec.z_gap_low, std::min(spec.z_gap_high, local_target.z()));
      local_target += spec.mode_template.leader_bias_local;
      break;
    case ObstacleType::RING:
    case ObstacleType::NONE:
    default:
      local_target += spec.mode_template.leader_bias_local;
      break;
  }

  return spec.center + spec.world_from_local * local_target;
}

double SwarmMasterCoordinator::computeObstacleSignedClearance(const FixedObstacleSpec &spec,
                                                              const Eigen::Vector3d &point_world,
                                                              double radius) const
{
  const double effective_radius = std::max(0.0, radius);
  switch (spec.type)
  {
    case ObstacleType::DOOR_FRAME:
      return payload_geometry::doorFrameSignedClearance(
          payload_geometry::LocalFrame{spec.center, spec.world_from_local},
          point_world, spec.gap_center_y, spec.gap_width, effective_radius,
          spec.physical_half_extent);
    case ObstacleType::Z_SLIT:
      return payload_geometry::slitSignedClearance(
          payload_geometry::LocalFrame{spec.center, spec.world_from_local},
          point_world, spec.z_gap_low, spec.z_gap_high, effective_radius,
          spec.physical_half_extent);
    case ObstacleType::RING:
      return payload_geometry::ringSignedClearance(
          payload_geometry::LocalFrame{spec.center, spec.world_from_local},
          point_world, spec.major_r, spec.minor_r, effective_radius);
    case ObstacleType::NONE:
    default:
      return std::numeric_limits<double>::quiet_NaN();
  }
}

double SwarmMasterCoordinator::getSegmentDuration() const
{
  return latest_leader_bezier_.segment_durations.empty() ? 0.1 : latest_leader_bezier_.segment_durations[0];
}

Eigen::Vector3d SwarmMasterCoordinator::evalBezierPosition(const Eigen::MatrixXd &ctrl_pts, double t) const
{
  const int order = latest_leader_bezier_.order > 0 ? latest_leader_bezier_.order : 3;
  const double ts = getSegmentDuration();
  if (ctrl_pts.cols() < order + 1 || ts <= 1e-6)
  {
    if (ctrl_pts.cols() > 0)
      return ctrl_pts.col(0);
    return Eigen::Vector3d::Zero();
  }

  const int num_seg = (ctrl_pts.cols() - 1) / order;
  const double total_time = num_seg * ts;
  const double clamped_t = std::max(0.0, std::min(t, std::max(0.0, total_time - 1e-6)));
  const int seg = std::min(num_seg - 1, static_cast<int>(clamped_t / ts));
  const double local_t = std::max(0.0, clamped_t - seg * ts);
  const double u = std::min(1.0, local_t / ts);
  const double mu = 1.0 - u;
  const int base = seg * order;
  return mu * mu * mu * ctrl_pts.col(base) +
         3.0 * mu * mu * u * ctrl_pts.col(base + 1) +
         3.0 * mu * u * u * ctrl_pts.col(base + 2) +
         u * u * u * ctrl_pts.col(base + 3);
}

Eigen::Vector3d SwarmMasterCoordinator::evalCurrentLeaderNominalPosition(const Eigen::MatrixXd &leader_ctrl_pts) const
{
  const double t_now = (ros::Time::now() - latest_leader_bezier_.start_time).toSec();
  return evalBezierPosition(leader_ctrl_pts, t_now);
}

bool SwarmMasterCoordinator::computePayloadPositionFromStates(Eigen::Vector3d &payload_center) const
{
  if (!leader_state_.valid || agent_ids_.size() < 2)
    return false;

  auto it1 = agent_states_.find(agent_ids_[0]);
  auto it2 = agent_states_.find(agent_ids_[1]);
  if (it1 == agent_states_.end() || it2 == agent_states_.end() || !it1->second.valid || !it2->second.valid)
    return false;

  const payload_geometry::PayloadSolution payload = payload_geometry::solvePayloadCenterLowerBranch(
      leader_state_.pos, it1->second.pos, it2->second.pos, payload_rope_length_);
  if (!payload.valid)
    return false;

  payload_center = payload.selected_center;
  return true;
}

void SwarmMasterCoordinator::generateFormationGuidance(
    const Eigen::MatrixXd &leader_ctrl_pts,
    std::unordered_map<int, Eigen::MatrixXd> &agent_ctrl_pts_map,
    const std::vector<FormationCommand>& formation_cmds)
{
  const double angle_rad = formation_angle_deg_ * M_PI / 180.0;
  const int N = leader_ctrl_pts.cols();

  // 一次性计算整条曲线上所有控制点的平滑航向 (Layer-A)
  const Eigen::MatrixXd headings = computeSmoothedHeadings(leader_ctrl_pts);

  // ── 窄通道检测 ────────────────────────────────────────────────────────────
  // detectNarrowPassage 内部已处理降级（地图为空时全 false），
  // 此处转型为 const_cast 调用（成员函数非 const，但不修改逻辑状态）。
  std::vector<bool> is_narrow_xy(static_cast<size_t>(N), false);
  std::vector<bool> is_narrow_z(static_cast<size_t>(N), false);
  if (!use_fixed_obstacle_schedule_ && !disable_online_classification_)
  {
    const_cast<SwarmMasterCoordinator*>(this)
        ->detectNarrowPassage(leader_ctrl_pts, is_narrow_xy, is_narrow_z);
  }

  // ── 预计算过渡扩展区域 ────────────────────────────────────────────────────
  // 在每个窄通道段的入口前 / 出口后各延伸 transition_ctrl_pts_ 个控制点，
  // 在扩展区中用 alpha ∈ (0,1) 线性插值 Y⟷Z 偏移，确保编队平滑切换。
  // transition_mask[c] ∈ [0,1]：0 = 纯水平V，1 = 纯垂直V，中间为过渡
  std::vector<double> transition_mask(static_cast<size_t>(N), 0.0);
  for (int c = 0; c < N; ++c)
  {
    if (is_narrow_xy[static_cast<size_t>(c)])
    {
      transition_mask[static_cast<size_t>(c)] = 1.0;

      // 向前扩展（入口过渡）
      for (int t = 1; t <= transition_ctrl_pts_; ++t)
      {
        const int idx = c - t;
        if (idx < 0) break;
        const double ratio = static_cast<double>(t) /
                              static_cast<double>(transition_ctrl_pts_ + 1);
        const double alpha = 0.5 * (1.0 + std::cos(M_PI * ratio));
        transition_mask[static_cast<size_t>(idx)] =
            std::max(transition_mask[static_cast<size_t>(idx)], alpha);
      }

      // 向后扩展（出口过渡）—— 2倍长度确保从机有足够距离回到水平编队
      const int exit_transition = transition_ctrl_pts_ * 2;
      for (int t = 1; t <= exit_transition; ++t)
      {
        const int idx = c + t;
        if (idx >= N) break;
        const double ratio = static_cast<double>(t) /
                              static_cast<double>(exit_transition + 1);
        const double alpha = 0.5 * (1.0 + std::cos(M_PI * ratio));
        transition_mask[static_cast<size_t>(idx)] =
            std::max(transition_mask[static_cast<size_t>(idx)], alpha);
      }
    }
  }

  // ── 初始化 per-control-point 类型/混合因子数组 ──────────────────────────
  // cmd_type[c]  — 障碍物类型（优先级高于 detectNarrowPassage 的 transition_mask）
  // cmd_blend[c] — 混合因子 [0,1]：0=默认V形，1=完整目标编队
  std::vector<ObstacleType> cmd_type(static_cast<size_t>(N), ObstacleType::NONE);
  std::vector<double>       cmd_blend(static_cast<size_t>(N), 0.0);
  std::vector<int>          cmd_obstacle_id(static_cast<size_t>(N), -1);

  // 从 formation_cmds 提取类型和 blend
  if (!formation_cmds.empty()) {
    for (size_t i = 0; i < std::min(static_cast<size_t>(N), formation_cmds.size()); ++i) {
      if (formation_cmds[i].type != ObstacleType::NONE) {
        cmd_type[i]  = formation_cmds[i].type;
        cmd_blend[i] = formation_cmds[i].blend;
        cmd_obstacle_id[i] = formation_cmds[i].obstacle_id;
      }
    }
  }

  // detectNarrowPassage 的 transition_mask 作为 DOOR_FRAME 回落：
  // 若 cmd_type 尚为 NONE 且 transition_mask 非零，则认定为 DOOR_FRAME
  if (!use_fixed_obstacle_schedule_)
  {
    for (int c = 0; c < N; ++c) {
      const size_t ci = static_cast<size_t>(c);
      if (cmd_type[ci] == ObstacleType::NONE && transition_mask[ci] > 1e-3) {
        cmd_type[ci] = ObstacleType::DOOR_FRAME;
      }
      if (cmd_type[ci] == ObstacleType::DOOR_FRAME) {
        cmd_blend[ci] = std::max(cmd_blend[ci], transition_mask[ci]);
      }
    }
  }

  // Z 轴窄缝优先级最高（覆写 XY DOOR_FRAME 请求）
  if (!use_fixed_obstacle_schedule_)
  {
    for (int c = 0; c < N; ++c) {
      if (is_narrow_z[static_cast<size_t>(c)]) {
        cmd_type[static_cast<size_t>(c)]  = ObstacleType::Z_SLIT;
        cmd_blend[static_cast<size_t>(c)] = 1.0;
      }
    }
  }

  for (size_t k = 0; k < agent_ids_.size(); ++k)
  {
    const int agent_id = agent_ids_[k];

    const int level = static_cast<int>(k / 2) + 1;
    const int side = (k % 2 == 0) ? 1 : -1;

    const double back_dist    = level * formation_spacing_ * std::cos(angle_rad);
    const double lateral_dist = side * level * formation_spacing_ * std::sin(angle_rad);

    Eigen::MatrixXd follower_ctrl = leader_ctrl_pts;

    for (int c = 0; c < N; ++c)
    {
      const Eigen::Vector3d forward = headings.col(c);  // 平滑的真实切线航向
      Eigen::Vector3d left_dir(-forward.y(), forward.x(), 0.0);
      if (left_dir.norm() < 1e-6)
        left_dir = Eigen::Vector3d(0.0, 1.0, 0.0);
      else
        left_dir.normalize();

      const double alpha = cmd_blend[static_cast<size_t>(c)];  // 混合因子 [0,1]
      Eigen::Vector3d offset;
      const Eigen::Vector3d default_offset =
          -forward * back_dist + left_dir * lateral_dist + Eigen::Vector3d(0.0, 0.0, formation_z_offset_);

      const int obstacle_id = cmd_obstacle_id[static_cast<size_t>(c)];
      if (use_fixed_obstacle_schedule_ &&
          obstacle_id >= 0 &&
          obstacle_id < static_cast<int>(fixed_obstacle_schedule_.size()) &&
          cmd_type[static_cast<size_t>(c)] != ObstacleType::NONE &&
          agent_ids_.size() == 2)
      {
        const FixedObstacleSpec &spec = fixed_obstacle_schedule_[static_cast<size_t>(obstacle_id)];
        const double aux_abs_min = triangle_area_min_ / std::max(spec.mode_template.back_offset, 0.2);
        const double aux_span = std::max(std::abs(spec.mode_template.aux_span), aux_abs_min);
        const double sign = (k == 0) ? 1.0 : -1.0;
        const Eigen::Vector3d desired_offset =
            -spec.mode_template.back_offset * spec.forward_axis +
            sign * 0.5 * spec.mode_template.primary_span * spec.primary_axis +
            sign * aux_span * spec.auxiliary_axis +
            Eigen::Vector3d(0.0, 0.0, formation_z_offset_);
        offset = (1.0 - alpha) * default_offset + alpha * desired_offset;
      }
      else
      {

      switch (cmd_type[static_cast<size_t>(c)]) {

        // ── DOOR_FRAME：门框型窄通道 ──────────────────────────────────────
        // 偏置公式：
        //   P_i = P_leader_i − back·fwd + lateral·(1−α)·left + lateral·α·ẑ
        // Bezier 凸包保持：施加在控制点上的偏置 δ_i = f(α_i) 是关于 α 的连续
        //   仿射函数；仿射变换将凸包映射为凸包（仿射不变性），相邻段间 α 通过
        //   余弦过渡连续变化，保证 C0/C1 接缝处梯度光滑。
        case ObstacleType::DOOR_FRAME: {
          const double y_comp = lateral_dist * (1.0 - alpha);
          const double z_comp = lateral_dist * alpha;
          offset = -forward * back_dist + left_dir * y_comp;
          offset.z() += z_comp + formation_z_offset_;
          break;
        }

        // ── Z_SLIT：Z 轴窄缝，展开水平钝角三角形 ────────────────────────
        // 偏置公式（均匀放大 spread）：
        //   spread = 1 + α·(slit_spread_factor−1)
        //   P_i = P_leader_i − back·spread·fwd + lateral·spread·left
        //                     + formation_z_offset·ẑ
        // 数学依据：spread 是标量缩放，等价于以 P_leader 为中心对编队凸包做
        //   均匀仿射放大；放大后的凸包仍为凸集，嵌套于更大凸包内，且 α∈[0,1]
        //   连续变化使控制点偏移量 Lipschitz 连续，不破坏 C0/C1 边界条件。
        case ObstacleType::Z_SLIT: {
          const double spread       = 1.0 + alpha * (slit_spread_factor_ - 1.0);
          const double wide_lateral = lateral_dist * spread;
          const double wide_back    = back_dist    * spread;
          offset = -forward * wide_back + left_dir * wide_lateral;
          offset.z() += formation_z_offset_;  // 保持水平，不做 Z 偏移
          break;
        }

        // ── RING：圆环障碍，收缩为紧凑纵列 ──────────────────────────────
        // 偏置公式（横向渐消 + 纵向重分布）：
        //   eff_lateral = lateral_dist·[(1−α)+α·ring_compact_lateral_scale]
        //   eff_back    = back_dist·(1−α) + ring_longitudinal_spacing·level·α
        //   P_i = P_leader_i − eff_back·fwd + eff_lateral·left
        //                     + formation_z_offset·(1−α)·ẑ
        // 凸包保持说明：偏置向量在仿射空间中随 α 连续变化，构成仿射路径；
        //   分段 Bezier 曲线对控制点施加线性偏移后，曲线仍位于新控制点凸包内
        //   （凸包性质），且相邻段偏移差量有界，不影响 C0/C1 连续性。
        // α=0 退化为默认V形；α=1 收敛为“紧凑人字形”（非重合），Z 对齐主机。
        case ObstacleType::RING: {
          const double ring_back   = ring_longitudinal_spacing_
                                     * static_cast<double>(level);
          const double lateral_scale = (1.0 - alpha)
                                     + alpha * std::max(0.0, std::min(1.0, ring_compact_lateral_scale_));
          const double eff_lateral = lateral_dist * lateral_scale;
          const double eff_back    = back_dist    * (1.0 - alpha)
                                   + ring_back    * alpha;
          offset = -forward * eff_back + left_dir * eff_lateral;
          offset.z() = formation_z_offset_ * (1.0 - alpha);
          break;
        }

        // ── NONE（默认）：标准水平 V 形 ──────────────────────────────────
        // 偏置公式：P_i = P_leader_i − back·fwd + lateral·left + z_offset·ẑ
        // 所有控制点施加相同仿射偏移，凸包整体平移，Bezier 阶数和曲线形状不变。
        default: {
          offset = default_offset;
          break;
        }
      }
      }

      // ── Z 轴安全钳制：防止撞天花板或穿地 ──────────────────────────────
      const double leader_z  = leader_ctrl_pts(2, c);
      const double raw_z     = leader_z + offset.z();
      const double clamped_z = std::max(z_formation_min_,
                                        std::min(z_formation_max_, raw_z));
      offset.z() += (clamped_z - raw_z);  // 补偿钳制量

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
            // Layer-C: 有界二次惩罚替代指数惩罚，消除优化对抗
            const double gain = std::min(collision_weight_ * err * err, collision_gain_cap_);
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

void SwarmMasterCoordinator::applyLeaderBoundaryCorrection(Eigen::MatrixXd &leader_ctrl_pts) const
{
  if (!leader_state_.valid || leader_ctrl_pts.cols() < 2)
    return;

  const double ts_seg = getSegmentDuration();
  const int order = latest_leader_bezier_.order > 0 ? latest_leader_bezier_.order : 3;
  const Eigen::Vector3d ideal_p0 = leader_ctrl_pts.col(0);
  const double jump = (ideal_p0 - leader_state_.pos).norm();
  if (jump < 0.03)
    return;

  leader_ctrl_pts.col(0) = leader_state_.pos;
  const Eigen::Vector3d ideal_p1 = leader_ctrl_pts.col(1);
  const Eigen::Vector3d vel_p1 =
      leader_state_.pos + (ts_seg / static_cast<double>(order)) * leader_state_.vel;
  const double blend = std::min(1.0, jump / std::max(guidance_c0_blend_dist_, 1e-3));
  leader_ctrl_pts.col(1) = (1.0 - blend) * ideal_p1 + blend * vel_p1;

  if (leader_ctrl_pts.cols() >= 3 && blend > 1e-3 && leader_state_.vel.norm() > 0.3)
  {
    const Eigen::Vector3d ideal_p2 = leader_ctrl_pts.col(2);
    const double seg_len = (ideal_p2 - ideal_p1).norm();
    if (seg_len > 1e-4)
    {
      Eigen::Vector3d vel_dir = leader_state_.vel.normalized();
      const Eigen::Vector3d vel_p2 = leader_ctrl_pts.col(1) + vel_dir * seg_len;
      const double c2_alpha = std::min(1.0, blend * guidance_c2_blend_weight_);
      leader_ctrl_pts.col(2) = (1.0 - c2_alpha) * ideal_p2 + c2_alpha * vel_p2;
    }
  }
}

void SwarmMasterCoordinator::applyFollowerBoundaryCorrections(
    std::unordered_map<int, Eigen::MatrixXd> &agent_ctrl_pts_map,
    bool same_traj_refresh) const
{
  const double ts_seg = getSegmentDuration();
  const int order = latest_leader_bezier_.order > 0 ? latest_leader_bezier_.order : 3;
  const bool damp_same_traj_refresh = same_traj_refresh && !use_fixed_obstacle_schedule_;

  for (auto &kv : agent_ctrl_pts_map)
  {
    const int agent_id = kv.first;
    Eigen::MatrixXd &ctrl = kv.second;
    if (ctrl.cols() < 2)
      continue;

    auto state_it = agent_states_.find(agent_id);
    if (state_it == agent_states_.end() || !state_it->second.valid)
      continue;
    const AgentState &state = state_it->second;

    const double state_age =
        std::max(0.0, (ros::Time::now() - state.stamp).toSec());
    const double pred_dt =
        std::min(state_age, std::max(0.0, guidance_state_prediction_max_dt_));
    const Eigen::Vector3d predicted_pos =
        state.pos + (guidance_state_prediction_gain_ * pred_dt) * state.vel;

    const Eigen::Vector3d ideal_p0 = ctrl.col(0);
    Eigen::Vector3d target_p0 = predicted_pos;
    const Eigen::Vector3d raw_delta = target_p0 - ideal_p0;
    const double raw_jump = raw_delta.norm();

    if (raw_jump < guidance_c0_deadzone_)
      continue;

    if (damp_same_traj_refresh && guidance_same_traj_c0_max_step_ > kFixedObsEps &&
        raw_jump > guidance_same_traj_c0_max_step_)
    {
      target_p0 =
          ideal_p0 + raw_delta * (guidance_same_traj_c0_max_step_ / raw_jump);
    }

    const double jump = (target_p0 - ideal_p0).norm();
    if (jump < guidance_c0_deadzone_)
      continue;

    ctrl.col(0) = target_p0;
    const Eigen::Vector3d ideal_p1 = ctrl.col(1);
    const Eigen::Vector3d vel_p1 = target_p0 + (ts_seg / static_cast<double>(order)) * state.vel;
    double blend = std::min(1.0, jump / std::max(guidance_c0_blend_dist_, 1e-3));
    if (damp_same_traj_refresh)
    {
      // 同一 leader 轨迹的高频刷新场景下，降低速度方向注入，避免起飞期左右摆动。
      blend = std::min(blend, 0.35);
    }
    ctrl.col(1) = (1.0 - blend) * ideal_p1 + blend * vel_p1;

    const Eigen::Vector3d implied_vel =
        (static_cast<double>(order) / ts_seg) * (ctrl.col(1) - ctrl.col(0));
    const double speed = implied_vel.norm();
    if (speed > follower_max_vel_ && speed > 1e-6)
    {
      ctrl.col(1) = ctrl.col(0) + (ts_seg / static_cast<double>(order)) * (implied_vel * (follower_max_vel_ / speed));
      ROS_WARN_THROTTLE(0.5,
                        "[SwarmMaster] agent=%d vel clamped: %.2f -> %.2f m/s",
                        agent_id, speed, follower_max_vel_);
    }

    if (!damp_same_traj_refresh && ctrl.cols() >= 3 && blend > 1e-3 &&
        state.vel.norm() > 0.3)
    {
      const Eigen::Vector3d ideal_p2 = ctrl.col(2);
      const double seg_len = (ideal_p2 - ideal_p1).norm();
      if (seg_len > 1e-4)
      {
        const Eigen::Vector3d vel_dir = state.vel.normalized();
        const Eigen::Vector3d vel_p2 = ctrl.col(1) + vel_dir * seg_len;
        const double c2_alpha = std::min(1.0, blend * guidance_c2_blend_weight_);
        ctrl.col(2) = (1.0 - c2_alpha) * ideal_p2 + c2_alpha * vel_p2;
      }
    }

    if (jump > guidance_log_jump_threshold_)
    {
      ROS_WARN_THROTTLE(0.5,
                        "[SwarmMaster] agent=%d C0_jump=%.3f m (raw=%.3f age=%.2f) blend=%.2f corrected.",
                        agent_id, jump, raw_jump, state_age, blend);
    }
  }
}

bool SwarmMasterCoordinator::shouldPublishFixedScheduleUpdate(bool same_leader_traj,
                                                              int obstacle_id,
                                                              FixedObstacleState state,
                                                              bool blanket_hold,
                                                              double command_scale,
                                                              double min_clear_along) const
{
  (void)min_clear_along;
  if (!same_leader_traj || !refresh_guidance_on_same_leader_traj_)
    return true;

  if (!have_fixed_schedule_publish_snapshot_)
    return true;

  if (obstacle_id != last_fixed_schedule_publish_obstacle_id_ ||
      state != last_fixed_schedule_publish_state_ ||
      blanket_hold != last_fixed_schedule_publish_blanket_hold_)
  {
    return true;
  }

  const double elapsed = (ros::Time::now() - last_fixed_schedule_publish_stamp_).toSec();
  double gate_interval = fixed_schedule_same_traj_refresh_min_interval_;
  if (fixed_schedule_same_traj_force_refresh_interval_ > kFixedObsEps)
  {
    gate_interval = std::min(gate_interval, fixed_schedule_same_traj_force_refresh_interval_);
  }

  if (elapsed + kFixedObsEps < gate_interval)
    return false;

  const double blend_delta = std::abs(command_scale - last_fixed_schedule_publish_blend_);
  if (blend_delta + kFixedObsEps >= fixed_schedule_same_traj_refresh_min_blend_delta_)
    return true;

  // 同轨迹 HOLD/RELEASE 阶段 blend 可能长时间不变。若不做心跳重发，
  // 从机短时轨迹执行完成后会进入保持，造成“全编队卡死”的假象。
  if (fixed_schedule_same_traj_force_refresh_interval_ > kFixedObsEps &&
      elapsed + kFixedObsEps >= fixed_schedule_same_traj_force_refresh_interval_)
  {
    return true;
  }

  return false;
}

void SwarmMasterCoordinator::cacheFixedSchedulePublishSnapshot(int obstacle_id,
                                                               FixedObstacleState state,
                                                               bool blanket_hold,
                                                               double command_scale,
                                                               double min_clear_along,
                                                               const ros::Time &stamp)
{
  have_fixed_schedule_publish_snapshot_ = true;
  last_fixed_schedule_publish_obstacle_id_ = obstacle_id;
  last_fixed_schedule_publish_state_ = state;
  last_fixed_schedule_publish_blanket_hold_ = blanket_hold;
  last_fixed_schedule_publish_blend_ = command_scale;
  last_fixed_schedule_publish_min_clear_along_ = min_clear_along;
  last_fixed_schedule_publish_stamp_ = stamp;
}

void SwarmMasterCoordinator::buildFixedFormationCommands(const Eigen::MatrixXd &leader_ctrl_pts,
                                                         const FormationCommand &active_cmd,
                                                         double release_blend,
                                                         bool blanket_hold,
                                                         std::vector<FormationCommand> &commands) const
{
  commands.assign(static_cast<size_t>(leader_ctrl_pts.cols()), FormationCommand());
  if (active_cmd.obstacle_id < 0 || active_cmd.obstacle_id >= static_cast<int>(fixed_obstacle_schedule_.size()))
    return;

  const FixedObstacleSpec &spec = fixed_obstacle_schedule_[static_cast<size_t>(active_cmd.obstacle_id)];
  if (blanket_hold)
  {
    for (FormationCommand &cmd : commands)
    {
      cmd.type = active_cmd.type;
      cmd.blend = release_blend;
      cmd.obstacle_id = active_cmd.obstacle_id;
    }
    return;
  }

  const double enter_start = spec.activation_window_enter - spec.blend_in;
  const double exit_end = spec.activation_window_exit + spec.blend_out;
  for (int c = 0; c < leader_ctrl_pts.cols(); ++c)
  {
    const double along = obstacleAlong(active_cmd.obstacle_id, leader_ctrl_pts.col(c));
    double spatial_blend = 0.0;
    if (along >= enter_start && along < spec.activation_window_enter)
      spatial_blend = blendRamp(along, enter_start, spec.activation_window_enter);
    else if (along >= spec.activation_window_enter && along <= spec.activation_window_exit)
      spatial_blend = 1.0;
    else if (along > spec.activation_window_exit && along <= exit_end)
      spatial_blend = 1.0 - blendRamp(along, spec.activation_window_exit, exit_end);

    spatial_blend *= release_blend;
    if (spatial_blend <= 1e-3)
      continue;

    commands[static_cast<size_t>(c)].type = active_cmd.type;
    commands[static_cast<size_t>(c)].blend = spatial_blend;
    commands[static_cast<size_t>(c)].obstacle_id = active_cmd.obstacle_id;
  }
}

double SwarmMasterCoordinator::applyFixedObstacleLeaderTraction(Eigen::MatrixXd &leader_ctrl_pts,
                                                                const FixedObstacleSpec &spec,
                                                                double leader_along) const
{
  const FixedObstacleLeaderTraction &traction = spec.leader_traction;
  if (!traction.enabled || leader_ctrl_pts.cols() <= 0)
    return 0.0;

  const double enter_start = traction.window_enter - traction.blend_in;
  const double exit_end = traction.window_exit + traction.blend_out;

  bool applied = false;
  double max_alpha = 0.0;
  int max_col = -1;
  Eigen::Vector3d max_nominal_local = Eigen::Vector3d::Zero();
  Eigen::Vector3d max_target_local = Eigen::Vector3d::Zero();
  Eigen::Vector3d max_corrected_local = Eigen::Vector3d::Zero();

  for (int c = 0; c < leader_ctrl_pts.cols(); ++c)
  {
    const double along = obstacleAlong(spec.id, leader_ctrl_pts.col(c));
    double alpha = 0.0;
    if (along >= enter_start && along < traction.window_enter)
      alpha = blendRamp(along, enter_start, traction.window_enter);
    else if (along >= traction.window_enter && along <= traction.window_exit)
      alpha = 1.0;
    else if (along > traction.window_exit && along <= exit_end)
      alpha = 1.0 - blendRamp(along, traction.window_exit, exit_end);

    if (alpha <= kFixedObsEps)
      continue;

    const Eigen::Vector3d nominal_local = obstacleLocal(spec.id, leader_ctrl_pts.col(c));
    Eigen::Vector3d target_local = nominal_local;

    switch (spec.type)
    {
      case ObstacleType::DOOR_FRAME:
      {
        const double safe_half = std::max(0.0, 0.5 * spec.gap_width - traction.clearance_margin);
        double desired_y = spec.gap_center_y + traction.bias_local.y();
        desired_y = std::max(spec.gap_center_y - safe_half,
                             std::min(spec.gap_center_y + safe_half, desired_y));
        target_local.y() = desired_y;
        break;
      }
      case ObstacleType::Z_SLIT:
      {
        double safe_low = spec.z_gap_low + traction.clearance_margin;
        double safe_high = spec.z_gap_high - traction.clearance_margin;
        if (safe_high < safe_low)
        {
          safe_low = spec.z_gap_low;
          safe_high = spec.z_gap_high;
        }
        double desired_z = 0.5 * (safe_low + safe_high) + traction.bias_local.z();
        desired_z = std::max(safe_low, std::min(safe_high, desired_z));
        target_local.z() = desired_z;
        break;
      }
      case ObstacleType::RING:
      case ObstacleType::NONE:
      default:
        continue;
    }

    const Eigen::Vector3d corrected_local =
        nominal_local + alpha * (target_local - nominal_local);
    leader_ctrl_pts.col(c) = spec.center + spec.world_from_local * corrected_local;

    applied = true;
    if (alpha > max_alpha)
    {
      max_alpha = alpha;
      max_col = c;
      max_nominal_local = nominal_local;
      max_target_local = target_local;
      max_corrected_local = corrected_local;
    }
  }

  if (applied && (leader_traction_debug_ || traction.debug))
  {
    ROS_INFO_THROTTLE(
        0.5,
        "[SwarmMaster][LeaderTraction] obstacle=%s type=%s leader_along=%.2f max_alpha=%.2f cp=%d "
        "nominal_local=(%.2f, %.2f, %.2f) target_local=(%.2f, %.2f, %.2f) corrected_local=(%.2f, %.2f, %.2f)",
        spec.name.c_str(), obstacleTypeName(spec.type).c_str(), leader_along, max_alpha, max_col,
        max_nominal_local.x(), max_nominal_local.y(), max_nominal_local.z(),
        max_target_local.x(), max_target_local.y(), max_target_local.z(),
        max_corrected_local.x(), max_corrected_local.y(), max_corrected_local.z());
  }

  return max_alpha;
}

bool SwarmMasterCoordinator::optimizeFixedObstacleWindow(
    Eigen::MatrixXd &leader_corrected_ctrl,
    const Eigen::MatrixXd &leader_nominal_ctrl,
    std::unordered_map<int, Eigen::MatrixXd> &agent_ctrl_pts_map,
    const FormationCommand &active_cmd,
    int seg_begin, int seg_end)
{
  if (active_cmd.obstacle_id < 0 ||
      active_cmd.obstacle_id >= static_cast<int>(fixed_obstacle_schedule_.size()) ||
      agent_ids_.size() < 2 || seg_begin < 0 || seg_end < seg_begin)
    return false;

  const int order = latest_leader_bezier_.order > 0 ? latest_leader_bezier_.order : 3;
  const int start_col = seg_begin * order;
  const int end_col = std::min<int>(leader_corrected_ctrl.cols() - 1, seg_end * order + order);
  const int num_cols = end_col - start_col + 1;
  if (num_cols < 7)
    return false;

  auto it1 = agent_ctrl_pts_map.find(agent_ids_[0]);
  auto it2 = agent_ctrl_pts_map.find(agent_ids_[1]);
  if (it1 == agent_ctrl_pts_map.end() || it2 == agent_ctrl_pts_map.end())
    return false;

  CooperativePayloadOptimizer optimizer;
  optimizer.setEnvironment(grid_map_);

  CooperativePayloadOptParams params;
  params.w_smooth = cooperative_w_smooth_;
  params.w_feas = cooperative_w_feas_;
  params.w_uav_obs = cooperative_w_uav_obs_;
  params.w_leader_ref = cooperative_w_leader_ref_;
  params.w_mode_shape = cooperative_w_mode_shape_;
  params.w_payload_feas = cooperative_w_payload_feas_;
  params.w_payload_obs = cooperative_w_payload_obs_;
  params.w_sep = cooperative_w_sep_;
  params.payload_invalid_penalty = cooperative_payload_invalid_penalty_;
  params.uav_clearance = follower_clearance_;
  params.max_vel = follower_max_vel_;
  params.max_acc = 3.0;
  params.rope_length = payload_rope_length_;
  params.payload_radius = payload_radius_;
  params.payload_extra_margin = payload_extra_margin_;
  params.triangle_area_min = triangle_area_min_;
  params.inter_uav_sep_min = inter_uav_sep_min_;
  params.samples_per_seg = payload_samples_per_seg_;
  params.max_iterations = cooperative_max_iterations_;
  params.finite_diff_eps = cooperative_fd_eps_;
  params.obstacle_search_radius = cooperative_obstacle_search_radius_;
  optimizer.setParams(params);

  const FixedObstacleSpec &spec = fixed_obstacle_schedule_[static_cast<size_t>(active_cmd.obstacle_id)];
  CooperativeObstacleSpec coop_spec;
  coop_spec.type = (spec.type == ObstacleType::DOOR_FRAME) ? CooperativeObstacleType::DOOR_FRAME :
                   (spec.type == ObstacleType::RING)       ? CooperativeObstacleType::RING :
                   (spec.type == ObstacleType::Z_SLIT)     ? CooperativeObstacleType::Z_SLIT :
                                                             CooperativeObstacleType::NONE;
  coop_spec.frame.center = spec.center;
  coop_spec.frame.world_from_local = spec.world_from_local;
  coop_spec.forward_axis = spec.forward_axis;
  coop_spec.primary_axis = spec.primary_axis;
  coop_spec.auxiliary_axis = spec.auxiliary_axis;
  coop_spec.physical_half_extent = spec.physical_half_extent;
  coop_spec.gap_center_y = spec.gap_center_y;
  coop_spec.gap_width = spec.gap_width;
  coop_spec.z_gap_low = spec.z_gap_low;
  coop_spec.z_gap_high = spec.z_gap_high;
  coop_spec.major_radius = spec.major_r;
  coop_spec.minor_radius = spec.minor_r;
  coop_spec.back_offset = spec.mode_template.back_offset;
  coop_spec.primary_span = spec.mode_template.primary_span;
  coop_spec.aux_span = spec.mode_template.aux_span;
  coop_spec.leader_bias_world = spec.world_from_local * spec.mode_template.leader_bias_local;

  CooperativeWindowInput input;
  input.leader_nominal = leader_nominal_ctrl.block(0, start_col, 3, num_cols);
  input.leader_initial = leader_corrected_ctrl.block(0, start_col, 3, num_cols);
  input.follower1_initial = it1->second.block(0, start_col, 3, num_cols);
  input.follower2_initial = it2->second.block(0, start_col, 3, num_cols);
  input.segment_duration = getSegmentDuration();
  input.obstacle = coop_spec;

  const CooperativeWindowOutput output = optimizer.optimize(input);
  if (!output.success || !output.payload_valid)
  {
    ROS_WARN("[SwarmMaster] Cooperative payload optimization failed for obstacle %s, fallback to fixed template.",
             spec.name.c_str());
    return false;
  }

  leader_corrected_ctrl.block(0, start_col, 3, num_cols) = output.leader_ctrl;
  it1->second.block(0, start_col, 3, num_cols) = output.follower1_ctrl;
  it2->second.block(0, start_col, 3, num_cols) = output.follower2_ctrl;
  return true;
}

bool SwarmMasterCoordinator::optimizeOnlinePayloadWindow(
    Eigen::MatrixXd &leader_corrected_ctrl,
    const Eigen::MatrixXd &leader_nominal_ctrl,
    std::unordered_map<int, Eigen::MatrixXd> &agent_ctrl_pts_map)
{
  if (agent_ids_.size() < 2 || leader_corrected_ctrl.cols() < 7)
    return false;

  auto it1 = agent_ctrl_pts_map.find(agent_ids_[0]);
  auto it2 = agent_ctrl_pts_map.find(agent_ids_[1]);
  if (it1 == agent_ctrl_pts_map.end() || it2 == agent_ctrl_pts_map.end())
    return false;

  const int order = latest_leader_bezier_.order > 0 ? latest_leader_bezier_.order : 3;
  const int total_seg = (leader_corrected_ctrl.cols() - 1) / order;
  if (total_seg < 2)
    return false;

  const double ts_seg = std::max(1e-3, getSegmentDuration());
  const double t_now = std::max(0.0, (ros::Time::now() - latest_leader_bezier_.start_time).toSec());
  const int current_seg = std::min(total_seg - 1, std::max(0, static_cast<int>(std::floor(t_now / ts_seg))));

  const int window_segs = std::max(2, online_payload_window_segs_);
  int seg_begin = std::min(current_seg, total_seg - 2);
  int seg_end = std::min(total_seg - 1, seg_begin + window_segs - 1);
  if (seg_end - seg_begin + 1 < 2)
  {
    seg_begin = std::max(0, total_seg - 2);
    seg_end = total_seg - 1;
  }

  const int start_col = seg_begin * order;
  const int end_col = std::min<int>(leader_corrected_ctrl.cols() - 1, seg_end * order + order);
  const int num_cols = end_col - start_col + 1;
  if (num_cols < 7)
    return false;

  ROS_INFO_THROTTLE(1.0,
                    "[SwarmMaster][OnlinePayloadOpt] active=1 seg=[%d,%d] current_seg=%d t=%.2f",
                    seg_begin, seg_end, current_seg, t_now);

  CooperativePayloadOptimizer optimizer;
  optimizer.setEnvironment(grid_map_);

  CooperativePayloadOptParams params;
  params.w_smooth = cooperative_w_smooth_;
  params.w_feas = cooperative_w_feas_;
  params.w_uav_obs = cooperative_w_uav_obs_;
  params.w_leader_ref = cooperative_w_leader_ref_;
  params.w_mode_shape = cooperative_w_mode_shape_;
  params.w_payload_feas = cooperative_w_payload_feas_;
  params.w_payload_obs = cooperative_w_payload_obs_;
  params.w_sep = cooperative_w_sep_;
  params.payload_invalid_penalty = cooperative_payload_invalid_penalty_;
  params.uav_clearance = follower_clearance_;
  params.max_vel = follower_max_vel_;
  params.max_acc = 3.0;
  params.rope_length = payload_rope_length_;
  params.payload_radius = payload_radius_;
  params.payload_extra_margin = payload_extra_margin_;
  params.triangle_area_min = triangle_area_min_;
  params.inter_uav_sep_min = inter_uav_sep_min_;
  params.samples_per_seg = payload_samples_per_seg_;
  params.max_iterations = cooperative_max_iterations_;
  params.finite_diff_eps = cooperative_fd_eps_;
  params.obstacle_search_radius = cooperative_obstacle_search_radius_;
  optimizer.setParams(params);

  CooperativeObstacleSpec coop_spec;
  coop_spec.type = CooperativeObstacleType::NONE;

  Eigen::Vector3d forward = Eigen::Vector3d::UnitX();
  if (leader_nominal_ctrl.cols() > start_col + 1)
  {
    forward = leader_nominal_ctrl.col(start_col + 1) - leader_nominal_ctrl.col(start_col);
    if (forward.norm() < 1e-3)
      forward = Eigen::Vector3d::UnitX();
    else
      forward.normalize();
  }

  Eigen::Vector3d left(-forward.y(), forward.x(), 0.0);
  if (left.norm() < 1e-3)
    left = Eigen::Vector3d::UnitY();
  else
    left.normalize();

  Eigen::Vector3d up = forward.cross(left);
  if (up.norm() < 1e-3)
    up = Eigen::Vector3d::UnitZ();
  else
    up.normalize();

  coop_spec.frame.center = leader_nominal_ctrl.col(start_col);
  coop_spec.frame.world_from_local.col(0) = forward;
  coop_spec.frame.world_from_local.col(1) = left;
  coop_spec.frame.world_from_local.col(2) = up;
  coop_spec.forward_axis = forward;
  coop_spec.primary_axis = left;
  coop_spec.auxiliary_axis = up;

  const double angle_rad = formation_angle_deg_ * M_PI / 180.0;
  coop_spec.back_offset = std::max(0.2, formation_spacing_ * std::cos(angle_rad));
  coop_spec.primary_span = 2.0 * std::abs(formation_spacing_ * std::sin(angle_rad));
  coop_spec.aux_span = 0.0;
  coop_spec.leader_bias_world = Eigen::Vector3d::Zero();

  CooperativeWindowInput input;
  input.leader_nominal = leader_nominal_ctrl.block(0, start_col, 3, num_cols);
  input.leader_initial = leader_corrected_ctrl.block(0, start_col, 3, num_cols);
  input.follower1_initial = it1->second.block(0, start_col, 3, num_cols);
  input.follower2_initial = it2->second.block(0, start_col, 3, num_cols);
  input.segment_duration = getSegmentDuration();
  input.obstacle = coop_spec;

  const CooperativeWindowOutput output = optimizer.optimize(input);
  if (!output.success || !output.payload_valid)
  {
    ROS_WARN_THROTTLE(1.0,
                      "[SwarmMaster][OnlinePayloadOpt] failed (success=%d payload_valid=%d seg=[%d,%d]), keep nominal guidance.",
                      static_cast<int>(output.success), static_cast<int>(output.payload_valid),
                      seg_begin, seg_end);
    return false;
  }

  leader_corrected_ctrl.block(0, start_col, 3, num_cols) = output.leader_ctrl;
  it1->second.block(0, start_col, 3, num_cols) = output.follower1_ctrl;
  it2->second.block(0, start_col, 3, num_cols) = output.follower2_ctrl;
  return true;
}

bool SwarmMasterCoordinator::runFixedObstacleSchedule(const Eigen::MatrixXd &leader_ctrl_pts,
                                                      const ros::Time &publish_start_time,
                                                      bool same_leader_traj)
{
  Eigen::MatrixXd corrected_leader_ctrl = leader_ctrl_pts;
  applyLeaderBoundaryCorrection(corrected_leader_ctrl);
  std::unordered_map<int, Eigen::MatrixXd> agent_ctrl_pts_map;
  std::vector<FormationCommand> commands;
  FormationCommand active_cmd;
  bool blanket_hold = false;
  double command_scale = 1.0;
  int publish_obstacle_id = -1;
  FixedObstacleState publish_state = FixedObstacleState::IDLE;
  bool publish_blanket_hold = false;
  double publish_command_scale = 0.0;
  double publish_min_clear_along = std::numeric_limits<double>::quiet_NaN();

  while (current_fixed_obstacle_idx_ < static_cast<int>(fixed_obstacle_runtimes_.size()) &&
         fixed_obstacle_runtimes_[static_cast<size_t>(current_fixed_obstacle_idx_)].completed)
  {
    ++current_fixed_obstacle_idx_;
  }

  if (current_fixed_obstacle_idx_ < static_cast<int>(fixed_obstacle_schedule_.size()))
  {
    FixedObstacleSpec &spec = fixed_obstacle_schedule_[static_cast<size_t>(current_fixed_obstacle_idx_)];
    FixedObstacleRuntime &runtime = fixed_obstacle_runtimes_[static_cast<size_t>(current_fixed_obstacle_idx_)];
    // fixed schedule 的状态推进必须跟随 leader 实际执行位置，而不是名义轨迹时钟。
    // 否则同一条 leader nominal 轨迹被持续刷新时，障碍物状态机会“按旧 start_time 快进”，
    // 在 ring/slit 附近过早切换 ACTIVE/HOLD/RELEASE，和实际编队位置脱节后造成卡死。
    const Eigen::Vector3d leader_world_now =
        leader_state_.valid ? leader_state_.pos : evalCurrentLeaderNominalPosition(corrected_leader_ctrl);
    const double leader_along = obstacleAlong(spec.id, leader_world_now);

    Eigen::Vector3d payload_center = Eigen::Vector3d::Zero();
    bool payload_valid = computePayloadPositionFromStates(payload_center);
    bool payload_using_cached_center = false;
    if (payload_valid)
    {
      runtime.last_valid_payload_center = payload_center;
      runtime.have_last_valid_payload = true;
    }
    else if (runtime.have_last_valid_payload)
    {
      payload_center = runtime.last_valid_payload_center;
      payload_using_cached_center = true;
    }

    double min_clear_along = std::numeric_limits<double>::infinity();
    bool all_followers_clear = true;
    for (size_t i = 0; i < std::min<size_t>(2, agent_ids_.size()); ++i)
    {
      auto st_it = agent_states_.find(agent_ids_[i]);
      if (st_it == agent_states_.end() || !st_it->second.valid)
      {
        all_followers_clear = false;
        break;
      }
      const double along = obstacleAlong(spec.id, st_it->second.pos);
      min_clear_along = std::min(min_clear_along, along);
      if (along < spec.activation_window_exit + spec.release_margin)
        all_followers_clear = false;
    }

    bool payload_clear = true;
    if (fixed_obstacle_release_require_payload_clear_)
    {
      payload_clear = false;
      if (payload_valid || payload_using_cached_center)
      {
        const double payload_along = obstacleAlong(spec.id, payload_center);
        min_clear_along = std::min(min_clear_along, payload_along);
        payload_clear = payload_along >= spec.activation_window_exit + spec.release_margin;
      }
      else
      {
        ROS_WARN_THROTTLE(1.0,
                          "[SwarmMaster][FixedSchedule] obstacle=%s payload state invalid and no cached center, keep HOLD until followers clear or payload gate is disabled.",
                          spec.name.c_str());
      }
    }

    if (runtime.state == FixedObstacleState::IDLE &&
        leader_along >= spec.activation_window_enter - spec.blend_in)
      runtime.state = FixedObstacleState::APPROACH;
    if (runtime.state == FixedObstacleState::APPROACH &&
        leader_along >= spec.activation_window_enter)
      runtime.state = FixedObstacleState::ACTIVE;
    if (runtime.state == FixedObstacleState::ACTIVE &&
        leader_along >= spec.activation_window_exit)
    {
      runtime.state = FixedObstacleState::HOLD;
      runtime.hold_latched = true;
    }
    if (runtime.state == FixedObstacleState::HOLD && all_followers_clear && payload_clear)
      runtime.state = FixedObstacleState::RELEASE;

    if (runtime.state == FixedObstacleState::APPROACH)
      command_scale = blendRamp(leader_along, spec.activation_window_enter - spec.blend_in,
                                spec.activation_window_enter);
    else if (runtime.state == FixedObstacleState::ACTIVE || runtime.state == FixedObstacleState::HOLD)
      command_scale = 1.0;
    else if (runtime.state == FixedObstacleState::RELEASE)
    {
      const double release_start = spec.activation_window_exit + spec.release_margin;
      command_scale = 1.0 - blendRamp(min_clear_along, release_start, release_start + spec.blend_out);
      runtime.last_release_blend = command_scale;
      if (command_scale <= 1e-3)
      {
        runtime.state = FixedObstacleState::DONE;
        runtime.completed = true;
        runtime.hold_latched = false;
        ++current_fixed_obstacle_idx_;
      }
    }
    else if (runtime.state == FixedObstacleState::DONE)
    {
      runtime.completed = true;
      ++current_fixed_obstacle_idx_;
      command_scale = 0.0;
    }

    if (!runtime.completed && runtime.state != FixedObstacleState::IDLE)
    {
      active_cmd.type = spec.type;
      active_cmd.obstacle_id = spec.id;
      active_cmd.blend = command_scale;
      blanket_hold = (runtime.state == FixedObstacleState::HOLD) ||
                     (runtime.state == FixedObstacleState::RELEASE &&
                      leader_along > spec.activation_window_exit);
      publish_obstacle_id = spec.id;
      publish_state = runtime.state;
      publish_blanket_hold = blanket_hold;
      publish_command_scale = command_scale;
      publish_min_clear_along = min_clear_along;

      const int order = latest_leader_bezier_.order > 0 ? latest_leader_bezier_.order : 3;
      int first_cp = -1;
      int last_cp = -1;
      for (int c = 0; c < corrected_leader_ctrl.cols(); ++c)
      {
        const double along = obstacleAlong(spec.id, corrected_leader_ctrl.col(c));
        if (along >= spec.activation_window_enter - spec.blend_in &&
            along <= spec.activation_window_exit + spec.blend_out)
        {
          if (first_cp < 0)
            first_cp = c;
          last_cp = c;
        }
      }
      if (first_cp >= 0 && last_cp >= 0)
      {
        const int max_window_end = static_cast<int>((corrected_leader_ctrl.cols() - 1) / order) - 1;
        runtime.window_seg_begin = std::max(0, first_cp / order - spec.opt_window_margin);
        runtime.window_seg_end = std::min(max_window_end,
                                          last_cp / order + spec.opt_window_margin);
      }

      // Keep a traction-preserving reference for the cooperative window, but capture
      // it before the explicit leader mode-bias is added to avoid double-counting.
      const Eigen::MatrixXd leader_opt_reference_ctrl = corrected_leader_ctrl;

      buildFixedFormationCommands(corrected_leader_ctrl, active_cmd, command_scale, blanket_hold, commands);
      std::vector<FormationCommand> leader_bias_cmds;
      buildFixedFormationCommands(corrected_leader_ctrl, active_cmd, command_scale, false, leader_bias_cmds);
      for (int c = 0; c < corrected_leader_ctrl.cols() && c < static_cast<int>(leader_bias_cmds.size()); ++c)
      {
        const FormationCommand &bias_cmd = leader_bias_cmds[static_cast<size_t>(c)];
        if (bias_cmd.type == ObstacleType::NONE || bias_cmd.obstacle_id < 0 ||
            bias_cmd.obstacle_id >= static_cast<int>(fixed_obstacle_schedule_.size()))
          continue;

        const FixedObstacleSpec &bias_spec = fixed_obstacle_schedule_[static_cast<size_t>(bias_cmd.obstacle_id)];
        corrected_leader_ctrl.col(c) +=
            bias_cmd.blend * (bias_spec.world_from_local * bias_spec.mode_template.leader_bias_local);
      }
      generateFormationGuidance(corrected_leader_ctrl, agent_ctrl_pts_map, commands);

      if (follower_optimization_enabled_ &&
          !blanket_hold &&
          runtime.window_seg_begin >= 0 &&
          runtime.window_seg_end >= runtime.window_seg_begin)
      {
        // Keep the cooperative window anchored to the traction-corrected leader geometry
        // so it does not get pulled back to the raw nominal line inside the obstacle window.
        optimizeFixedObstacleWindow(corrected_leader_ctrl, leader_opt_reference_ctrl, agent_ctrl_pts_map,
                                    active_cmd, runtime.window_seg_begin, runtime.window_seg_end);
      }

      ROS_INFO_THROTTLE(0.5,
                        "[SwarmMaster][FixedSchedule] obstacle=%s state=%d blend=%.2f hold=%d "
                        "leader_along=%.2f min_clear=%.2f payload_clear=%d payload_cached=%d",
                        spec.name.c_str(), static_cast<int>(runtime.state), command_scale,
                        static_cast<int>(blanket_hold), leader_along, min_clear_along,
                        static_cast<int>(payload_clear),
                        static_cast<int>(payload_using_cached_center));
    }
  }

  if (!shouldPublishFixedScheduleUpdate(same_leader_traj, publish_obstacle_id, publish_state,
                                        publish_blanket_hold, publish_command_scale,
                                        publish_min_clear_along))
  {
    ROS_INFO_THROTTLE(0.5,
                      "[SwarmMaster][FixedSchedule] skip same-traj republish obstacle=%d state=%d blend=%.2f min_clear=%.2f",
                      publish_obstacle_id, static_cast<int>(publish_state), publish_command_scale,
                      publish_min_clear_along);
    return false;
  }

  if (agent_ctrl_pts_map.empty())
    generateFormationGuidance(corrected_leader_ctrl, agent_ctrl_pts_map, commands);

  applySwarmCollisionPenalty(agent_ctrl_pts_map);
  applyFollowerBoundaryCorrections(agent_ctrl_pts_map, same_leader_traj);
  applyLeaderBoundaryCorrection(corrected_leader_ctrl);

  const ego_planner::Bezier corrected_msg =
      buildAgentBezierMsg(0, corrected_leader_ctrl, publish_start_time, ++out_traj_id_);
  leader_corrected_pub_.publish(corrected_msg);

  for (const auto &kv : agent_ctrl_pts_map)
  {
    auto pub_it = guidance_pubs_.find(kv.first);
    if (pub_it == guidance_pubs_.end())
      continue;
    pub_it->second.publish(buildAgentBezierMsg(kv.first, kv.second, publish_start_time, ++out_traj_id_));
  }

  cacheFixedSchedulePublishSnapshot(publish_obstacle_id, publish_state,
                                    publish_blanket_hold, publish_command_scale,
                                    publish_min_clear_along,
                                    ros::Time::now());
  return true;
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

// ─────────────────────────────────────────────────────────────────────────────
// detectNarrowPassage
//   对每个主机控制点，沿航向垂直方向（XY 平面 left_dir）双向射线检测；
//   若两侧首个障碍物之间的净间距 < narrow_passage_threshold_，则标记为 true。
//
//   降级设计：
//   ① grid_map_ 为空或地图无有效数据 → 全 false，行为与改动前完全一致
//   ② 某侧射线超出地图范围也未碰障碍 → 视该侧间距为 max_ray（保守估计通畅）
// ─────────────────────────────────────────────────────────────────────────────
bool SwarmMasterCoordinator::detectNarrowPassage(
    const Eigen::MatrixXd& leader_ctrl_pts,
    std::vector<bool>& is_narrow_xy,
    std::vector<bool>& is_narrow_z,
    double ray_step)
{
  const int N = leader_ctrl_pts.cols();
  is_narrow_xy.assign(static_cast<size_t>(N), false);
  is_narrow_z.assign(static_cast<size_t>(N), false);

  // ── 降级条件1：地图不可用 ──────────────────────────────────────────────────
  if (!grid_map_)
    return false;

  {
    const Eigen::Vector3d map_min = grid_map_->getMapMinBoundary();
    const Eigen::Vector3d map_max = grid_map_->getMapMaxBoundary();
    const Eigen::Vector3d range   = map_max - map_min;
    if (range.x() < 1e-3 || range.y() < 1e-3 || range.z() < 1e-3)
      return false;  // 地图尚未接收到传感器数据
  }

  // 最远探测距离：取阈值的两倍，避免无限循环
  const double max_ray = narrow_passage_threshold_ * 2.0;
  const int    max_steps = static_cast<int>(max_ray / std::max(ray_step, 1e-3));

  // 复用 computeSmoothedHeadings() 计算平滑航向
  const Eigen::MatrixXd headings = computeSmoothedHeadings(leader_ctrl_pts);

  bool any_narrow = false;

  for (int c = 0; c < N; ++c)
  {
    const Eigen::Vector3d pos  = leader_ctrl_pts.col(c);
    const Eigen::Vector3d fwd  = headings.col(c);  // 已归一化，Z=0

    // XY 平面左方向（右手系：left = Z × forward）
    Eigen::Vector3d left(-fwd.y(), fwd.x(), 0.0);
    if (left.norm() < 1e-6) left = Eigen::Vector3d(0.0, 1.0, 0.0);
    else                     left.normalize();

    // ── 左侧射线 ──────────────────────────────────────────────────────────
    double dist_left = max_ray;
    for (int s = 1; s <= max_steps; ++s)
    {
      const Eigen::Vector3d probe = pos + left * (s * ray_step);
      const int occ = grid_map_->getInflateOccupancy(probe);
      if (occ < 0) { dist_left = max_ray; break; }  // 出界 → 保守视为通畅
      if (occ > 0) { dist_left = s * ray_step; break; }
    }

    // ── 右侧射线 ──────────────────────────────────────────────────────────
    double dist_right = max_ray;
    for (int s = 1; s <= max_steps; ++s)
    {
      const Eigen::Vector3d probe = pos - left * (s * ray_step);
      const int occ = grid_map_->getInflateOccupancy(probe);
      if (occ < 0) { dist_right = max_ray; break; }
      if (occ > 0) { dist_right = s * ray_step; break; }
    }

    const double width = dist_left + dist_right;
    if (width < narrow_passage_threshold_)
    {
      is_narrow_xy[static_cast<size_t>(c)] = true;
      any_narrow = true;
      ROS_DEBUG("[SwarmMaster] detectNarrowPassage: ctrl[%d] width=%.2f m "
                "(left=%.2f right=%.2f) < threshold=%.2f → NARROW_XY",
                c, width, dist_left, dist_right, narrow_passage_threshold_);
    }

    // ── Z 轴射线检测（上下）────────────────────────────────
    double dist_up = max_ray;
    for (int s = 1; s <= max_steps; ++s)
    {
      Eigen::Vector3d probe = pos + Eigen::Vector3d(0, 0, s * ray_step);
      int occ = grid_map_->getInflateOccupancy(probe);
      if (occ < 0) { dist_up = max_ray; break; }
      if (occ > 0) { dist_up = s * ray_step; break; }
    }

    double dist_down = max_ray;
    for (int s = 1; s <= max_steps; ++s)
    {
      Eigen::Vector3d probe = pos - Eigen::Vector3d(0, 0, s * ray_step);
      int occ = grid_map_->getInflateOccupancy(probe);
      if (occ < 0) { dist_down = max_ray; break; }
      if (occ > 0) { dist_down = s * ray_step; break; }
    }

    const double z_clearance = dist_up + dist_down;
    // Z 轴窤缝阈値：编队 Z 轴展开需要 ~1.83m + 安全余量
    const double z_narrow_threshold = 2.5;
    if (z_clearance < z_narrow_threshold)
    {
      is_narrow_z[static_cast<size_t>(c)] = true;
      any_narrow = true;
      ROS_DEBUG("[SwarmMaster] detectNarrowPassage: ctrl[%d] z_clearance=%.2f m "
                "(up=%.2f down=%.2f) < %.2f → NARROW_Z",
                c, z_clearance, dist_up, dist_down, z_narrow_threshold);
    }
  }

  if (any_narrow)
  {
    ROS_INFO_THROTTLE(1.0,
        "[SwarmMaster] Narrow passage detected along leader trajectory "
        "(threshold=%.2f m). Switching to vertical V-formation.",
        narrow_passage_threshold_);

    // ── 记录新发现的窄通道到 active_narrow_passages_ ──────────────────
    int first_narrow = -1, last_narrow = -1;
    for (int c = 0; c < N; ++c) {
      if (is_narrow_xy[static_cast<size_t>(c)] || is_narrow_z[static_cast<size_t>(c)]) {
        if (first_narrow < 0) first_narrow = c;
        last_narrow = c;
      }
    }
    if (first_narrow >= 0 && last_narrow >= 0) {
      NarrowPassageRecord rec;
      const int mid = (first_narrow + last_narrow) / 2;
      rec.center   = leader_ctrl_pts.col(mid);
      rec.forward  = headings.col(mid);
      // extent = 首末窄通道控制点沿飞行方向投影距离的一半
      const Eigen::Vector3d span =
          leader_ctrl_pts.col(last_narrow) - leader_ctrl_pts.col(first_narrow);
      rec.extent   = std::max(0.5, span.dot(rec.forward) / 2.0);
      rec.detected_time = ros::Time::now();
      rec.type = is_narrow_xy[static_cast<size_t>(mid)] ?
                 ObstacleType::DOOR_FRAME : ObstacleType::Z_SLIT;

      // 避免重复记录同一窄通道（中心距 < 2m 视为同一个）
      bool already_recorded = false;
      for (const auto& existing : active_narrow_passages_) {
        if ((existing.center - rec.center).norm() < 2.0) {
          already_recorded = true;
          break;
        }
      }
      if (!already_recorded) {
        active_narrow_passages_.push_back(rec);
        ROS_INFO("[SwarmMaster] Recorded narrow passage at "
                 "(%.1f, %.1f, %.1f), extent=%.2f m",
                 rec.center.x(), rec.center.y(), rec.center.z(), rec.extent);
      }
    }
  }

  return any_narrow;
}

// ─────────────────────────────────────────────────────────────────────────────
// classifyObstacleType
//   对每个主机控制点分类障碍物类型：DOOR_FRAME / RING / Z_SLIT / NONE。
//   同时为非NONE控制点前后各扩展过渡区（余弦 blend），
//   并将结果写入 active_narrow_passages_（Trailing Protection）。
// ─────────────────────────────────────────────────────────────────────────────
bool SwarmMasterCoordinator::classifyObstacleType(
    const Eigen::MatrixXd& leader_ctrl_pts,
    std::vector<FormationCommand>& commands,
    double ray_step)
{
  const int N = leader_ctrl_pts.cols();
  commands.assign(static_cast<size_t>(N), FormationCommand());

  // 降级：地图不可用时全部返回 NONE
  if (!grid_map_) return false;
  {
    const Eigen::Vector3d map_min = grid_map_->getMapMinBoundary();
    const Eigen::Vector3d map_max = grid_map_->getMapMaxBoundary();
    const Eigen::Vector3d range   = map_max - map_min;
    if (range.x() < 1e-3 || range.y() < 1e-3 || range.z() < 1e-3)
      return false;
  }

  const double max_ray  = narrow_passage_threshold_ * 2.0;
  const int max_steps   = static_cast<int>(max_ray / std::max(ray_step, 1e-3));
  const Eigen::MatrixXd headings = computeSmoothedHeadings(leader_ctrl_pts);
  const double z_narrow_threshold = 2.5;

  bool any_obstacle = false;

  for (int c = 0; c < N; ++c)
  {
    const Eigen::Vector3d pos = leader_ctrl_pts.col(c);
    const Eigen::Vector3d fwd = headings.col(c);

    Eigen::Vector3d left(-fwd.y(), fwd.x(), 0.0);
    if (left.norm() < 1e-6) left = Eigen::Vector3d(0.0, 1.0, 0.0);
    else                     left.normalize();

    // ── 1. XY 左右射线（同 detectNarrowPassage）──
    double dist_left = max_ray;
    for (int s = 1; s <= max_steps; ++s) {
      Eigen::Vector3d probe = pos + left * (s * ray_step);
      int occ = grid_map_->getInflateOccupancy(probe);
      if (occ < 0) { dist_left = max_ray; break; }
      if (occ > 0) { dist_left = s * ray_step; break; }
    }
    double dist_right = max_ray;
    for (int s = 1; s <= max_steps; ++s) {
      Eigen::Vector3d probe = pos - left * (s * ray_step);
      int occ = grid_map_->getInflateOccupancy(probe);
      if (occ < 0) { dist_right = max_ray; break; }
      if (occ > 0) { dist_right = s * ray_step; break; }
    }
    const double xy_width = dist_left + dist_right;
    const bool xy_narrow  = (xy_width < narrow_passage_threshold_);

    // ── 2. Z 轴上下射线 ──
    double dist_up = max_ray;
    for (int s = 1; s <= max_steps; ++s) {
      Eigen::Vector3d probe = pos + Eigen::Vector3d(0, 0, s * ray_step);
      int occ = grid_map_->getInflateOccupancy(probe);
      if (occ < 0) { dist_up = max_ray; break; }
      if (occ > 0) { dist_up = s * ray_step; break; }
    }
    double dist_down = max_ray;
    for (int s = 1; s <= max_steps; ++s) {
      Eigen::Vector3d probe = pos - Eigen::Vector3d(0, 0, s * ray_step);
      int occ = grid_map_->getInflateOccupancy(probe);
      if (occ < 0) { dist_down = max_ray; break; }
      if (occ > 0) { dist_down = s * ray_step; break; }
    }
    const double z_clearance = dist_up + dist_down;
    const bool z_narrow      = (z_clearance < z_narrow_threshold);

    // ── 3. 圆环检测：多距离前向扫描「中心通畅 + 四向对称命中 + 周向包围」──
    // 修复：原单点探测（pos + fwd * ring_detect_radius_）在圆环距离不等于
    // ring_detect_radius_ 时完全漏检。改为在 [0.5x, 2.0x] 范围内均匀扫描
    // ring_scan_steps_ 个候选中心，任意一个满足条件即判定为 RING。
    bool is_ring = false;
    {
      const int n_scan = std::max(ring_scan_steps_, 2);
      const double scan_min = 0.5 * ring_detect_radius_;
      const double scan_max = 2.0 * ring_detect_radius_;
      const double scan_step_dist = (scan_max - scan_min) / (n_scan - 1);
      const Eigen::Vector3d up_dir(0.0, 0.0, 1.0);
      const int n_angles = std::max(ring_detect_angles_, 4);
      const int ring_max_steps = static_cast<int>(ring_detect_radius_ / std::max(ray_step, 1e-3));

      for (int si = 0; si < n_scan && !is_ring; ++si)
      {
        const double scan_d = scan_min + si * scan_step_dist;
        const Eigen::Vector3d front_center = pos + fwd * scan_d;

        // ① 候选中心点本身必须无碰
        const int center_occ = grid_map_->getInflateOccupancy(front_center);
        bool center_clear = (center_occ == 0);

        // ② 中心通畅性增强：6方向在 ring_center_clear_dist_ 范围内均无障碍
        if (center_clear)
        {
          const double clear_d = std::max(0.2, ring_center_clear_dist_);
          const Eigen::Vector3d test_dirs[6] = {
              left, -left, up_dir, -up_dir, fwd, -fwd};
          for (const auto& d : test_dirs)
          {
            const int occ_near = grid_map_->getInflateOccupancy(front_center + d * clear_d);
            if (occ_near != 0) { center_clear = false; break; }
          }
        }
        if (!center_clear) continue;

        // ③ 四向命中距离（左/右/上/下）
        auto rayHitDist = [&](const Eigen::Vector3d& dir, double max_dist) -> double {
          const int steps = static_cast<int>(max_dist / std::max(ray_step, 1e-3));
          for (int s = 1; s <= steps; ++s) {
            const Eigen::Vector3d probe = front_center + dir * (s * ray_step);
            const int occ = grid_map_->getInflateOccupancy(probe);
            if (occ > 0) return s * ray_step;
            if (occ < 0) break;
          }
          return max_dist + 1.0;
        };

        const double d_left  = rayHitDist(left,    ring_detect_radius_);
        const double d_right = rayHitDist(-left,   ring_detect_radius_);
        const double d_up    = rayHitDist(up_dir,  ring_detect_radius_);
        const double d_down  = rayHitDist(-up_dir, ring_detect_radius_);

        const bool lateral_pair_hit = (d_left  <= ring_detect_radius_ && d_right <= ring_detect_radius_);
        const bool vertical_pair_hit = (d_up   <= ring_detect_radius_ && d_down  <= ring_detect_radius_);

        // ④ 对称性检查：改用比例约束（≤2.5），比绝对差值更鲁棒
        //    避免大/小圆环因绝对差值大而被误拒
        const double lat_ratio = (d_left  < d_right)
            ? (d_right / std::max(d_left,  1e-3))
            : (d_left  / std::max(d_right, 1e-3));
        const double ver_ratio = (d_up    < d_down)
            ? (d_down  / std::max(d_up,    1e-3))
            : (d_up    / std::max(d_down,  1e-3));
        const bool pair_symmetric = (lat_ratio <= 2.5) && (ver_ratio <= 2.5);

        // ⑤ 周向射线包围比例
        int hit_count = 0;
        for (int a = 0; a < n_angles; ++a)
        {
          const double angle = 2.0 * M_PI * a / n_angles;
          const Eigen::Vector3d ray_dir = left * std::cos(angle) + up_dir * std::sin(angle);
          for (int s = 1; s <= ring_max_steps; ++s) {
            Eigen::Vector3d probe = front_center + ray_dir * (s * ray_step);
            int occ = grid_map_->getInflateOccupancy(probe);
            if (occ > 0) { ++hit_count; break; }
            if (occ < 0) break;
          }
        }
        const double surround_ratio = static_cast<double>(hit_count) / static_cast<double>(n_angles);

        if (surround_ratio >= ring_surround_ratio_ &&
            lateral_pair_hit && vertical_pair_hit && pair_symmetric)
        {
          is_ring = true;
          ROS_DEBUG("[SwarmMaster] classifyObstacle: ctrl[%d] RING at scan_d=%.2f "
                    "(surround=%.1f%%, dl/dr=%.2f/%.2f, du/dd=%.2f/%.2f)",
                    c, scan_d, surround_ratio * 100.0, d_left, d_right, d_up, d_down);
        }
      }  // end scan loop
    }

    // ── 4. 优先级判定：Z_SLIT > RING > DOOR_FRAME > NONE ──
    if (z_narrow && !xy_narrow) {
      commands[static_cast<size_t>(c)].type  = ObstacleType::Z_SLIT;
      commands[static_cast<size_t>(c)].blend = 1.0;
      any_obstacle = true;
    } else if (is_ring) {
      commands[static_cast<size_t>(c)].type  = ObstacleType::RING;
      commands[static_cast<size_t>(c)].blend = 1.0;
      any_obstacle = true;
    } else if (xy_narrow && !z_narrow) {
      commands[static_cast<size_t>(c)].type  = ObstacleType::DOOR_FRAME;
      commands[static_cast<size_t>(c)].blend = 1.0;
      any_obstacle = true;
    }
    // 否则保留默认 NONE + blend=0
  }

  // ── 5. 过渡区扩展：为每个非NONE控制点前后各扩展 transition_ctrl_pts_ 个点 ──
  {
    std::vector<FormationCommand> smoothed = commands;
    for (int c = 0; c < N; ++c)
    {
      if (commands[static_cast<size_t>(c)].type == ObstacleType::NONE)
        continue;

      const ObstacleType src_type = commands[static_cast<size_t>(c)].type;

      // 入口过渡
      for (int t = 1; t <= transition_ctrl_pts_; ++t) {
        const int idx = c - t;
        if (idx < 0) break;
        if (commands[static_cast<size_t>(idx)].type != ObstacleType::NONE)
          break;
        const double ratio = static_cast<double>(t) /
                              static_cast<double>(transition_ctrl_pts_ + 1);
        const double alpha = 0.5 * (1.0 + std::cos(M_PI * ratio));
        if (alpha > smoothed[static_cast<size_t>(idx)].blend) {
          smoothed[static_cast<size_t>(idx)].type  = src_type;
          smoothed[static_cast<size_t>(idx)].blend = alpha;
        }
      }

      // 出口过渡（2倍长度）
      const int exit_transition = transition_ctrl_pts_ * 2;
      for (int t = 1; t <= exit_transition; ++t) {
        const int idx = c + t;
        if (idx >= N) break;
        if (commands[static_cast<size_t>(idx)].type != ObstacleType::NONE)
          break;
        const double ratio = static_cast<double>(t) /
                              static_cast<double>(exit_transition + 1);
        const double alpha = 0.5 * (1.0 + std::cos(M_PI * ratio));
        if (alpha > smoothed[static_cast<size_t>(idx)].blend) {
          smoothed[static_cast<size_t>(idx)].type  = src_type;
          smoothed[static_cast<size_t>(idx)].blend = alpha;
        }
      }
    }
    commands = smoothed;
  }

  // ── 6. 记录到 active_narrow_passages_（用于 Trailing Protection）──
  if (any_obstacle)
  {
    const Eigen::MatrixXd& h = headings;
    int first_obs = -1, last_obs = -1;
    ObstacleType dominant_type = ObstacleType::NONE;

    for (int c = 0; c < N; ++c) {
      if (commands[static_cast<size_t>(c)].type != ObstacleType::NONE &&
          commands[static_cast<size_t>(c)].blend >= 1.0) {
        if (first_obs < 0) first_obs = c;
        last_obs = c;
        dominant_type = commands[static_cast<size_t>(c)].type;
      }
    }

    if (first_obs >= 0) {
      NarrowPassageRecord rec;
      const int mid = (first_obs + last_obs) / 2;
      rec.center  = leader_ctrl_pts.col(mid);
      rec.forward = h.col(mid);
      const Eigen::Vector3d span =
          leader_ctrl_pts.col(last_obs) - leader_ctrl_pts.col(first_obs);
      rec.extent        = std::max(0.5, span.dot(rec.forward) / 2.0);
      rec.detected_time = ros::Time::now();
      rec.type          = dominant_type;

      bool already = false;
      for (const auto& ex : active_narrow_passages_)
        if ((ex.center - rec.center).norm() < 2.0) { already = true; break; }

      if (!already) {
        active_narrow_passages_.push_back(rec);
        const char* type_str =
            (dominant_type == ObstacleType::DOOR_FRAME) ? "DOOR_FRAME" :
            (dominant_type == ObstacleType::RING)       ? "RING" :
            (dominant_type == ObstacleType::Z_SLIT)     ? "Z_SLIT" : "NONE";
        ROS_INFO("[SwarmMaster] Recorded obstacle at (%.1f, %.1f, %.1f), "
                 "type=%s, extent=%.2f m",
                 rec.center.x(), rec.center.y(), rec.center.z(),
                 type_str, rec.extent);
      }
    }
  }

  return any_obstacle;
}

bool SwarmMasterCoordinator::checkFollowerCollision(
    const Eigen::MatrixXd& follower_ctrl_pts,
    std::vector<int>&      collision_indices)
{
  // === 坐标变换说明 ===
  // 本系统中主从机均使用 ENU 全局坐标系, 里程计输出均在同一世界坐标系下。
  // 因此从机偏移后的控制点可直接在主机的 GridMap 中查询碰撞，无需额外
  // SE(3) 变换。
  //
  // 若未来主从机使用独立局部坐标系, 需要:
  //   T_world_follower = T_world_master * T_master_formation_offset
  //   p_in_master_map = T_world_master^(-1) * p_follower_world
  // 其中 T 为 4x4 齐次变换矩阵, 包含 R(yaw) 和 t(position)。

  collision_indices.clear();

  if (!grid_map_)
  {
    ROS_WARN_THROTTLE(2.0, "[SwarmMaster] checkFollowerCollision: grid_map_ is null, skipping.");
    return false;
  }

  // ── 地图有效性检查 ──────────────────────────────────────────────────
  // GridMap 在 initMap() 阶段从 ROS 参数读取 map_size 设置边界。
  // 若无传感器数据源（点云/深度话题未配置或未发布），地图边界退化为零体积。
  // 此时碰撞查询无意义，直接返回 false（无碰撞），回退到原始 V 形偏移。
  {
    const Eigen::Vector3d map_min = grid_map_->getMapMinBoundary();
    const Eigen::Vector3d map_max = grid_map_->getMapMaxBoundary();
    const Eigen::Vector3d map_range = map_max - map_min;
    if (map_range.x() < 1e-3 || map_range.y() < 1e-3 || map_range.z() < 1e-3)
    {
      ROS_WARN_THROTTLE(5.0,
          "[SwarmMaster] checkFollowerCollision: GridMap has no valid data "
          "(range=[%.2f, %.2f, %.2f]), skipping collision check.",
          map_range.x(), map_range.y(), map_range.z());
      return false;
    }
  }

  const int N = follower_ctrl_pts.cols();
  if (N < 2)
    return false;

  // ─────────────────────────────────────────────────────────────────────────
  // Phase-1: 利用贝塞尔凸包性质检查控制点
  // 贝塞尔曲线位于其控制点凸包内。若所有控制点均无碰，则曲线必无碰。
  // (凸包无碰 → 曲线无碰) 是充分条件；反向不成立，因此 Phase-2 进一步采样确认。
  // ─────────────────────────────────────────────────────────────────────────
  for (int c = 0; c < N; ++c)
  {
    const Eigen::Vector3d pt = follower_ctrl_pts.col(c);
    // getInflateOccupancy: 1=碰撞, 0=无碰, -1=地图外(保守不计入碰撞)
    const int occ = grid_map_->getInflateOccupancy(pt);
    if (occ > 0)
      collision_indices.push_back(c);
  }

  // ─────────────────────────────────────────────────────────────────────────
  // Phase-2: De Casteljau 曲线采样 (t = 0.25, 0.50, 0.75)
  // 凸包是必要条件，对于窄通道场景（障碍物尺度接近分辨率）可能不够充分。
  // 对每段贝塞尔曲线额外在三个参数点采样，捕捉控制点凸包“漏检”的碰撞。
  // ─────────────────────────────────────────────────────────────────────────
  const int order   = (latest_leader_bezier_.order > 0) ? latest_leader_bezier_.order : 3;
  const int num_seg = (N - 1) / order;

  // 用集合容纳已记录索引，避免重复插入
  std::set<int> recorded(collision_indices.begin(), collision_indices.end());

  const double sample_ts[3] = {0.25, 0.50, 0.75};

  for (int seg = 0; seg < num_seg; ++seg)
  {
    const int base = seg * order;  // 该段首控制点列索引

    // 拷贝该段 order+1 个控制点到局部向量下备迭代
    std::vector<Eigen::Vector3d> dp(static_cast<size_t>(order + 1));
    for (int i = 0; i <= order; ++i)
      dp[static_cast<size_t>(i)] = follower_ctrl_pts.col(base + i);

    for (const double t : sample_ts)
    {
      // De Casteljau 算法：原地迭代差唃插値，O(order^2)
      std::vector<Eigen::Vector3d> work = dp;
      for (int r = 1; r <= order; ++r)
        for (int i = 0; i <= order - r; ++i)
          work[static_cast<size_t>(i)] = (1.0 - t) * work[static_cast<size_t>(i)]
                                        + t        * work[static_cast<size_t>(i + 1)];

      const Eigen::Vector3d sample_pt = work[0];
      const int occ = grid_map_->getInflateOccupancy(sample_pt);
      if (occ > 0)
      {
        // 以所属段的首控制点索引为标记，保持接口与 Phase-1 一致
        if (recorded.insert(base).second)
          collision_indices.push_back(base);

        ROS_WARN_THROTTLE(0.5,
            "[SwarmMaster] checkFollowerCollision: De Casteljau collision "
            "at seg=%d t=%.2f pos=(%.2f, %.2f, %.2f)",
            seg, t, sample_pt.x(), sample_pt.y(), sample_pt.z());
      }
    }
  }

  if (!collision_indices.empty())
    ROS_WARN_THROTTLE(0.5,
        "[SwarmMaster] checkFollowerCollision: %zu collision(s) found in follower trajectory.",
        collision_indices.size());

  return !collision_indices.empty();
}

// ─────────────────────────────────────────────────────────────────────────
// optimizeFollowerTrajectory  —  利用 BezierOptimizer::FORMATION 模式
//                               通过 L-BFGS 消除从机引导轨迹与障碍物的碰撞。
//
// 代价函数（全解析梯度）：
//   J = λ1·smooth + λ2·distance(rebound) + λ3·feasibility + λ5·formation
//   其中 λ2(~5000) >> λ5(~20) 确保安全优先，编队保持为软约束。
// ─────────────────────────────────────────────────────────────────────────
bool SwarmMasterCoordinator::optimizeFollowerTrajectory(
    const Eigen::MatrixXd& initial_ctrl_pts,
    const Eigen::MatrixXd& leader_ctrl_pts,
    int                    follower_index,
    double                 segment_duration,
    Eigen::MatrixXd&       optimized_ctrl_pts)
{
  // 未使用的参数（编队索引保留给未来多层编队扩展）
  (void)leader_ctrl_pts;
  (void)follower_index;

  if (!grid_map_ || initial_ctrl_pts.cols() < 4)
  {
    optimized_ctrl_pts = initial_ctrl_pts;
    return false;
  }

  // ── Step 1: 构造 BezierOptimizer 并注入共享地图 ──
  ego_planner::BezierOptimizer::Ptr follower_opt =
      std::make_unique<ego_planner::BezierOptimizer>();
  follower_opt->setEnvironment(grid_map_);

  // 使用 setParamDirect() 直接注入参数值，完全绕过 ROS 参数服务器。
  // 原因：nh_ 命名空间下只有 swarm_master/* 参数，optimization/* 不存在，
  // setParam(nh_) 会将所有优化权重读为默认值 -1.0，导致优化器必然内爆。
  // dist0 使用 follower_clearance_（由 YAML 正确读取），确保检测阈值与
  // 优化器安全距离严格一致，无硬编码。
  follower_opt->setParamDirect(
      10.0,                // lambda_smooth
      500.0,               // lambda_collision（降低避免深陷障碍物时梯度爆炸）
      1.0,                 // lambda_feasibility（提高确保速度/加速度约束有效）
      1.0,                 // lambda_fitness
      formation_weight_,   // lambda_formation = swarm_master/formation_weight
      follower_clearance_, // dist0 = 从 YAML swarm_master/follower_clearance 读取
      follower_max_vel_,   // max_vel = 从 YAML swarm_master/follower_max_vel 读取
      3.0,                 // max_acc  (m/s²)
      3                    // order: cubic Bezier
  );
  ROS_DEBUG("[SwarmMaster] follower_opt setParamDirect: lambda1=10.0 lambda2=500.0 "
            "lambda3=1.0 lambda5=%.1f dist0=%.3f max_vel=%.1f max_acc=3.0",
            formation_weight_, follower_clearance_, follower_max_vel_);

  // ── Step 2: 设置编队参考点（初始 V 形偏移控制点，即 initial_ctrl_pts）──
  {
    std::vector<Eigen::Vector3d> formation_refs;
    formation_refs.reserve(static_cast<size_t>(initial_ctrl_pts.cols()));
    for (int i = 0; i < initial_ctrl_pts.cols(); ++i)
      formation_refs.push_back(initial_ctrl_pts.col(i));
    follower_opt->setFormationReference(formation_refs);
  }

  // ── Step 3: 初始化控制点碰撞约束（射线采样，找附近障碍体素）──
  //   flag_first_init=false 表示使用当前控制点重建约束，不重置地图
  Eigen::MatrixXd ctrl_pts_copy = initial_ctrl_pts;  // initControlPoints 会修改输入
  follower_opt->initControlPoints(ctrl_pts_copy, true);

  // ── Step 4: 执行 FORMATION 模式 L-BFGS 优化 ──
  optimized_ctrl_pts = initial_ctrl_pts;
  const int opt_result =
      follower_opt->BezierOptimizeTrajFormation(optimized_ctrl_pts, segment_duration);

  // ── Step 5: 验证优化结果 ──
  if (opt_result == ego_planner::BezierOptimizer::OPTIMAL ||
      opt_result == ego_planner::BezierOptimizer::SATISFIED)
  {
    std::vector<int> remaining_collisions;
    if (!checkFollowerCollision(optimized_ctrl_pts, remaining_collisions))
      return true;  // 优化成功且无碰撞

    ROS_WARN("[SwarmMaster] optimizeFollowerTrajectory: "
             "L-BFGS %s but %zu collision(s) remain after optimization.",
             (opt_result == ego_planner::BezierOptimizer::OPTIMAL ? "converged" : "satisfied"),
             remaining_collisions.size());
  }
  else
  {
    ROS_WARN("[SwarmMaster] optimizeFollowerTrajectory: L-BFGS FAILED (code=%d).",
             opt_result);
  }

  return false;
}  // end optimizeFollowerTrajectory


void SwarmMasterCoordinator::runOnce()
{
  if (!checkReady())
    return;

  // 默认只在 leader 轨迹发生变化时下发一次引导，避免高频重复发布同一轨迹。
  // 但 fixed obstacle schedule 依赖 leader / follower 实际位置持续推进 HOLD/RELEASE，
  // 因此在该模式下允许对“同一条 nominal leader 轨迹”持续刷新修正引导。
  const bool same_leader_traj =
      (latest_leader_bezier_.traj_id == last_published_leader_traj_id_) &&
      (std::fabs((latest_leader_bezier_.start_time - last_published_leader_start_time_).toSec()) < 1e-4);

    const bool startup_sync_preview_refresh =
      startup_sync_enabled_ && !startup_sync_released_ &&
      same_leader_traj && refresh_guidance_on_same_leader_traj_;

  const bool allow_nonfixed_preview_refresh =
      startup_sync_preview_refresh &&
      !use_fixed_obstacle_schedule_ &&
      allow_same_traj_refresh_without_fixed_schedule_;

  const bool allow_same_traj_refresh =
      (use_fixed_obstacle_schedule_ && refresh_guidance_on_same_leader_traj_) ||
      allow_nonfixed_preview_refresh ||
      startup_sync_preview_refresh;

  if (same_leader_traj && !allow_same_traj_refresh)
    return;

  if (same_leader_traj && startup_sync_preview_refresh)
  {
    double min_refresh_interval = nonfixed_same_traj_refresh_interval_;
    if (startup_sync_same_traj_refresh_rate_hz_ > 1e-6)
    {
      min_refresh_interval = std::max(min_refresh_interval, 1.0 / startup_sync_same_traj_refresh_rate_hz_);
    }

    if (min_refresh_interval > 1e-6 && !last_nonfixed_same_traj_publish_stamp_.isZero())
    {
      const double dt = (ros::Time::now() - last_nonfixed_same_traj_publish_stamp_).toSec();
      if (dt < min_refresh_interval)
        return;
    }

    last_nonfixed_same_traj_publish_stamp_ = ros::Time::now();

    ROS_INFO_THROTTLE(1.0,
                      "[SwarmMaster] startup-sync previewing same-traj refresh before release.");
  }

  // 当 fixed schedule 需要对同一条 nominal leader 轨迹持续刷新时，
  // 控制点已经通过 boundary correction 重新锚定到当前状态。
  // 若继续沿用旧 start_time，traj_server 会把“新轨迹”当作已执行到中后段
  // 甚至已结束，导致 leader / follower 在障碍物附近进入悬停。
  // 因此 same-traj refresh 时需要把输出轨迹时间戳重锚到当前时刻。
  const ros::Time publish_start_time =
      (same_leader_traj && allow_same_traj_refresh) ? ros::Time::now()
                                                    : latest_leader_bezier_.start_time;

  Eigen::MatrixXd leader_ctrl = leaderBezierToCtrlPts(latest_leader_bezier_);
  if (leader_ctrl.cols() < 4)
    return;

  // 【预平滑】对 leader 控制点中间列做窗口=3 加权滑动平均。
  // 主机每次重规划后段间控制点存在高频跳变；直接叠加编队偏置会把抖动放大传给从机。
  // 首列/末列保持不动，维持轨迹端点边界条件（C0/C1 约束）。
  // 权重：前邻 0.25 + 当前 0.50 + 后邻 0.25，不引入新变量，操作原地完成。
  {
    const int N = leader_ctrl.cols();
    Eigen::MatrixXd smoothed = leader_ctrl;
    for (int c = 1; c < N - 1; ++c)
    {
      smoothed.col(c) = 0.25 * leader_ctrl.col(c - 1)
                      + 0.50 * leader_ctrl.col(c)
                      + 0.25 * leader_ctrl.col(c + 1);
    }
    leader_ctrl = smoothed;  // 首末列未动，中间列已平滑
  }

  if (use_fixed_obstacle_schedule_)
  {
    runFixedObstacleSchedule(leader_ctrl, publish_start_time, same_leader_traj);
    last_published_leader_traj_id_ = latest_leader_bezier_.traj_id;
    last_published_leader_start_time_ = latest_leader_bezier_.start_time;
    return;
  }

  std::unordered_map<int, Eigen::MatrixXd> agent_ctrl_pts_map;

  // ── Trailing Protection：检查从机是否已安全通过所有活跃窄通道 ──────
  // 根因修复：即使主机重规划后新轨迹起点已在障碍物后方，
  // 只要从机尚未飞过窄通道出口+安全余量，就持续注入 Z 轴编队保护。
  std::vector<double> trailing_z_mask;
  std::vector<ObstacleType> trailing_type_mask;  // 与 trailing_z_mask 同步：记录每个控制点的障碍物类型
  {
    const ros::Time now = ros::Time::now();

    // 清除超时记录，防止永久残留
    active_narrow_passages_.erase(
        std::remove_if(active_narrow_passages_.begin(),
                       active_narrow_passages_.end(),
                       [&](const NarrowPassageRecord& rec) {
                         return (now - rec.detected_time).toSec() >
                                narrow_passage_max_age_;
                       }),
        active_narrow_passages_.end());

    if (!active_narrow_passages_.empty())
    {
      const int N_trail = leader_ctrl.cols();
      trailing_z_mask.assign(static_cast<size_t>(N_trail), 0.0);
      trailing_type_mask.assign(static_cast<size_t>(N_trail), ObstacleType::NONE);

      for (const auto& passage : active_narrow_passages_)
      {
        // 检查所有从机是否已安全通过此窄通道
        bool all_followers_clear = true;
        for (const int fid : agent_ids_)
        {
          auto st = agent_states_.find(fid);
          if (st == agent_states_.end() || !st->second.valid) {
            all_followers_clear = false;
            break;
          }
          // 从机位置相对窄通道中心，沿飞行方向的投影
          const Eigen::Vector3d diff = st->second.pos - passage.center;
          const double along = diff.dot(passage.forward);
          // 从机必须超过窄通道出口(extent) + safety_margin 才算安全通过
          if (along < passage.extent + trailing_safety_margin_) {
            all_followers_clear = false;
            break;
          }
        }

        if (all_followers_clear)
          continue;  // 此窄通道从机已安全通过，不注入保护

        // ── 关键修复：主机轨迹超前检查 ──────────────────────────────────────
        // 问题根因：主机重规划后新轨迹起点已在障碍物后方，所有控制点的 along_c
        // 都超出 [−prot_range, +prot_range+margin]，导致位置插值法 mask 全零，
        // Trailing Protection 失效，从机尚在穿越时队形被立即恢复 → 碰撞风险。
        //
        // 修复方案：若存在任意从机当前位置仍在障碍物保护区内，
        // 则对所有控制点强制注入 blend=1.0 的全量保护（"毯式保护"），
        // 直到所有从机飞出保护区后再恢复位置插值渐退。
        bool any_follower_in_zone = false;
        for (const int fid : agent_ids_) {
          auto st = agent_states_.find(fid);
          if (st != agent_states_.end() && st->second.valid) {
            const Eigen::Vector3d diff_f = st->second.pos - passage.center;
            const double along_f = diff_f.dot(passage.forward);
            if (along_f < passage.extent + trailing_safety_margin_) {
              any_follower_in_zone = true;
              break;
            }
          }
        }

        if (any_follower_in_zone)
        {
          // 主机已超前，但从机还在穿越 → 对全部控制点强制注入全量保护
          for (int c = 0; c < N_trail; ++c) {
            trailing_z_mask[static_cast<size_t>(c)]    = 1.0;
            trailing_type_mask[static_cast<size_t>(c)] = passage.type;
          }
          ROS_INFO_THROTTLE(0.5,
              "[SwarmMaster] Blanket trailing protection: "
              "leader past obstacle but follower still in passage zone.");
          continue;  // 毯式保护已注入，跳过位置插值
        }

        // 从机尚未进入障碍物区域但轨迹即将靠近 → 位置插值渐入保护
        for (int c = 0; c < N_trail; ++c)
        {
          const Eigen::Vector3d ctrl_pos = leader_ctrl.col(c);
          const Eigen::Vector3d diff_c   = ctrl_pos - passage.center;
          const double along_c = diff_c.dot(passage.forward);
          const double prot_range = passage.extent + trailing_safety_margin_;

          if (along_c >= -prot_range && along_c <= prot_range)
          {
            // 核心保护区：完全编队变换
            trailing_z_mask[static_cast<size_t>(c)] = 1.0;
            trailing_type_mask[static_cast<size_t>(c)] = passage.type;
          }
          else if (along_c > prot_range &&
                   along_c < prot_range + trailing_safety_margin_)
          {
            // 出口退出过渡区（余弦插值）
            const double exit_ratio =
                (along_c - prot_range) / trailing_safety_margin_;
            const double alpha = 0.5 * (1.0 + std::cos(M_PI * exit_ratio));
            const double prev_exit = trailing_z_mask[static_cast<size_t>(c)];
            trailing_z_mask[static_cast<size_t>(c)] = std::max(prev_exit, alpha);
            if (alpha > prev_exit)
              trailing_type_mask[static_cast<size_t>(c)] = passage.type;
          }
          else if (along_c < -prot_range &&
                   along_c > -prot_range - trailing_safety_margin_)
          {
            // 入口过渡区（余弦插值）
            const double entry_ratio =
                (-prot_range - along_c) / trailing_safety_margin_;
            const double alpha = 0.5 * (1.0 + std::cos(M_PI * entry_ratio));
            const double prev_entry = trailing_z_mask[static_cast<size_t>(c)];
            trailing_z_mask[static_cast<size_t>(c)] = std::max(prev_entry, alpha);
            if (alpha > prev_entry)
              trailing_type_mask[static_cast<size_t>(c)] = passage.type;
          }
        }
      }

      // 再次清理从机已完全通过的窄通道记录
      active_narrow_passages_.erase(
          std::remove_if(active_narrow_passages_.begin(),
                         active_narrow_passages_.end(),
                         [&](const NarrowPassageRecord& rec) {
                           for (const int fid : agent_ids_) {
                             auto st = agent_states_.find(fid);
                             if (st == agent_states_.end() || !st->second.valid)
                               return false;
                             Eigen::Vector3d d = st->second.pos - rec.center;
                             if (d.dot(rec.forward) <
                                 rec.extent + trailing_safety_margin_)
                               return false;
                           }
                           return true;  // 所有从机已安全通过
                         }),
          active_narrow_passages_.end());

      // 若 mask 全为 0，则无需传入（清空以触发 generateFormationGuidance 内部默认逻辑）
      bool has_trailing = false;
      for (double v : trailing_z_mask)
        if (v > 1e-3) { has_trailing = true; break; }
      if (!has_trailing) {
        trailing_z_mask.clear();
        trailing_type_mask.clear();
      } else
        ROS_INFO_THROTTLE(0.5, "[SwarmMaster] Trailing protection active: "
                          "%zu passage(s), injecting Z-mask into guidance.",
                          active_narrow_passages_.size());
    }
  }  // end Trailing Protection

  // 将 trailing_z_mask 转换为 FormationCommand（类型从 active_narrow_passages_ 记录的 rec.type 获取）
  std::vector<FormationCommand> trailing_cmds;
  if (!trailing_z_mask.empty()) {
    trailing_cmds.resize(trailing_z_mask.size());
    for (size_t i = 0; i < trailing_z_mask.size(); ++i) {
      trailing_cmds[i].type  = (trailing_z_mask[i] > 1e-3)
                                 ? trailing_type_mask[i]   // 使用通道记录的实际障碍物类型
                                 : ObstacleType::NONE;
      trailing_cmds[i].blend = trailing_z_mask[i];
    }
  }
  // 调用实时障碍物分类，获取每个控制点的类型（DOOR_FRAME / RING / Z_SLIT / NONE）
  std::vector<FormationCommand> classify_cmds;
  if (!disable_online_classification_)
    classifyObstacleType(leader_ctrl, classify_cmds);
  else
    classify_cmds.assign(static_cast<size_t>(leader_ctrl.cols()), FormationCommand());

  // ── 障碍物识别去抖动：时间窗口确认缓冲 ──────────────────────────────────
  // 统计 classify_cmds 中出现次数最多的非 NONE 类型（dominant type）。
  // 只有连续 obs_confirm_frames_ 帧识别到同一类型才真正确认并下发编队指令。
  // 确认后保持至少 obs_release_duration_ 秒，抑制单帧误识别引起的队形闪变。
  // 去抖后的结果直接覆写 classify_cmds，后续可视化和合并均基于确认类型。
  {
    int cnt_ring = 0, cnt_door = 0, cnt_slit = 0;
    for (const auto& cmd : classify_cmds) {
      if      (cmd.type == ObstacleType::RING)       ++cnt_ring;
      else if (cmd.type == ObstacleType::DOOR_FRAME) ++cnt_door;
      else if (cmd.type == ObstacleType::Z_SLIT)     ++cnt_slit;
    }
    // 优先级与编队控制一致：Z_SLIT > RING > DOOR_FRAME > NONE
    ObstacleType raw_dominant = ObstacleType::NONE;
    if      (cnt_slit > 0) raw_dominant = ObstacleType::Z_SLIT;
    else if (cnt_ring > 0) raw_dominant = ObstacleType::RING;
    else if (cnt_door > 0) raw_dominant = ObstacleType::DOOR_FRAME;

    // 连续帧计数：类型变化时重置
    if (raw_dominant == obs_last_raw_type_) {
      ++obs_consecutive_count_;
    } else {
      obs_consecutive_count_ = 1;
      obs_last_raw_type_ = raw_dominant;
    }

    const ros::Time now_db = ros::Time::now();
    if (raw_dominant != ObstacleType::NONE &&
        obs_consecutive_count_ >= obs_confirm_frames_)
    {
      // 连续帧数达到阈值 → 确认（刷新时间戳，类型可能不变也要刷新以防保持超时）
      if (raw_dominant != obs_confirmed_type_)
        ROS_INFO("[SwarmMaster][Debounce] Type confirmed: %s (after %d frames)",
                 raw_dominant == ObstacleType::RING       ? "RING" :
                 raw_dominant == ObstacleType::DOOR_FRAME ? "DOOR_FRAME" :
                 raw_dominant == ObstacleType::Z_SLIT     ? "Z_SLIT" : "NONE",
                 obs_consecutive_count_);
      obs_confirmed_type_ = raw_dominant;
      obs_confirm_time_   = now_db;
    }
    else if (raw_dominant == ObstacleType::NONE)
    {
      // 当前帧没有识别到障碍物，判断是否已超过最少保持时间
      if (obs_confirmed_type_ != ObstacleType::NONE &&
          !obs_confirm_time_.isZero() &&
          (now_db - obs_confirm_time_).toSec() >= obs_release_duration_)
      {
        ROS_INFO("[SwarmMaster][Debounce] Type released: %s → NONE",
                 obs_confirmed_type_ == ObstacleType::RING       ? "RING" :
                 obs_confirmed_type_ == ObstacleType::DOOR_FRAME ? "DOOR_FRAME" :
                 obs_confirmed_type_ == ObstacleType::Z_SLIT     ? "Z_SLIT" : "NONE");
        obs_confirmed_type_ = ObstacleType::NONE;
      }
      // 否则保持上次确认类型（obs_confirm_time_ 不刷新，继续倒计时）
    }

    // 用确认后的类型覆写 classify_cmds（过滤掉所有不匹配的非NONE条目）
    for (auto& cmd : classify_cmds) {
      if (cmd.type != ObstacleType::NONE && cmd.type != obs_confirmed_type_) {
        cmd.type  = ObstacleType::NONE;
        cmd.blend = 0.0;
      }
    }
    // 若确认类型为 NONE 但 classify 有残余，也清零
    if (obs_confirmed_type_ == ObstacleType::NONE) {
      for (auto& cmd : classify_cmds) { cmd.type = ObstacleType::NONE; cmd.blend = 0.0; }
    }
  }


  // ── 障碍物识别状态可视化（终端） ─────────────────────────────────────
  // 用 classify_cmds（实时识别结果）输出主机当前障碍物状态：
  //   None / Circle / door / Narrow
  // 注：这里不使用 merged_cmds，避免 Trailing Protection 历史记忆影响“前方识别”观察。
  {
    size_t cnt_none = 0, cnt_circle = 0, cnt_door = 0, cnt_narrow = 0;
    for (const auto& cmd : classify_cmds)
    {
      switch (cmd.type)
      {
        case ObstacleType::RING:
          ++cnt_circle;
          break;
        case ObstacleType::DOOR_FRAME:
          ++cnt_door;
          break;
        case ObstacleType::Z_SLIT:
          ++cnt_narrow;
          break;
        case ObstacleType::NONE:
        default:
          ++cnt_none;
          break;
      }
    }

    std::string state = "None";
    const char* color = "\033[1;32m";  // 绿色
    const char* icon  = ".";
    // 与编队控制一致的优先级：Narrow > Circle > door > None
    if (cnt_narrow > 0) {
      state = "Narrow";
      color = "\033[1;31m";            // 红色
      icon  = "||";
    } else if (cnt_circle > 0) {
      state = "Circle";
      color = "\033[1;35m";            // 紫色
      icon  = "O";
    } else if (cnt_door > 0) {
      state = "door";
      color = "\033[1;33m";            // 黄色
      icon  = "[]";
    }

    // 仅在状态变化时打印，或每 2s 保底刷新一次，避免刷屏。
    static std::string last_state = "";
    static ros::Time last_print_time(0.0);
    const ros::Time now = ros::Time::now();
    const bool state_changed = (state != last_state);
    const bool periodic_refresh = (last_print_time.isZero() || (now - last_print_time).toSec() > 2.0);
    if (state_changed || periodic_refresh)
    {
      ROS_INFO("%s[SwarmMaster][ObstacleState] %s %s  (None=%zu, Circle=%zu, door=%zu, Narrow=%zu)\033[0m",
               color, icon, state.c_str(), cnt_none, cnt_circle, cnt_door, cnt_narrow);
      last_state = state;
      last_print_time = now;
    }

    // ── RViz 障碍物状态 TEXT Marker（跟随主机位置实时更新）──────────
    {
      visualization_msgs::Marker mk;
      mk.header.frame_id = "world";
      mk.header.stamp    = ros::Time::now();
      mk.ns              = "obs_state";
      mk.id              = 9999;
      mk.type            = visualization_msgs::Marker::TEXT_VIEW_FACING;
      mk.action          = visualization_msgs::Marker::ADD;
      mk.scale.z         = 0.50;
      mk.pose.orientation.w = 1.0;

      // 位置：主机轨迹第一个控制点正上方
      const Eigen::Vector3d leader_pt = leader_ctrl.col(0);
      mk.pose.position.x = leader_pt.x();
      mk.pose.position.y = leader_pt.y();
      mk.pose.position.z = leader_pt.z() + 1.3;

      // 颜色随状态变化：绿=None，红=Narrow，紫=Circle，黄=door
      if (state == "Narrow") {
        mk.color.r = 1.0f; mk.color.g = 0.2f; mk.color.b = 0.2f;
      } else if (state == "Circle") {
        mk.color.r = 0.8f; mk.color.g = 0.2f; mk.color.b = 1.0f;
      } else if (state == "door") {
        mk.color.r = 1.0f; mk.color.g = 0.85f; mk.color.b = 0.1f;
      } else {
        mk.color.r = 0.2f; mk.color.g = 1.0f; mk.color.b = 0.2f;
      }
      mk.color.a  = 1.0f;
      mk.text     = state;
      mk.lifetime = ros::Duration(0.6);  // 超时自动消失

      obs_state_marker_pub_.publish(mk);
    }
  }

  // 按优先级合并：trailing_cmds 优先（安全保底），其次 classify_cmds（实时识别）
  // 修复根因：原来 classify > trailing，当主机重规划后 classify 全为 NONE，
  // 直接覆盖了 trailing 的保护，导致从机未通过障碍物时队形被错误恢复。
  // 现在 trailing 优先：已知未通过的障碍物保护 > 实时检测的当前环境分类。
  const size_t n_pts = static_cast<size_t>(leader_ctrl.cols());
  std::vector<FormationCommand> merged_cmds(n_pts);
  for (size_t i = 0; i < n_pts; ++i) {
    if (!trailing_cmds.empty() && i < trailing_cmds.size() &&
        trailing_cmds[i].type != ObstacleType::NONE) {
      merged_cmds[i] = trailing_cmds[i];  // trailing 优先：保护从机安全通过
    } else if (i < classify_cmds.size() &&
               classify_cmds[i].type != ObstacleType::NONE) {
      merged_cmds[i] = classify_cmds[i];  // 实时分类：主机前方障碍物响应
    }
    // 否则保持默认构造的 NONE
  }

  generateFormationGuidance(leader_ctrl, agent_ctrl_pts_map, merged_cmds);

  // === Step 4.5: 从机轨迹碰撞检测与编队变换避障 ===
  // ① 由 follower_collision_check_ 开关控制（YAML: follower_collision_check）
  // ② 地图不可用时直接跳过，保留原始 V 形偏置
  // ③ 碰撞时构造 forced_z_mask，通过 generateFormationGuidance 统一重新生成
  //   （避免与 transition_mask 双重叠加，确保过渡平滑）
  {
    const bool map_ok = (grid_map_ != nullptr);
    if (!map_ok)
      ROS_WARN_THROTTLE(2.0, "[SwarmMaster] Step 4.5 skipped: grid_map_ not"
                        " available, using raw V-formation guidance.");

    for (size_t k = 0; k < agent_ids_.size(); ++k)
    {
      const int agent_id = agent_ids_[k];
      auto it = agent_ctrl_pts_map.find(agent_id);
      if (it == agent_ctrl_pts_map.end()) continue;
      Eigen::MatrixXd& ctrl = it->second;

      if (!follower_collision_check_ || !map_ok)
        continue;

      std::vector<int> collision_indices;
      const bool has_collision = checkFollowerCollision(ctrl, collision_indices);
      if (!has_collision)
        continue;

      ROS_WARN("[SwarmMaster] Agent %d: %zu collision(s), rebuilding with forced Z-mask.",
               agent_id, collision_indices.size());

      // === 构造 forced_z_mask（碰撞点=1.0，前后各 2*transition_ctrl_pts_ 余弦过渡）===
      const int N_ctrl = ctrl.cols();
      std::vector<double> forced_z_mask(static_cast<size_t>(N_ctrl), 0.0);
      const int extend = transition_ctrl_pts_ * 2;
      for (int ci : collision_indices) {
        forced_z_mask[static_cast<size_t>(ci)] = 1.0;
        for (int t = 1; t <= extend; ++t) {
          const double ratio = static_cast<double>(t) /
                                static_cast<double>(extend + 1);
          const double alpha = 0.5 * (1.0 + std::cos(M_PI * ratio));
          if (ci - t >= 0)
            forced_z_mask[static_cast<size_t>(ci - t)] =
                std::max(forced_z_mask[static_cast<size_t>(ci - t)], alpha);
          if (ci + t < N_ctrl)
            forced_z_mask[static_cast<size_t>(ci + t)] =
                std::max(forced_z_mask[static_cast<size_t>(ci + t)], alpha);
        }
      }

      // 将 forced_z_mask 转换为 FormationCommand（回查 classify_cmds 获取实际障碍物类型）
      std::vector<FormationCommand> forced_cmds(static_cast<size_t>(N_ctrl));
      for (size_t i = 0; i < static_cast<size_t>(N_ctrl); ++i) {
        if (forced_z_mask[i] > 1e-3) {
          // 优先使用 classifyObstacleType 检测到的实际障碍物类型
          forced_cmds[i].type = (i < classify_cmds.size() && classify_cmds[i].type != ObstacleType::NONE)
                                  ? classify_cmds[i].type
                                  : ObstacleType::DOOR_FRAME;  // fallback
        }
        forced_cmds[i].blend = forced_z_mask[i];
      }

      // 以 forced_cmds 重新生成所有从机编队引导控制点，只取当前 agent_id 的结果覆盖
      std::unordered_map<int, Eigen::MatrixXd> rebuilt_map;
      generateFormationGuidance(leader_ctrl, rebuilt_map, forced_cmds);
      auto new_it = rebuilt_map.find(agent_id);
      if (new_it != rebuilt_map.end())
      {
        ctrl = new_it->second;
        ROS_INFO("[SwarmMaster] Agent %d: forced Z-mask applied, guidance rebuilt.", agent_id);
      }
    }  // end for each agent
  }  // end Step 4.5

  applySwarmCollisionPenalty(agent_ctrl_pts_map);

  // 从机边界修正：统一通过 helper 执行，避免 runOnce/fixed-schedule 双份逻辑漂移。
  applyFollowerBoundaryCorrections(agent_ctrl_pts_map, same_leader_traj);

  Eigen::MatrixXd leader_corrected_ctrl = leader_ctrl;
  if (online_payload_opt_enabled_)
  {
    optimizeOnlinePayloadWindow(leader_corrected_ctrl, leader_ctrl, agent_ctrl_pts_map);
  }
  applyLeaderBoundaryCorrection(leader_corrected_ctrl);

  const ego_planner::Bezier corrected_msg =
      buildAgentBezierMsg(0, leader_corrected_ctrl, publish_start_time, ++out_traj_id_);
  leader_corrected_pub_.publish(corrected_msg);

  for (const auto &kv : agent_ctrl_pts_map)
  {
    const int agent_id = kv.first;
    auto pub_it = guidance_pubs_.find(agent_id);
    if (pub_it == guidance_pubs_.end())
      continue;

    const ego_planner::Bezier msg =
        buildAgentBezierMsg(agent_id, kv.second, publish_start_time, ++out_traj_id_);
    pub_it->second.publish(msg);
  }

  if (allow_nonfixed_preview_refresh)
  {
    last_nonfixed_same_traj_publish_stamp_ = ros::Time::now();
  }

  last_published_leader_traj_id_ = latest_leader_bezier_.traj_id;
  last_published_leader_start_time_ = latest_leader_bezier_.start_time;
}

} // namespace ego_planner
