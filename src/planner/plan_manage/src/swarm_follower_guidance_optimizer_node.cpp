#include <bezier_opt/bezier_optimizer.h>
#include <ego_planner/Bezier.h>
#include <plan_env/grid_map.h>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

#include <algorithm>
#include <cmath>

class SwarmFollowerGuidanceOptimizer
{
public:
  void init(ros::NodeHandle &nh)
  {
    nh_ = nh;

    nh_.param("enable_local_refine", enable_local_refine_, true);
    nh_.param("enable_collision_rebound", enable_collision_rebound_, false);
    nh_.param("min_start_time_delay", min_start_time_delay_, 0.03);
    nh_.param("fallback_segment_dt", fallback_segment_dt_, 0.1);
    // 滚动轨迹接续参数
    // stitch_time_margin_: 从 0.05s 扩大到 0.12s
    // 理由：更大的前向裕量使接续点远离已过期段，给优化器更多可调控制点
    nh_.param("refine_window_segs", refine_window_segs_, 5);
    // refine_window_segs_: 从 3 扩大到 5
    // 理由：更大的优化窗口涵盖 C2 修正影响范围（P2 已被移动），
    //        BezierOptimizeTrajRefine 有足够自由度消化段间不连续
    nh_.param("stitch_time_margin", stitch_time_margin_, 0.12);
    have_running_traj_ = false;

    nh_.param("use_local_map", use_local_map_, false);
    if (use_local_map_)
    {
      grid_map_.reset(new GridMap);
      grid_map_->initMap(nh_);
      ROS_INFO("[FollowerGuidance] local GridMap enabled.");
    }

    optimizer_.reset(new ego_planner::BezierOptimizer);
    optimizer_->setParam(nh_);
    if (grid_map_)
    {
      optimizer_->setEnvironment(grid_map_);
      optimizer_->a_star_.reset(new AStar);
      optimizer_->a_star_->initGridMap(grid_map_, Eigen::Vector3i(100, 100, 100));
    }

    sub_ = nh_.subscribe("guidance_in", 10, &SwarmFollowerGuidanceOptimizer::guidanceCallback, this);
    pub_ = nh_.advertise<ego_planner::Bezier>("planning/bezier", 10);

    // -------------------------------------------------------------------------
    // 被动避障初始化：odom 订阅 + 安全定时器（仅 use_local_map_ 时生效）
    // -------------------------------------------------------------------------
    nh_.param("safety_look_ahead", safety_look_ahead_, 2.0);
    nh_.param("safety_check_step", safety_check_step_, 0.05);
    nh_.param("safety_cooldown", safety_cooldown_, 0.25);
    last_safety_replan_time_ = ros::Time(0);

    nh_.param("warmup_frames", warmup_total_frames_, 15);
    nh_.param("guidance_lock_frames", guidance_lock_frames_, 5);
    nh_.param("hover_hold_frames", hover_hold_frames_, 8);
    nh_.param("bridge_frames", bridge_frames_, 15);
    nh_.param("guidance_lock_pos_tol", guidance_lock_pos_tol_, 0.25);

    nh_.param("startup_hold_enabled", startup_hold_enabled_, true);
    nh_.param("startup_guidance_ready_frames", guidance_lock_frames_, guidance_lock_frames_);
    nh_.param("startup_bridge_duration", startup_bridge_duration_, 0.75);
    nh_.param("startup_hover_timeout", startup_hover_timeout_, 0.40);
    nh_.param("startup_nominal_guidance_rate", startup_nominal_guidance_rate_, 20.0);
    nh_.param("startup_bridge_stitch_pos_margin", startup_bridge_stitch_pos_margin_, 0.15);
    nh_.param("startup_bridge_max_stitch_speed", startup_bridge_max_stitch_speed_, 0.25);
    nh_.param("startup_bridge_use_s_curve_alpha", startup_bridge_use_s_curve_alpha_, true);

    if (startup_bridge_duration_ > 1e-3)
    {
      bridge_frames_ = std::max(1, static_cast<int>(std::round(startup_bridge_duration_ *
                                                                std::max(1.0, startup_nominal_guidance_rate_))));
    }
    if (startup_hover_timeout_ > 1e-3)
    {
      hover_hold_frames_ = std::max(1, static_cast<int>(std::round(startup_hover_timeout_ *
                                                                    std::max(1.0, startup_nominal_guidance_rate_))));
    }

    active_warmup_frames_ = std::max(1, warmup_total_frames_);
    startup_stage_ = startup_hold_enabled_ ? StartupStage::WAIT_GUIDANCE : StartupStage::TRACK_FORMATION;
    stage_enter_stamp_ = ros::Time::now();

    // 始终订阅 odom（无论 use_local_map_），用于首次 guidance p_stitch 修正和 warmup 过渡
    odom_sub_ = nh_.subscribe("odom_in", 10,
                              &SwarmFollowerGuidanceOptimizer::odomCallback,
                              this);
    ROS_INFO("[FollowerGuidance] Odom subscriber always active (warmup_frames=%d).",
             warmup_total_frames_);

    if (use_local_map_)
    {
      // 安全检测定时器：20Hz，独立于 guidanceCallback 运行
      safety_timer_ = nh_.createTimer(
          ros::Duration(safety_check_step_),
          &SwarmFollowerGuidanceOptimizer::safetyCheckCallback, this);

      ROS_INFO("[FollowerGuidance] Passive obstacle avoidance ENABLED. "
               "look_ahead=%.2f s, cooldown=%.2f s",
               safety_look_ahead_, safety_cooldown_);
    }

    ROS_INFO("[FollowerGuidance] initialized. refine=%d rebound=%d local_map=%d",
             static_cast<int>(enable_local_refine_),
             static_cast<int>(enable_collision_rebound_),
             static_cast<int>(use_local_map_));
    ROS_INFO("[FollowerGuidance] startup FSM: enabled=%d lock_frames=%d hold_frames=%d bridge_frames=%d lock_tol=%.2f",
         static_cast<int>(startup_hold_enabled_), guidance_lock_frames_, hover_hold_frames_, bridge_frames_,
         guidance_lock_pos_tol_);
    ROS_INFO("[FollowerGuidance] startup bridge guard: pos_margin=%.2f m max_stitch_speed=%.2f m/s s_curve_alpha=%d",
         startup_bridge_stitch_pos_margin_, startup_bridge_max_stitch_speed_,
         static_cast<int>(startup_bridge_use_s_curve_alpha_));
  }

private:
  enum class StartupStage
  {
    WAIT_GUIDANCE = 0,
    HOVER_HOLD,
    BRIDGE_TO_FORMATION,
    TRACK_FORMATION
  };

  void publishHoverCommand(const ego_planner::Bezier &tmpl)
  {
    ego_planner::Bezier out = tmpl;
    if (out.pos_pts.empty())
      return;

    geometry_msgs::Point p;
    p.x = hover_ref_pos_.x();
    p.y = hover_ref_pos_.y();
    p.z = hover_ref_pos_.z();
    for (auto &cp : out.pos_pts)
      cp = p;

    const double delay = std::max(0.0, min_start_time_delay_);
    out.start_time = ros::Time::now() + ros::Duration(delay);
    pub_.publish(out);
  }

  static Eigen::MatrixXd toCtrlPts(const ego_planner::Bezier &msg)
  {
    Eigen::MatrixXd ctrl(3, msg.pos_pts.size());
    for (size_t i = 0; i < msg.pos_pts.size(); ++i)
    {
      ctrl(0, i) = msg.pos_pts[i].x;
      ctrl(1, i) = msg.pos_pts[i].y;
      ctrl(2, i) = msg.pos_pts[i].z;
    }
    return ctrl;
  }

  static void fillCtrlPts(ego_planner::Bezier &msg, const Eigen::MatrixXd &ctrl)
  {
    msg.pos_pts.clear();
    msg.pos_pts.reserve(ctrl.cols());
    for (int i = 0; i < ctrl.cols(); ++i)
    {
      geometry_msgs::Point p;
      p.x = ctrl(0, i);
      p.y = ctrl(1, i);
      p.z = ctrl(2, i);
      msg.pos_pts.push_back(p);
    }
  }

  static Eigen::Vector3d clampVecNorm(const Eigen::Vector3d &v, double max_norm)
  {
    if (max_norm <= 1e-9)
      return v;
    const double n = v.norm();
    if (n <= max_norm || n <= 1e-9)
      return v;
    return v * (max_norm / n);
  }

  static Eigen::Vector3d evalBezierAt(const Eigen::MatrixXd &ctrl, double ts, int order, double t)
  {
    if (order != 3 || ctrl.cols() < 4 || ts <= 1e-6)
    {
      if (ctrl.cols() > 0)
        return ctrl.col(0);
      return Eigen::Vector3d::Zero();
    }

    // 正确的段数 = (总列数 - 1) / order
    // 例: 10 列 (3段) → (10-1)/3 = 3 ✓；原公式 10/3=3 碰巧对，
    // 但对非 3N+1 的退化输入（如悬停全同控制点）可能产生越界访问。
    const int seg_num = static_cast<int>((ctrl.cols() - 1) / order);
    if (seg_num <= 0)
      return ctrl.col(0);

    const int seg = std::min(static_cast<int>(t / ts), seg_num - 1);
    const double local_t = std::max(0.0, t - seg * ts);
    const double u = std::min(1.0, local_t / ts);
    const int base = seg * order;

    const Eigen::Vector3d p01 = (1.0 - u) * ctrl.col(base) + u * ctrl.col(base + 1);
    const Eigen::Vector3d p12 = (1.0 - u) * ctrl.col(base + 1) + u * ctrl.col(base + 2);
    const Eigen::Vector3d p23 = (1.0 - u) * ctrl.col(base + 2) + u * ctrl.col(base + 3);
    const Eigen::Vector3d p012 = (1.0 - u) * p01 + u * p12;
    const Eigen::Vector3d p123 = (1.0 - u) * p12 + u * p23;
    return (1.0 - u) * p012 + u * p123;
  }

  void setRefPtsFromRawGuidance(int win_start, int win_cols)
  {
    if (!optimizer_ || !have_raw_guidance_ || last_raw_guidance_ctrl_.cols() <= 0 || win_cols <= 1)
      return;

    optimizer_->ref_pts_.clear();
    const int ref_count = win_cols - 1;  // 必须严格等于 win_cols - 1
    optimizer_->ref_pts_.reserve(ref_count);

    for (int i = 0; i < ref_count; ++i)
    {
      int src_idx = win_start + i;
      Eigen::Vector3d formation_pt;
      if (src_idx < last_raw_guidance_ctrl_.cols())
      {
        formation_pt = last_raw_guidance_ctrl_.col(src_idx);
      }
      else
      {
        formation_pt = last_raw_guidance_ctrl_.col(last_raw_guidance_ctrl_.cols() - 1);
      }
      // 起飞过渡插值：current_warmup_alpha_ 从 0 渐变到 1，将 ref_pts 从 odom 位置渐变到编队位置
      const Eigen::Vector3d ref_pt =
          (1.0 - current_warmup_alpha_) * warmup_start_pos_ + current_warmup_alpha_ * formation_pt;
      optimizer_->ref_pts_.push_back(ref_pt);
    }
  }

  void guidanceCallback(const ego_planner::BezierConstPtr &msg)
  {
    // -----------------------------------------------------------------------
    // 步骤 0：早返回检查——控制点过少时直接转发，不做任何修改
    // -----------------------------------------------------------------------
    if (static_cast<int>(msg->pos_pts.size()) < 4)
    {
      ego_planner::Bezier out = *msg;
      out.start_time = msg->start_time;
      pub_.publish(out);
      return;
    }

    // -----------------------------------------------------------------------
    // 步骤 1：提取新 guidance 的控制点和时间参数
    // ts：每段时长；order：贝塞尔阶数（默认3）；total_segs：总段数 N
    // -----------------------------------------------------------------------
    const double ts = (!msg->segment_durations.empty())
                          ? msg->segment_durations[0]
                          : fallback_segment_dt_;
    const int order = (msg->order > 0) ? msg->order : 3;
    Eigen::MatrixXd guide_ctrl = toCtrlPts(*msg);
    const int total_cols = guide_ctrl.cols();         // 3N+1 列
    const int total_segs = (total_cols - 1) / order;  // N 段

    // 段数不合法时直接转发，保持与原有代码一致的早返回逻辑
    if (total_segs <= 0 || total_cols < 4)
    {
      ego_planner::Bezier out = *msg;
      out.start_time = msg->start_time;
      pub_.publish(out);
      return;
    }

    // 保存主机下发的原始引导控制点（后续 fitness 回归基准）
    last_raw_guidance_ctrl_ = guide_ctrl;
    have_raw_guidance_ = true;

    if (startup_hold_enabled_ && !have_running_traj_ && startup_stage_ != StartupStage::TRACK_FORMATION)
    {
      const Eigen::Vector3d curr_p0 = guide_ctrl.col(0);
      const bool start_time_ready =
          (msg->start_time - ros::Time::now()).toSec() <= std::max(0.01, min_start_time_delay_ + 0.02);

      if (startup_stage_ == StartupStage::WAIT_GUIDANCE && !have_odom_)
      {
        guidance_stable_frames_ = 0;
        ROS_WARN_THROTTLE(1.0,
                          "[FollowerGuidance] WAIT_GUIDANCE: odom not ready, keep holding.");
        return;
      }

      if (!have_last_guidance_p0_)
      {
        last_guidance_p0_ = curr_p0;
        have_last_guidance_p0_ = true;
        guidance_stable_frames_ = start_time_ready ? 1 : 0;
      }
      else
      {
        const double p0_jump = (curr_p0 - last_guidance_p0_).norm();
        if (p0_jump <= guidance_lock_pos_tol_ && start_time_ready)
          ++guidance_stable_frames_;
        else
          guidance_stable_frames_ = start_time_ready ? 1 : 0;
        last_guidance_p0_ = curr_p0;
      }

      if (startup_stage_ == StartupStage::WAIT_GUIDANCE)
      {
        if (guidance_stable_frames_ < std::max(1, guidance_lock_frames_))
        {
          ROS_INFO_THROTTLE(0.5,
                            "[FollowerGuidance] WAIT_GUIDANCE: stable=%d/%d, holding before bridge.",
                            guidance_stable_frames_, guidance_lock_frames_);
          return;
        }

        startup_stage_ = StartupStage::HOVER_HOLD;
        stage_enter_stamp_ = ros::Time::now();
        hover_hold_count_ = 0;
        hover_ref_pos_ = have_odom_ ? odom_pos_ : curr_p0;
        ROS_INFO("[FollowerGuidance] Guidance locked, entering HOVER_HOLD (%d frames).",
                 hover_hold_frames_);
      }

      if (startup_stage_ == StartupStage::HOVER_HOLD)
      {
        publishHoverCommand(*msg);
        ++hover_hold_count_;
        const double hold_elapsed = (ros::Time::now() - stage_enter_stamp_).toSec();
        ROS_INFO_THROTTLE(0.5,
                          "[FollowerGuidance] HOVER_HOLD frame %d/%d elapsed=%.2fs",
                          hover_hold_count_, std::max(1, hover_hold_frames_), hold_elapsed);
        const bool hold_frames_ready = (hover_hold_count_ >= std::max(1, hover_hold_frames_));
        const bool hold_time_ready = (startup_hover_timeout_ > 1e-3 && hold_elapsed >= startup_hover_timeout_);
        if (hold_frames_ready || hold_time_ready)
        {
          startup_stage_ = StartupStage::BRIDGE_TO_FORMATION;
          stage_enter_stamp_ = ros::Time::now();
          active_warmup_frames_ = std::max(1, bridge_frames_);
          ROS_INFO("[FollowerGuidance] Entering BRIDGE_TO_FORMATION (%d frames).",
                   active_warmup_frames_);
        }
        return;
      }
    }

    // ── 退化轨迹检测 ──────────────────────────────────────────────────
    // 主机端优化失败时可能下发“悬停轨迹”（所有控制点重合），
    // 该退化贝塞尔曲线的导数矩阵为全零，在 getDerivative().evaluate() 时
    // 可能产生空矩阵访问 → SIGBUS 崩溃。
    // 检测方法：首末控制点距离 < 阈值 → 视为悬停命令，直接转发不优化。
    {
      const double span = (guide_ctrl.col(0) - guide_ctrl.col(total_cols - 1)).norm();
      if (span < 1e-4)
      {
        ROS_WARN_THROTTLE(1.0, "[FollowerGuidance] Degenerate hover trajectory detected "
                                "(span=%.6f m), forwarding without optimization.", span);
        ego_planner::Bezier out = *msg;
        out.start_time = msg->start_time;
        pub_.publish(out);
        // 不更新 running_traj_，避免污染后续接续逻辑
        return;
      }
    }

    // -----------------------------------------------------------------------
    // 步骤 2：确定接续起点（C0 位置 + C1 速度）
    // 若已有执行中的轨迹，从当前执行位置接续，保证位置/速度连续；
    // 否则（首次收到 guidance），从 guidance 首点推算初始速度。
    // -----------------------------------------------------------------------
    // evaluate() 返回 Eigen::VectorXd，统一使用动态类型避免固定/动态尺寸混用
    Eigen::VectorXd p_stitch, v_stitch;

    if (have_running_traj_)
    {
      // 计算当前轨迹已执行的时长，加前向裕量避免接续点落到已过去的位置
      double t_cur = (ros::Time::now() - running_traj_start_time_).toSec()
                     + stitch_time_margin_;
      double t_total = running_traj_.getTimeSum();
      t_cur = std::max(0.0, std::min(t_cur, t_total - 1e-3));

      // 从执行中的轨迹求当前位置和速度
      p_stitch = running_traj_.evaluate(t_cur);
      ego_planner::PiecewiseBezier vel_traj = running_traj_.getDerivative();
      // ── 安全保护：防止退化轨迹的导数矩阵为空 ──────────────
      if (vel_traj.getTimeSum() > 1e-6)
      {
        double t_vel = std::max(0.0, std::min(t_cur, vel_traj.getTimeSum() - 1e-3));
        v_stitch = vel_traj.evaluate(t_vel);
      }
      else
      {
        // 退化轨迹（悬停）：导数为零，使用零速度
        v_stitch = Eigen::VectorXd::Zero(3);
        ROS_WARN_THROTTLE(1.0, "[FollowerGuidance] Velocity trajectory degenerate, "
                                "using zero velocity for stitch.");
      }
    }
    else
    {
      // 首次：使用从机当前 odom 位置/速度作为接续起点，避免硬位置跳变
      if (have_odom_)
      {
        p_stitch = odom_pos_;
        v_stitch = odom_vel_;

        // 起飞桥接首帧对 odom 快照做安全限幅，抑制单机瞬态噪声触发突兀抖动。
        if (startup_stage_ == StartupStage::BRIDGE_TO_FORMATION)
        {
          const Eigen::Vector3d delta_from_hold = p_stitch.head<3>() - hover_ref_pos_;
          const double delta_norm = delta_from_hold.norm();
          if (startup_bridge_stitch_pos_margin_ > 1e-6 &&
              delta_norm > startup_bridge_stitch_pos_margin_)
          {
            p_stitch = hover_ref_pos_ +
                       delta_from_hold * (startup_bridge_stitch_pos_margin_ / delta_norm);
            ROS_WARN_THROTTLE(0.5,
                              "[FollowerGuidance] BRIDGE stitch pos limited: %.3f -> %.3f m",
                              delta_norm, startup_bridge_stitch_pos_margin_);
          }

          if (startup_bridge_max_stitch_speed_ > 1e-6)
          {
            const Eigen::Vector3d v_raw = v_stitch.head<3>();
            const double raw_speed = v_raw.norm();
            const Eigen::Vector3d v_limited = clampVecNorm(v_raw, startup_bridge_max_stitch_speed_);
            v_stitch = v_limited;
            if (raw_speed - v_limited.norm() > 1e-6)
            {
              ROS_WARN_THROTTLE(0.5,
                                "[FollowerGuidance] BRIDGE stitch speed limited: %.3f -> %.3f m/s",
                                raw_speed, v_limited.norm());
            }
          }
        }

        warmup_start_pos_ = (startup_stage_ == StartupStage::BRIDGE_TO_FORMATION)
                                ? hover_ref_pos_
                                : odom_pos_;
      }
      else
      {
        // 无 odom 时降级到 P0/P1 推算（保持旧行为）
        p_stitch = guide_ctrl.col(0);
        v_stitch = (static_cast<double>(order) / ts)
                   * (guide_ctrl.col(1) - guide_ctrl.col(0));
        warmup_start_pos_ = Eigen::Vector3d(guide_ctrl(0, 0), guide_ctrl(1, 0), guide_ctrl(2, 0));
        ROS_WARN("[FollowerGuidance] No odom on first guidance, falling back to P0.");
      }
      warmup_frame_count_ = 0;
      is_warmup_ = true;
      if (startup_stage_ == StartupStage::BRIDGE_TO_FORMATION)
      {
        ROS_INFO("[FollowerGuidance] BRIDGE started from hold point (%d frames).", active_warmup_frames_);
      }
      else
      {
        active_warmup_frames_ = std::max(1, warmup_total_frames_);
        ROS_INFO("[FollowerGuidance] First guidance received. Starting warmup (%d frames).",
                 active_warmup_frames_);
      }
    }

    // -----------------------------------------------------------------------
    // 步骤 2.5：计算本帧 warmup alpha（首次 guidance 后的起飞过渡期）
    // -----------------------------------------------------------------------
    if (is_warmup_)
    {
      const double warmup_alpha_linear = (active_warmup_frames_ > 0)
          ? std::min(1.0, static_cast<double>(warmup_frame_count_) / active_warmup_frames_)
          : 1.0;
      if (startup_bridge_use_s_curve_alpha_ && startup_stage_ == StartupStage::BRIDGE_TO_FORMATION)
      {
        // smoothstep: alpha=3x^2-2x^3，确保桥接起止速度更平滑
        const double x = std::max(0.0, std::min(1.0, warmup_alpha_linear));
        current_warmup_alpha_ = x * x * (3.0 - 2.0 * x);
      }
      else
      {
        current_warmup_alpha_ = warmup_alpha_linear;
      }
      ROS_INFO("[FollowerGuidance] Warmup frame %d/%d alpha=%.2f",
               warmup_frame_count_, active_warmup_frames_, current_warmup_alpha_);
      warmup_frame_count_++;
      if (warmup_frame_count_ > active_warmup_frames_)
      {
        is_warmup_ = false;
        current_warmup_alpha_ = 1.0;
        if (startup_stage_ == StartupStage::BRIDGE_TO_FORMATION)
        {
          startup_stage_ = StartupStage::TRACK_FORMATION;
          stage_enter_stamp_ = ros::Time::now();
          ROS_INFO("[FollowerGuidance] BRIDGE done, entering TRACK_FORMATION.");
        }
        active_warmup_frames_ = std::max(1, warmup_total_frames_);
        ROS_INFO("[FollowerGuidance] Warmup complete, entering formation mode.");
      }
    }
    else
    {
      current_warmup_alpha_ = 1.0;
    }

    // -----------------------------------------------------------------------
    // 步骤 3：将接续起点写入 guidance 控制点，确保 C0/C1 连续
    // C0：起点强制对齐当前执行位置
    // C1：由速度通过贝塞尔微分关系逆推：vel=(order/ts)*(P1-P0) → P1=P0+(ts/order)*vel
    // -----------------------------------------------------------------------
    guide_ctrl.col(0) = p_stitch;
    guide_ctrl.col(1) = p_stitch + (ts / static_cast<double>(order)) * v_stitch;

    // -----------------------------------------------------------------------
    // 步骤 3.5：引导轨迹预检（接收时立即扫描碰撞）
    // -----------------------------------------------------------------------
    bool guidance_in_collision = false;
    if (grid_map_ && use_local_map_)
    {
      const double total_time = total_segs * ts;
      const double scan_end = std::min(total_time, safety_look_ahead_);
      for (double t = 0.0; t < scan_end; t += safety_check_step_)
      {
        Eigen::Vector3d pos = evalBezierAt(guide_ctrl, ts, order, t);
        if (grid_map_->getInflateOccupancy(pos) == 1)
        {
          guidance_in_collision = true;
          ROS_WARN("[Follower] Guidance in collision at t=%.2f!", t);
          break;
        }
      }
    }

    // -----------------------------------------------------------------------
    // 步骤 4：提取未来固定窗口内的控制点，仅对该窗口做局部优化
    // 窗口大小不超过总段数，避免越界；优化失败时降级使用原始窗口控制点
    // -----------------------------------------------------------------------
    const int win_segs = std::min(refine_window_segs_, total_segs);
    const int win_cols = win_segs * order + 1;  // 窗口内控制点列数

    Eigen::MatrixXd window_ctrl = guide_ctrl.leftCols(win_cols);
    Eigen::MatrixXd opt_window = window_ctrl;
    bool ok = false;

    if ((enable_collision_rebound_ && grid_map_) || guidance_in_collision)
    {
      // 先做碰撞弹开优化（依赖地图），再做平滑精化
      optimizer_->initControlPoints(opt_window, true);
      ok = optimizer_->BezierOptimizeTrajRebound(opt_window, ts);
      if (ok)
      {
        Eigen::MatrixXd tmp = opt_window;
        setRefPtsFromRawGuidance(0, win_cols);
        ok = optimizer_->BezierOptimizeTrajRefine(tmp, ts, opt_window);
      }
    }
    if (!ok)
    {
      // 无碰撞弹开或弹开失败：仅做平滑精化
      setRefPtsFromRawGuidance(0, win_cols);
      ok = optimizer_->BezierOptimizeTrajRefine(window_ctrl, ts, opt_window);
    }
    if (!ok)
    {
      // 优化彻底失败，降级使用原始窗口控制点，并发出警告
      opt_window = window_ctrl;
      ROS_WARN_THROTTLE(1.0, "[FollowerGuidance] window refine failed, using raw guidance.");
    }

    // -----------------------------------------------------------------------
    // 步骤 5：拼接优化窗口 + guidance 剩余段，构建完整轨迹控制点
    // 接缝处强制 C0 对齐（将窗口末点写入剩余段首列），保证轨迹位置连续
    // -----------------------------------------------------------------------
    Eigen::MatrixXd full_ctrl(3, total_cols);
    full_ctrl.leftCols(win_cols) = opt_window;

    if (total_cols > win_cols)
    {
      // 保留 guidance 中未被优化的剩余段控制点（原样拷贝）
      full_ctrl.rightCols(total_cols - win_cols) =
          guide_ctrl.rightCols(total_cols - win_cols);
      // C0 接缝：强制剩余段的接缝控制点与优化窗口末点完全对齐
      full_ctrl.col(win_cols) = opt_window.col(win_cols - 1);
    }

    // -----------------------------------------------------------------------
    // 步骤 6：更新 running_traj_（记录当前执行中轨迹），并发布优化后轨迹
    // 使用 msg->start_time（主机绝对时间戳），保证主从全局时钟同步
    // -----------------------------------------------------------------------
    ego_planner::PiecewiseBezier new_traj;
    new_traj.setBezierCurve(full_ctrl, order, ts);
    running_traj_ = new_traj;
    running_traj_start_time_ = msg->start_time;  // 主机绝对时间戳，不使用 ros::Time::now()
    have_running_traj_ = true;

    // 打包并发布优化后的完整轨迹
    ego_planner::Bezier out = *msg;
    fillCtrlPts(out, full_ctrl);
    out.start_time = msg->start_time;  // 保持主机绝对时间戳
    pub_.publish(out);

    // 保存本次完整控制点和时间参数，供被动避障定时器使用
    last_full_ctrl_ = full_ctrl;
    last_ts_ = ts;
    last_order_ = order;
    last_guide_start_time_ = msg->start_time;
    have_last_full_ = true;
  }

  void odomCallback(const nav_msgs::OdometryConstPtr &msg)
  {
    odom_pos_(0) = msg->pose.pose.position.x;
    odom_pos_(1) = msg->pose.pose.position.y;
    odom_pos_(2) = msg->pose.pose.position.z;
    odom_vel_(0) = msg->twist.twist.linear.x;
    odom_vel_(1) = msg->twist.twist.linear.y;
    odom_vel_(2) = msg->twist.twist.linear.z;
    have_odom_ = true;
  }

  void safetyCheckCallback(const ros::TimerEvent & /*e*/)
  {
    // ------------------------------------------------------------------
    // 前置条件检查：必须有地图、有执行轨迹、有里程计
    // ------------------------------------------------------------------
    if (!use_local_map_ || !grid_map_ || !have_running_traj_ || !have_last_full_)
      return;

    // ------------------------------------------------------------------
    // 冷却期保护：距上次主动重规划不足 safety_cooldown_ 秒时跳过
    // ------------------------------------------------------------------
    const ros::Time now = ros::Time::now();
    if ((now - last_safety_replan_time_).toSec() < safety_cooldown_)
      return;

    // ------------------------------------------------------------------
    // 计算当前在 running_traj_ 上的执行时刻 t_cur
    // ------------------------------------------------------------------
    double t_cur = (now - running_traj_start_time_).toSec();
    double t_total = running_traj_.getTimeSum();
    if (t_cur < 0.0 || t_cur >= t_total - 1e-3)
      return;  // 轨迹尚未开始或已结束

    // ------------------------------------------------------------------
    // 前向扫描：从 t_cur 到 min(t_cur + look_ahead, t_total)
    // 步长 safety_check_step_，检测每个采样点是否在障碍物中
    // ------------------------------------------------------------------
    double t_collision = -1.0;  // 记录第一个碰撞时刻，-1 表示无碰撞
    double t_scan_end = std::min(t_cur + safety_look_ahead_, t_total);

    for (double t = t_cur; t < t_scan_end; t += safety_check_step_)
    {
      Eigen::VectorXd pos = running_traj_.evaluate(t);
      if (grid_map_->getInflateOccupancy(pos.head<3>()) == 1)
      {
        t_collision = t;
        break;
      }
    }

    if (t_collision < 0.0)
      return;  // 无碰撞，本次不触发

    ROS_WARN("[FollowerGuidance] Safety check: collision detected at t=%.3f s "
             "(t_cur=%.3f). Triggering local rebound.",
             t_collision, t_cur);

    // ------------------------------------------------------------------
    // 从 last_full_ctrl_ 中提取接续窗口控制点
    // ------------------------------------------------------------------
    const double ts = last_ts_;
    const int order = last_order_;
    const int total_cols = static_cast<int>(last_full_ctrl_.cols());
    const int total_segs = (total_cols - 1) / order;

    // 接续时刻（保持前向裕量）
    double t_stitch = t_cur + stitch_time_margin_;
    t_stitch = std::max(0.0, std::min(t_stitch, t_total - 1e-3));

    // 求接续位置和速度
    Eigen::VectorXd p_stitch = running_traj_.evaluate(t_stitch);
    ego_planner::PiecewiseBezier vel_traj_safety = running_traj_.getDerivative();
    Eigen::VectorXd v_stitch;
    if (vel_traj_safety.getTimeSum() > 1e-6)
    {
      double t_vel = std::max(0.0, std::min(t_stitch, vel_traj_safety.getTimeSum() - 1e-3));
      v_stitch = vel_traj_safety.evaluate(t_vel);
    }
    else
    {
      v_stitch = Eigen::VectorXd::Zero(3);
    }

    // 将 t_stitch 映射到 last_full_ctrl_ 的段号
    // last_full_ctrl_ 的时间轴与 running_traj_ 对齐（同一 start_time、同一 ts）
    int stitch_seg = static_cast<int>(std::floor(t_stitch / ts));
    stitch_seg = std::max(0, std::min(stitch_seg, total_segs - 1));

    // 窗口大小：不超过从 stitch_seg 起剩余的段数
    const int win_segs = std::min(refine_window_segs_, total_segs - stitch_seg);
    if (win_segs <= 0)
      return;

    const int win_start_col = stitch_seg * order;  // 窗口在 last_full_ctrl_ 中的起始列
    const int win_cols = win_segs * order + 1;     // 窗口列数

    // 边界保护：不越界
    if (win_start_col + win_cols > total_cols)
      return;

    // 提取窗口控制点
    Eigen::MatrixXd window_ctrl =
        last_full_ctrl_.block(0, win_start_col, 3, win_cols);

    // C0/C1：用接续点覆盖窗口头两列
    window_ctrl.col(0) = p_stitch.head<3>();
    window_ctrl.col(1) = p_stitch.head<3>()
                         + (ts / static_cast<double>(order)) * v_stitch.head<3>();

    // ------------------------------------------------------------------
    // Rebound 优化：将障碍物中的控制点弹开
    // ------------------------------------------------------------------
    Eigen::MatrixXd opt_window = window_ctrl;
    bool ok = false;

    optimizer_->initControlPoints(opt_window, true);
    ok = optimizer_->BezierOptimizeTrajRebound(opt_window, ts);

    if (ok)
    {
      // Refine 优化：以 Rebound 结果为初值，将轨迹拉回引导位置
      Eigen::MatrixXd refined_window;
      setRefPtsFromRawGuidance(win_start_col, win_cols);
      ok = optimizer_->BezierOptimizeTrajRefine(opt_window, ts, refined_window);
      if (ok)
        opt_window = refined_window;
    }

    if (!ok)
    {
      // Rebound 或 Refine 失败：降级使用 Rebound 结果或原始窗口
      ROS_WARN_THROTTLE(1.0, "[FollowerGuidance] Safety rebound/refine failed, "
                             "using best available result.");
    }

    // ------------------------------------------------------------------
    // 将优化窗口写回 last_full_ctrl_ 对应列（原地更新）
    // ------------------------------------------------------------------
    last_full_ctrl_.block(0, win_start_col, 3, win_cols) = opt_window;
    if (win_start_col + win_cols < total_cols)
    {
      // C0 接缝：强制剩余段首列与窗口末列对齐
      last_full_ctrl_.col(win_start_col + win_cols) = opt_window.col(win_cols - 1);
    }

    // ------------------------------------------------------------------
    // 更新 running_traj_ 并发布修正后的完整轨迹
    // start_time 继承自最近一次 guidance，保证主从时钟同步
    // ------------------------------------------------------------------
    ego_planner::PiecewiseBezier new_traj;
    new_traj.setBezierCurve(last_full_ctrl_, order, ts);
    running_traj_ = new_traj;
    running_traj_start_time_ = last_guide_start_time_;

    // 打包发布
    ego_planner::Bezier out;
    out.order = order;
    out.start_time = last_guide_start_time_;
    out.traj_id = 0;  // safety replan 不从 guidance 继承 id，设为 0 区分
    if (last_ts_ > 0.0)
      out.segment_durations.push_back(last_ts_);
    fillCtrlPts(out, last_full_ctrl_);
    pub_.publish(out);

    last_safety_replan_time_ = now;

    ROS_INFO("[FollowerGuidance] Safety replan done. Collision was at t=%.3f, "
             "stitch_seg=%d, win_segs=%d.",
             t_collision, stitch_seg, win_segs);
  }

private:
  ros::NodeHandle nh_;
  ros::Subscriber sub_;
  ros::Publisher pub_;

  bool enable_local_refine_{true};
  bool enable_collision_rebound_{false};
  bool use_local_map_{false};
  double min_start_time_delay_{0.03};
  double fallback_segment_dt_{0.1};

  GridMap::Ptr grid_map_;
  ego_planner::BezierOptimizer::Ptr optimizer_;

  // -------------------------------------------------------------------------
  // 滚动轨迹接续状态
  // -------------------------------------------------------------------------
  ego_planner::PiecewiseBezier running_traj_;  // 当前正在执行的完整贝塞尔轨迹
  ros::Time running_traj_start_time_;          // 该轨迹的绝对起始时刻（来自 msg->start_time）
  bool have_running_traj_{false};              // 是否已有执行中的轨迹

  // 滚动优化参数（从 ROS param 读取）
  int refine_window_segs_{3};        // 每次只对未来这么多段做局部优化（默认 3 段）
  double stitch_time_margin_{0.05};  // t_cur 前向裕量（秒），避免接续点落到已过去的位置

  // =========================================================================
  // 里程计（始终订阅，用于 warmup 和被动避障）
  // =========================================================================
  ros::Subscriber odom_sub_;
  Eigen::Vector3d odom_pos_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d odom_vel_{Eigen::Vector3d::Zero()};  // 线速度（来自 odom twist）
  bool have_odom_{false};

  // =========================================================================
  // 起飞过渡（Warmup）：首次 guidance 后平滑汇入编队
  // =========================================================================
  int warmup_total_frames_{15};   // 过渡总帧数（ROS param: warmup_frames）
  int active_warmup_frames_{15};
  int warmup_frame_count_{0};     // 当前已处理帧数
  bool is_warmup_{false};         // 是否处于过渡期
  Eigen::Vector3d warmup_start_pos_{Eigen::Vector3d::Zero()};  // 过渡起始位置（首次 guidance 时的 odom）
  double current_warmup_alpha_{1.0};  // 本帧插值系数（0=odom 位置, 1=编队位置）

  StartupStage startup_stage_{StartupStage::WAIT_GUIDANCE};
  int guidance_lock_frames_{5};
  int hover_hold_frames_{8};
  int bridge_frames_{15};
  double guidance_lock_pos_tol_{0.25};
  int guidance_stable_frames_{0};
  int hover_hold_count_{0};
  bool have_last_guidance_p0_{false};
  Eigen::Vector3d last_guidance_p0_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d hover_ref_pos_{Eigen::Vector3d::Zero()};
  bool startup_hold_enabled_{true};
  double startup_bridge_duration_{0.75};
  double startup_hover_timeout_{0.40};
  double startup_nominal_guidance_rate_{20.0};
  double startup_bridge_stitch_pos_margin_{0.15};
  double startup_bridge_max_stitch_speed_{0.25};
  bool startup_bridge_use_s_curve_alpha_{true};
  ros::Time stage_enter_stamp_;

  // =========================================================================
  // 被动避障：安全检测定时器
  // =========================================================================
  ros::Timer safety_timer_;
  double safety_look_ahead_{2.0};   // 向前检测时间窗口（秒）
  double safety_check_step_{0.05};  // 碰撞采样步长（秒），与定时器周期一致
  double safety_cooldown_{0.25};    // 两次主动重规划最小间隔（秒）
  ros::Time last_safety_replan_time_;

  // =========================================================================
  // 被动避障：保存最近一次发布的完整轨迹控制点信息
  // =========================================================================
  Eigen::MatrixXd last_full_ctrl_;  // 最近一次发布的完整控制点（3 x total_cols）
  double last_ts_{0.1};
  int last_order_{3};
  ros::Time last_guide_start_time_;
  bool have_last_full_{false};

  // 主机原始引导轨迹（永不被避障修改），用于 fitness 回归约束
  Eigen::MatrixXd last_raw_guidance_ctrl_;
  bool have_raw_guidance_{false};
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "swarm_follower_guidance_optimizer_node");
  ros::NodeHandle nh("~");

  SwarmFollowerGuidanceOptimizer node;
  node.init(nh);

  ros::spin();
  return 0;
}
