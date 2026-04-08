/**
 * @file ego_replan_fsm.cpp
 * @brief Implementation of EGO Replanning FSM with Piecewise Bezier Curves
 *        Enhanced with mandatory waypoint reaching functionality
 * 
 * 核心改进: 航点强制到达机制
 * - 航点成为无人机真实轨迹必须到达的点
 * - 基于无人机实际位置检测航点到达
 * - 只有实际到达航点后才切换到下一个航点
 */

#include <plan_manage/ego_replan_fsm.h>

namespace ego_planner
{

  void EGOReplanFSM::init(ros::NodeHandle &nh)
  {
    current_wp_ = 0;
    exec_state_ = FSM_EXEC_STATE::INIT;
    have_target_ = false;
    have_odom_ = false;

    /*  FSM parameters  */
    nh.param("fsm/flight_type", target_type_, -1);
    nh.param("fsm/thresh_replan", replan_thresh_, -1.0);
    nh.param("fsm/thresh_no_replan", no_replan_thresh_, -1.0);
    nh.param("fsm/planning_horizon", planning_horizen_, -1.0);
    nh.param("fsm/planning_horizen_time", planning_horizen_time_, -1.0);
    nh.param("fsm/emergency_time_", emergency_time_, 1.0);
    nh.param("fsm/goal_reach_thresh", goal_reach_thresh_, 1.0);
    
    /* 航点强制到达参数 */
    nh.param("fsm/use_mandatory_waypoints", use_mandatory_waypoints_, true);
    nh.param("fsm/waypoint_reach_thresh", waypoint_reach_thresh_, 0.8);
    nh.param("fsm/max_waypoint_replan_attempts", max_waypoint_replan_attempts_, 20);
    nh.param("fsm/max_waypoint_approach_time", max_waypoint_approach_time_, 60.0);

    nh.param("fsm/waypoint_num", waypoint_num_, -1);
    for (int i = 0; i < waypoint_num_; i++)
    {
      nh.param("fsm/waypoint" + to_string(i) + "_x", waypoints_[i][0], -1.0);
      nh.param("fsm/waypoint" + to_string(i) + "_y", waypoints_[i][1], -1.0);
      nh.param("fsm/waypoint" + to_string(i) + "_z", waypoints_[i][2], -1.0);
    }
    
    /* 初始化航点追踪系统 */
    current_waypoint_idx_ = 0;
    waypoint_reached_ = false;
    all_waypoints_completed_ = false;
    waypoint_replan_count_ = 0;
    waypoint_approach_time_ = 0.0;
    last_waypoint_check_time_ = ros::Time::now();

    /* Initialize main modules */
    visualization_.reset(new PlanningVisualization(nh));
    planner_manager_.reset(new EGOPlannerManager);
    planner_manager_->initPlanModules(nh, visualization_);
    local_frame_.reset(new LocalCoordinateSystem);
    local_frame_->init(nh);

    /* Callbacks */
    exec_timer_ = nh.createTimer(ros::Duration(0.01), &EGOReplanFSM::execFSMCallback, this);
    safety_timer_ = nh.createTimer(ros::Duration(0.05), &EGOReplanFSM::checkCollisionCallback, this);
    
    /* 航点检测定时器 - 每0.1秒检测一次是否到达航点 */
    waypoint_check_timer_ = nh.createTimer(ros::Duration(0.1), &EGOReplanFSM::waypointCheckCallback, this);

    odom_sub_ = nh.subscribe("/odom_world", 1, &EGOReplanFSM::odometryCallback, this);

    bezier_pub_ = nh.advertise<ego_planner::Bezier>("/planning/bezier", 10);
    data_disp_pub_ = nh.advertise<ego_planner::DataDisp>("/planning/data_display", 100);
    
    
    /* 航点状态发布器 */
    waypoint_status_pub_ = nh.advertise<std_msgs::Int32>("/planning/waypoint_status", 10);
    current_waypoint_pub_ = nh.advertise<std_msgs::Int32>("/planning/current_waypoint_idx", 10);
    waypoint_distance_pub_ = nh.advertise<std_msgs::Float32>("/planning/waypoint_distance", 10);

    if (target_type_ == TARGET_TYPE::MANUAL_TARGET)
      waypoint_sub_ = nh.subscribe("/waypoint_generator/waypoints", 1, &EGOReplanFSM::waypointCallback, this);
    else if (target_type_ == TARGET_TYPE::PRESET_TARGET)
    {
      ros::Duration(1.0).sleep();
      while (ros::ok() && !have_odom_)
        ros::spinOnce();
      initWaypointTracking();
      planGlobalTrajbyGivenWps();
    }
    else
      cout << "Wrong target_type_ value! target_type_=" << target_type_ << endl;
  }
  
  void EGOReplanFSM::initWaypointTracking()
  {
    /* 初始化航点列表 */
    all_waypoints_.clear();
    for (int i = 0; i < waypoint_num_; i++)
    {
      Eigen::Vector3d wp(waypoints_[i][0], waypoints_[i][1], waypoints_[i][2]);
      all_waypoints_.push_back(wp);
    }
    
    current_waypoint_idx_ = 0;
    waypoint_reached_ = false;
    all_waypoints_completed_ = false;
    waypoint_replan_count_ = 0;
    waypoint_approach_time_ = 0.0;
    
    if (!all_waypoints_.empty())
    {
      current_target_waypoint_ = all_waypoints_[0];
      ROS_INFO("[WaypointTracker] Initialized with %zu waypoints. First target: (%.2f, %.2f, %.2f)",
               all_waypoints_.size(), current_target_waypoint_.x(), current_target_waypoint_.y(), current_target_waypoint_.z());
    }
  }

  void EGOReplanFSM::planGlobalTrajbyGivenWps()
  {
    std::vector<Eigen::Vector3d> wps(waypoint_num_);
    for (int i = 0; i < waypoint_num_; i++)
    {
      wps[i](0) = waypoints_[i][0];
      wps[i](1) = waypoints_[i][1];
      wps[i](2) = waypoints_[i][2];
      end_pt_ = wps.back();
    }
    
    /* 航点强制到达模式：设置第一个航点为当前目标 */
    if (use_mandatory_waypoints_ && !wps.empty())
    {
      current_target_waypoint_ = wps[0];
      ROS_INFO("[WaypointTracker] Starting mandatory waypoint mode. Target waypoint 0: (%.2f, %.2f, %.2f)",
               current_target_waypoint_.x(), current_target_waypoint_.y(), current_target_waypoint_.z());
    }
    
    // 关键修改：使用当前速度作为起始边界，只有最终目标使用零速度
    Eigen::Vector3d start_vel = odom_vel_;
    if (start_vel.norm() < 0.1 && !wps.empty())
    {
      Eigen::Vector3d dir = (wps[0] - odom_pos_).normalized();
      start_vel = dir * planner_manager_->pp_.max_vel_ * 0.3;
    }
    
    bool success = planner_manager_->planGlobalTrajWaypoints(odom_pos_, start_vel, 
                                                              Eigen::Vector3d::Zero(), wps, 
                                                              Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());

    for (size_t i = 0; i < (size_t)waypoint_num_; i++)
    {
      // 当前目标航点用红色显示，其他用青色
      Eigen::Vector4d color = (use_mandatory_waypoints_ && (int)i == current_waypoint_idx_) 
                              ? Eigen::Vector4d(1, 0, 0, 1) 
                              : Eigen::Vector4d(0, 0.5, 0.5, 1);
      visualization_->displayGoalPoint(wps[i], color, 0.3, i);
      ros::Duration(0.001).sleep();
    }

    if (success)
    {
      constexpr double step_size_t = 0.1;
      int i_end = floor(planner_manager_->global_data_.global_duration_ / step_size_t);
      std::vector<Eigen::Vector3d> gloabl_traj(i_end);
      for (int i = 0; i < i_end; i++)
      {
        gloabl_traj[i] = planner_manager_->global_data_.global_traj_.evaluate(i * step_size_t);
      }

      end_vel_.setZero();
      have_target_ = true;
      have_new_target_ = true;

      changeFSMExecState(GEN_NEW_TRAJ, "TRIG");

      ros::Duration(0.001).sleep();
      visualization_->displayGlobalPathList(gloabl_traj, 0.1, 0);
      ros::Duration(0.001).sleep();
    }
    else
    {
      ROS_ERROR("Unable to generate global trajectory!");
    }
  }

  void EGOReplanFSM::waypointCallback(const nav_msgs::PathConstPtr &msg)
  {
    if (msg->poses[0].pose.position.z < -0.1)
      return;

    cout << "Triggered!" << endl;
    trigger_ = true;
    init_pt_ = odom_pos_;

    bool success = false;
    end_pt_ << msg->poses[0].pose.position.x, msg->poses[0].pose.position.y, 1.0;
    success = planner_manager_->planGlobalTraj(odom_pos_, odom_vel_, Eigen::Vector3d::Zero(), 
                                               end_pt_, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());

    visualization_->displayGoalPoint(end_pt_, Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, 0);

    if (success)
    {
      constexpr double step_size_t = 0.1;
      int i_end = floor(planner_manager_->global_data_.global_duration_ / step_size_t);
      vector<Eigen::Vector3d> gloabl_traj(i_end);
      for (int i = 0; i < i_end; i++)
      {
        gloabl_traj[i] = planner_manager_->global_data_.global_traj_.evaluate(i * step_size_t);
      }

      end_vel_.setZero();
      have_target_ = true;
      have_new_target_ = true;

      if (exec_state_ == WAIT_TARGET)
        changeFSMExecState(GEN_NEW_TRAJ, "TRIG");
      else if (exec_state_ == EXEC_TRAJ)
        changeFSMExecState(REPLAN_TRAJ, "TRIG");

      visualization_->displayGlobalPathList(gloabl_traj, 0.1, 0);
    }
    else
    {
      ROS_ERROR("Unable to generate global trajectory!");
    }
  }

  void EGOReplanFSM::odometryCallback(const nav_msgs::OdometryConstPtr &msg)
  {
    odom_pos_(0) = msg->pose.pose.position.x;
    odom_pos_(1) = msg->pose.pose.position.y;
    odom_pos_(2) = msg->pose.pose.position.z;

    odom_vel_(0) = msg->twist.twist.linear.x;
    odom_vel_(1) = msg->twist.twist.linear.y;
    odom_vel_(2) = msg->twist.twist.linear.z;

    odom_orient_.w() = msg->pose.pose.orientation.w;
    odom_orient_.x() = msg->pose.pose.orientation.x;
    odom_orient_.y() = msg->pose.pose.orientation.y;
    odom_orient_.z() = msg->pose.pose.orientation.z;

    have_odom_ = true;
    
    // ========== 更新局部坐标系原点 ==========
    if (local_frame_->isEnabled() && have_odom_)
    {
      if (local_frame_->updateOrigin(odom_pos_))
      {
        ROS_INFO("[FSM] Local frame origin updated to: [%.2f, %.2f, %.2f]",
                 odom_pos_.x(), odom_pos_.y(), odom_pos_.z());
      }
    }
  }

  void EGOReplanFSM::changeFSMExecState(FSM_EXEC_STATE new_state, string pos_call)
  {
    if (new_state == exec_state_)
      continously_called_times_++;
    else
      continously_called_times_ = 1;

    static string state_str[7] = {"INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ", "EMERGENCY_STOP"};
    int pre_s = int(exec_state_);
    exec_state_ = new_state;
//     cout << "[" + pos_call + "]: from " + state_str[pre_s] + " to " + state_str[int(new_state)] << endl;
  }

  std::pair<int, EGOReplanFSM::FSM_EXEC_STATE> EGOReplanFSM::timesOfConsecutiveStateCalls()
  {
    return std::pair<int, FSM_EXEC_STATE>(continously_called_times_, exec_state_);
  }

  void EGOReplanFSM::printFSMExecState()
  {
    static string state_str[7] = {"INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ", "EMERGENCY_STOP"};
    cout << "[FSM]: state: " + state_str[int(exec_state_)] << endl;
  }

  void EGOReplanFSM::execFSMCallback(const ros::TimerEvent &e)
  {
    static int fsm_num = 0;
    fsm_num++;
    if (fsm_num == 100)
    {
      printFSMExecState();
      if (!have_odom_)
        cout << "no odom." << endl;
      if (!trigger_)
        cout << "wait for goal." << endl;
      fsm_num = 0;
    }

    switch (exec_state_)
    {
    case INIT:
    {
      if (!have_odom_) return;
      if (!trigger_) return;
      changeFSMExecState(WAIT_TARGET, "FSM");
      break;
    }

    case WAIT_TARGET:
    {
      if (!have_target_) return;
      else changeFSMExecState(GEN_NEW_TRAJ, "FSM");
      break;
    }

    case GEN_NEW_TRAJ:
    {
      start_pt_ = odom_pos_;
      start_vel_ = odom_vel_;
      start_acc_.setZero();

      bool flag_random_poly_init;
      if (timesOfConsecutiveStateCalls().first == 1)
        flag_random_poly_init = false;
      else
        flag_random_poly_init = true;

      bool success = callReboundReplan(true, flag_random_poly_init);
      if (success)
      {
        changeFSMExecState(EXEC_TRAJ, "FSM");
        flag_escape_emergency_ = true;
      }
      else
      {
        changeFSMExecState(GEN_NEW_TRAJ, "FSM");
      }
      break;
    }

    case REPLAN_TRAJ:
    {
      if (planFromCurrentTraj())
      {
        changeFSMExecState(EXEC_TRAJ, "FSM");
      }
      else
      {
        changeFSMExecState(REPLAN_TRAJ, "FSM");
      }
      break;
    }

    case EXEC_TRAJ:
    {
      LocalTrajData *info = &planner_manager_->local_data_;
      ros::Time time_now = ros::Time::now();
      double t_cur = (time_now - info->start_time_).toSec();
      t_cur = min(info->duration_, t_cur);

      Eigen::Vector3d pos = info->position_traj_.evaluateT(t_cur);
      
      /* ========== 航点强制到达模式 ========== */
      if (use_mandatory_waypoints_ && !all_waypoints_completed_)
      {
        // 检查是否到达当前目标航点（基于实际位置）
        double dist_to_waypoint = (odom_pos_ - current_target_waypoint_).norm();
        
        // 轨迹时间结束但未到达航点：重规划继续接近
        if (t_cur > info->duration_ - 1e-2)
        {
          if (dist_to_waypoint > waypoint_reach_thresh_)
          {
            waypoint_replan_count_++;
            ROS_INFO("[WaypointTracker] Trajectory ended but waypoint %d not reached. Distance: %.2f, Replanning... (attempt %d)",
                     current_waypoint_idx_, dist_to_waypoint, waypoint_replan_count_);
            
            // 检查是否超过最大重规划次数
            if (waypoint_replan_count_ > max_waypoint_replan_attempts_)
            {
              ROS_WARN("[WaypointTracker] Max replan attempts reached for waypoint %d. Trying direct approach.",
                       current_waypoint_idx_);
              if (planDirectToWaypoint())
              {
                have_new_target_ = true;
                changeFSMExecState(GEN_NEW_TRAJ, "DIRECT_APPROACH");
              }
              else
              {
                changeFSMExecState(REPLAN_TRAJ, "FSM");
              }
              return;
            }
            
            // 重新规划到当前航点
            changeFSMExecState(REPLAN_TRAJ, "FSM");
            return;
          }
          else
          {
            // 到达当前航点，waypointCheckCallback会处理切换
            return;
          }
        }
        
        // 检查距离当前航点的进度
        if (dist_to_waypoint < no_replan_thresh_)
        {
          // 接近航点，继续执行
          return;
        }
        else if ((info->start_pos_ - pos).norm() < replan_thresh_)
        {
          return;
        }
        else
        {
          changeFSMExecState(REPLAN_TRAJ, "FSM");
        }
      }
      else
      {
        /* ========== 原始模式（非强制航点） ========== */
        // 轨迹时间结束判断 - 使用实际位置进行辅助验证
        if (t_cur > info->duration_ - 1e-2)
        {
          // 检查实际位置到目标的距离
          double actual_dist = (end_pt_ - odom_pos_).norm();
          // 如果距离目标还比较远（超过到达阈值），需要重规划继续接近
          if (actual_dist > goal_reach_thresh_)
          {
            changeFSMExecState(REPLAN_TRAJ, "FSM");
            return;
          }
          have_target_ = false;
          changeFSMExecState(WAIT_TARGET, "FSM");
          return;
        }
        else if ((end_pt_ - pos).norm() < no_replan_thresh_)
        {
          return;
        }
        else if ((info->start_pos_ - pos).norm() < replan_thresh_)
        {
          return;
        }
        else
        {
          changeFSMExecState(REPLAN_TRAJ, "FSM");
        }
      }
      break;
    }

    case EMERGENCY_STOP:
    {
      if (flag_escape_emergency_)
      {
        callEmergencyStop(odom_pos_);
      }
      else
      {
        if (odom_vel_.norm() < 0.1)
          changeFSMExecState(GEN_NEW_TRAJ, "FSM");
      }
      flag_escape_emergency_ = false;
      break;
    }
    
    case REACHING_WAYPOINT:
    {
      // 预留状态：正在逼近航点（当前由waypointCheckCallback处理）
      // 可以在此添加更精细的航点逼近控制
      break;
    }
    }

    data_disp_.header.stamp = ros::Time::now();
    data_disp_pub_.publish(data_disp_);
  }

  bool EGOReplanFSM::planFromCurrentTraj()
  {
    LocalTrajData *info = &planner_manager_->local_data_;
    ros::Time time_now = ros::Time::now();
    double t_cur = (time_now - info->start_time_).toSec();

    start_pt_ = info->position_traj_.evaluateT(t_cur);
    start_vel_ = info->velocity_traj_.evaluateT(t_cur);
    start_acc_ = info->acceleration_traj_.evaluateT(t_cur);

    bool success = callReboundReplan(false, false);

    if (!success)
    {
      success = callReboundReplan(true, false);
      if (!success)
      {
        success = callReboundReplan(true, true);
        if (!success) return false;
      }
    }

    return true;
  }

  void EGOReplanFSM::checkCollisionCallback(const ros::TimerEvent &e)
  {
    LocalTrajData *info = &planner_manager_->local_data_;
    auto map = planner_manager_->grid_map_;

    if (exec_state_ == WAIT_TARGET || info->start_time_.toSec() < 1e-5)
      return;

    constexpr double time_step = 0.01;
    double t_cur = (ros::Time::now() - info->start_time_).toSec();
    double t_2_3 = info->duration_ * 2 / 3;
    for (double t = t_cur; t < info->duration_; t += time_step)
    {
      if (t_cur < t_2_3 && t >= t_2_3)
        break;

      if (map->getInflateOccupancy(info->position_traj_.evaluateT(t)))
      {
        if (planFromCurrentTraj())
        {
          changeFSMExecState(EXEC_TRAJ, "SAFETY");
          return;
        }
        else
        {
          if (t - t_cur < emergency_time_)
          {
            ROS_WARN("Suddenly discovered obstacles. emergency stop! time=%f", t - t_cur);
            changeFSMExecState(EMERGENCY_STOP, "SAFETY");
          }
          else
          {
            changeFSMExecState(REPLAN_TRAJ, "SAFETY");
          }
          return;
        }
        break;
      }
    }
  }

  bool EGOReplanFSM::callReboundReplan(bool flag_use_poly_init, bool flag_randomPolyTraj)
  {
    getLocalTarget();

    bool plan_success = planner_manager_->reboundReplan(start_pt_, start_vel_, start_acc_, 
                                                        local_target_pt_, local_target_vel_, 
                                                        (have_new_target_ || flag_use_poly_init), 
                                                        flag_randomPolyTraj);
    have_new_target_ = false;

//     cout << "final_plan_success=" << plan_success << endl;

    if (plan_success)
    {
      auto info = &planner_manager_->local_data_;

      /* Publish Bezier trajectory */
      ego_planner::Bezier bezier;
      bezier.order = 3;
      bezier.start_time = info->start_time_;
      bezier.traj_id = info->traj_id_;

      Eigen::MatrixXd pos_pts = info->position_traj_.getControlPoint();
      bezier.pos_pts.reserve(pos_pts.cols());
      for (int i = 0; i < pos_pts.cols(); ++i)
      {
        geometry_msgs::Point pt;
        pt.x = pos_pts(0, i);
        pt.y = pos_pts(1, i);
        pt.z = pos_pts(2, i);
        bezier.pos_pts.push_back(pt);
      }

      bezier.segment_durations.push_back(info->position_traj_.getInterval());

      bezier_pub_.publish(bezier);

      visualization_->displayOptimalList(info->position_traj_.getControlPoint(), 0);
    }

    return plan_success;
  }

  bool EGOReplanFSM::callEmergencyStop(Eigen::Vector3d stop_pos)
  {
    planner_manager_->EmergencyStop(stop_pos);

    auto info = &planner_manager_->local_data_;

    /* Publish Bezier trajectory */
    ego_planner::Bezier bezier;
    bezier.order = 3;
    bezier.start_time = info->start_time_;
    bezier.traj_id = info->traj_id_;

    Eigen::MatrixXd pos_pts = info->position_traj_.getControlPoint();
    bezier.pos_pts.reserve(pos_pts.cols());
    for (int i = 0; i < pos_pts.cols(); ++i)
    {
      geometry_msgs::Point pt;
      pt.x = pos_pts(0, i);
      pt.y = pos_pts(1, i);
      pt.z = pos_pts(2, i);
      bezier.pos_pts.push_back(pt);
    }

    bezier.segment_durations.push_back(info->position_traj_.getInterval());

    bezier_pub_.publish(bezier);

    return true;
  }

  void EGOReplanFSM::getLocalTarget()
  {
    double t;
    double t_step = planning_horizen_ / 20 / planner_manager_->pp_.max_vel_;
    double dist_min = 9999, dist_min_t = 0.0;
    
    // 关键修复：使用无人机实际位置(odom_pos_)而不是规划起点(start_pt_)来计算进度
    // 这样可以避免因轨迹跟踪误差导致的进度错误前移
    Eigen::Vector3d reference_pos = odom_pos_;
    
    /* ========== 航点强制到达模式：限制局部目标不超过当前航点 ========== */
    if (use_mandatory_waypoints_ && !all_waypoints_completed_ && 
        current_waypoint_idx_ < (int)all_waypoints_.size())
    {
      // 计算到当前航点的距离
      double dist_to_current_waypoint = (current_target_waypoint_ - odom_pos_).norm();
      
      // 如果当前航点在规划视野内，直接将其设为局部目标
      if (dist_to_current_waypoint <= planning_horizen_)
      {
        local_target_pt_ = current_target_waypoint_;
        
        // 关键修改：不再强制零速度，而是计算合理的穿越速度
        // 只有当这是最终目标点时才使用零速度
        bool is_final_waypoint = (current_waypoint_idx_ == (int)all_waypoints_.size() - 1);
        
        if (is_final_waypoint && dist_to_current_waypoint < waypoint_reach_thresh_ * 2.0)
        {
          // 最终航点且非常接近时，使用零速度以确保精确停止
          local_target_vel_ = Eigen::Vector3d::Zero();
        }
        else
        {
          // 中间航点或距离较远时，使用穿越速度
          // 计算指向下一航点的方向（如果存在）
          Eigen::Vector3d next_dir;
          if (current_waypoint_idx_ + 1 < (int)all_waypoints_.size())
          {
            // 使用Catmull-Rom风格的切线：当前方向与下一段方向的平均
            Eigen::Vector3d to_current = (current_target_waypoint_ - odom_pos_).normalized();
            Eigen::Vector3d to_next = (all_waypoints_[current_waypoint_idx_ + 1] - current_target_waypoint_).normalized();
            next_dir = (to_current + to_next).normalized();
          }
          else
          {
            next_dir = (current_target_waypoint_ - odom_pos_).normalized();
          }
          
          // 根据距离调整速度大小，但保持非零
          double speed_factor = std::min(1.0, dist_to_current_waypoint / planning_horizen_);
          speed_factor = std::max(0.3, speed_factor);  // 最低保持30%的速度
          local_target_vel_ = next_dir * planner_manager_->pp_.max_vel_ * speed_factor;
        }
        
        ROS_DEBUG("[WaypointTracker] Local target set to current waypoint %d: (%.2f, %.2f, %.2f), dist: %.2f",
                  current_waypoint_idx_, local_target_pt_.x(), local_target_pt_.y(), local_target_pt_.z(),
                  dist_to_current_waypoint);
        return;
      }
      
      // 当前航点超出规划视野，使用全局轨迹找中间点，但限制不超过当前航点
      // 首先找到全局轨迹上最接近当前航点的时间点
      double waypoint_t = planner_manager_->global_data_.global_duration_;
      double min_dist_to_wp = 9999;
      for (double t_search = 0.0; 
           t_search < planner_manager_->global_data_.global_duration_; t_search += t_step)
      {
        Eigen::Vector3d pos_t = planner_manager_->global_data_.getPosition(t_search);
        double dist = (pos_t - current_target_waypoint_).norm();
        if (dist < min_dist_to_wp)
        {
          min_dist_to_wp = dist;
          waypoint_t = t_search;
        }
      }
      
      // 在当前位置到航点之间找一个规划视野内的点
      double actual_progress_t = 0.0;
      double min_dist_to_traj = 9999;
      for (double t_search = 0.0; t_search <= waypoint_t; t_search += t_step)
      {
        Eigen::Vector3d pos_t = planner_manager_->global_data_.getPosition(t_search);
        double dist_to_traj = (pos_t - reference_pos).norm();
        if (dist_to_traj < min_dist_to_traj)
        {
          min_dist_to_traj = dist_to_traj;
          actual_progress_t = t_search;
        }
      }
      
      // 从当前进度搜索局部目标，但不超过航点时间
      for (t = actual_progress_t; t <= waypoint_t; t += t_step)
      {
        Eigen::Vector3d pos_t = planner_manager_->global_data_.getPosition(t);
        double dist = (pos_t - reference_pos).norm();
        
        if (dist < dist_min)
        {
          dist_min = dist;
          dist_min_t = t;
        }
        if (dist >= planning_horizen_)
        {
          local_target_pt_ = pos_t;
          planner_manager_->global_data_.last_progress_time_ = dist_min_t;
          break;
        }
      }
      
      // 如果没有找到规划视野外的点，使用航点作为目标
      if (t > waypoint_t)
      {
        local_target_pt_ = current_target_waypoint_;
        // 修改：使用全局轨迹在航点处的速度，而非零速度
        local_target_vel_ = planner_manager_->global_data_.getVelocity(waypoint_t);
        // 如果全局轨迹速度太小，使用指向下一航点的方向
        if (local_target_vel_.norm() < 0.1 && current_waypoint_idx_ + 1 < (int)all_waypoints_.size())
        {
          Eigen::Vector3d to_next = (all_waypoints_[current_waypoint_idx_ + 1] - current_target_waypoint_).normalized();
          local_target_vel_ = to_next * planner_manager_->pp_.max_vel_ * 0.5;
        }
        return;
      }
      
      // 设置局部目标速度 - 修改：只有最终航点才使用零速度
      bool is_final_waypoint = (current_waypoint_idx_ == (int)all_waypoints_.size() - 1);
      double stopping_dist = (planner_manager_->pp_.max_vel_ * planner_manager_->pp_.max_vel_) / 
                             (2 * planner_manager_->pp_.max_acc_);
      
      if (is_final_waypoint && (current_target_waypoint_ - local_target_pt_).norm() < stopping_dist)
      {
        local_target_vel_ = Eigen::Vector3d::Zero();
      }
      else
      {
        local_target_vel_ = planner_manager_->global_data_.getVelocity(t);
        // 确保速度不会太小
        if (local_target_vel_.norm() < planner_manager_->pp_.max_vel_ * 0.2)
        {
          Eigen::Vector3d dir = (local_target_pt_ - odom_pos_).normalized();
          local_target_vel_ = dir * planner_manager_->pp_.max_vel_ * 0.3;
        }
      }
      return;
    }
    
    /* ========== 原始模式（非强制航点） ========== */
    // 首先，根据实际位置找到全局轨迹上最近的点，确定当前真实进度
    double actual_progress_t = 0.0;
    double min_dist_to_traj = 9999;
    for (double t_search = 0.0; 
         t_search < planner_manager_->global_data_.global_duration_; t_search += t_step)
    {
      Eigen::Vector3d pos_t = planner_manager_->global_data_.getPosition(t_search);
      double dist_to_traj = (pos_t - reference_pos).norm();
      if (dist_to_traj < min_dist_to_traj)
      {
        min_dist_to_traj = dist_to_traj;
        actual_progress_t = t_search;
      }
    }
    
    // 进度保护：只允许进度向前推进，不允许大幅度跳跃
    // 允许的最大进度跳跃量（基于最大速度和规划视野）
    double max_progress_jump = planning_horizen_ / planner_manager_->pp_.max_vel_;
    double old_progress = planner_manager_->global_data_.last_progress_time_;
    
    // 如果计算出的进度比上次进度落后太多，说明可能有问题，使用上次进度
    if (actual_progress_t < old_progress - t_step * 2)
    {
      actual_progress_t = old_progress;
    }
    // 如果进度跳跃太大，限制跳跃量
    else if (actual_progress_t > old_progress + max_progress_jump)
    {
      actual_progress_t = old_progress + max_progress_jump * 0.5;
    }
    
    // 从当前实际进度开始搜索局部目标点
    for (t = actual_progress_t; 
         t < planner_manager_->global_data_.global_duration_; t += t_step)
    {
      Eigen::Vector3d pos_t = planner_manager_->global_data_.getPosition(t);
      // 使用实际位置计算到轨迹点的距离
      double dist = (pos_t - reference_pos).norm();

      if (t < actual_progress_t + 1e-5 && dist > planning_horizen_)
      {
        ROS_WARN("getLocalTarget: reference position too far from trajectory, dist=%.2f", dist);
        // 不直接返回，而是尝试使用 start_pt_ 作为备选
        reference_pos = start_pt_;
        dist = (pos_t - reference_pos).norm();
      }
      if (dist < dist_min)
      {
        dist_min = dist;
        dist_min_t = t;
      }
      if (dist >= planning_horizen_)
      {
        local_target_pt_ = pos_t;
        // 只有当无人机实际位置确实接近这个进度点时，才更新进度
        Eigen::Vector3d progress_pos = planner_manager_->global_data_.getPosition(dist_min_t);
        double actual_dist_to_progress = (progress_pos - odom_pos_).norm();
        // 如果实际距离小于目标到达阈值的2倍，才更新进度
        if (actual_dist_to_progress < goal_reach_thresh_ * 2.0)
        {
          planner_manager_->global_data_.last_progress_time_ = dist_min_t;
        }
        break;
      }
    }
    if (t > planner_manager_->global_data_.global_duration_)
    {
      local_target_pt_ = end_pt_;
    }

    // 修改：只有非常接近最终目标时才使用零速度
    double stopping_dist = (planner_manager_->pp_.max_vel_ * planner_manager_->pp_.max_vel_) / 
                           (2 * planner_manager_->pp_.max_acc_);
    double dist_to_end = (end_pt_ - local_target_pt_).norm();
    
    if (dist_to_end < stopping_dist && (end_pt_ - odom_pos_).norm() < stopping_dist * 1.5)
    {
      // 真正接近终点时使用零速度
      local_target_vel_ = Eigen::Vector3d::Zero();
    }
    else
    {
      local_target_vel_ = planner_manager_->global_data_.getVelocity(t);
      // 确保速度不会太小导致停顿
      if (local_target_vel_.norm() < planner_manager_->pp_.max_vel_ * 0.2)
      {
        Eigen::Vector3d dir = (local_target_pt_ - odom_pos_).normalized();
        local_target_vel_ = dir * planner_manager_->pp_.max_vel_ * 0.3;
      }
    }
  }

  /* ========== 航点强制到达核心实现 ========== */
  
  void EGOReplanFSM::waypointCheckCallback(const ros::TimerEvent &e)
  {
    if (!use_mandatory_waypoints_ || !have_odom_ || all_waypoints_completed_)
      return;
    
    // 更新接近时间
    ros::Time now = ros::Time::now();
    double dt = (now - last_waypoint_check_time_).toSec();
    last_waypoint_check_time_ = now;
    
    if (have_target_ && !all_waypoints_.empty())
    {
      waypoint_approach_time_ += dt;
    }
    
    // 发布航点状态信息
    publishWaypointStatus();
    
    // 可视化当前目标航点
    visualizeCurrentWaypoint();
    
    // 检测航点是否到达
    if (checkWaypointReached())
    {
      ROS_INFO("[WaypointTracker] ========================================");
      ROS_INFO("[WaypointTracker] WAYPOINT %d REACHED! Position: (%.2f, %.2f, %.2f)",
               current_waypoint_idx_, current_target_waypoint_.x(), 
               current_target_waypoint_.y(), current_target_waypoint_.z());
      ROS_INFO("[WaypointTracker] Actual position: (%.2f, %.2f, %.2f)",
               odom_pos_.x(), odom_pos_.y(), odom_pos_.z());
      ROS_INFO("[WaypointTracker] ========================================");
      
      waypoint_reached_ = true;
      
      // 尝试前进到下一个航点
      if (!advanceToNextWaypoint())
      {
        // 所有航点已完成
        ROS_INFO("[WaypointTracker] ALL WAYPOINTS COMPLETED!");
        all_waypoints_completed_ = true;
        have_target_ = false;
        changeFSMExecState(WAIT_TARGET, "WAYPOINT_COMPLETE");
      }
      else
      {
        // 触发重规划到新航点
        ROS_INFO("[WaypointTracker] Advancing to waypoint %d: (%.2f, %.2f, %.2f)",
                 current_waypoint_idx_, current_target_waypoint_.x(), 
                 current_target_waypoint_.y(), current_target_waypoint_.z());
        
        // 重置计数器
        waypoint_replan_count_ = 0;
        waypoint_approach_time_ = 0.0;
        
        // 规划到新航点
        if (planToCurrentWaypoint())
        {
          have_target_ = true;
          have_new_target_ = false;          // REPLAN 分支不需要 poly_init 标志
          changeFSMExecState(REPLAN_TRAJ, "NEW_WAYPOINT");
        }
      }
    }
  }
  
  bool EGOReplanFSM::checkWaypointReached()
  {
    if (all_waypoints_.empty() || current_waypoint_idx_ >= (int)all_waypoints_.size())
      return false;
    
    // 使用无人机实际位置判断是否到达航点
    double dist = (odom_pos_ - current_target_waypoint_).norm();
    
    return dist < waypoint_reach_thresh_;
  }
  
  bool EGOReplanFSM::advanceToNextWaypoint()
  {
    current_waypoint_idx_++;
    
    if (current_waypoint_idx_ >= (int)all_waypoints_.size())
    {
      // 没有更多航点
      return false;
    }
    
    current_target_waypoint_ = all_waypoints_[current_waypoint_idx_];
    waypoint_reached_ = false;
    
    return true;
  }
  
  bool EGOReplanFSM::planToCurrentWaypoint()
  {
    if (all_waypoints_.empty() || current_waypoint_idx_ >= (int)all_waypoints_.size())
      return false;
    
    // 构建从当前航点到最终航点的路径
    std::vector<Eigen::Vector3d> remaining_wps;
    for (int i = current_waypoint_idx_; i < (int)all_waypoints_.size(); i++)
    {
      remaining_wps.push_back(all_waypoints_[i]);
    }
    
    // 更新终点为最终航点
    end_pt_ = all_waypoints_.back();
    
    // 关键修改：使用当前实际速度作为起始边界，而非零速度
    // 这样可以实现轨迹的平滑衔接
    Eigen::Vector3d start_vel = odom_vel_;
    
    // 如果当前速度太小，使用指向第一个航点的方向
    if (start_vel.norm() < 0.1 && !remaining_wps.empty())
    {
      Eigen::Vector3d dir = (remaining_wps[0] - odom_pos_).normalized();
      start_vel = dir * planner_manager_->pp_.max_vel_ * 0.3;
    }
    
    // 计算全局轨迹末端速度：
    // - 若还有多个 remaining 航点，末速指向最后一段方向，保持穿越动量
    // - 若只剩一个（最终目标），末速为零以确保精确停止
    Eigen::Vector3d end_vel = Eigen::Vector3d::Zero();
    if (remaining_wps.size() >= 2)
    {
      Eigen::Vector3d last_seg =
          (remaining_wps.back() - remaining_wps[remaining_wps.size() - 2]).normalized();
      end_vel = last_seg * planner_manager_->pp_.max_vel_ * 0.3;
    }

    // 规划全局轨迹 - 只有最终目标使用零速度
    bool success = planner_manager_->planGlobalTrajWaypoints(
        odom_pos_, start_vel, Eigen::Vector3d::Zero(),
        remaining_wps,
        end_vel, Eigen::Vector3d::Zero());
    
    if (success)
    {
      // 可视化剩余航点
      for (size_t i = 0; i < remaining_wps.size(); i++)
      {
        Eigen::Vector4d color = (i == 0) ? Eigen::Vector4d(1, 0, 0, 1) : Eigen::Vector4d(0, 0.5, 0.5, 1);
        visualization_->displayGoalPoint(remaining_wps[i], color, 0.3, current_waypoint_idx_ + i);
        ros::Duration(0.001).sleep();
      }
      
      // 可视化全局轨迹
      constexpr double step_size_t = 0.1;
      int i_end = floor(planner_manager_->global_data_.global_duration_ / step_size_t);
      std::vector<Eigen::Vector3d> global_traj(i_end);
      for (int i = 0; i < i_end; i++)
      {
        global_traj[i] = planner_manager_->global_data_.global_traj_.evaluate(i * step_size_t);
      }
      visualization_->displayGlobalPathList(global_traj, 0.1, 0);
      
      ROS_INFO("[WaypointTracker] Successfully planned trajectory to waypoint %d", current_waypoint_idx_);
    }
    else
    {
      ROS_WARN("[WaypointTracker] Failed to plan trajectory to waypoint %d", current_waypoint_idx_);
    }
    
    return success;
  }
  
  Eigen::Vector3d EGOReplanFSM::getCurrentTargetWaypoint()
  {
    if (all_waypoints_.empty() || current_waypoint_idx_ >= (int)all_waypoints_.size())
      return end_pt_;
    
    return current_target_waypoint_;
  }
  
  bool EGOReplanFSM::isAllWaypointsCompleted()
  {
    return all_waypoints_completed_;
  }
  
  void EGOReplanFSM::publishWaypointStatus()
  {
    // 发布当前航点索引
    std_msgs::Int32 idx_msg;
    idx_msg.data = current_waypoint_idx_;
    current_waypoint_pub_.publish(idx_msg);
    
    // 发布到当前航点的距离
    if (!all_waypoints_.empty() && current_waypoint_idx_ < (int)all_waypoints_.size())
    {
      std_msgs::Float32 dist_msg;
      dist_msg.data = (odom_pos_ - current_target_waypoint_).norm();
      waypoint_distance_pub_.publish(dist_msg);
    }
    
    // 发布航点完成状态
    std_msgs::Int32 status_msg;
    if (all_waypoints_completed_)
      status_msg.data = -1;  // 全部完成
    else
      status_msg.data = current_waypoint_idx_;  // 当前正在前往的航点
    waypoint_status_pub_.publish(status_msg);
  }
  
  void EGOReplanFSM::visualizeCurrentWaypoint()
  {
    if (all_waypoints_.empty() || current_waypoint_idx_ >= (int)all_waypoints_.size())
      return;
    
    // 用红色高亮显示当前目标航点
    visualization_->displayGoalPoint(current_target_waypoint_, Eigen::Vector4d(1, 0, 0, 1), 0.5, 100);
  }
  
  bool EGOReplanFSM::planDirectToWaypoint()
  {
    if (all_waypoints_.empty() || current_waypoint_idx_ >= (int)all_waypoints_.size())
      return false;
    
    // 直接规划到当前航点，不考虑后续航点
    std::vector<Eigen::Vector3d> single_wp;
    single_wp.push_back(current_target_waypoint_);
    
    end_pt_ = current_target_waypoint_;
    
    bool success = planner_manager_->planGlobalTrajWaypoints(
        odom_pos_, odom_vel_, Eigen::Vector3d::Zero(),
        single_wp,
        Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
    
    return success;
  }

} // namespace ego_planner
