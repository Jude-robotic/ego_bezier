/**
 * @file planner_manager.cpp
 * @brief Implementation of EGO Planner Manager with Piecewise Bezier Curves
 */

#include <plan_manage/planner_manager.h>
#include <thread>

namespace ego_planner
{

  EGOPlannerManager::EGOPlannerManager() {}

  EGOPlannerManager::~EGOPlannerManager() { std::cout << "des manager" << std::endl; }

  void EGOPlannerManager::initPlanModules(ros::NodeHandle &nh, PlanningVisualization::Ptr vis)
  {
    /* Read algorithm parameters */
    nh.param("manager/max_vel", pp_.max_vel_, -1.0);
    nh.param("manager/max_acc", pp_.max_acc_, -1.0);
    nh.param("manager/max_jerk", pp_.max_jerk_, -1.0);
    nh.param("manager/feasibility_tolerance", pp_.feasibility_tolerance_, 0.0);
    nh.param("manager/control_points_distance", pp_.ctrl_pt_dist, -1.0);
    nh.param("manager/planning_horizon", pp_.planning_horizen_, 5.0);

    local_data_.traj_id_ = 0;
    grid_map_.reset(new GridMap);
    grid_map_->initMap(nh);

    bezier_optimizer_rebound_.reset(new BezierOptimizer);
    bezier_optimizer_rebound_->setParam(nh);
    bezier_optimizer_rebound_->setEnvironment(grid_map_);
    bezier_optimizer_rebound_->a_star_.reset(new AStar);
    bezier_optimizer_rebound_->a_star_->initGridMap(grid_map_, Eigen::Vector3i(100, 100, 100));

    
    visualization_ = vis;
    
  }

  bool EGOPlannerManager::reboundReplan(Eigen::Vector3d start_pt, Eigen::Vector3d start_vel,
                                        Eigen::Vector3d start_acc, Eigen::Vector3d local_target_pt,
                                        Eigen::Vector3d local_target_vel, bool flag_polyInit, bool flag_randomPolyTraj)
  {
    static int count = 0;
    std::cout << endl << "[Bezier replan]: -------------------------------------" << count++ << std::endl;
    cout.precision(3);
    
    // Get height limits from grid_map boundaries - use actual map limits
    // For inclined corridor scenarios, we need the full Z range
    Eigen::Vector3d map_max = grid_map_->getMapMaxBoundary();
    Eigen::Vector3d map_min = grid_map_->getMapMinBoundary();
    double max_flight_height = map_max.z() - 0.5;  // Safety margin from ceiling
    double min_flight_height = map_min.z() + 0.3;  // Safety margin from ground
    
    start_pt.z() = std::max(min_flight_height, std::min(max_flight_height, start_pt.z()));
    local_target_pt.z() = std::max(min_flight_height, std::min(max_flight_height, local_target_pt.z()));
    
    cout << "start: " << start_pt.transpose() << ", " << start_vel.transpose() 
         << "\ngoal:" << local_target_pt.transpose() << ", " << local_target_vel.transpose() << endl;
    cout << "height limits: [" << min_flight_height << ", " << max_flight_height << "]" << endl;

    if ((start_pt - local_target_pt).norm() < 0.2)
    {
      cout << "Close to goal" << endl;
      continous_failures_count_++;
      return false;
    }

    ros::Time t_start = ros::Time::now();
    ros::Duration t_init, t_opt, t_refine;

    /*** STEP 1: INIT ***/
    double ts = (start_pt - local_target_pt).norm() > 0.1 
                ? pp_.ctrl_pt_dist / pp_.max_vel_ * 1.2 
                : pp_.ctrl_pt_dist / pp_.max_vel_ * 5;
    
    vector<Eigen::Vector3d> point_set, start_end_derivatives;
    static bool flag_first_call = true, flag_force_polynomial = false;
    bool flag_regenerate = false;
    
    do
    {
      point_set.clear();
      start_end_derivatives.clear();
      flag_regenerate = false;

      if (flag_first_call || flag_polyInit || flag_force_polynomial)
      {
        flag_first_call = false;
        flag_force_polynomial = false;

        PolynomialTraj gl_traj;
        double dist = (start_pt - local_target_pt).norm();
        double time = pow(pp_.max_vel_, 2) / pp_.max_acc_ > dist 
                      ? sqrt(dist / pp_.max_acc_) 
                      : (dist - pow(pp_.max_vel_, 2) / pp_.max_acc_) / pp_.max_vel_ + 2 * pp_.max_vel_ / pp_.max_acc_;

        if (!flag_randomPolyTraj)
        {
          gl_traj = PolynomialTraj::one_segment_traj_gen(start_pt, start_vel, start_acc, 
                                                         local_target_pt, local_target_vel, 
                                                         Eigen::Vector3d::Zero(), time);
        }
        else
        {
          Eigen::Vector3d horizen_dir = ((start_pt - local_target_pt).cross(Eigen::Vector3d(0, 0, 1))).normalized();
          Eigen::Vector3d vertical_dir = ((start_pt - local_target_pt).cross(horizen_dir)).normalized();
          Eigen::Vector3d random_inserted_pt = (start_pt + local_target_pt) / 2 +
              (((double)rand()) / RAND_MAX - 0.5) * (start_pt - local_target_pt).norm() * horizen_dir * 0.8 * (-0.978 / (continous_failures_count_ + 0.989) + 0.989) +
              (((double)rand()) / RAND_MAX - 0.5) * (start_pt - local_target_pt).norm() * vertical_dir * 0.4 * (-0.978 / (continous_failures_count_ + 0.989) + 0.989);
          
          Eigen::MatrixXd pos(3, 3);
          pos.col(0) = start_pt;
          pos.col(1) = random_inserted_pt;
          pos.col(2) = local_target_pt;
          Eigen::VectorXd t(2);
          t(0) = t(1) = time / 2;
          gl_traj = PolynomialTraj::minSnapTraj(pos, start_vel, local_target_vel, start_acc, Eigen::Vector3d::Zero(), t);
        }

        double t;
        bool flag_too_far;
        ts *= 1.5;
        do
        {
          ts /= 1.5;
          point_set.clear();
          flag_too_far = false;
          Eigen::Vector3d last_pt = gl_traj.evaluate(0);
          // Clamp initial point height
          last_pt.z() = std::max(min_flight_height, std::min(max_flight_height, last_pt.z()));
          for (t = 0; t < time; t += ts)
          {
            Eigen::Vector3d pt = gl_traj.evaluate(t);
            // CRITICAL: Clamp each trajectory point to height limits
            pt.z() = std::max(min_flight_height, std::min(max_flight_height, pt.z()));
            if ((last_pt - pt).norm() > pp_.ctrl_pt_dist * 1.5)
            {
              flag_too_far = true;
              break;
            }
            last_pt = pt;
            point_set.push_back(pt);
          }
        } while (flag_too_far || point_set.size() < 7);
        t -= ts;
        start_end_derivatives.push_back(gl_traj.evaluateVel(0));
        start_end_derivatives.push_back(local_target_vel);
        start_end_derivatives.push_back(gl_traj.evaluateAcc(0));
        start_end_derivatives.push_back(gl_traj.evaluateAcc(t));
      }
      else
      {
        double t;
        double t_cur = (ros::Time::now() - local_data_.start_time_).toSec();

        vector<double> pseudo_arc_length;
        vector<Eigen::Vector3d> segment_point;
        pseudo_arc_length.push_back(0.0);
        for (t = t_cur; t < local_data_.duration_ + 1e-3; t += ts)
        {
          segment_point.push_back(local_data_.position_traj_.evaluateT(t));
          if (t > t_cur)
          {
            pseudo_arc_length.push_back((segment_point.back() - segment_point[segment_point.size() - 2]).norm() + pseudo_arc_length.back());
          }
        }
        t -= ts;

        double poly_time = (local_data_.position_traj_.evaluateT(t) - local_target_pt).norm() / pp_.max_vel_ * 2;
        if (poly_time > ts)
        {
          PolynomialTraj gl_traj = PolynomialTraj::one_segment_traj_gen(
              local_data_.position_traj_.evaluateT(t),
              local_data_.velocity_traj_.evaluateT(t),
              local_data_.acceleration_traj_.evaluateT(t),
              local_target_pt, local_target_vel, Eigen::Vector3d::Zero(), poly_time);

          for (t = ts; t < poly_time; t += ts)
          {
            if (!pseudo_arc_length.empty())
            {
              segment_point.push_back(gl_traj.evaluate(t));
              pseudo_arc_length.push_back((segment_point.back() - segment_point[segment_point.size() - 2]).norm() + pseudo_arc_length.back());
            }
            else
            {
              ROS_ERROR("pseudo_arc_length is empty, return!");
              continous_failures_count_++;
              return false;
            }
          }
        }

        double sample_length = 0;
        double cps_dist = pp_.ctrl_pt_dist * 1.5;
        size_t id = 0;
        do
        {
          cps_dist /= 1.5;
          point_set.clear();
          sample_length = 0;
          id = 0;
          while ((id <= pseudo_arc_length.size() - 2) && sample_length <= pseudo_arc_length.back())
          {
            if (sample_length >= pseudo_arc_length[id] && sample_length < pseudo_arc_length[id + 1])
            {
              point_set.push_back((sample_length - pseudo_arc_length[id]) / (pseudo_arc_length[id + 1] - pseudo_arc_length[id]) * segment_point[id + 1] +
                                  (pseudo_arc_length[id + 1] - sample_length) / (pseudo_arc_length[id + 1] - pseudo_arc_length[id]) * segment_point[id]);
              sample_length += cps_dist;
            }
            else
              id++;
          }
          point_set.push_back(local_target_pt);
        } while (point_set.size() < 7);

        start_end_derivatives.push_back(local_data_.velocity_traj_.evaluateT(t_cur));
        start_end_derivatives.push_back(local_target_vel);
        start_end_derivatives.push_back(local_data_.acceleration_traj_.evaluateT(t_cur));
        start_end_derivatives.push_back(Eigen::Vector3d::Zero());

        if (point_set.size() > pp_.planning_horizen_ / pp_.ctrl_pt_dist * 3)
        {
          flag_force_polynomial = true;
          flag_regenerate = true;
        }
      }
    } while (flag_regenerate);

    // CRITICAL: Clamp all point heights to flight limits
    for (auto& pt : point_set)
    {
      pt.z() = std::max(min_flight_height, std::min(max_flight_height, pt.z()));
    }

    Eigen::MatrixXd ctrl_pts;
    PiecewiseBezier::parameterizeToBezier(ts, point_set, start_end_derivatives, ctrl_pts);
    
    // Also clamp control points z-coordinates
    for (int i = 0; i < ctrl_pts.cols(); ++i)
    {
      ctrl_pts(2, i) = std::max(min_flight_height, std::min(max_flight_height, ctrl_pts(2, i)));
    }

    vector<vector<Eigen::Vector3d>> a_star_pathes;
    a_star_pathes = bezier_optimizer_rebound_->initControlPoints(ctrl_pts, true);

    t_init = ros::Time::now() - t_start;

    static int vis_id = 0;
    visualization_->displayInitPathList(point_set, 0.2, 0);
    visualization_->displayAStarList(a_star_pathes, vis_id);

    t_start = ros::Time::now();

    /*** STEP 2: OPTIMIZE ***/
    bool flag_step_1_success = bezier_optimizer_rebound_->BezierOptimizeTrajRebound(ctrl_pts, ts);
    cout << "first_optimize_step_success=" << flag_step_1_success << endl;
    
    // 如果直接优化失败，尝试使用 A* 搜索安全路径作为初始轨迹
    if (!flag_step_1_success)
    {
      ROS_WARN("[Planner] Direct optimization failed, trying A* path search...");
      
      // 使用 A* 搜索从起点到目标点的安全路径
      double a_star_step = grid_map_->getResolution() * 2.0;  // 搜索步长
      bool a_star_success = bezier_optimizer_rebound_->a_star_->AstarSearch(a_star_step, start_pt, local_target_pt);
      
      if (a_star_success)
      {
        ROS_INFO("[Planner] A* found a safe path, re-optimizing...");
        
        // 获取 A* 路径
        std::vector<Eigen::Vector3d> a_star_path = bezier_optimizer_rebound_->a_star_->getPath();
        
        // 简化路径点（降采样）
        std::vector<Eigen::Vector3d> sparse_path;
        sparse_path.push_back(start_pt);  // 确保起点正确
        
        double min_dist = pp_.ctrl_pt_dist * 0.8;  // 控制点间最小距离
        Eigen::Vector3d last_added = start_pt;
        
        for (size_t i = 1; i < a_star_path.size() - 1; ++i)
        {
          if ((a_star_path[i] - last_added).norm() >= min_dist)
          {
            // Clamp A* path point heights
            Eigen::Vector3d clamped_pt = a_star_path[i];
            clamped_pt.z() = std::max(min_flight_height, std::min(max_flight_height, clamped_pt.z()));
            sparse_path.push_back(clamped_pt);
            last_added = clamped_pt;
          }
        }
        sparse_path.push_back(local_target_pt);  // 确保终点正确
        
        // 确保至少有7个控制点
        while (sparse_path.size() < 7)
        {
          // 在相邻点之间插值
          std::vector<Eigen::Vector3d> new_path;
          for (size_t i = 0; i < sparse_path.size() - 1; ++i)
          {
            new_path.push_back(sparse_path[i]);
            Eigen::Vector3d mid = (sparse_path[i] + sparse_path[i+1]) / 2.0;
            // Clamp interpolated point height
            mid.z() = std::max(min_flight_height, std::min(max_flight_height, mid.z()));
            new_path.push_back(mid);
          }
          new_path.push_back(sparse_path.back());
          sparse_path = new_path;
        }
        
        // 使用 A* 路径重新生成控制点
        std::vector<Eigen::Vector3d> a_star_derivatives;
        a_star_derivatives.push_back(start_vel);  // 起始速度
        a_star_derivatives.push_back(local_target_vel);  // 终点速度
        a_star_derivatives.push_back(start_acc);  // 起始加速度
        a_star_derivatives.push_back(Eigen::Vector3d::Zero());  // 终点加速度
        
        Eigen::MatrixXd a_star_ctrl_pts;
        PiecewiseBezier::parameterizeToBezier(ts, sparse_path, a_star_derivatives, a_star_ctrl_pts);
        
        // Clamp A* control points height
        for (int i = 0; i < a_star_ctrl_pts.cols(); ++i)
        {
          a_star_ctrl_pts(2, i) = std::max(min_flight_height, std::min(max_flight_height, a_star_ctrl_pts(2, i)));
        }
        
        // 重新初始化控制点和约束
        bezier_optimizer_rebound_->initControlPoints(a_star_ctrl_pts, true);
        
        // 可视化 A* 路径
        visualization_->displayInitPathList(sparse_path, 0.15, 1);
        
        // 重新优化
        flag_step_1_success = bezier_optimizer_rebound_->BezierOptimizeTrajRebound(a_star_ctrl_pts, ts);
        cout << "A* guided optimization success=" << flag_step_1_success << endl;
        
        if (flag_step_1_success)
        {
          ctrl_pts = a_star_ctrl_pts;  // 使用 A* 引导的控制点
        }
      }
      else
      {
        ROS_WARN("[Planner] A* path search also failed!");
      }
      
      if (!flag_step_1_success)
      {
        continous_failures_count_++;
        return false;
      }
    }

    t_opt = ros::Time::now() - t_start;
    t_start = ros::Time::now();

    /*** STEP 3: REFINE (RE-ALLOCATE TIME) IF NECESSARY ***/
    PiecewiseBezier pos = PiecewiseBezier(ctrl_pts, 3, ts);
    pos.setPhysicalLimits(pp_.max_vel_, pp_.max_acc_, pp_.feasibility_tolerance_);

    double ratio;
    bool flag_step_2_success = true;
    if (!pos.checkFeasibility(ratio, false))
    {
      cout << "Need to reallocate time." << endl;

      Eigen::MatrixXd optimal_control_points;
      flag_step_2_success = refineTrajAlgo(pos, start_end_derivatives, ratio, ts, optimal_control_points);
      if (flag_step_2_success)
        pos = PiecewiseBezier(optimal_control_points, 3, ts);
    }

    if (!flag_step_2_success)
    {
      printf("\033[34mThis refined trajectory hits obstacles. It doesn't matter if appears occasionally. But if continuously appearing, Increase parameter \"lambda_fitness\".\n\033[0m");
      continous_failures_count_++;
      return false;
    }

    t_refine = ros::Time::now() - t_start;

    // Save planned results
    updateTrajInfo(pos, ros::Time::now());

    cout << "total time:\033[42m" << (t_init + t_opt + t_refine).toSec() << "\033[0m,optimize:" << (t_init + t_opt).toSec() << ",refine:" << t_refine.toSec() << endl;

    continous_failures_count_ = 0;
    return true;
  }

  bool EGOPlannerManager::EmergencyStop(Eigen::Vector3d stop_pos)
  {
    Eigen::MatrixXd control_points(3, 6);
    for (int i = 0; i < 6; i++)
    {
      control_points.col(i) = stop_pos;
    }

    updateTrajInfo(PiecewiseBezier(control_points, 3, 1.0), ros::Time::now());

    return true;
  }

  bool EGOPlannerManager::planGlobalTrajWaypoints(const Eigen::Vector3d &start_pos, const Eigen::Vector3d &start_vel, const Eigen::Vector3d &start_acc,
                                                  const std::vector<Eigen::Vector3d> &waypoints, const Eigen::Vector3d &end_vel, const Eigen::Vector3d &end_acc)
  {
    // Get height limits from grid_map - use actual map limits for inclined corridors
    Eigen::Vector3d map_max = grid_map_->getMapMaxBoundary();
    Eigen::Vector3d map_min = grid_map_->getMapMinBoundary();
    double max_flight_height = map_max.z() - 0.5;  // Safety margin from ceiling
    double min_flight_height = map_min.z() + 0.3;  // Safety margin from ground
    
    ROS_INFO("[PlannerManager] Global traj height limits: min=%.2f, max=%.2f", min_flight_height, max_flight_height);
    
    vector<Eigen::Vector3d> points;
    // Clamp start position height
    Eigen::Vector3d clamped_start = start_pos;
    clamped_start.z() = std::max(min_flight_height, std::min(max_flight_height, clamped_start.z()));
    points.push_back(clamped_start);

    for (size_t wp_i = 0; wp_i < waypoints.size(); wp_i++)
    {
      // Clamp each waypoint height
      Eigen::Vector3d wp = waypoints[wp_i];
      wp.z() = std::max(min_flight_height, std::min(max_flight_height, wp.z()));
      points.push_back(wp);
    }

    double total_len = 0;
    total_len += (clamped_start - points[1]).norm();
    for (size_t i = 0; i < waypoints.size() - 1; i++)
    {
      total_len += (waypoints[i + 1] - waypoints[i]).norm();
    }

    vector<Eigen::Vector3d> inter_points;
    double dist_thresh = max(total_len / 8, 4.0);

    for (size_t i = 0; i < points.size() - 1; ++i)
    {
      inter_points.push_back(points.at(i));
      double dist = (points.at(i + 1) - points.at(i)).norm();

      if (dist > dist_thresh)
      {
        int id_num = floor(dist / dist_thresh) + 1;
        for (int j = 1; j < id_num; ++j)
        {
          Eigen::Vector3d inter_pt = points.at(i) * (1.0 - double(j) / id_num) + points.at(i + 1) * double(j) / id_num;
          inter_points.push_back(inter_pt);
        }
      }
    }

    inter_points.push_back(points.back());

    int pt_num = inter_points.size();
    Eigen::MatrixXd pos(3, pt_num);
    for (int i = 0; i < pt_num; ++i)
      pos.col(i) = inter_points[i];

    Eigen::Vector3d zero(0, 0, 0);
    Eigen::VectorXd time(pt_num - 1);
    for (int i = 0; i < pt_num - 1; ++i)
    {
      time(i) = (pos.col(i + 1) - pos.col(i)).norm() / (pp_.max_vel_);
    }

    time(0) *= 2.0;
    time(time.rows() - 1) *= 2.0;

    PolynomialTraj gl_traj;
    if (pos.cols() >= 3)
      gl_traj = PolynomialTraj::minSnapTraj(pos, start_vel, end_vel, start_acc, end_acc, time);
    else if (pos.cols() == 2)
      gl_traj = PolynomialTraj::one_segment_traj_gen(start_pos, start_vel, start_acc, pos.col(1), end_vel, end_acc, time(0));
    else
      return false;

    auto time_now = ros::Time::now();
    global_data_.setGlobalTraj(gl_traj, time_now);

    return true;
  }

  bool EGOPlannerManager::planGlobalTraj(const Eigen::Vector3d &start_pos, const Eigen::Vector3d &start_vel, const Eigen::Vector3d &start_acc,
                                         const Eigen::Vector3d &end_pos, const Eigen::Vector3d &end_vel, const Eigen::Vector3d &end_acc)
  {
    // Get height limits from grid_map - use actual map limits for inclined corridors
    Eigen::Vector3d map_max = grid_map_->getMapMaxBoundary();
    Eigen::Vector3d map_min = grid_map_->getMapMinBoundary();
    double max_flight_height = map_max.z() - 0.5;  // Safety margin from ceiling
    double min_flight_height = map_min.z() + 0.3;  // Safety margin from ground
    
    // Clamp positions
    Eigen::Vector3d clamped_start = start_pos;
    Eigen::Vector3d clamped_end = end_pos;
    clamped_start.z() = std::max(min_flight_height, std::min(max_flight_height, clamped_start.z()));
    clamped_end.z() = std::max(min_flight_height, std::min(max_flight_height, clamped_end.z()));
    
    vector<Eigen::Vector3d> points;
    points.push_back(clamped_start);
    points.push_back(clamped_end);

    vector<Eigen::Vector3d> inter_points;
    const double dist_thresh = 4.0;

    for (size_t i = 0; i < points.size() - 1; ++i)
    {
      inter_points.push_back(points.at(i));
      double dist = (points.at(i + 1) - points.at(i)).norm();

      if (dist > dist_thresh)
      {
        int id_num = floor(dist / dist_thresh) + 1;
        for (int j = 1; j < id_num; ++j)
        {
          Eigen::Vector3d inter_pt = points.at(i) * (1.0 - double(j) / id_num) + points.at(i + 1) * double(j) / id_num;
          // Also clamp interpolated points
          inter_pt.z() = std::max(min_flight_height, std::min(max_flight_height, inter_pt.z()));
          inter_points.push_back(inter_pt);
        }
      }
    }

    inter_points.push_back(points.back());

    int pt_num = inter_points.size();
    Eigen::MatrixXd pos(3, pt_num);
    for (int i = 0; i < pt_num; ++i)
      pos.col(i) = inter_points[i];

    Eigen::Vector3d zero(0, 0, 0);
    Eigen::VectorXd time(pt_num - 1);
    for (int i = 0; i < pt_num - 1; ++i)
    {
      time(i) = (pos.col(i + 1) - pos.col(i)).norm() / (pp_.max_vel_);
    }

    time(0) *= 2.0;
    time(time.rows() - 1) *= 2.0;

    PolynomialTraj gl_traj;
    if (pos.cols() >= 3)
      gl_traj = PolynomialTraj::minSnapTraj(pos, start_vel, end_vel, start_acc, end_acc, time);
    else if (pos.cols() == 2)
      gl_traj = PolynomialTraj::one_segment_traj_gen(start_pos, start_vel, start_acc, end_pos, end_vel, end_acc, time(0));
    else
      return false;

    auto time_now = ros::Time::now();
    global_data_.setGlobalTraj(gl_traj, time_now);

    return true;
  }

  bool EGOPlannerManager::refineTrajAlgo(PiecewiseBezier &traj, vector<Eigen::Vector3d> &start_end_derivative, 
                                         double ratio, double &ts, Eigen::MatrixXd &optimal_control_points)
  {
    double t_inc;
    Eigen::MatrixXd ctrl_pts;

    reparamBezier(traj, start_end_derivative, ratio, ctrl_pts, ts, t_inc);

    traj = PiecewiseBezier(ctrl_pts, 3, ts);

    // Generate reference points for Bezier curve (must match control point count)
    int num_ctrl_pts = ctrl_pts.cols();
    bezier_optimizer_rebound_->ref_pts_.clear();
    bezier_optimizer_rebound_->ref_pts_.resize(num_ctrl_pts);

    for (int i = 0; i < num_ctrl_pts; ++i) {
      bezier_optimizer_rebound_->ref_pts_[i] = ctrl_pts.col(i);
    }

    bool success = bezier_optimizer_rebound_->BezierOptimizeTrajRefine(ctrl_pts, ts, optimal_control_points);

    return success;
  }

  void EGOPlannerManager::updateTrajInfo(const PiecewiseBezier &position_traj, const ros::Time time_now)
  {
    local_data_.start_time_ = time_now;
    local_data_.position_traj_ = position_traj;
    local_data_.velocity_traj_ = local_data_.position_traj_.getDerivative();
    local_data_.acceleration_traj_ = local_data_.velocity_traj_.getDerivative();
    local_data_.start_pos_ = local_data_.position_traj_.evaluateT(0.0);
    local_data_.duration_ = local_data_.position_traj_.getTimeSum();
    local_data_.traj_id_ += 1;
  }

  void EGOPlannerManager::reparamBezier(PiecewiseBezier &bezier, vector<Eigen::Vector3d> &start_end_derivative, double ratio,
                                        Eigen::MatrixXd &ctrl_pts, double &dt, double &time_inc)
  {
    double time_origin = bezier.getTimeSum();
    int seg_num = (bezier.getControlPoint().cols() - 1) / 3;

    bezier.lengthenTime(ratio);
    double duration = bezier.getTimeSum();
    dt = duration / double(seg_num);
    time_inc = duration - time_origin;

    vector<Eigen::Vector3d> point_set;
    for (double time = 0.0; time <= duration + 1e-4; time += dt)
    {
      point_set.push_back(bezier.evaluateT(time));
    }
    PiecewiseBezier::parameterizeToBezier(dt, point_set, start_end_derivative, ctrl_pts);
  }

} // namespace ego_planner
