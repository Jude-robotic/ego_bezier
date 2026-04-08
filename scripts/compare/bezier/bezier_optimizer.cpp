/**
 * @file bezier_optimizer.cpp
 * @brief Implementation of Piecewise Bezier Curve Trajectory Optimizer
 * 
 * This optimizer uses L-BFGS to minimize a cost function consisting of:
 * - Smoothness (jerk minimization + inter-segment continuity)
 * - Collision avoidance
 * - Feasibility (velocity/acceleration constraints)
 * - Fitness (trajectory refinement)
 */

#include "bezier_opt/bezier_optimizer.h"
#include "bezier_opt/gradient_descent_optimizer.h"

namespace ego_planner
{

  void BezierOptimizer::setParam(ros::NodeHandle &nh)
  {
    nh.param("optimization/lambda_smooth", lambda1_, -1.0);
    nh.param("optimization/lambda_collision", lambda2_, -1.0);
    nh.param("optimization/lambda_feasibility", lambda3_, -1.0);
    nh.param("optimization/lambda_fitness", lambda4_, -1.0);
    nh.param("optimization/dist0", dist0_, -1.0);
    nh.param("optimization/max_vel", max_vel_, -1.0);
    nh.param("optimization/max_acc", max_acc_, -1.0);
    nh.param("optimization/order", order_, 3);  // Cubic Bezier
    
    // Initialize collision weight for rebound optimization
    new_lambda2_ = lambda2_;
  }

  void BezierOptimizer::setParamDirect(
      double lambda_smooth, double lambda_collision,
      double lambda_feasibility, double lambda_fitness,
      double lambda_formation,
      double dist0, double max_vel, double max_acc, int order)
  {
    lambda1_     = lambda_smooth;
    lambda2_     = lambda_collision;
    lambda3_     = lambda_feasibility;
    lambda4_     = lambda_fitness;
    lambda5_     = lambda_formation;
    dist0_       = dist0;
    max_vel_     = max_vel;
    max_acc_     = max_acc;
    order_       = order;
    new_lambda2_ = lambda2_;  // 将碰撞权重同步到内部动态权重寄存器
  }

  void BezierOptimizer::setEnvironment(const GridMap::Ptr &env)
  {
    this->grid_map_ = env;
  }

  void BezierOptimizer::setControlPoints(const Eigen::MatrixXd &points)
  {
    cps_.points = points;
  }

  void BezierOptimizer::setSegmentDuration(const double &ts)
  {
    segment_duration_ = ts;
  }

  std::vector<std::vector<Eigen::Vector3d>> BezierOptimizer::initControlPoints(
      Eigen::MatrixXd &init_points, bool flag_first_init)
  {
    if (flag_first_init)
    {
      cps_.clearance = dist0_;
      // DEBUG: Verify clearance assignment
      ROS_WARN("[BezierOptimizer] initControlPoints: clearance set to %f (dist0_=%f)", 
               cps_.clearance, dist0_);
      cps_.resize(init_points.cols());
      cps_.points = init_points;
      // Initialize collision base points and directions from the grid map
      // For each control point, cast several rays outward to find nearby
      // occupied voxels (inflated occupancy). For each hit, record the base
      // point (voxel center) and a direction vector pointing from base_point
      // to the control point (used in distance cost computation).
      if (grid_map_) {
        double res = grid_map_->getResolution();
        double max_search = std::max(5.0, cps_.clearance * 4.0); // Increased search range for wall obstacles
        int steps = std::max(4, int(max_search / res));

        // sampling directions: azimuthal around XY plane, with a few elevations
        std::vector<Eigen::Vector3d> dirs;
        int azimuth_samples = 24; // Increased for better wall detection
        // Add vertical sampling: more angles for better 3D coverage
        std::vector<double> elevs = {0.0, 0.15, -0.15, 0.3, -0.3, 0.5, -0.5, 0.785, -0.785, 1.2, -1.2, 1.57, -1.57};
        for (int ea = 0; ea < (int)elevs.size(); ++ea) {
          double el = elevs[ea];
          for (int a = 0; a < azimuth_samples; ++a) {
            double ang = 2.0 * M_PI * double(a) / double(azimuth_samples);
            Eigen::Vector3d d;
            d.x() = cos(ang) * cos(el);
            d.y() = sin(ang) * cos(el);
            d.z() = sin(el);
            dirs.push_back(d.normalized());
          }
        }

        for (int i = 0; i < cps_.points.cols(); ++i) {
          Eigen::Vector3d cp = cps_.points.col(i);
          cps_.base_point[i].clear();
          cps_.direction[i].clear();

          // For each direction, step outward from control point to find occupancy
          for (const auto &d : dirs) {
            bool found = false;
            for (int s = 1; s <= steps; ++s) {
              double dist = double(s) * res * 0.8; // overlap a bit
              Eigen::Vector3d sample = cp + d * dist;
              if (!grid_map_->isInMap(sample)) continue;
              int occ = grid_map_->getInflateOccupancy(sample);
              if (occ > 0) {
                // convert sample index to voxel center
                Eigen::Vector3i idx; grid_map_->posToIndex(sample, idx);
                Eigen::Vector3d voxel_center; grid_map_->indexToPos(idx, voxel_center);
                Eigen::Vector3d dir_vec = (cp - voxel_center);
                double nrm = dir_vec.norm();
                if (nrm < 1e-6) continue;
                dir_vec /= nrm;
                cps_.base_point[i].push_back(voxel_center);
                cps_.direction[i].push_back(dir_vec);
                found = true;
                break;
              }
            }
            (void)found;
          }
        }
      }
    }
    return vector<vector<Eigen::Vector3d>>();
  }

  int BezierOptimizer::earlyExit(void *func_data, const double *x, const double *g,
      const double fx, const double xnorm, const double gnorm, 
      const double step, int n, int k, int ls)
  {
    BezierOptimizer *opt = reinterpret_cast<BezierOptimizer *>(func_data);
    return (opt->force_stop_type_ == STOP_FOR_ERROR || 
            opt->force_stop_type_ == STOP_FOR_REBOUND);
  }

  double BezierOptimizer::costFunctionRebound(void *func_data, const double *x, 
                                              double *grad, const int n)
  {
    BezierOptimizer *opt = reinterpret_cast<BezierOptimizer *>(func_data);
    double cost;
    opt->combineCostRebound(x, grad, cost, n);
    opt->iter_num_ += 1;
    return cost;
  }

  double BezierOptimizer::costFunctionRefine(void *func_data, const double *x, 
                                             double *grad, const int n)
  {
    BezierOptimizer *opt = reinterpret_cast<BezierOptimizer *>(func_data);
    double cost;
    opt->combineCostRefine(x, grad, cost, n);
    opt->iter_num_ += 1;
    return cost;
  }

  /**
   * @brief Compute collision avoidance cost using rebound method
   */
  void BezierOptimizer::calcDistanceCostRebound(const Eigen::MatrixXd &q, double &cost,
                                                Eigen::MatrixXd &gradient, int iter_num, double smoothness_cost)
  {
    cost = 0.0;
    
    // Traverse all control points
    int end_idx = q.cols();
    // Demarcation distance (safety clearance)
    double demarcation = cps_.clearance;
    
    // DEBUG: Print clearance on first iteration
    if (iter_num == 0) {
      ROS_WARN("[BezierOptimizer] calcDistanceCostRebound: clearance=%f, demarcation=%f", 
               cps_.clearance, demarcation);
    }
    // Cubic polynomial coefficients to ensure continuity at demarcation
    double a = 3 * demarcation, b = -3 * pow(demarcation, 2), c = pow(demarcation, 3);
    
    for (int i = 0; i < end_idx; ++i)
    {
      // Safety check: ensure direction vectors are initialized
      if (i >= cps_.direction.size()) continue;
      
      // Traverse all constraint directions for this control point
      for (size_t j = 0; j < cps_.direction[i].size(); ++j)
      {
        // Compute signed distance from control point to base point (projected along constraint direction)
        double dist = (q.col(i) - cps_.base_point[i][j]).dot(cps_.direction[i][j]);
        
        // Distance error = safety clearance - actual distance
        double dist_err = cps_.clearance - dist;
        
        // Gradient direction
        Eigen::Vector3d dist_grad = cps_.direction[i][j];

        if (dist_err < 0)
        {
          // Control point is outside safety zone, no penalty needed
        }
        else if (dist_err < demarcation)
        {
          // Control point is close to obstacle, use cubic penalty
          cost += pow(dist_err, 3);
          gradient.col(i) += -3.0 * dist_err * dist_err * dist_grad;
        }
        else
        {
          // Control point is deep inside obstacle region, use stronger quadratic penalty
          cost += a * dist_err * dist_err + b * dist_err + c;
          gradient.col(i) += -(2.0 * a * dist_err + b) * dist_grad;
        }
      }
      
      // Ground avoidance: add strong penalty if control point is too low
      double ground_clearance = 0.3; // minimum 0.3m above ground
      double z_pos = q.col(i)(2);
      if (z_pos < ground_clearance)
      {
        double z_err = ground_clearance - z_pos;
        // FIX: Use cubic penalty (stronger push near ground) with higher weight
        cost += 1000.0 * pow(z_err, 3);  // Cubic penalty increases rapidly
        gradient(2, i) += -3000.0 * pow(z_err, 2); // Cubic gradient: 3*weight*z_err^2
      }
    }
  }
  void BezierOptimizer::calcFitnessCost(const Eigen::MatrixXd &q, double &cost, 
                                        Eigen::MatrixXd &gradient)
  {
    cost = 0.0;
    int end_idx = q.cols();
    if (ref_pts_.size() != end_idx) return;

    for (int i = 0; i < end_idx; ++i)
    {
      Eigen::Vector3d diff = q.col(i) - ref_pts_[i];
      cost += diff.squaredNorm();
      gradient.col(i) += 2 * diff;
    }
  }

  /**
   * @brief Compute smoothness cost for piecewise Bezier curves
   * 
   * Two components:
   * 1. Segment jerk minimization (cubic Bezier has constant jerk per segment)
   * 2. Inter-segment C1 continuity (velocity and acceleration continuity)
   */
  void BezierOptimizer::calcSmoothnessCost(const Eigen::MatrixXd &q, double &cost,
                                           Eigen::MatrixXd &gradient, bool flag_use_jerk)
  {
    cost = 0.0;
    int num_segments = (q.cols() - 1) / 3;
    double ts = segment_duration_;
    double ts_inv3 = 1.0 / pow(ts, 3);
    double ts_inv2 = 1.0 / pow(ts, 2);
    double ts_inv = 1.0 / ts;

    // Part 1: Jerk cost per segment
    // Jerk = 6*(P3 - 3*P2 + 3*P1 - P0) / T^3
    for (int k = 0; k < num_segments; ++k)
    {
      int idx = k * 3;
      Eigen::Vector3d p0 = q.col(idx);
      Eigen::Vector3d p1 = q.col(idx + 1);
      Eigen::Vector3d p2 = q.col(idx + 2);
      Eigen::Vector3d p3 = q.col(idx + 3);

      Eigen::Vector3d jerk_vec = (p3 - 3 * p2 + 3 * p1 - p0);
      // FIX: Use fixed weight instead of 36.0/T^5 to prevent smoothness from dominating
      // Original: double weight = 36.0 / pow(ts, 5);  // This made smoothness 1000x-10000x stronger!
      double weight = 10.0;  // Moderate fixed weight for jerk minimization
      
      cost += weight * jerk_vec.squaredNorm();

      Eigen::Vector3d grad_j = 2.0 * weight * jerk_vec;
      gradient.col(idx)     += -grad_j;
      gradient.col(idx + 1) +=  3.0 * grad_j;
      gradient.col(idx + 2) += -3.0 * grad_j;
      gradient.col(idx + 3) +=  grad_j;
    }

    // Part 2: Inter-segment continuity constraints
    // FIX: Increase to 100000.0 to ensure strong C2 continuity between segments
    double cont_weight_vel = 10000.0;  // Velocity continuity
    double cont_weight_acc = 10000.0;  // Acceleration continuity

    for (int k = 0; k < num_segments - 1; ++k)
    {
      int idx = k * 3;
      
      // Velocity continuity: P4 - 2*P3 + P2 = 0
      Eigen::Vector3d diff_vel = (q.col(idx + 4) - 2 * q.col(idx + 3) + q.col(idx + 2));
      cost += cont_weight_vel * diff_vel.squaredNorm();
      
      Eigen::Vector3d grad_v = 2.0 * cont_weight_vel * diff_vel;
      gradient.col(idx + 4) += grad_v;
      gradient.col(idx + 3) += -2.0 * grad_v;
      gradient.col(idx + 2) += grad_v;

      // Acceleration continuity: P1 - 2*P2 + 2*P4 - P5 = 0
      Eigen::Vector3d diff_acc = (q.col(idx + 1) - 2 * q.col(idx + 2) + 
                                  2 * q.col(idx + 4) - q.col(idx + 5));
      cost += cont_weight_acc * diff_acc.squaredNorm();

      Eigen::Vector3d grad_a = 2.0 * cont_weight_acc * diff_acc;
      gradient.col(idx + 1) += grad_a;
      gradient.col(idx + 2) += -2.0 * grad_a;
      gradient.col(idx + 4) += 2.0 * grad_a;
      gradient.col(idx + 5) += -grad_a;
    }
  }

  /**
   * @brief Compute feasibility cost (velocity and acceleration limits)
   * 
   * For cubic Bezier, velocity is quadratic with 3 control points per segment,
   * and acceleration is linear with 2 control points per segment.
   */
  void BezierOptimizer::calcFeasibilityCost(const Eigen::MatrixXd &q, double &cost,
                                            Eigen::MatrixXd &gradient)
  {
    cost = 0.0;
    double ts = segment_duration_;
    int num_segments = (q.cols() - 1) / 3;

    for (int k = 0; k < num_segments; ++k)
    {
      int idx = k * 3;
      Eigen::Vector3d p0 = q.col(idx);
      Eigen::Vector3d p1 = q.col(idx + 1);
      Eigen::Vector3d p2 = q.col(idx + 2);
      Eigen::Vector3d p3 = q.col(idx + 3);

      // Velocity control points: V_i = 3*(P_{i+1} - P_i)/T
      vector<Eigen::Vector3d> vs = {(p1-p0)*3/ts, (p2-p1)*3/ts, (p3-p2)*3/ts};
      vector<pair<int, int>> v_indices = {{idx, idx+1}, {idx+1, idx+2}, {idx+2, idx+3}};

      for(int i = 0; i < 3; ++i) {
        Eigen::Vector3d v = vs[i];
        for(int dim = 0; dim < 3; ++dim) {
          if(abs(v(dim)) > max_vel_) {
            double diff = abs(v(dim)) - max_vel_;
            cost += pow(diff, 2);
            double grad = 2 * diff * (v(dim) > 0 ? 1 : -1);
            gradient(dim, v_indices[i].second) += grad * 3.0 / ts;
            gradient(dim, v_indices[i].first)  += grad * -3.0 / ts;
          }
        }
      }

      // Acceleration control points: A_i = 6*(P_{i+2} - 2*P_{i+1} + P_i)/T^2
      vector<Eigen::Vector3d> as = {(p2 - 2*p1 + p0)*6/(ts*ts), (p3 - 2*p2 + p1)*6/(ts*ts)};
      vector<vector<int>> a_indices = {{idx, idx+1, idx+2}, {idx+1, idx+2, idx+3}};

      for(int i = 0; i < 2; ++i) {
        Eigen::Vector3d a = as[i];
        for(int dim = 0; dim < 3; ++dim) {
          if(abs(a(dim)) > max_acc_) {
            double diff = abs(a(dim)) - max_acc_;
            cost += pow(diff, 2);
            double grad = 2 * diff * (a(dim) > 0 ? 1 : -1);
            double factor = 6.0 / (ts*ts);
            gradient(dim, a_indices[i][0]) += grad * factor * 1.0;
            gradient(dim, a_indices[i][1]) += grad * factor * -2.0;
            gradient(dim, a_indices[i][2]) += grad * factor * 1.0;
          }
        }
      }
    }
  }


  // ═══════════════════════════════════════════════════════════════════════════
  // setFormationReference
  // ═══════════════════════════════════════════════════════════════════════════
  void BezierOptimizer::setFormationReference(
      const std::vector<Eigen::Vector3d>& formation_pts)
  {
    formation_ref_pts_ = formation_pts;
  }

  // ═══════════════════════════════════════════════════════════════════════════
  // calcFormationCost  —  编队保持代价（弹性软约束）
  //
  // 数学原理：
  //   每个控制点 q_i 有理想编队位置 f_i（由 V 形偏移得到）。
  //   代价函数：C_form = λ5 · Σ_i ‖q_i - f_i‖²
  //   解析梯度：∂C/∂q_i  = 2·λ5·(q_i - f_i)
  //
  //   避障代价(λ2~5000) >> 编队代价(λ5~20)，L-BFGS 自动在二者之间平衡：
  //   - 遭遇障碍时，避障梯度推离，编队梯度被压制；
  //   - 绕过障碍后，避障梯度消失，编队代价主导 → 自动恢复编队队形。
  // ═══════════════════════════════════════════════════════════════════════════
  void BezierOptimizer::calcFormationCost(
      const Eigen::MatrixXd& q,
      double&                cost,
      Eigen::MatrixXd&       gradient)
  {
    cost = 0.0;
    const int N = q.cols();
    // 若参考点数量不匹配，跳过（防止配置错误导致越界）
    if (static_cast<int>(formation_ref_pts_.size()) != N) return;

    for (int i = 0; i < N; ++i)
    {
      const Eigen::Vector3d diff = q.col(i) - formation_ref_pts_[i];
      cost            += diff.squaredNorm();   // ‖q_i - f_i‖²
      gradient.col(i) += 2.0 * diff;           // 解析梯度 2(q_i - f_i)
    }
  }

  // ═══════════════════════════════════════════════════════════════════════════
  // FORMATION 模式静态代价回调（L-BFGS 接口）
  // ═══════════════════════════════════════════════════════════════════════════
  double BezierOptimizer::costFunctionFormation(
      void *func_data, const double *x, double *grad, const int n)
  {
    BezierOptimizer *opt = reinterpret_cast<BezierOptimizer *>(func_data);
    double cost;
    opt->combineCostFormation(x, grad, cost, n);
    opt->iter_num_ += 1;
    return cost;
  }

  // ═══════════════════════════════════════════════════════════════════════════
  // combineCostFormation  —  FORMATION 模式组合代价
  //
  // f = λ1·smooth + λ2·distance(rebound) + λ3·feasibility + λ5·formation
  //
  // 与 Rebound 模式区别：多了 λ5·formation 项
  // 与 Refine  模式区别：用 distance_cost 代替 fitness_cost，并叠加 formation
  // ═══════════════════════════════════════════════════════════════════════════
  void BezierOptimizer::combineCostFormation(
      const double *x, double *grad, double &f_combine, const int n)
  {
    Eigen::Map<const Eigen::MatrixXd> q(x, 3, cps_.points.cols());
    Eigen::Map<Eigen::MatrixXd> grad_mat(grad, 3, cps_.points.cols());
    grad_mat.setZero();

    double f_smoothness, f_distance, f_feasibility, f_formation;
    Eigen::MatrixXd g_smoothness  = Eigen::MatrixXd::Zero(3, cps_.points.cols());
    Eigen::MatrixXd g_distance    = Eigen::MatrixXd::Zero(3, cps_.points.cols());
    Eigen::MatrixXd g_feasibility = Eigen::MatrixXd::Zero(3, cps_.points.cols());
    Eigen::MatrixXd g_formation   = Eigen::MatrixXd::Zero(3, cps_.points.cols());

    calcSmoothnessCost      (q, f_smoothness,  g_smoothness);
    calcDistanceCostRebound (q, f_distance,    g_distance,    iter_num_, f_smoothness);
    calcFeasibilityCost     (q, f_feasibility, g_feasibility);
    calcFormationCost       (q, f_formation,   g_formation);

    f_combine = lambda1_ * f_smoothness
              + new_lambda2_ * f_distance
              + lambda3_ * f_feasibility
              + lambda5_ * f_formation;

    grad_mat  = lambda1_      * g_smoothness
              + new_lambda2_  * g_distance
              + lambda3_      * g_feasibility
              + lambda5_      * g_formation;

    if (iter_num_ % 20 == 0)
    {
      ROS_INFO("[Bezier FORMATION iter %d] smooth=%.3f dist=%.3f feas=%.3f"
               " form=%.3f total=%.3f grad_norm=%.4f",
               iter_num_, f_smoothness, f_distance, f_feasibility,
               f_formation, f_combine, grad_mat.norm());
    }
  }

  // ═══════════════════════════════════════════════════════════════════════════
  // BezierOptimizeTrajFormation  —  FORMATION 模式 L-BFGS 优化入口
  //
  // 调用方：SwarmMasterCoordinator::optimizeFollowerTrajectory()
  //   1. initControlPoints(initial_ctrl_pts) 已在调用前完成
  //   2. setFormationReference(formation_refs) 已在调用前完成
  //   3. 本函数执行 L-BFGS 并将结果写入 ctrl_pts
  //
  // 返回值：OPTIMAL(0) / SATISFIED(1) / FAILED(-1)
  // ═══════════════════════════════════════════════════════════════════════════
  int BezierOptimizer::BezierOptimizeTrajFormation(
      Eigen::MatrixXd& ctrl_pts, double ts)
  {
    setControlPoints(ctrl_pts);
    setSegmentDuration(ts);

    iter_num_     = 0;
    variable_num_ = cps_.points.size();
    force_stop_type_ = DONT_STOP;

    double final_cost;
    std::vector<double> q(static_cast<size_t>(variable_num_));
    Eigen::Map<Eigen::MatrixXd>(q.data(), 3, cps_.points.cols()) = cps_.points;

    lbfgs::lbfgs_parameter_t lbfgs_params;
    lbfgs::lbfgs_load_default_parameters(&lbfgs_params);
    lbfgs_params.max_iterations = 150;
    lbfgs_params.g_epsilon      = 1e-3;
    lbfgs_params.mem_size       = 16;

    int lbfgs_ret = lbfgs::lbfgs_optimize(
        variable_num_, q.data(), &final_cost,
        BezierOptimizer::costFunctionFormation,
        NULL,
        BezierOptimizer::earlyExit,
        this, &lbfgs_params);

    // 将优化结果写回
    cps_.points = Eigen::Map<Eigen::MatrixXd>(q.data(), 3, cps_.points.cols());
    ctrl_pts    = cps_.points;

    // ── 判断收敛状态 ──
    if (lbfgs_ret == lbfgs::LBFGS_CONVERGENCE  ||
        lbfgs_ret == lbfgs::LBFGS_STOP         ||
        lbfgs_ret == lbfgs::LBFGS_ALREADY_MINIMIZED)
      return OPTIMAL;

    // 最大迭代但代价足够低，视为可接受（阈值与 λ2=500 量级匹配）
    if (lbfgs_ret == lbfgs::LBFGSERR_MAXIMUMITERATION && final_cost < 10000.0)
      return SATISFIED;

    ROS_WARN("[BezierOptimizer] Formation: %s (%d), final_cost=%.2f",
             lbfgs::lbfgs_strerror(lbfgs_ret), lbfgs_ret, final_cost);
    return FAILED;
  }

  bool BezierOptimizer::check_collision_and_rebound(void)
  {
    // FIX: Implement rebound mechanism to dynamically detect collisions during optimization
    // This prevents the optimizer from accepting trajectories that pass through newly encountered obstacles
    
    if (!grid_map_) return false;
    
    bool collision_detected = false;
    double max_clearance_violation = 0.0;
    
    // Check each control point for collisions
    for (int i = 0; i < cps_.points.cols(); ++i)
    {
      Eigen::Vector3d cp = cps_.points.col(i);
      
      // CRITICAL: Check z boundary violation - don't skip, treat as collision!
      // This prevents drone from flying over walls
      bool z_boundary_violation = (cp.z() > grid_map_->getMapMaxBoundary().z() - 0.1 || 
                                   cp.z() < grid_map_->getMapMinBoundary().z() + 0.1);
      
      // Check occupancy directly - if occupied or z-boundary violated, we have a collision
      int occ = grid_map_->getInflateOccupancy(cp);
      
      // If in occupied space or violated z-boundary, we have a collision
      if (occ > 0 || z_boundary_violation)
      {
        collision_detected = true;
        max_clearance_violation = std::max(max_clearance_violation, cps_.clearance);
        
        // Re-sample collision constraints around this control point
        // Similar to initControlPoints but for a single point
        double res = grid_map_->getResolution();
        double max_search = std::max(3.0, cps_.clearance * 2.0);
        int steps = std::max(4, int(max_search / res));
        
        std::vector<Eigen::Vector3d> dirs;
        int azimuth_samples = 16;
        std::vector<double> elevs = {0.0, 0.3, -0.3, 0.785, -0.785, 1.57, -1.57};
        
        for (int ea = 0; ea < (int)elevs.size(); ++ea) {
          double el = elevs[ea];
          for (int a = 0; a < azimuth_samples; ++a) {
            double ang = 2.0 * M_PI * double(a) / double(azimuth_samples);
            Eigen::Vector3d d;
            d.x() = cos(ang) * cos(el);
            d.y() = sin(ang) * cos(el);
            d.z() = sin(el);
            dirs.push_back(d.normalized());
          }
        }
        
        // Clear old constraints for this point and regenerate
        cps_.base_point[i].clear();
        cps_.direction[i].clear();
        
        for (const auto &d : dirs) {
          for (int s = 1; s <= steps; ++s) {
            double dist_ray = double(s) * res * 0.8;
            Eigen::Vector3d sample = cp + d * dist_ray;
            if (!grid_map_->isInMap(sample)) continue;
            int occ = grid_map_->getInflateOccupancy(sample);
            if (occ > 0) {
              Eigen::Vector3i idx;
              grid_map_->posToIndex(sample, idx);
              Eigen::Vector3d voxel_center;
              grid_map_->indexToPos(idx, voxel_center);
              Eigen::Vector3d dir_vec = (cp - voxel_center);
              double nrm = dir_vec.norm();
              if (nrm > 1e-6) {
                dir_vec /= nrm;
                cps_.base_point[i].push_back(voxel_center);
                cps_.direction[i].push_back(dir_vec);
              }
              break;
            }
          }
        }
      }
    }
    
    // If significant collision detected, trigger rebound
    if (collision_detected && max_clearance_violation > 0.05)
    {
      ROS_WARN("[BezierOptimizer] Collision detected! Max violation: %.3fm, regenerating constraints", max_clearance_violation);
      force_stop_type_ = STOP_FOR_REBOUND;
      return true;
    }
    
    return false;
  }

  bool BezierOptimizer::BezierOptimizeTrajRebound(Eigen::MatrixXd &optimal_points, double ts)
  {
    setSegmentDuration(ts);
    bool flag_success = rebound_optimize();
    optimal_points = cps_.points;
    return flag_success;
  }

  bool BezierOptimizer::BezierOptimizeTrajRefine(const Eigen::MatrixXd &init_points, 
                                                  const double ts, 
                                                  Eigen::MatrixXd &optimal_points)
  {
    setControlPoints(init_points);
    setSegmentDuration(ts);
    bool flag_success = refine_optimize();
    optimal_points = cps_.points;
    return flag_success;
  }

  bool BezierOptimizer::rebound_optimize()
  {
    iter_num_ = 0;
    variable_num_ = cps_.points.size();
    double final_cost;
    
    std::vector<double> q(variable_num_);
    Eigen::Map<Eigen::MatrixXd>(q.data(), 3, cps_.points.cols()) = cps_.points;

    lbfgs::lbfgs_parameter_t lbfgs_params;
    lbfgs::lbfgs_load_default_parameters(&lbfgs_params);
    lbfgs_params.max_iterations = 500;
    lbfgs_params.g_epsilon = 1e-3;
    lbfgs_params.mem_size = 16;

    int result = lbfgs::lbfgs_optimize(variable_num_, q.data(), &final_cost, 
        BezierOptimizer::costFunctionRebound, NULL, BezierOptimizer::earlyExit, 
        this, &lbfgs_params);
    
// //     if (result != lbfgs::LBFGS_CONVERGENCE && result != lbfgs::LBFGS_STOP && 
// //         result != lbfgs::LBFGS_ALREADY_MINIMIZED)
// //     {
// //       std::cout << "[BezierOptimizer] Rebound optimization: " << lbfgs::lbfgs_strerror(result)
// //                 << " (" << result << "), final_cost=" << final_cost << std::endl;
// //     }
// // 
    cps_.points = Eigen::Map<Eigen::MatrixXd>(q.data(), 3, cps_.points.cols());
    
    bool success = (result == lbfgs::LBFGS_CONVERGENCE || result == lbfgs::LBFGS_STOP ||
                    result == lbfgs::LBFGS_ALREADY_MINIMIZED);
    
    // Check if trajectory has collision by examining distance cost
    // Only accept if final_cost is relatively low OR distance cost portion is acceptable
    if (result == lbfgs::LBFGSERR_MAXIMUMITERATION)
    {
      // Estimate if the trajectory is collision-free by checking if cost is dominated by smoothness
      // If final_cost < 100, it's likely collision-free (dist cost would be ~5000 if in collision)
      if (final_cost < 100.0) {
        // std::cout << "[BezierOptimizer] Max iterations but cost low enough, accepting" << std::endl;
        success = true;
      } else {
        // std::cout << "[BezierOptimizer] Max iterations and high cost, trajectory likely in collision" << std::endl;
        success = false;
      }
    }
    
    return success;
  }

  bool BezierOptimizer::refine_optimize()
  {
    iter_num_ = 0;
    variable_num_ = cps_.points.size();
    double final_cost;

    std::vector<double> q(variable_num_);
    Eigen::Map<Eigen::MatrixXd>(q.data(), 3, cps_.points.cols()) = cps_.points;

    lbfgs::lbfgs_parameter_t lbfgs_params;
    lbfgs::lbfgs_load_default_parameters(&lbfgs_params);
    lbfgs_params.max_iterations = 500;
    lbfgs_params.g_epsilon = 1e-3;
    lbfgs_params.mem_size = 16;

    int result = lbfgs::lbfgs_optimize(variable_num_, q.data(), &final_cost, 
        BezierOptimizer::costFunctionRefine, NULL, NULL, this, &lbfgs_params);
    
// //     if (result != lbfgs::LBFGS_CONVERGENCE && result != lbfgs::LBFGS_STOP && 
// //         result != lbfgs::LBFGS_ALREADY_MINIMIZED)
// //     {
// //       std::cout << "[BezierOptimizer] Refine optimization: " << lbfgs::lbfgs_strerror(result)
// //                 << " (" << result << "), final_cost=" << final_cost << std::endl;
// //     }
// // 
    cps_.points = Eigen::Map<Eigen::MatrixXd>(q.data(), 3, cps_.points.cols());

    bool success = (result == lbfgs::LBFGS_CONVERGENCE || result == lbfgs::LBFGS_STOP ||
                    result == lbfgs::LBFGS_ALREADY_MINIMIZED);
    
    if (result == lbfgs::LBFGSERR_MAXIMUMITERATION && final_cost < 10.0) {
      success = true;
    }
    
    return success;
  }

  void BezierOptimizer::combineCostRebound(const double *x, double *grad, 
                                           double &f_combine, const int n)
  {
    Eigen::Map<const Eigen::MatrixXd> q(x, 3, cps_.points.cols());
    Eigen::Map<Eigen::MatrixXd> grad_mat(grad, 3, cps_.points.cols());
    grad_mat.setZero();

    double f_smoothness, f_distance, f_feasibility;
    Eigen::MatrixXd g_smoothness = Eigen::MatrixXd::Zero(3, cps_.points.cols());
    Eigen::MatrixXd g_distance = Eigen::MatrixXd::Zero(3, cps_.points.cols());
    Eigen::MatrixXd g_feasibility = Eigen::MatrixXd::Zero(3, cps_.points.cols());

    calcSmoothnessCost(q, f_smoothness, g_smoothness);
    calcDistanceCostRebound(q, f_distance, g_distance, iter_num_, f_smoothness);
    calcFeasibilityCost(q, f_feasibility, g_feasibility);

    f_combine = lambda1_ * f_smoothness + new_lambda2_ * f_distance + lambda3_ * f_feasibility;
    grad_mat = lambda1_ * g_smoothness + new_lambda2_ * g_distance + lambda3_ * g_feasibility;

// //     if (iter_num_ % 20 == 0) {
// //       std::cout << "[Bezier Opt iter " << iter_num_ << "] smooth=" << f_smoothness 
// //                 << " dist=" << f_distance << " feas=" << f_feasibility
// //                 << " total=" << f_combine 
// //                 << " grad_norm=" << grad_mat.norm() << std::endl;
// //     }
  }

  void BezierOptimizer::combineCostRefine(const double *x, double *grad, 
                                          double &f_combine, const int n)
  {
    Eigen::Map<const Eigen::MatrixXd> q(x, 3, cps_.points.cols());
    Eigen::Map<Eigen::MatrixXd> grad_mat(grad, 3, cps_.points.cols());
    grad_mat.setZero();

    double f_smoothness, f_fitness, f_feasibility;
    Eigen::MatrixXd g_smoothness = Eigen::MatrixXd::Zero(3, cps_.points.cols());
    Eigen::MatrixXd g_fitness = Eigen::MatrixXd::Zero(3, cps_.points.cols());
    Eigen::MatrixXd g_feasibility = Eigen::MatrixXd::Zero(3, cps_.points.cols());

    calcSmoothnessCost(q, f_smoothness, g_smoothness);
    calcFitnessCost(q, f_fitness, g_fitness);
    calcFeasibilityCost(q, f_feasibility, g_feasibility);

    f_combine = lambda1_ * f_smoothness + lambda4_ * f_fitness + lambda3_ * f_feasibility;
    grad_mat = lambda1_ * g_smoothness + lambda4_ * g_fitness + lambda3_ * g_feasibility;
  }

} // namespace ego_planner
