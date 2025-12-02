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
      cps_.resize(init_points.cols());
      cps_.points = init_points;
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
    int end_idx = q.cols();
    double demarcation = cps_.clearance;
    double a = 3 * demarcation, b = -3 * pow(demarcation, 2), c = pow(demarcation, 3);

    for (int i = 0; i < end_idx; ++i)
    {
      if (i >= cps_.direction.size()) continue;
      
      for (size_t j = 0; j < cps_.direction[i].size(); ++j)
      {
        double dist = (q.col(i) - cps_.base_point[i][j]).dot(cps_.direction[i][j]);
        double dist_err = cps_.clearance - dist;
        Eigen::Vector3d dist_grad = cps_.direction[i][j];

        if (dist_err < 0) {
          // Safe region - no penalty
        }
        else if (dist_err < demarcation) {
          // Approaching obstacle - cubic penalty
          cost += pow(dist_err, 3);
          gradient.col(i) += -3.0 * dist_err * dist_err * dist_grad;
        }
        else {
          // Deep in obstacle - stronger quadratic penalty
          cost += a * dist_err * dist_err + b * dist_err + c;
          gradient.col(i) += -(2.0 * a * dist_err + b) * dist_grad;
        }
      }
    }
  }

  /**
   * @brief Compute fitness cost (deviation from reference trajectory)
   */
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
      double weight = 36.0 / pow(ts, 5);
      
      cost += weight * jerk_vec.squaredNorm();

      Eigen::Vector3d grad_j = 2.0 * weight * jerk_vec;
      gradient.col(idx)     += -grad_j;
      gradient.col(idx + 1) +=  3.0 * grad_j;
      gradient.col(idx + 2) += -3.0 * grad_j;
      gradient.col(idx + 3) +=  grad_j;
    }

    // Part 2: Inter-segment continuity constraints
    double cont_weight_vel = 1000.0;
    double cont_weight_acc = 1000.0;

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

  bool BezierOptimizer::check_collision_and_rebound(void)
  {
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
    
    if (result != lbfgs::LBFGS_CONVERGENCE && result != lbfgs::LBFGS_STOP && 
        result != lbfgs::LBFGS_ALREADY_MINIMIZED)
    {
      std::cout << "[BezierOptimizer] Rebound optimization: " << lbfgs::lbfgs_strerror(result)
                << " (" << result << "), final_cost=" << final_cost << std::endl;
    }

    cps_.points = Eigen::Map<Eigen::MatrixXd>(q.data(), 3, cps_.points.cols());
    
    bool success = (result == lbfgs::LBFGS_CONVERGENCE || result == lbfgs::LBFGS_STOP ||
                    result == lbfgs::LBFGS_ALREADY_MINIMIZED);
    
    if (result == lbfgs::LBFGSERR_MAXIMUMITERATION && final_cost < 10.0) {
      std::cout << "[BezierOptimizer] Max iterations reached but cost is low, accepting result" << std::endl;
      success = true;
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
    
    if (result != lbfgs::LBFGS_CONVERGENCE && result != lbfgs::LBFGS_STOP && 
        result != lbfgs::LBFGS_ALREADY_MINIMIZED)
    {
      std::cout << "[BezierOptimizer] Refine optimization: " << lbfgs::lbfgs_strerror(result)
                << " (" << result << "), final_cost=" << final_cost << std::endl;
    }

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

    if (iter_num_ % 20 == 0) {
      std::cout << "[Bezier Opt iter " << iter_num_ << "] smooth=" << f_smoothness 
                << " dist=" << f_distance << " feas=" << f_feasibility
                << " total=" << f_combine 
                << " grad_norm=" << grad_mat.norm() << std::endl;
    }
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
