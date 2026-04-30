/**
 * @file piecewise_bezier.cpp
 * @brief Implementation of Piecewise Cubic Bezier Curve for trajectory representation
 * 
 * This file implements a trajectory representation using piecewise cubic Bezier curves.
 * Each Bezier segment has 4 control points and connects smoothly at junction points.
 * 
 * Key properties of cubic Bezier curves:
 * - Position: B(t) = (1-t)^3*P0 + 3*(1-t)^2*t*P1 + 3*(1-t)*t^2*P2 + t^3*P3
 * - Velocity at endpoints: V(0) = 3*(P1-P0)/T, V(T) = 3*(P3-P2)/T
 * - Acceleration at endpoints: A(0) = 6*(P2-2*P1+P0)/T^2
 */

#include "bezier_opt/piecewise_bezier.h"
#include <ros/ros.h>
#include <cmath>

namespace ego_planner
{

  PiecewiseBezier::PiecewiseBezier(const Eigen::MatrixXd &points, const int &order,
                                   const double &interval)
  {
    setBezierCurve(points, order, interval);
  }

  PiecewiseBezier::~PiecewiseBezier() {}

  void PiecewiseBezier::setBezierCurve(const Eigen::MatrixXd &points, const int &order,
                                       const double &interval)
  {
    control_points_ = points;
    p_ = order;  // Bezier degree (3 for cubic)
    interval_ = interval;  // Duration per segment
    n_ = points.cols() - 1;  // Number of control points - 1
  }

  void PiecewiseBezier::setKnot(const Eigen::VectorXd &knot) {
    // Knot vector is not used for piecewise Bezier (kept for interface compatibility)
  }

  Eigen::VectorXd PiecewiseBezier::getKnot() {
    // Return empty vector - Bezier curves don't use explicit knot vectors
    return Eigen::VectorXd();
  }

  bool PiecewiseBezier::getTimeSpan(double &t_start, double &t_end)
  {
    t_start = 0.0;
    // Total duration = number of segments * segment duration
    int num_segments = n_ / p_;
    t_end = num_segments * interval_;
    return true;
  }

  Eigen::MatrixXd PiecewiseBezier::getControlPoint() { return control_points_; }

  /**
   * @brief Evaluate the Bezier curve at time t
   * @param t Time parameter in [0, total_duration]
   * @return 3D position at time t
   * 
   * Algorithm:
   * 1. Determine which segment t falls into
   * 2. Compute local parameter within segment [0, 1]
   * 3. Apply Bernstein polynomial evaluation
   */
  Eigen::VectorXd PiecewiseBezier::evaluate(const double &u)
  {
    double t = u;
    int num_segments = n_ / p_;
    double total_duration = num_segments * interval_;

    // Clamp t to valid range
    if (t < 0) t = 0;
    if (t > total_duration) t = total_duration;

    // Find segment index
    int segment_idx = floor(t / interval_);
    if (segment_idx >= num_segments) segment_idx = num_segments - 1;

    // Compute local parameter in [0, 1]
    double local_t = (t - segment_idx * interval_) / interval_;

    // Starting control point index for this segment
    int start_idx = segment_idx * p_;

    if (p_ == 3) {
      // Cubic Bezier evaluation
      if (start_idx + 3 >= control_points_.cols()) {
        start_idx = control_points_.cols() - 4;
        if (start_idx < 0) start_idx = 0;
      }

      Eigen::Vector3d p0 = control_points_.col(start_idx);
      Eigen::Vector3d p1 = control_points_.col(start_idx + 1);
      Eigen::Vector3d p2 = control_points_.col(start_idx + 2);
      Eigen::Vector3d p3 = control_points_.col(start_idx + 3);

      // Bernstein polynomials for cubic Bezier
      double mt = 1 - local_t;
      double b0 = mt * mt * mt;
      double b1 = 3 * mt * mt * local_t;
      double b2 = 3 * mt * local_t * local_t;
      double b3 = local_t * local_t * local_t;

      return p0 * b0 + p1 * b1 + p2 * b2 + p3 * b3;
    }
    else if (p_ == 2) {
      // Quadratic Bezier (for velocity curve)
      if (start_idx + 2 >= control_points_.cols()) {
        start_idx = control_points_.cols() - 3;
        if (start_idx < 0) start_idx = 0;
      }

      Eigen::Vector3d p0 = control_points_.col(start_idx);
      Eigen::Vector3d p1 = control_points_.col(start_idx + 1);
      Eigen::Vector3d p2 = control_points_.col(start_idx + 2);

      double mt = 1 - local_t;
      double b0 = mt * mt;
      double b1 = 2 * mt * local_t;
      double b2 = local_t * local_t;

      return p0 * b0 + p1 * b1 + p2 * b2;
    }
    else if (p_ == 1) {
      // Linear Bezier (for acceleration curve)
      if (start_idx + 1 >= control_points_.cols()) {
        start_idx = control_points_.cols() - 2;
        if (start_idx < 0) start_idx = 0;
      }

      Eigen::Vector3d p0 = control_points_.col(start_idx);
      Eigen::Vector3d p1 = control_points_.col(start_idx + 1);

      double mt = 1 - local_t;
      return p0 * mt + p1 * local_t;
    }

    return Eigen::Vector3d::Zero();
  }

  /**
   * @brief Compute the derivative Bezier curve
   * @return A PiecewiseBezier representing the derivative (velocity or acceleration)
   * 
   * For cubic Bezier, derivative is quadratic with control points:
   * Q_i = 3 * (P_{i+1} - P_i) / T
   */
  PiecewiseBezier PiecewiseBezier::getDerivative()
  {
    int num_segments = n_ / p_;
    int new_p = p_ - 1;
    if (new_p < 1) return *this;

    int new_cols = num_segments * new_p + 1;
    Eigen::MatrixXd deriv_points(3, new_cols);

    for (int k = 0; k < num_segments; ++k) {
      int p_idx = k * p_;
      int q_idx = k * new_p;

      Eigen::Vector3d p0 = control_points_.col(p_idx);
      Eigen::Vector3d p1 = control_points_.col(p_idx + 1);

      if (p_ == 3) {
        // Derivative of cubic Bezier
        Eigen::Vector3d p2 = control_points_.col(p_idx + 2);
        Eigen::Vector3d p3 = control_points_.col(p_idx + 3);

        deriv_points.col(q_idx)     = (p1 - p0) * p_ / interval_;
        deriv_points.col(q_idx + 1) = (p2 - p1) * p_ / interval_;
        deriv_points.col(q_idx + 2) = (p3 - p2) * p_ / interval_;
      } else if (p_ == 2) {
        // Derivative of quadratic Bezier
        Eigen::Vector3d p2 = control_points_.col(p_idx + 2);

        deriv_points.col(q_idx)     = (p1 - p0) * p_ / interval_;
        deriv_points.col(q_idx + 1) = (p2 - p1) * p_ / interval_;
      }
    }

    return PiecewiseBezier(deriv_points, new_p, interval_);
  }

  double PiecewiseBezier::getInterval() { return interval_; }

  void PiecewiseBezier::setPhysicalLimits(const double &vel, const double &acc, const double &tolerance)
  {
    limit_vel_ = vel;
    limit_acc_ = acc;
    limit_ratio_ = 1.1;
    feasibility_tolerance_ = tolerance;
  }

  /**
   * @brief Check if trajectory satisfies velocity and acceleration limits
   * @param ratio Output: factor needed to scale time to satisfy limits
   * @param show Whether to print violation details
   * @return true if feasible, false otherwise
   * 
   * For Bezier curves, we check the control points of velocity and acceleration curves,
   * as actual values are bounded by the convex hull of control points.
   */
  bool PiecewiseBezier::checkFeasibility(double &ratio, bool show)
  {
    bool fea = true;
    double max_vel = -1.0;
    double max_acc = -1.0;

    double enlarged_vel_lim = limit_vel_ * (1.0 + feasibility_tolerance_) + 1e-4;
    double enlarged_acc_lim = limit_acc_ * (1.0 + feasibility_tolerance_) + 1e-4;

    int num_segments = n_ / p_;

    for (int k = 0; k < num_segments; ++k) {
      int idx = k * p_;
      Eigen::Vector3d p0 = control_points_.col(idx);
      Eigen::Vector3d p1 = control_points_.col(idx + 1);
      Eigen::Vector3d p2 = control_points_.col(idx + 2);
      Eigen::Vector3d p3 = control_points_.col(idx + 3);

      // Velocity control points: V_i = 3*(P_{i+1} - P_i)/T
      Eigen::Vector3d v0 = (p1 - p0) * 3.0 / interval_;
      Eigen::Vector3d v1 = (p2 - p1) * 3.0 / interval_;
      Eigen::Vector3d v2 = (p3 - p2) * 3.0 / interval_;

      // Check velocity bounds
      for (const auto& v : {v0, v1, v2}) {
        if (v.cwiseAbs().maxCoeff() > enlarged_vel_lim) {
          if (show) cout << "[Bezier Check]: Velocity limit exceeded at segment " << k << ": " << v.transpose() << endl;
          fea = false;
        }
        max_vel = max(max_vel, v.cwiseAbs().maxCoeff());
      }

      // Acceleration control points: A_i = 6*(V_{i+1} - V_i)/T = 6*(P_{i+2} - 2*P_{i+1} + P_i)/T^2
      Eigen::Vector3d a0 = (v1 - v0) * 2.0 / interval_;
      Eigen::Vector3d a1 = (v2 - v1) * 2.0 / interval_;

      // Check acceleration bounds
      for (const auto& a : {a0, a1}) {
        if (a.cwiseAbs().maxCoeff() > enlarged_acc_lim) {
          if (show) cout << "[Bezier Check]: Acceleration limit exceeded at segment " << k << ": " << a.transpose() << endl;
          fea = false;
        }
        max_acc = max(max_acc, a.cwiseAbs().maxCoeff());
      }
    }

    // Compute required time scaling ratio
    ratio = max(max_vel / limit_vel_, sqrt(max_acc / limit_acc_));
    return fea;
  }

  void PiecewiseBezier::lengthenTime(const double &ratio)
  {
    interval_ *= ratio;
  }

  /**
   * @brief Convert waypoints to piecewise Bezier control points ensuring C1 continuity
   * @param ts Duration of each Bezier segment
   * @param point_set Waypoints (K points generate K-1 Bezier segments)
   * @param start_end_derivative Boundary conditions [start_vel, end_vel, start_acc, end_acc]
   * @param ctrl_pts Output: 3 x (3*(K-1)+1) matrix of control points
   * 
   * 关键修改：使用Catmull-Rom风格的切线计算，确保中间航点的平滑穿越
   * 
   * Bezier velocity property: V(0) = 3*(P1-P0)/T, V(T) = 3*(P3-P2)/T
   * Therefore: P1 = P0 + V*T/3, P2 = P3 - V*T/3
   */
  void PiecewiseBezier::parameterizeToBezier(const double &ts, const vector<Eigen::Vector3d> &point_set,
                                             const vector<Eigen::Vector3d> &start_end_derivative,
                                             Eigen::MatrixXd &ctrl_pts)
  {
    if (point_set.size() < 2) return;

    int K = point_set.size();           // Number of waypoints
    int num_segments = K - 1;           // Number of Bezier segments
    int num_ctrl_pts = num_segments * 3 + 1;  // Total control points

    ctrl_pts.resize(3, num_ctrl_pts);

    // Step 1: Set waypoints as segment endpoints (P0, P3, P6, ..., P_{3N})
    for (int i = 0; i < K; ++i) {
      ctrl_pts.col(i * 3) = point_set[i];
    }

    // Step 2: Compute tangent vectors at each waypoint using Catmull-Rom style
    // 关键修改：使用Catmull-Rom样条方法计算切线，确保更平滑的轨迹
    std::vector<Eigen::Vector3d> tangents(K);
    
    // 用于调整切线大小的张力参数 (0.5 是Catmull-Rom的标准值)
    const double tension = 0.5;

    for (int i = 0; i < K; ++i) {
      if (i == 0) {
        // Start point: use given start velocity or Catmull-Rom estimation
        if (start_end_derivative.size() >= 1 && start_end_derivative[0].norm() > 1e-6) {
          tangents[i] = start_end_derivative[0];
        } else {
          // Catmull-Rom: 使用第一段的方向
          tangents[i] = (point_set[1] - point_set[0]) / ts;
        }
      } else if (i == K - 1) {
        // End point: use given end velocity or Catmull-Rom estimation
        if (start_end_derivative.size() >= 2 && start_end_derivative[1].norm() > 1e-6) {
          tangents[i] = start_end_derivative[1];
        } else {
          // Catmull-Rom: 使用最后一段的方向
          tangents[i] = (point_set[K-1] - point_set[K-2]) / ts;
        }
      } else {
        // 关键修改：中间点使用Catmull-Rom切线公式
        // T_i = tension * (P_{i+1} - P_{i-1}) / (2 * ts)
        // 这确保了轨迹在中间点的平滑穿越
        Eigen::Vector3d chord = point_set[i+1] - point_set[i-1];
        tangents[i] = tension * chord / ts;  // 注意：这里用ts而不是2*ts，因为ts是每段的时间
        
        // 速度限制：确保切线不会导致超速
        double max_tangent_norm = chord.norm() / ts;  // 最大允许的切线大小
        if (tangents[i].norm() > max_tangent_norm) {
          tangents[i] = tangents[i].normalized() * max_tangent_norm;
        }
        
        // 确保切线不会太小（避免在航点处减速过多）
        double min_tangent_norm = chord.norm() / (3.0 * ts);  // 最小允许的切线大小
        if (tangents[i].norm() < min_tangent_norm && tangents[i].norm() > 1e-6) {
          tangents[i] = tangents[i].normalized() * min_tangent_norm;
        }
      }
    }

    // Step 3: Set intermediate control points P1 and P2 for each segment
    // Segment k has control points P_{3k}, P_{3k+1}, P_{3k+2}, P_{3k+3}
    // P_{3k+1} = P_{3k} + tangent[k] * ts / 3
    // P_{3k+2} = P_{3k+3} - tangent[k+1] * ts / 3
    for (int k = 0; k < num_segments; ++k) {
      int idx = k * 3;
      ctrl_pts.col(idx + 1) = ctrl_pts.col(idx) + tangents[k] * ts / 3.0;
      ctrl_pts.col(idx + 2) = ctrl_pts.col(idx + 3) - tangents[k + 1] * ts / 3.0;
    }

    ROS_DEBUG_STREAM("[Bezier Parameterize] waypoints=" << K << ", segments=" << num_segments
                    << ", control_points=" << num_ctrl_pts);
  }

  double PiecewiseBezier::getTimeSum()
  {
    double tm, tmp;
    getTimeSpan(tm, tmp);
    return tmp;
  }

  double PiecewiseBezier::getLength(const double &res)
  {
    double length = 0.0;
    double dur = getTimeSum();
    Eigen::VectorXd p_l = evaluateT(0.0), p_n;
    for (double t = res; t <= dur + 1e-4; t += res)
    {
      p_n = evaluateT(t);
      length += (p_n - p_l).norm();
      p_l = p_n;
    }
    return length;
  }

  /**
   * @brief Compute total squared jerk of the trajectory
   * @return Integrated squared jerk value
   * 
   * For cubic Bezier, jerk is constant within each segment:
   * J = 6*(P3 - 3*P2 + 3*P1 - P0) / T^3
   */
  double PiecewiseBezier::getJerk()
  {
    double total_jerk = 0.0;
    int num_segments = n_ / p_;
    for(int k = 0; k < num_segments; ++k) {
      int idx = k * p_;
      Eigen::Vector3d p0 = control_points_.col(idx);
      Eigen::Vector3d p1 = control_points_.col(idx + 1);
      Eigen::Vector3d p2 = control_points_.col(idx + 2);
      Eigen::Vector3d p3 = control_points_.col(idx + 3);

      Eigen::Vector3d J = (p3 - 3*p2 + 3*p1 - p0) * 6.0 / pow(interval_, 3);
      total_jerk += J.squaredNorm() * interval_;
    }
    return total_jerk;
  }

  void PiecewiseBezier::getMeanAndMaxVel(double &mean_v, double &max_v)
  {
    double dur = getTimeSum();
    double max_vel = -1.0, mean_vel = 0.0;
    int num = 0;
    for (double t = 0; t <= dur; t += 0.01)
    {
      Eigen::Vector3d p1 = evaluateT(t);
      Eigen::Vector3d p2 = evaluateT(t + 0.001);
      double vn = (p2 - p1).norm() / 0.001;

      mean_vel += vn;
      ++num;
      if (vn > max_vel) max_vel = vn;
    }
    mean_vel /= num;
    mean_v = mean_vel;
    max_v = max_vel;
  }

  void PiecewiseBezier::getMeanAndMaxAcc(double &mean_a, double &max_a)
  {
    double dur = getTimeSum();
    double max_acc = -1.0, mean_acc = 0.0;
    int num = 0;
    for (double t = 0; t <= dur; t += 0.01)
    {
      Eigen::Vector3d p1 = evaluateT(t);
      Eigen::Vector3d p2 = evaluateT(t + 0.001);
      Eigen::Vector3d p3 = evaluateT(t + 0.002);
      Eigen::Vector3d v1 = (p2 - p1) / 0.001;
      Eigen::Vector3d v2 = (p3 - p2) / 0.001;
      double an = (v2 - v1).norm() / 0.001;

      mean_acc += an;
      ++num;
      if (an > max_acc) max_acc = an;
    }
    mean_acc /= num;
    mean_a = mean_acc;
    max_a = max_acc;
  }

} // namespace ego_planner
