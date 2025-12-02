#include "bspline_opt/uniform_bspline.h"
#include <ros/ros.h>
#include <cmath>

namespace ego_planner
{

  UniformBspline::UniformBspline(const Eigen::MatrixXd &points, const int &order,
                                 const double &interval)
  {
    setUniformBspline(points, order, interval);
  }

  UniformBspline::~UniformBspline() {}

  void UniformBspline::setUniformBspline(const Eigen::MatrixXd &points, const int &order,
                                         const double &interval)
  {
    control_points_ = points;
    p_ = order; 
    interval_ = interval;
    n_ = points.cols() - 1;
  }

  void UniformBspline::setKnot(const Eigen::VectorXd &knot) {
      // No knot vector in uniform piecewise bezier (implicit)
  }

  Eigen::VectorXd UniformBspline::getKnot() {
      return Eigen::VectorXd();
  }

  bool UniformBspline::getTimeSpan(double &um, double &um_p)
  {
    um = 0.0;
    int num_segments = n_ / p_;
    um_p = num_segments * interval_;
    return true;
  }

  Eigen::MatrixXd UniformBspline::getControlPoint() { return control_points_; }

  Eigen::VectorXd UniformBspline::evaluateDeBoor(const double &u)
  {
    double t = u;
    int num_segments = n_ / p_;
    double total_duration = num_segments * interval_;

    if (t < 0) t = 0;
    if (t > total_duration) t = total_duration;

    int segment_idx = floor(t / interval_);
    if (segment_idx >= num_segments) segment_idx = num_segments - 1;

    double local_t = (t - segment_idx * interval_) / interval_; 

    int start_idx = segment_idx * p_;
    
    if (p_ == 3) {
        // Cubic Bezier
        if (start_idx + 3 >= control_points_.cols()) {
            start_idx = control_points_.cols() - 4;
            if (start_idx < 0) start_idx = 0;
        }

        Eigen::Vector3d p0 = control_points_.col(start_idx);
        Eigen::Vector3d p1 = control_points_.col(start_idx + 1);
        Eigen::Vector3d p2 = control_points_.col(start_idx + 2);
        Eigen::Vector3d p3 = control_points_.col(start_idx + 3);

        double mt = 1 - local_t;
        double b0 = mt * mt * mt;
        double b1 = 3 * mt * mt * local_t;
        double b2 = 3 * mt * local_t * local_t;
        double b3 = local_t * local_t * local_t;

        return p0 * b0 + p1 * b1 + p2 * b2 + p3 * b3;
    } 
    else if (p_ == 2) {
        // Quadratic Bezier (for derivative)
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
        // Linear Bezier (for 2nd derivative)
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

  UniformBspline UniformBspline::getDerivative()
  {
    int num_segments = n_ / p_;
    int new_p = p_ - 1;
    if (new_p < 1) return *this; // Should not happen for p=3

    int new_cols = num_segments * new_p + 1;
    Eigen::MatrixXd deriv_points(3, new_cols);

    for (int k = 0; k < num_segments; ++k) {
        int p_idx = k * p_;
        int q_idx = k * new_p;

        Eigen::Vector3d p0 = control_points_.col(p_idx);
        Eigen::Vector3d p1 = control_points_.col(p_idx + 1);
        
        if (p_ == 3) {
            Eigen::Vector3d p2 = control_points_.col(p_idx + 2);
            Eigen::Vector3d p3 = control_points_.col(p_idx + 3);

            deriv_points.col(q_idx)     = (p1 - p0) * p_ / interval_;
            deriv_points.col(q_idx + 1) = (p2 - p1) * p_ / interval_;
            deriv_points.col(q_idx + 2) = (p3 - p2) * p_ / interval_;
        } else if (p_ == 2) {
            Eigen::Vector3d p2 = control_points_.col(p_idx + 2);
            
            deriv_points.col(q_idx)     = (p1 - p0) * p_ / interval_;
            deriv_points.col(q_idx + 1) = (p2 - p1) * p_ / interval_;
        }
    }

    return UniformBspline(deriv_points, new_p, interval_);
  }

  double UniformBspline::getInterval() { return interval_; }

  void UniformBspline::setPhysicalLimits(const double &vel, const double &acc, const double &tolerance)
  {
    limit_vel_ = vel;
    limit_acc_ = acc;
    limit_ratio_ = 1.1;
    feasibility_tolerance_ = tolerance;
  }

  bool UniformBspline::checkFeasibility(double &ratio, bool show)
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

        // Velocity Control Points
        Eigen::Vector3d v0 = (p1 - p0) * 3.0 / interval_;
        Eigen::Vector3d v1 = (p2 - p1) * 3.0 / interval_;
        Eigen::Vector3d v2 = (p3 - p2) * 3.0 / interval_;

        // Check Velocity
        for (const auto& v : {v0, v1, v2}) {
            if (v.cwiseAbs().maxCoeff() > enlarged_vel_lim) {
                if (show) cout << "[Check]: Infeasible vel at seg " << k << ": " << v.transpose() << endl;
                fea = false;
            }
            max_vel = max(max_vel, v.cwiseAbs().maxCoeff());
        }

        // Acceleration Control Points
        Eigen::Vector3d a0 = (v1 - v0) * 2.0 / interval_;
        Eigen::Vector3d a1 = (v2 - v1) * 2.0 / interval_;

        // Check Acceleration
        for (const auto& a : {a0, a1}) {
            if (a.cwiseAbs().maxCoeff() > enlarged_acc_lim) {
                if (show) cout << "[Check]: Infeasible acc at seg " << k << ": " << a.transpose() << endl;
                fea = false;
            }
            max_acc = max(max_acc, a.cwiseAbs().maxCoeff());
        }
    }

    ratio = max(max_vel / limit_vel_, sqrt(max_acc / limit_acc_));
    return fea;
  }

  void UniformBspline::lengthenTime(const double &ratio)
  {
      interval_ *= ratio;
  }

  /**
   * @brief 将路径点转换为分段贝塞尔曲线控制点
   * @param ts 每段贝塞尔曲线的时间间隔
   * @param point_set 路径点集合（K个点）
   * @param start_end_derivative 起始和终止的速度/加速度约束
   *        [0]: start_vel, [1]: end_vel, [2]: start_acc (可选), [3]: end_acc (可选)
   * @param ctrl_pts 输出控制点矩阵 (3 x (3*(K-1)+1))
   * 
   * 说明：
   * - K个路径点生成K-1段三次贝塞尔曲线
   * - 需要 3*(K-1)+1 = 3*N+1 个控制点
   * - 保证C1连续性（位置和速度连续）
   * 
   * 贝塞尔曲线性质：
   * - 起点速度 V(0) = 3*(P1 - P0) / T
   * - 终点速度 V(T) = 3*(P3 - P2) / T
   * - 因此 P1 = P0 + V*T/3, P2 = P3 - V*T/3
   */
  void UniformBspline::parameterizeToBspline(const double &ts, const vector<Eigen::Vector3d> &point_set,
                                             const vector<Eigen::Vector3d> &start_end_derivative,
                                             Eigen::MatrixXd &ctrl_pts)
  {
    if (point_set.size() < 2) return;

    int K = point_set.size();           // 路径点数量
    int num_segments = K - 1;           // 贝塞尔曲线段数
    int num_ctrl_pts = num_segments * 3 + 1;  // 控制点数量

    ctrl_pts.resize(3, num_ctrl_pts);

    // Step 1: 设置所有连接点 (P0, P3, P6, ..., P_{3N})
    // 这些是贝塞尔曲线段的端点，也是路径点
    for (int i = 0; i < K; ++i) {
        ctrl_pts.col(i * 3) = point_set[i];
    }

    // Step 2: 计算每个连接点的切向量（速度）
    // 这些切向量用于设置P1和P2控制点
    std::vector<Eigen::Vector3d> tangents(K);
    
    for (int i = 0; i < K; ++i) {
        if (i == 0) {
            // 起点：使用给定的起始速度，否则用差分估计
            if (start_end_derivative.size() >= 1 && start_end_derivative[0].norm() > 1e-6) {
                tangents[i] = start_end_derivative[0];
            } else {
                tangents[i] = (point_set[1] - point_set[0]) / ts;
            }
        } else if (i == K - 1) {
            // 终点：使用给定的终止速度，否则用差分估计
            if (start_end_derivative.size() >= 2 && start_end_derivative[1].norm() > 1e-6) {
                tangents[i] = start_end_derivative[1];
            } else {
                tangents[i] = (point_set[K-1] - point_set[K-2]) / ts;
            }
        } else {
            // 中间点：使用两侧差分的平均作为切向量
            Eigen::Vector3d v_prev = (point_set[i] - point_set[i-1]) / ts;
            Eigen::Vector3d v_next = (point_set[i+1] - point_set[i]) / ts;
            tangents[i] = 0.5 * (v_prev + v_next);
        }
    }

    // Step 3: 设置每段的中间控制点 P1 和 P2
    // 对于第k段 (控制点 P_{3k}, P_{3k+1}, P_{3k+2}, P_{3k+3}):
    //   P_{3k+1} = P_{3k} + tangent[k] * ts / 3     (保证起点速度)
    //   P_{3k+2} = P_{3k+3} - tangent[k+1] * ts / 3 (保证终点速度)
    for (int k = 0; k < num_segments; ++k) {
        int idx = k * 3;
        // P1: 从起点沿切向量方向偏移 ts/3
        ctrl_pts.col(idx + 1) = ctrl_pts.col(idx) + tangents[k] * ts / 3.0;
        // P2: 从终点沿切向量反方向偏移 ts/3
        ctrl_pts.col(idx + 2) = ctrl_pts.col(idx + 3) - tangents[k + 1] * ts / 3.0;
    }
    
    // Debug输出
    ROS_DEBUG_STREAM("[Bezier Parameterize] K=" << K << ", segments=" << num_segments 
                    << ", ctrl_pts=" << num_ctrl_pts);
  }

  double UniformBspline::getTimeSum()
  {
      double tm, tmp;
      getTimeSpan(tm, tmp);
      return tmp;
  }

  double UniformBspline::getLength(const double &res)
  {
      double length = 0.0;
      double dur = getTimeSum();
      Eigen::VectorXd p_l = evaluateDeBoorT(0.0), p_n;
      for (double t = res; t <= dur + 1e-4; t += res)
      {
        p_n = evaluateDeBoorT(t);
        length += (p_n - p_l).norm();
        p_l = p_n;
      }
      return length;
  }

  double UniformBspline::getJerk()
  {
      double total_jerk = 0.0;
      int num_segments = n_ / p_;
      for(int k=0; k<num_segments; ++k) {
          int idx = k*p_;
          Eigen::Vector3d p0 = control_points_.col(idx);
          Eigen::Vector3d p1 = control_points_.col(idx+1);
          Eigen::Vector3d p2 = control_points_.col(idx+2);
          Eigen::Vector3d p3 = control_points_.col(idx+3);

          Eigen::Vector3d J = (p3 - 3*p2 + 3*p1 - p0) * 6.0 / pow(interval_, 3);
          total_jerk += J.squaredNorm() * interval_;
      }
      return total_jerk;
  }

  void UniformBspline::getMeanAndMaxVel(double &mean_v, double &max_v)
  {
      double dur = getTimeSum();
      double max_vel = -1.0, mean_vel = 0.0;
      int num = 0;
      for (double t = 0; t <= dur; t += 0.01)
      {
          Eigen::Vector3d p1 = evaluateDeBoorT(t);
          Eigen::Vector3d p2 = evaluateDeBoorT(t+0.001);
          double vn = (p2-p1).norm() / 0.001;

          mean_vel += vn;
          ++num;
          if (vn > max_vel) max_vel = vn;
      }
      mean_vel /= num;
      mean_v = mean_vel;
      max_v = max_vel;
  }

  void UniformBspline::getMeanAndMaxAcc(double &mean_a, double &max_a)
  {
      double dur = getTimeSum();
      double max_acc = -1.0, mean_acc = 0.0;
      int num = 0;
      for (double t = 0; t <= dur; t += 0.01)
      {
          Eigen::Vector3d p1 = evaluateDeBoorT(t);
          Eigen::Vector3d p2 = evaluateDeBoorT(t+0.001);
          Eigen::Vector3d p3 = evaluateDeBoorT(t+0.002);
          Eigen::Vector3d v1 = (p2-p1)/0.001;
          Eigen::Vector3d v2 = (p3-p2)/0.001;
          double an = (v2-v1).norm()/0.001;

          mean_acc += an;
          ++num;
          if (an > max_acc) max_acc = an;
      }
      mean_acc /= num;
      mean_a = mean_acc;
      max_a = max_acc;
  }

} // namespace ego_planner
