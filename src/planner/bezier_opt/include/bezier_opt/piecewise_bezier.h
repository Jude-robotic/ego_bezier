#ifndef _PIECEWISE_BEZIER_H_
#define _PIECEWISE_BEZIER_H_

#include <Eigen/Eigen>
#include <algorithm>
#include <iostream>

using namespace std;

namespace ego_planner
{
  /**
   * @brief Piecewise Cubic Bezier Curve Class
   * 
   * This class represents a trajectory as a sequence of connected cubic Bezier curve segments.
   * Each segment uses 4 control points: P_{3k}, P_{3k+1}, P_{3k+2}, P_{3k+3}
   * For N segments, we have 3N+1 control points total.
   * 
   * The class provides:
   * - Curve evaluation at any time t
   * - Derivative computation (velocity, acceleration)
   * - Feasibility checking for velocity/acceleration limits
   * - Parameterization from waypoints to Bezier control points
   */
  class PiecewiseBezier
  {
  private:
    // Control points matrix: each column is a 3D control point
    // For N Bezier segments: 3N+1 control points
    Eigen::MatrixXd control_points_;

    int p_;        // Bezier degree (always 3 for cubic)
    int n_;        // Number of control points - 1
    Eigen::VectorXd u_; // Legacy: kept for compatibility but not used
    double interval_;   // Duration of each Bezier segment

    Eigen::MatrixXd getDerivativeControlPoints();

    double limit_vel_, limit_acc_, limit_ratio_, feasibility_tolerance_;

  public:
    PiecewiseBezier() {}
    PiecewiseBezier(const Eigen::MatrixXd &points, const int &order, const double &interval);
    ~PiecewiseBezier();

    Eigen::MatrixXd get_control_points(void) { return control_points_; }

    // Initialize the Bezier curve with control points
    void setBezierCurve(const Eigen::MatrixXd &points, const int &order, const double &interval);

    // Get/set basic curve info
    void setKnot(const Eigen::VectorXd &knot);
    Eigen::VectorXd getKnot();
    Eigen::MatrixXd getControlPoint();
    double getInterval();
    bool getTimeSpan(double &t_start, double &t_end);

    // Evaluate position at parameter t
    Eigen::VectorXd evaluate(const double &t);
    inline Eigen::VectorXd evaluateT(const double &t) { return evaluate(t); }
    
    // Legacy compatibility methods
    inline Eigen::VectorXd evaluateDeBoor(const double &t) { return evaluate(t); }
    inline Eigen::VectorXd evaluateDeBoorT(const double &t) { return evaluate(t); }

    // Get derivative curve (velocity or acceleration)
    PiecewiseBezier getDerivative();

    /**
     * @brief Convert waypoints to piecewise Bezier control points
     * @param ts Duration of each Bezier segment
     * @param point_set Waypoints to interpolate
     * @param start_end_derivative Boundary conditions [start_vel, end_vel, start_acc, end_acc]
     * @param ctrl_pts Output control points matrix
     */
    static void parameterizeToBezier(const double &ts, const vector<Eigen::Vector3d> &point_set,
                                     const vector<Eigen::Vector3d> &start_end_derivative,
                                     Eigen::MatrixXd &ctrl_pts);

    // Feasibility checking
    void setPhysicalLimits(const double &vel, const double &acc, const double &tolerance);
    bool checkFeasibility(double &ratio, bool show = false);
    void lengthenTime(const double &ratio);

    // Performance metrics
    double getTimeSum();
    double getLength(const double &res = 0.01);
    double getJerk();
    void getMeanAndMaxVel(double &mean_v, double &max_v);
    void getMeanAndMaxAcc(double &mean_a, double &max_a);

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
  
  // Type alias for backward compatibility
  using UniformBspline = PiecewiseBezier;
  
} // namespace ego_planner

#endif
