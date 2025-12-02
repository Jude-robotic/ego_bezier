/**
 * @file bezier_optimizer.h
 * @brief Piecewise Bezier Curve Trajectory Optimizer
 * 
 * This optimizer uses L-BFGS to optimize piecewise cubic Bezier curve trajectories.
 * It minimizes a weighted sum of:
 * - Smoothness cost (jerk minimization + segment continuity)
 * - Collision cost (obstacle avoidance)
 * - Feasibility cost (velocity/acceleration limits)
 * - Fitness cost (trajectory refinement to reference points)
 */

#ifndef _BEZIER_OPTIMIZER_H_
#define _BEZIER_OPTIMIZER_H_

#include <Eigen/Eigen>
#include <path_searching/dyn_a_star.h>
#include <bezier_opt/piecewise_bezier.h>
#include <plan_env/grid_map.h>
#include <ros/ros.h>
#include "bezier_opt/lbfgs.hpp"

namespace ego_planner
{

  /**
   * @brief Container for Bezier control points with collision avoidance data
   */
  class ControlPoints
  {
  public:
    double clearance;  // Safety distance from obstacles
    int size;          // Number of control points
    Eigen::MatrixXd points;  // Control points matrix (3 x size)
    std::vector<std::vector<Eigen::Vector3d>> base_point;  // Collision base points
    std::vector<std::vector<Eigen::Vector3d>> direction;   // Collision avoidance directions
    std::vector<bool> flag_temp;  // Temporary flags for various operations

    void resize(const int size_set)
    {
      size = size_set;
      base_point.clear();
      direction.clear();
      flag_temp.clear();
      points.resize(3, size_set);
      base_point.resize(size);
      direction.resize(size);
      flag_temp.resize(size);
    }
  };

  /**
   * @brief Bezier curve trajectory optimizer using L-BFGS
   * 
   * The optimizer works with piecewise cubic Bezier curves where:
   * - Each segment has 4 control points
   * - N segments have 3N+1 total control points
   * - C1 continuity is enforced between segments
   */
  class BezierOptimizer
  {

  public:
    BezierOptimizer() {}
    ~BezierOptimizer() {}

    /* Main API */
    void setEnvironment(const GridMap::Ptr &env);
    void setParam(ros::NodeHandle &nh);
    Eigen::MatrixXd BezierOptimizeTraj(const Eigen::MatrixXd &points, const double &ts,
                                       const int &cost_function, int max_num_id, int max_time_id);

    /* Setup functions */
    void setControlPoints(const Eigen::MatrixXd &points);
    void setSegmentDuration(const double &ts);
    void setCostFunction(const int &cost_function);
    void setTerminateCond(const int &max_num_id, const int &max_time_id);

    /* Optional inputs */
    void setGuidePath(const vector<Eigen::Vector3d> &guide_pt);
    void setWaypoints(const vector<Eigen::Vector3d> &waypts, const vector<int> &waypt_idx);

    void optimize();
    Eigen::MatrixXd getControlPoints();

    AStar::Ptr a_star_;
    std::vector<Eigen::Vector3d> ref_pts_;  // Reference points for fitness cost

    std::vector<std::vector<Eigen::Vector3d>> initControlPoints(Eigen::MatrixXd &init_points, bool flag_first_init = true);
    bool BezierOptimizeTrajRebound(Eigen::MatrixXd &optimal_points, double ts);
    bool BezierOptimizeTrajRefine(const Eigen::MatrixXd &init_points, const double ts, Eigen::MatrixXd &optimal_points);

    inline int getOrder(void) { return order_; }

    // Legacy compatibility aliases
    inline void setBsplineInterval(const double &ts) { setSegmentDuration(ts); }
    inline bool BsplineOptimizeTrajRebound(Eigen::MatrixXd &optimal_points, double ts) {
      return BezierOptimizeTrajRebound(optimal_points, ts);
    }
    inline bool BsplineOptimizeTrajRefine(const Eigen::MatrixXd &init_points, const double ts, Eigen::MatrixXd &optimal_points) {
      return BezierOptimizeTrajRefine(init_points, ts, optimal_points);
    }

  private:
    GridMap::Ptr grid_map_;

    enum FORCE_STOP_OPTIMIZE_TYPE
    {
      DONT_STOP,
      STOP_FOR_REBOUND,
      STOP_FOR_ERROR
    } force_stop_type_;

    // Optimization state
    double segment_duration_;  // Duration of each Bezier segment
    int bezier_step_ = 3;      // Control points per segment (fixed for cubic)
    Eigen::Vector3d end_pt_;

    vector<Eigen::Vector3d> guide_pts_;
    vector<Eigen::Vector3d> waypoints_;
    vector<int> waypt_idx_;

    int max_num_id_, max_time_id_;
    int cost_function_;
    double start_time_;

    /* Optimization parameters */
    int order_;                    // Bezier degree (3 for cubic)
    double lambda1_;               // Smoothness weight
    double lambda2_, new_lambda2_; // Collision weight
    double lambda3_;               // Feasibility weight
    double lambda4_;               // Fitness weight

    int a;
    double dist0_;             // Safety distance threshold
    double max_vel_, max_acc_; // Dynamic limits

    int variable_num_;
    int iter_num_;
    Eigen::VectorXd best_variable_;
    double min_cost_;

    ControlPoints cps_;

    /* Cost function components */
    static double costFunction(const std::vector<double> &x, std::vector<double> &grad, void *func_data);
    void combineCost(const std::vector<double> &x, vector<double> &grad, double &cost);

    void calcSmoothnessCost(const Eigen::MatrixXd &q, double &cost,
                            Eigen::MatrixXd &gradient, bool flag_use_jerk = true);
    void calcFeasibilityCost(const Eigen::MatrixXd &q, double &cost,
                             Eigen::MatrixXd &gradient);
    void calcDistanceCostRebound(const Eigen::MatrixXd &q, double &cost, 
                                 Eigen::MatrixXd &gradient, int iter_num, double smoothness_cost);
    void calcFitnessCost(const Eigen::MatrixXd &q, double &cost, Eigen::MatrixXd &gradient);
    bool check_collision_and_rebound(void);

    static int earlyExit(void *func_data, const double *x, const double *g, const double fx,
                         const double xnorm, const double gnorm, const double step, int n, int k, int ls);
    static double costFunctionRebound(void *func_data, const double *x, double *grad, const int n);
    static double costFunctionRefine(void *func_data, const double *x, double *grad, const int n);

    bool rebound_optimize();
    bool refine_optimize();
    void combineCostRebound(const double *x, double *grad, double &f_combine, const int n);
    void combineCostRefine(const double *x, double *grad, double &f_combine, const int n);

  public:
    typedef unique_ptr<BezierOptimizer> Ptr;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
  
  // Type alias for backward compatibility
  using BsplineOptimizer = BezierOptimizer;

} // namespace ego_planner

#endif
