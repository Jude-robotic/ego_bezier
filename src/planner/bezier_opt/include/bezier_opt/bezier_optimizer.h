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

    /** @brief 优化模式枚举 */
    enum OptMode { REBOUND, REFINE, FORMATION };

    /** @brief BezierOptimizeTrajFormation 返回值常量 */
    static constexpr int OPTIMAL   =  0;  ///< L-BFGS 收敛
    static constexpr int SATISFIED =  1;  ///< 最大迭代但代价可接受
    static constexpr int FAILED    = -1;  ///< 优化失败

    /* Main API */
    void setEnvironment(const GridMap::Ptr &env);
    void setParam(ros::NodeHandle &nh);
    /** @brief 直接传入参数数值，不依赖 ROS 参数服务器。
     *  适用于 swarm_master 等无 optimization/* 命名空间的节点。 */
    void setParamDirect(double lambda_smooth, double lambda_collision,
                        double lambda_feasibility, double lambda_fitness,
                        double lambda_formation,
                        double dist0, double max_vel, double max_acc, int order);
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
    std::vector<Eigen::Vector3d> formation_ref_pts_;  ///< FORMATION 模式理想编队控制点

    std::vector<std::vector<Eigen::Vector3d>> initControlPoints(Eigen::MatrixXd &init_points, bool flag_first_init = true);
    bool BezierOptimizeTrajRebound(Eigen::MatrixXd &optimal_points, double ts);
    bool BezierOptimizeTrajRefine(const Eigen::MatrixXd &init_points, const double ts, Eigen::MatrixXd &optimal_points);

    /** @brief 设置 FORMATION 模式的编队参考控制点（理想 V 形偏移位置）。
     *  @param formation_pts 长度 = 控制点总数，与 initial_ctrl_pts 列数对齐 */
    void setFormationReference(const std::vector<Eigen::Vector3d>& formation_pts);

    /** @brief FORMATION 模式 L-BFGS 优化入口。
     *  代价 = λ1·smooth + λ2·distance(rebound) + λ3·feasibility + λ5·formation
     *  @param ctrl_pts  输入初始控制点，优化后原地修改
     *  @param ts        每段贝塞尔时长
     *  @return OPTIMAL / SATISFIED / FAILED */
    int BezierOptimizeTrajFormation(Eigen::MatrixXd& ctrl_pts, double ts);

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
    double lambda5_{20.0};         ///< FORMATION 编队保持权重（软约束，远小于 λ2 确保安全优先）

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
    /** @brief 编队保持代价（软约束弹簧）：C_form = λ5·Σ||q_i - f_i||²
     *         解析梯度：∂C/∂q_i = 2·λ5·(q_i - f_i) */
    void calcFormationCost(const Eigen::MatrixXd& q, double& cost,
                           Eigen::MatrixXd& gradient);
    bool check_collision_and_rebound(void);

    static int earlyExit(void *func_data, const double *x, const double *g, const double fx,
                         const double xnorm, const double gnorm, const double step, int n, int k, int ls);
    static double costFunctionRebound(void *func_data, const double *x, double *grad, const int n);
    static double costFunctionRefine(void *func_data, const double *x, double *grad, const int n);
    static double costFunctionFormation(void *func_data, const double *x,
                                        double *grad, const int n);

    bool rebound_optimize();
    bool refine_optimize();
    bool formation_optimize();
    void combineCostRebound(const double *x, double *grad, double &f_combine, const int n);
    void combineCostRefine(const double *x, double *grad, double &f_combine, const int n);
    void combineCostFormation(const double *x, double *grad,
                              double &f_combine, const int n);

  public:
    typedef unique_ptr<BezierOptimizer> Ptr;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
  
  // Type alias for backward compatibility
  using BsplineOptimizer = BezierOptimizer;

} // namespace ego_planner

#endif
