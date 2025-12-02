/**
 * @file planner_manager.h
 * @brief EGO Planner Manager with Piecewise Bezier Curve Trajectories
 */

#ifndef _PLANNER_MANAGER_H_
#define _PLANNER_MANAGER_H_

#include <stdlib.h>

#include <bezier_opt/bezier_optimizer.h>
#include <bezier_opt/piecewise_bezier.h>
#include <ego_planner/DataDisp.h>
#include <plan_env/grid_map.h>
#include <plan_manage/plan_container.hpp>
#include <ros/ros.h>
#include <traj_utils/planning_visualization.h>

namespace ego_planner
{

  /**
   * @brief Main planning manager for EGO-Planner with Bezier curves
   * 
   * This class coordinates:
   * - Path searching (A*)
   * - Trajectory optimization (L-BFGS with Bezier curves)
   * - Trajectory refinement and time reallocation
   */
  class EGOPlannerManager
  {
  public:
    EGOPlannerManager();
    ~EGOPlannerManager();

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /* Main planning interface */
    bool reboundReplan(Eigen::Vector3d start_pt, Eigen::Vector3d start_vel, Eigen::Vector3d start_acc,
                       Eigen::Vector3d end_pt, Eigen::Vector3d end_vel, bool flag_polyInit, bool flag_randomPolyTraj);
    bool EmergencyStop(Eigen::Vector3d stop_pos);
    bool planGlobalTraj(const Eigen::Vector3d &start_pos, const Eigen::Vector3d &start_vel, const Eigen::Vector3d &start_acc,
                        const Eigen::Vector3d &end_pos, const Eigen::Vector3d &end_vel, const Eigen::Vector3d &end_acc);
    bool planGlobalTrajWaypoints(const Eigen::Vector3d &start_pos, const Eigen::Vector3d &start_vel, const Eigen::Vector3d &start_acc,
                                 const std::vector<Eigen::Vector3d> &waypoints, const Eigen::Vector3d &end_vel, const Eigen::Vector3d &end_acc);

    void initPlanModules(ros::NodeHandle &nh, PlanningVisualization::Ptr vis = NULL);

    PlanParameters pp_;
    LocalTrajData local_data_;
    GlobalTrajData global_data_;
    GridMap::Ptr grid_map_;

  private:
    /* Main planning algorithms & modules */
    PlanningVisualization::Ptr visualization_;

    BezierOptimizer::Ptr bezier_optimizer_rebound_;

    int continous_failures_count_{0};

    void updateTrajInfo(const PiecewiseBezier &position_traj, const ros::Time time_now);

    void reparamBezier(PiecewiseBezier &bezier, vector<Eigen::Vector3d> &start_end_derivative, 
                       double ratio, Eigen::MatrixXd &ctrl_pts, double &dt, double &time_inc);

    bool refineTrajAlgo(PiecewiseBezier &traj, vector<Eigen::Vector3d> &start_end_derivative, 
                        double ratio, double &ts, Eigen::MatrixXd &optimal_control_points);

  public:
    typedef unique_ptr<EGOPlannerManager> Ptr;
  };
} // namespace ego_planner

#endif
