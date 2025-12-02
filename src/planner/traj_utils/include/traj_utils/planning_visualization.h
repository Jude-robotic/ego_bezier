/**
 * @file planning_visualization.h
 * @brief Visualization tools for planning with Bezier curves
 */

#ifndef _PLANNING_VISUALIZATION_H_
#define _PLANNING_VISUALIZATION_H_

#include <Eigen/Eigen>
#include <algorithm>
#include <iostream>
#include <ros/ros.h>
#include <vector>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <stdlib.h>

#include <bezier_opt/piecewise_bezier.h>
#include <traj_utils/polynomial_traj.h>

using std::vector;

namespace ego_planner
{
  class PlanningVisualization
  {
  private:
    ros::NodeHandle node;

    ros::Publisher goal_point_pub;
    ros::Publisher global_list_pub;
    ros::Publisher init_list_pub;
    ros::Publisher optimal_list_pub;
    ros::Publisher failed_list_pub;
    ros::Publisher a_star_list_pub;
    ros::Publisher guide_vector_pub;
    ros::Publisher intermediate_state_pub;

    int last_goal_point_num_;
    int last_global_list_num_;
    int last_init_list_num_;
    int last_optimal_list_num_;
    int last_failed_list_num_;
    int last_a_star_list_num_;
    int last_guide_vector_num_;
    int last_intermediate_state_num_;

  public:
    PlanningVisualization() {}
    ~PlanningVisualization() {}
    PlanningVisualization(ros::NodeHandle &nh);

    typedef std::shared_ptr<PlanningVisualization> Ptr;

    /**
     * @brief Display marker list
     */
    void displayMarkerList(ros::Publisher &pub, const vector<Eigen::Vector3d> &list, 
                           double scale, Eigen::Vector4d color, int id, bool clear = false);

    /**
     * @brief Display goal point
     */
    void displayGoalPoint(Eigen::Vector3d goal_point, Eigen::Vector4d color, 
                          const double scale, int id);

    /**
     * @brief Display global path list
     */
    void displayGlobalPathList(vector<Eigen::Vector3d> global_pts, 
                               const double scale, int id);

    /**
     * @brief Display initial path list from optimization
     */
    void displayInitPathList(vector<Eigen::Vector3d> init_pts, 
                             const double scale, int id);

    /**
     * @brief Display optimized Bezier trajectory
     */
    void displayOptimalList(Eigen::MatrixXd optimal_pts, int id);

    /**
     * @brief Display failed optimization list
     */
    void displayFailedList(Eigen::MatrixXd failed_pts, int id);

    /**
     * @brief Display A* path list
     */
    void displayAStarList(std::vector<std::vector<Eigen::Vector3d>> a_star_paths, 
                          int id);

    /**
     * @brief Display arrow between two points
     */
    void displayArrowList(ros::Publisher &pub, 
                          const vector<Eigen::Vector3d> &list, 
                          double scale, Eigen::Vector4d color, int id);

    /**
     * @brief Display intermediate state
     */
    void displayIntermediateState(ros::Publisher &state_pub, 
                                  ego_planner::PiecewiseBezier &pos, 
                                  double sleep_time = 0.0, 
                                  string method = "ORIGINAL");

    /**
     * @brief Generate path display for Bezier curve
     */
    void generatePathDisplayArray(visualization_msgs::MarkerArray &array,
                                  const vector<Eigen::Vector3d> &list, 
                                  double scale, Eigen::Vector4d color, int id);

    /**
     * @brief Display Bezier control points
     */
    void displayBezierControlPoints(Eigen::MatrixXd ctrl_pts, int id);

    /**
     * @brief Display Bezier trajectory curve
     */
    void displayBezierCurve(ego_planner::PiecewiseBezier &bezier_traj, int id);

  };
} // namespace ego_planner

#endif
