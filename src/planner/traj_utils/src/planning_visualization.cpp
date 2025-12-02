/**
 * @file planning_visualization.cpp
 * @brief Implementation of visualization tools for Bezier curve planning
 */

#include <traj_utils/planning_visualization.h>

using std::cout;
using std::endl;

namespace ego_planner
{
  PlanningVisualization::PlanningVisualization(ros::NodeHandle &nh)
  {
    node = nh;

    goal_point_pub = nh.advertise<visualization_msgs::Marker>("goal_point", 2);
    global_list_pub = nh.advertise<visualization_msgs::Marker>("global_list", 2);
    init_list_pub = nh.advertise<visualization_msgs::Marker>("init_list", 2);
    optimal_list_pub = nh.advertise<visualization_msgs::Marker>("optimal_list", 2);
    failed_list_pub = nh.advertise<visualization_msgs::Marker>("failed_list", 2);
    a_star_list_pub = nh.advertise<visualization_msgs::Marker>("a_star_list", 20);
    guide_vector_pub = nh.advertise<visualization_msgs::MarkerArray>("guide_vector", 2);
    intermediate_state_pub = nh.advertise<visualization_msgs::Marker>("intermediate_state", 20);

    last_goal_point_num_ = 0;
    last_global_list_num_ = 0;
    last_init_list_num_ = 0;
    last_optimal_list_num_ = 0;
    last_failed_list_num_ = 0;
    last_a_star_list_num_ = 0;
    last_guide_vector_num_ = 0;
    last_intermediate_state_num_ = 0;
  }

  void PlanningVisualization::displayMarkerList(ros::Publisher &pub, 
                                                const vector<Eigen::Vector3d> &list, 
                                                double scale, Eigen::Vector4d color, 
                                                int id, bool clear)
  {
    visualization_msgs::Marker sphere, line_strip;
    sphere.header.frame_id = line_strip.header.frame_id = "world";
    sphere.header.stamp = line_strip.header.stamp = ros::Time::now();
    sphere.type = visualization_msgs::Marker::SPHERE_LIST;
    line_strip.type = visualization_msgs::Marker::LINE_STRIP;
    sphere.action = line_strip.action = visualization_msgs::Marker::ADD;
    sphere.id = id;
    line_strip.id = id + 1000;

    sphere.pose.orientation.w = line_strip.pose.orientation.w = 1.0;
    sphere.color.r = line_strip.color.r = color(0);
    sphere.color.g = line_strip.color.g = color(1);
    sphere.color.b = line_strip.color.b = color(2);
    sphere.color.a = line_strip.color.a = clear ? 0 : color(3);
    sphere.scale.x = scale;
    sphere.scale.y = scale;
    sphere.scale.z = scale;
    line_strip.scale.x = scale / 2;
    geometry_msgs::Point pt;
    for (int i = 0; i < int(list.size()); i++)
    {
      pt.x = list[i](0);
      pt.y = list[i](1);
      pt.z = list[i](2);
      sphere.points.push_back(pt);
      line_strip.points.push_back(pt);
    }
    pub.publish(sphere);
    pub.publish(line_strip);
  }

  void PlanningVisualization::displayGoalPoint(Eigen::Vector3d goal_point, 
                                               Eigen::Vector4d color, 
                                               const double scale, int id)
  {
    visualization_msgs::Marker sphere;
    sphere.header.frame_id = "world";
    sphere.header.stamp = ros::Time::now();
    sphere.type = visualization_msgs::Marker::SPHERE;
    sphere.action = visualization_msgs::Marker::ADD;
    sphere.id = id;

    sphere.pose.orientation.w = 1.0;
    sphere.color.r = color(0);
    sphere.color.g = color(1);
    sphere.color.b = color(2);
    sphere.color.a = color(3);
    sphere.scale.x = scale;
    sphere.scale.y = scale;
    sphere.scale.z = scale;
    sphere.pose.position.x = goal_point(0);
    sphere.pose.position.y = goal_point(1);
    sphere.pose.position.z = goal_point(2);

    goal_point_pub.publish(sphere);
  }

  void PlanningVisualization::displayGlobalPathList(vector<Eigen::Vector3d> global_pts, 
                                                     const double scale, int id)
  {
    if (global_pts.empty())
    {
      displayMarkerList(global_list_pub, global_pts, scale, 
                        Eigen::Vector4d(0, 0.5, 0.5, 1), id, true);
      return;
    }
    displayMarkerList(global_list_pub, global_pts, scale, 
                      Eigen::Vector4d(0, 0.5, 0.5, 1), id);
  }

  void PlanningVisualization::displayInitPathList(vector<Eigen::Vector3d> init_pts, 
                                                   const double scale, int id)
  {
    if (init_pts.empty())
    {
      displayMarkerList(init_list_pub, init_pts, scale, 
                        Eigen::Vector4d(0, 0, 1, 1), id, true);
      return;
    }
    displayMarkerList(init_list_pub, init_pts, scale, 
                      Eigen::Vector4d(0, 0, 1, 1), id);
  }

  void PlanningVisualization::displayOptimalList(Eigen::MatrixXd optimal_pts, int id)
  {
    vector<Eigen::Vector3d> list;
    for (int i = 0; i < optimal_pts.cols(); i++)
    {
      Eigen::Vector3d pt = optimal_pts.col(i).transpose();
      list.push_back(pt);
    }
    displayMarkerList(optimal_list_pub, list, 0.15, 
                      Eigen::Vector4d(1, 0, 0, 1), id);
  }

  void PlanningVisualization::displayFailedList(Eigen::MatrixXd failed_pts, int id)
  {
    vector<Eigen::Vector3d> list;
    for (int i = 0; i < failed_pts.cols(); i++)
    {
      Eigen::Vector3d pt = failed_pts.col(i).transpose();
      list.push_back(pt);
    }
    displayMarkerList(failed_list_pub, list, 0.15, 
                      Eigen::Vector4d(0.5, 0, 0.5, 1), id);
  }

  void PlanningVisualization::displayAStarList(std::vector<std::vector<Eigen::Vector3d>> a_star_paths, 
                                               int id)
  {
    visualization_msgs::Marker line_strip;
    line_strip.header.frame_id = "world";
    line_strip.header.stamp = ros::Time::now();
    line_strip.type = visualization_msgs::Marker::LINE_STRIP;
    line_strip.action = visualization_msgs::Marker::ADD;

    line_strip.id = id;
    line_strip.scale.x = 0.05;
    line_strip.pose.orientation.w = 1.0;
    line_strip.color.r = 0.5;
    line_strip.color.g = 0.5;
    line_strip.color.b = 0.0;
    line_strip.color.a = 0.6;

    for (size_t i = 0; i < a_star_paths.size(); i++)
    {
      line_strip.points.clear();
      for (size_t j = 0; j < a_star_paths[i].size(); j++)
      {
        geometry_msgs::Point pt;
        pt.x = a_star_paths[i][j](0);
        pt.y = a_star_paths[i][j](1);
        pt.z = a_star_paths[i][j](2);
        line_strip.points.push_back(pt);
      }
      line_strip.id = id + i;
      a_star_list_pub.publish(line_strip);
    }
  }

  void PlanningVisualization::displayArrowList(ros::Publisher &pub, 
                                               const vector<Eigen::Vector3d> &list, 
                                               double scale, Eigen::Vector4d color, int id)
  {
    visualization_msgs::MarkerArray array;
    
    visualization_msgs::Marker delete_all;
    delete_all.action = visualization_msgs::Marker::DELETEALL;
    array.markers.push_back(delete_all);

    for (size_t i = 0; i + 1 < list.size(); i += 2)
    {
      visualization_msgs::Marker arrow;
      arrow.header.frame_id = "world";
      arrow.header.stamp = ros::Time::now();
      arrow.type = visualization_msgs::Marker::ARROW;
      arrow.action = visualization_msgs::Marker::ADD;
      arrow.id = id + i;

      arrow.pose.orientation.w = 1.0;
      arrow.color.r = color(0);
      arrow.color.g = color(1);
      arrow.color.b = color(2);
      arrow.color.a = color(3);
      arrow.scale.x = scale;
      arrow.scale.y = scale * 2;
      arrow.scale.z = scale * 2;

      geometry_msgs::Point start, end;
      start.x = list[i](0);
      start.y = list[i](1);
      start.z = list[i](2);
      end.x = list[i + 1](0);
      end.y = list[i + 1](1);
      end.z = list[i + 1](2);
      arrow.points.push_back(start);
      arrow.points.push_back(end);

      array.markers.push_back(arrow);
    }

    pub.publish(array);
  }

  void PlanningVisualization::displayIntermediateState(ros::Publisher &state_pub, 
                                                       ego_planner::PiecewiseBezier &pos, 
                                                       double sleep_time, 
                                                       string method)
  {
    double t_min, t_max;
    pos.getTimeSpan(t_min, t_max);
    
    visualization_msgs::Marker line_strip;
    line_strip.header.frame_id = "world";
    line_strip.header.stamp = ros::Time::now();
    line_strip.type = visualization_msgs::Marker::LINE_STRIP;
    line_strip.action = visualization_msgs::Marker::ADD;
    line_strip.id = 0;

    line_strip.scale.x = 0.03;
    line_strip.pose.orientation.w = 1.0;

    if (method == "ORIGINAL")
    {
      line_strip.color.r = 1.0;
      line_strip.color.g = 0.0;
      line_strip.color.b = 0.0;
    }
    else if (method == "OPTIMIZED")
    {
      line_strip.color.r = 0.0;
      line_strip.color.g = 1.0;
      line_strip.color.b = 0.0;
    }
    else
    {
      line_strip.color.r = 0.0;
      line_strip.color.g = 0.0;
      line_strip.color.b = 1.0;
    }
    line_strip.color.a = 1.0;

    for (double t = t_min; t <= t_max; t += 0.01)
    {
      Eigen::Vector3d pt = pos.evaluate(t);
      geometry_msgs::Point p;
      p.x = pt(0);
      p.y = pt(1);
      p.z = pt(2);
      line_strip.points.push_back(p);
    }

    state_pub.publish(line_strip);

    if (sleep_time > 0)
    {
      ros::Duration(sleep_time).sleep();
    }
  }

  void PlanningVisualization::generatePathDisplayArray(visualization_msgs::MarkerArray &array,
                                                        const vector<Eigen::Vector3d> &list, 
                                                        double scale, Eigen::Vector4d color, int id)
  {
    visualization_msgs::Marker sphere, line_strip;
    sphere.header.frame_id = line_strip.header.frame_id = "world";
    sphere.header.stamp = line_strip.header.stamp = ros::Time::now();
    sphere.type = visualization_msgs::Marker::SPHERE_LIST;
    line_strip.type = visualization_msgs::Marker::LINE_STRIP;
    sphere.action = line_strip.action = visualization_msgs::Marker::ADD;
    sphere.id = id;
    line_strip.id = id + 1000;

    sphere.pose.orientation.w = line_strip.pose.orientation.w = 1.0;
    sphere.color.r = line_strip.color.r = color(0);
    sphere.color.g = line_strip.color.g = color(1);
    sphere.color.b = line_strip.color.b = color(2);
    sphere.color.a = line_strip.color.a = color(3);
    sphere.scale.x = scale;
    sphere.scale.y = scale;
    sphere.scale.z = scale;
    line_strip.scale.x = scale / 2;

    geometry_msgs::Point pt;
    for (int i = 0; i < int(list.size()); i++)
    {
      pt.x = list[i](0);
      pt.y = list[i](1);
      pt.z = list[i](2);
      sphere.points.push_back(pt);
      line_strip.points.push_back(pt);
    }

    array.markers.push_back(sphere);
    array.markers.push_back(line_strip);
  }

  void PlanningVisualization::displayBezierControlPoints(Eigen::MatrixXd ctrl_pts, int id)
  {
    visualization_msgs::Marker sphere;
    sphere.header.frame_id = "world";
    sphere.header.stamp = ros::Time::now();
    sphere.type = visualization_msgs::Marker::SPHERE_LIST;
    sphere.action = visualization_msgs::Marker::ADD;
    sphere.id = id;

    sphere.pose.orientation.w = 1.0;
    sphere.color.r = 0.0;
    sphere.color.g = 1.0;
    sphere.color.b = 0.0;
    sphere.color.a = 1.0;
    sphere.scale.x = 0.1;
    sphere.scale.y = 0.1;
    sphere.scale.z = 0.1;

    for (int i = 0; i < ctrl_pts.cols(); i++)
    {
      geometry_msgs::Point pt;
      pt.x = ctrl_pts(0, i);
      pt.y = ctrl_pts(1, i);
      pt.z = ctrl_pts(2, i);
      sphere.points.push_back(pt);
    }

    optimal_list_pub.publish(sphere);
  }

  void PlanningVisualization::displayBezierCurve(ego_planner::PiecewiseBezier &bezier_traj, int id)
  {
    double t_min, t_max;
    bezier_traj.getTimeSpan(t_min, t_max);

    visualization_msgs::Marker line_strip;
    line_strip.header.frame_id = "world";
    line_strip.header.stamp = ros::Time::now();
    line_strip.type = visualization_msgs::Marker::LINE_STRIP;
    line_strip.action = visualization_msgs::Marker::ADD;
    line_strip.id = id;

    line_strip.scale.x = 0.05;
    line_strip.pose.orientation.w = 1.0;
    line_strip.color.r = 1.0;
    line_strip.color.g = 0.5;
    line_strip.color.b = 0.0;
    line_strip.color.a = 1.0;

    for (double t = t_min; t <= t_max; t += 0.02)
    {
      Eigen::Vector3d pt = bezier_traj.evaluate(t);
      geometry_msgs::Point p;
      p.x = pt(0);
      p.y = pt(1);
      p.z = pt(2);
      line_strip.points.push_back(p);
    }

    optimal_list_pub.publish(line_strip);
  }

} // namespace ego_planner
