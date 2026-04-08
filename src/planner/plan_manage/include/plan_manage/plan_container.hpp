/**
 * @file plan_container.hpp
 * @brief Data containers for planning with Piecewise Bezier Curves
 */

#ifndef _PLAN_CONTAINER_H_
#define _PLAN_CONTAINER_H_

#include <Eigen/Eigen>
#include <vector>
#include <algorithm>
#include <ros/ros.h>

#include <bezier_opt/piecewise_bezier.h>
#include <traj_utils/polynomial_traj.h>

using std::vector;

namespace ego_planner
{

  /**
   * @brief Container for global trajectory data
   */
  class GlobalTrajData
  {
  private:
  public:
    PolynomialTraj global_traj_;
    vector<PiecewiseBezier> local_traj_;

    double global_duration_;
    ros::Time global_start_time_;
    double local_start_time_, local_end_time_;
    double time_increase_;
    double last_time_inc_;
    double last_progress_time_;

    GlobalTrajData() {}
    ~GlobalTrajData() {}

    bool localTrajReachTarget() { return fabs(local_end_time_ - global_duration_) < 0.1; }

    void setGlobalTraj(const PolynomialTraj &traj, const ros::Time &time)
    {
      global_traj_ = traj;
      global_traj_.init();
      global_duration_ = global_traj_.getTimeSum();
      global_start_time_ = time;

      local_traj_.clear();
      local_start_time_ = -1;
      local_end_time_ = -1;
      time_increase_ = 0.0;
      last_time_inc_ = 0.0;
      last_progress_time_ = 0.0;
    }

    void setLocalTraj(PiecewiseBezier traj, double local_ts, double local_te, double time_inc)
    {
      local_traj_.resize(3);
      local_traj_[0] = traj;
      local_traj_[1] = local_traj_[0].getDerivative();
      local_traj_[2] = local_traj_[1].getDerivative();

      local_start_time_ = local_ts;
      local_end_time_ = local_te;
      global_duration_ += time_inc;
      time_increase_ += time_inc;
      last_time_inc_ = time_inc;
    }

    Eigen::Vector3d getPosition(double t)
    {
      if (local_traj_.size() < 3 || local_start_time_ < 0.0 || local_end_time_ < 0.0)
      {
        double t_safe = std::max(0.0, std::min(t, global_duration_));
        return global_traj_.evaluate(t_safe);
      }

      if (t >= -1e-3 && t <= local_start_time_)
      {
        return global_traj_.evaluate(t - time_increase_ + last_time_inc_);
      }
      else if (t >= local_end_time_ && t <= global_duration_ + 1e-3)
      {
        return global_traj_.evaluate(t - time_increase_);
      }
      else
      {
        double tm, tmp;
        local_traj_[0].getTimeSpan(tm, tmp);
        return local_traj_[0].evaluate(tm + t - local_start_time_);
      }
    }

    Eigen::Vector3d getVelocity(double t)
    {
      if (local_traj_.size() < 3 || local_start_time_ < 0.0 || local_end_time_ < 0.0)
      {
        double t_safe = std::max(0.0, std::min(t, global_duration_));
        return global_traj_.evaluateVel(t_safe);
      }

      if (t >= -1e-3 && t <= local_start_time_)
      {
        return global_traj_.evaluateVel(t);
      }
      else if (t >= local_end_time_ && t <= global_duration_ + 1e-3)
      {
        return global_traj_.evaluateVel(t - time_increase_);
      }
      else
      {
        double tm, tmp;
        local_traj_[0].getTimeSpan(tm, tmp);
        return local_traj_[1].evaluate(tm + t - local_start_time_);
      }
    }

    Eigen::Vector3d getAcceleration(double t)
    {
      if (local_traj_.size() < 3 || local_start_time_ < 0.0 || local_end_time_ < 0.0)
      {
        double t_safe = std::max(0.0, std::min(t, global_duration_));
        return global_traj_.evaluateAcc(t_safe);
      }

      if (t >= -1e-3 && t <= local_start_time_)
      {
        return global_traj_.evaluateAcc(t);
      }
      else if (t >= local_end_time_ && t <= global_duration_ + 1e-3)
      {
        return global_traj_.evaluateAcc(t - time_increase_);
      }
      else
      {
        double tm, tmp;
        local_traj_[0].getTimeSpan(tm, tmp);
        return local_traj_[2].evaluate(tm + t - local_start_time_);
      }
    }

    /**
     * @brief Get Bezier parameterization data within a sphere
     */
    void getTrajByRadius(const double &start_t, const double &des_radius, const double &dist_pt,
                         vector<Eigen::Vector3d> &point_set, vector<Eigen::Vector3d> &start_end_derivative,
                         double &dt, double &seg_duration)
    {
      double seg_length = 0.0;
      double seg_time = 0.0;
      double radius = 0.0;

      double delt = 0.2;
      Eigen::Vector3d first_pt = getPosition(start_t);
      Eigen::Vector3d prev_pt = first_pt;
      Eigen::Vector3d cur_pt;

      while (radius < des_radius && seg_time < global_duration_ - start_t - 1e-3)
      {
        seg_time += delt;
        seg_time = min(seg_time, global_duration_ - start_t);

        cur_pt = getPosition(start_t + seg_time);
        seg_length += (cur_pt - prev_pt).norm();
        prev_pt = cur_pt;
        radius = (cur_pt - first_pt).norm();
      }

      int seg_num = floor(seg_length / dist_pt);

      seg_duration = seg_time;
      dt = seg_time / seg_num;

      for (double tp = 0.0; tp <= seg_time + 1e-4; tp += dt)
      {
        cur_pt = getPosition(start_t + tp);
        point_set.push_back(cur_pt);
      }

      start_end_derivative.push_back(getVelocity(start_t));
      start_end_derivative.push_back(getVelocity(start_t + seg_time));
      start_end_derivative.push_back(getAcceleration(start_t));
      start_end_derivative.push_back(getAcceleration(start_t + seg_time));
    }

    /**
     * @brief Get Bezier parameterization data for fixed duration
     */
    void getTrajByDuration(double start_t, double duration, int seg_num,
                           vector<Eigen::Vector3d> &point_set,
                           vector<Eigen::Vector3d> &start_end_derivative, double &dt)
    {
      dt = duration / seg_num;
      Eigen::Vector3d cur_pt;
      for (double tp = 0.0; tp <= duration + 1e-4; tp += dt)
      {
        cur_pt = getPosition(start_t + tp);
        point_set.push_back(cur_pt);
      }

      start_end_derivative.push_back(getVelocity(start_t));
      start_end_derivative.push_back(getVelocity(start_t + duration));
      start_end_derivative.push_back(getAcceleration(start_t));
      start_end_derivative.push_back(getAcceleration(start_t + duration));
    }
  };

  /**
   * @brief Planning parameters
   */
  struct PlanParameters
  {
    double max_vel_, max_acc_, max_jerk_;
    double ctrl_pt_dist;                  // Distance between control points
    double feasibility_tolerance_;
    double planning_horizen_;

    double time_search_ = 0.0;
    double time_optimize_ = 0.0;
    double time_adjust_ = 0.0;
  };

  /**
   * @brief Local trajectory data container
   */
  struct LocalTrajData
  {
    int traj_id_;
    double duration_;
    double global_time_offset;
    ros::Time start_time_;
    Eigen::Vector3d start_pos_;
    PiecewiseBezier position_traj_, velocity_traj_, acceleration_traj_;
  };

} // namespace ego_planner

#endif
