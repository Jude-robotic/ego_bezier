/**
 * @file sliding_map.cpp
 * @brief Implementation of Sliding Window Local Map
 */

#include "plan_env/sliding_map.h"

namespace ego_planner
{

void SlidingMap::init(ros::NodeHandle& nh)
{
  // Read parameters from ROS
  nh.param("sliding_map/enable", params_.enable_sliding_, true);
  nh.param("sliding_map/local_size_x", params_.local_size_x_, 20.0);
  nh.param("sliding_map/local_size_y", params_.local_size_y_, 20.0);
  nh.param("sliding_map/local_size_z", params_.local_size_z_, 10.0);
  nh.param("sliding_map/resolution", params_.resolution_, 0.1);
  nh.param("sliding_map/shift_thresh_x", params_.shift_thresh_x_, 5.0);
  nh.param("sliding_map/shift_thresh_y", params_.shift_thresh_y_, 5.0);
  nh.param("sliding_map/shift_thresh_z", params_.shift_thresh_z_, 3.0);
  nh.param("sliding_map/ground_clearance", params_.ground_clearance_, 0.3);
  nh.param("sliding_map/unknown_as_occupied", params_.unknown_as_occupied_, 0.0);

  if (!params_.enable_sliding_)
  {
    ROS_INFO("[SlidingMap] Sliding map DISABLED");
    initialized_ = false;
    return;
  }

  // Initialize map origin at (0,0,0) - will be updated on first odometry
  map_origin_ = Eigen::Vector3d::Zero();
  map_center_ = Eigen::Vector3d::Zero();
  last_center_ = Eigen::Vector3d::Zero();
  initialized_ = false;  // Will be set true on first updateMapCenter call

  ROS_INFO("[SlidingMap] Initialized: size=(%.1f,%.1f,%.1f), shift_thresh=(%.1f,%.1f,%.1f)",
           params_.local_size_x_, params_.local_size_y_, params_.local_size_z_,
           params_.shift_thresh_x_, params_.shift_thresh_y_, params_.shift_thresh_z_);
}

bool SlidingMap::updateMapCenter(const Eigen::Vector3d& drone_pos)
{
  if (!params_.enable_sliding_)
    return false;

  bool shifted = false;

  // First call - initialize center
  if (!initialized_)
  {
    map_center_ = drone_pos;
    map_origin_ = map_center_ - Eigen::Vector3d(
        params_.local_size_x_ / 2.0,
        params_.local_size_y_ / 2.0,
        params_.local_size_z_ / 2.0);
    last_center_ = map_center_;
    initialized_ = true;
    ROS_INFO("[SlidingMap] First update: center=(%.2f,%.2f,%.2f), origin=(%.2f,%.2f,%.2f)",
             map_center_.x(), map_center_.y(), map_center_.z(),
             map_origin_.x(), map_origin_.y(), map_origin_.z());
    return true;
  }

  // Check if we need to shift the map
  Eigen::Vector3d delta = drone_pos - map_center_;

  // Shift X
  if (fabs(delta.x()) > params_.shift_thresh_x_)
  {
    double shift = (delta.x() > 0) ? params_.shift_thresh_x_ : -params_.shift_thresh_x_;
    map_center_.x() += shift;
    map_origin_.x() += shift;
    shifted = true;
  }

  // Shift Y
  if (fabs(delta.y()) > params_.shift_thresh_y_)
  {
    double shift = (delta.y() > 0) ? params_.shift_thresh_y_ : -params_.shift_thresh_y_;
    map_center_.y() += shift;
    map_origin_.y() += shift;
    shifted = true;
  }

  // Shift Z - THIS IS THE KEY FOR NEGATIVE Z-AXIS FLIGHT
  if (fabs(delta.z()) > params_.shift_thresh_z_)
  {
    double shift = (delta.z() > 0) ? params_.shift_thresh_z_ : -params_.shift_thresh_z_;
    map_center_.z() += shift;
    map_origin_.z() += shift;
    shifted = true;
  }

  if (shifted)
  {
    ROS_INFO_THROTTLE(1.0, "[SlidingMap] Map shifted: center=(%.2f,%.2f,%.2f), origin=(%.2f,%.2f,%.2f)",
             map_center_.x(), map_center_.y(), map_center_.z(),
             map_origin_.x(), map_origin_.y(), map_origin_.z());
  }

  last_center_ = map_center_;
  return shifted;
}

bool SlidingMap::worldToLocal(const Eigen::Vector3d& world_pos, Eigen::Vector3d& local_pos) const
{
  if (!initialized_)
  {
    local_pos = world_pos;  // Fallback: no transformation
    return true;
  }

  // Transform: P_local = P_world - map_origin
  local_pos = world_pos - map_origin_;

  // Check if within local map bounds
  if (local_pos.x() < 0 || local_pos.x() > params_.local_size_x_ ||
      local_pos.y() < 0 || local_pos.y() > params_.local_size_y_ ||
      local_pos.z() < 0 || local_pos.z() > params_.local_size_z_)
  {
    return false;
  }

  return true;
}

Eigen::Vector3d SlidingMap::localToWorld(const Eigen::Vector3d& local_pos) const
{
  if (!initialized_)
    return local_pos;

  return local_pos + map_origin_;
}

bool SlidingMap::isInLocalMap(const Eigen::Vector3d& world_pos) const
{
  if (!initialized_)
    return true;  // Before initialization, consider everything "in map"

  Eigen::Vector3d local;
  return worldToLocal(world_pos, local);
}

void SlidingMap::getWorldBounds(Eigen::Vector3d& min_bound, Eigen::Vector3d& max_bound) const
{
  if (!initialized_)
  {
    // Return default large bounds
    min_bound = Eigen::Vector3d(-1000, -1000, -1000);
    max_bound = Eigen::Vector3d(1000, 1000, 1000);
    return;
  }

  min_bound = map_origin_;
  max_bound = map_origin_ + Eigen::Vector3d(params_.local_size_x_, 
                                             params_.local_size_y_, 
                                             params_.local_size_z_);
}

void SlidingMap::getDynamicHeightBounds(double current_z, double& min_z, double& max_z) const
{
  if (!initialized_ || !params_.enable_sliding_)
  {
    // Fallback to traditional bounds
    min_z = params_.ground_clearance_;
    max_z = 100.0;  // Very high ceiling
    return;
  }

  // Dynamic bounds centered around current position
  // This is the KEY innovation for Z-negative flight
  min_z = map_origin_.z() + params_.ground_clearance_;
  max_z = map_origin_.z() + params_.local_size_z_ - params_.ground_clearance_;

  // Ensure reasonable range
  if (max_z <= min_z)
  {
    max_z = min_z + params_.local_size_z_;
  }
}

} // namespace ego_planner
