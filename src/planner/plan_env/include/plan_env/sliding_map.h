/**
 * @file sliding_map.h
 * @brief Sliding Window Local Map for Unbounded Z-axis Navigation
 * 
 * This module implements a rolling buffer approach for maintaining a local occupancy grid
 * that follows the drone's position. The key innovation is:
 * 
 * 1. The map origin moves with the drone (centered around current odometry)
 * 2. When the drone moves beyond a threshold, the map "rolls" to follow
 * 3. World coordinates are converted to local coordinates for all queries
 * 4. This allows navigation in unbounded Z-axis scenarios (like inclined corridors)
 * 
 * Key concepts:
 * - map_origin_: The world-frame origin of the current local map
 * - local coordinates: P_local = P_world - map_origin_
 * - The buffer wraps around when the drone moves
 */

#ifndef _SLIDING_MAP_H_
#define _SLIDING_MAP_H_

#include <Eigen/Eigen>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>

namespace ego_planner
{

/**
 * @brief Parameters for the sliding local map
 */
struct SlidingMapParams
{
  // Map dimensions (local frame)
  double local_size_x_, local_size_y_, local_size_z_;
  double resolution_;
  
  // Sliding thresholds (when to shift the map)
  double shift_thresh_x_, shift_thresh_y_, shift_thresh_z_;
  
  // Safety margins
  double ground_clearance_;    // Minimum height above ground to consider safe
  double unknown_as_occupied_; // Treat unknown space as occupied (0/1)
  
  // Enable flag
  bool enable_sliding_;
  
  SlidingMapParams() :
    local_size_x_(20.0), local_size_y_(20.0), local_size_z_(10.0),
    resolution_(0.1),
    shift_thresh_x_(5.0), shift_thresh_y_(5.0), shift_thresh_z_(3.0),
    ground_clearance_(0.3),
    unknown_as_occupied_(0),
    enable_sliding_(true)
  {}
};

/**
 * @brief Sliding window local map manager
 * 
 * This class wraps around a fixed-size occupancy buffer but maintains
 * a moving origin that follows the drone. The key operations are:
 * 
 * 1. worldToLocal(): Convert world coordinates to local buffer coordinates
 * 2. isInLocalMap(): Check if a world point is within the current local map
 * 3. updateMapOrigin(): Shift the map origin when drone moves
 */
class SlidingMap
{
public:
  SlidingMap() : initialized_(false) {}
  ~SlidingMap() {}

  /**
   * @brief Initialize the sliding map with parameters
   */
  void init(ros::NodeHandle& nh);

  /**
   * @brief Update the map center based on drone's current position
   * @param drone_pos Current drone position in world frame
   * @return true if the map origin was shifted
   */
  bool updateMapCenter(const Eigen::Vector3d& drone_pos);

  /**
   * @brief Convert world coordinates to local buffer coordinates
   * @param world_pos Position in world frame
   * @param local_pos Output position in local buffer frame
   * @return true if the point is within the local map
   */
  bool worldToLocal(const Eigen::Vector3d& world_pos, Eigen::Vector3d& local_pos) const;

  /**
   * @brief Convert local buffer coordinates to world coordinates
   */
  Eigen::Vector3d localToWorld(const Eigen::Vector3d& local_pos) const;

  /**
   * @brief Check if a world position is within the current local map
   */
  bool isInLocalMap(const Eigen::Vector3d& world_pos) const;

  /**
   * @brief Get the current map origin in world frame
   */
  Eigen::Vector3d getMapOrigin() const { return map_origin_; }

  /**
   * @brief Get the current map center (drone position) in world frame
   */
  Eigen::Vector3d getMapCenter() const { return map_center_; }

  /**
   * @brief Get local map boundaries in world frame
   */
  void getWorldBounds(Eigen::Vector3d& min_bound, Eigen::Vector3d& max_bound) const;

  /**
   * @brief Get the dynamic height bounds based on current position
   * This is the key function for enabling Z-axis negative flight
   * @param current_z Current drone Z position
   * @param min_z Output minimum safe Z (relative to drone)
   * @param max_z Output maximum safe Z (relative to drone)
   */
  void getDynamicHeightBounds(double current_z, double& min_z, double& max_z) const;

  /**
   * @brief Get parameters
   */
  const SlidingMapParams& getParams() const { return params_; }

  /**
   * @brief Check if sliding map is enabled and initialized
   */
  bool isEnabled() const { return params_.enable_sliding_ && initialized_; }

  typedef std::shared_ptr<SlidingMap> Ptr;

private:
  SlidingMapParams params_;
  
  // Current map state
  Eigen::Vector3d map_origin_;   // World-frame origin of local map (corner, not center)
  Eigen::Vector3d map_center_;   // World-frame center of local map (follows drone)
  
  // Initialization flag
  bool initialized_;
  
  // Last update position (to detect movement)
  Eigen::Vector3d last_center_;
};

} // namespace ego_planner

#endif // _SLIDING_MAP_H_
