/**
 * @file local_coordinate_system.cpp
 * @brief 局部参考坐标系统实现
 */

#include <plan_manage/local_coordinate_system.h>

namespace ego_planner
{

void LocalCoordinateSystem::init(ros::NodeHandle &nh)
{
    node_ = nh;
    
    // 读取参数
    node_.param("local_frame/enable", enable_local_frame_, true);
    node_.param("local_frame/auto_update", auto_update_origin_, true);
    node_.param("local_frame/update_thresh", update_distance_thresh_, 5.0);
    
    double local_x, local_y, local_z;
    node_.param("local_frame/map_size_x", local_x, 40.0);
    node_.param("local_frame/map_size_y", local_y, 40.0);
    node_.param("local_frame/map_size_z", local_z, 5.0);
    local_map_size_ << local_x / 2.0, local_y / 2.0, local_z / 2.0;  // 存储半径
    
    // 历史记录
    node_.param("local_frame/max_history", max_history_size_, 100);
    origin_history_.reserve(max_history_size_);
    
    initialized_ = false;
    
    ROS_INFO("[LocalCoordinateSystem] Initialized");
    ROS_INFO("  Enable: %s", enable_local_frame_ ? "true" : "false");
    ROS_INFO("  Auto Update: %s", auto_update_origin_ ? "true" : "false");
    ROS_INFO("  Update Threshold: %.2f m", update_distance_thresh_);
    ROS_INFO("  Local Map Size: [%.1f, %.1f, %.1f] m", 
             local_map_size_.x() * 2, local_map_size_.y() * 2, local_map_size_.z() * 2);
}

void LocalCoordinateSystem::setOrigin(const Eigen::Vector3d &global_pos)
{
    local_origin_ = global_pos;
    initialized_ = true;
    
    // 记录历史
    origin_history_.push_back(global_pos);
    if (origin_history_.size() > max_history_size_)
    {
        origin_history_.erase(origin_history_.begin());
    }
    
    ROS_INFO("[LocalCoordinateSystem] Origin set to: [%.2f, %.2f, %.2f]", 
             global_pos.x(), global_pos.y(), global_pos.z());
}

bool LocalCoordinateSystem::updateOrigin(const Eigen::Vector3d &current_global_pos)
{
    if (!enable_local_frame_ || !auto_update_origin_)
        return false;
    
    if (!initialized_)
    {
        setOrigin(current_global_pos);
        return true;
    }
    
    // 检查是否需要更新
    double dist = (current_global_pos - local_origin_).norm();
    if (dist > update_distance_thresh_)
    {
        ROS_INFO("[LocalCoordinateSystem] Updating origin (distance: %.2f > %.2f)", 
                 dist, update_distance_thresh_);
        setOrigin(current_global_pos);
        return true;
    }
    
    return false;
}

Eigen::Vector3d LocalCoordinateSystem::globalToLocal(const Eigen::Vector3d &global_pos) const
{
    if (!enable_local_frame_ || !initialized_)
    {
        return global_pos;  // 如果未启用，直接返回全局坐标
    }
    
    return global_pos - local_origin_;
}

Eigen::Vector3d LocalCoordinateSystem::localToGlobal(const Eigen::Vector3d &local_pos) const
{
    if (!enable_local_frame_ || !initialized_)
    {
        return local_pos;  // 如果未启用，直接返回
    }
    
    return local_pos + local_origin_;
}

bool LocalCoordinateSystem::isInLocalBounds(const Eigen::Vector3d &local_pos) const
{
    // 检查是否在局部坐标系的边界内
    return (fabs(local_pos.x()) <= local_map_size_.x() &&
            fabs(local_pos.y()) <= local_map_size_.y() &&
            fabs(local_pos.z()) <= local_map_size_.z());
}

bool LocalCoordinateSystem::isGlobalPosValid(const Eigen::Vector3d &global_pos) const
{
    if (!enable_local_frame_)
        return true;  // 如果未启用局部坐标系，认为所有位置都有效
    
    Eigen::Vector3d local_pos = globalToLocal(global_pos);
    return isInLocalBounds(local_pos);
}

double LocalCoordinateSystem::getDistanceToOrigin(const Eigen::Vector3d &global_pos) const
{
    if (!initialized_)
        return 0.0;
    
    return (global_pos - local_origin_).norm();
}

void LocalCoordinateSystem::printDebugInfo() const
{
    std::cout << "\n========== Local Coordinate System ==========" << std::endl;
    std::cout << "Enabled: " << (enable_local_frame_ ? "Yes" : "No") << std::endl;
    std::cout << "Initialized: " << (initialized_ ? "Yes" : "No") << std::endl;
    
    if (initialized_)
    {
        std::cout << "Origin (Global): [" << local_origin_.x() << ", " 
                  << local_origin_.y() << ", " << local_origin_.z() << "]" << std::endl;
        std::cout << "Map Size: [" << local_map_size_.x() * 2 << ", " 
                  << local_map_size_.y() * 2 << ", " << local_map_size_.z() * 2 << "]" << std::endl;
        std::cout << "Auto Update: " << (auto_update_origin_ ? "Yes" : "No") << std::endl;
        std::cout << "Update Threshold: " << update_distance_thresh_ << " m" << std::endl;
        std::cout << "Origin History: " << origin_history_.size() << " entries" << std::endl;
    }
    std::cout << "============================================\n" << std::endl;
}

} // namespace ego_planner
