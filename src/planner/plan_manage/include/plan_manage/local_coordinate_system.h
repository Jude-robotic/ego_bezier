/**
 * @file local_coordinate_system.h
 * @brief 局部参考坐标系统 - 将绝对位置转换为相对位置
 * 
 * 功能:
 * 1. 维护一个动态移动的局部坐标系原点
 * 2. 提供全局坐标 <-> 局部坐标的变换
 * 3. 支持滑动窗口式的工作空间管理
 * 4. 适配倾斜巷道等复杂环境
 * 
 * 作者: Jude
 * 日期: 2026-01-16
 */

#ifndef _LOCAL_COORDINATE_SYSTEM_H_
#define _LOCAL_COORDINATE_SYSTEM_H_

#include <Eigen/Eigen>
#include <ros/ros.h>
#include <iostream>

namespace ego_planner
{

/**
 * @brief 局部参考坐标系统
 * 
 * 核心思想：
 * - 以无人机当前位置为中心建立局部坐标系
 * - 所有规划在局部坐标系中进行
 * - 局部坐标系随无人机移动而滑动
 */
class LocalCoordinateSystem
{
public:
    typedef std::shared_ptr<LocalCoordinateSystem> Ptr;
    LocalCoordinateSystem() : initialized_(false), 
                             enable_local_frame_(false),
                             auto_update_origin_(true),
                             update_distance_thresh_(5.0) {}
    
    ~LocalCoordinateSystem() {}

    /**
     * @brief 初始化局部坐标系
     * @param nh ROS节点句柄
     */
    void init(ros::NodeHandle &nh);

    /**
     * @brief 设置局部坐标系原点
     * @param global_pos 全局坐标系下的位置
     */
    void setOrigin(const Eigen::Vector3d &global_pos);

    /**
     * @brief 更新局部坐标系原点（如果满足更新条件）
     * @param current_global_pos 无人机当前全局位置
     * @return true 如果原点被更新
     */
    bool updateOrigin(const Eigen::Vector3d &current_global_pos);

    /**
     * @brief 全局坐标转局部坐标
     * @param global_pos 全局坐标
     * @return 局部坐标
     */
    Eigen::Vector3d globalToLocal(const Eigen::Vector3d &global_pos) const;

    /**
     * @brief 局部坐标转全局坐标
     * @param local_pos 局部坐标
     * @return 全局坐标
     */
    Eigen::Vector3d localToGlobal(const Eigen::Vector3d &local_pos) const;

    /**
     * @brief 检查局部坐标是否在有效范围内
     * @param local_pos 局部坐标
     * @return true 如果在有效范围内
     */
    bool isInLocalBounds(const Eigen::Vector3d &local_pos) const;

    /**
     * @brief 检查全局坐标是否在有效范围内（转换为局部坐标后检查）
     * @param global_pos 全局坐标
     * @return true 如果在有效范围内
     */
    bool isGlobalPosValid(const Eigen::Vector3d &global_pos) const;

    /**
     * @brief 获取当前局部坐标系原点（全局坐标）
     * @return 局部坐标系原点
     */
    Eigen::Vector3d getOrigin() const { return local_origin_; }

    /**
     * @brief 获取局部坐标系的边界尺寸
     * @return 边界尺寸 (x_size, y_size, z_size)
     */
    Eigen::Vector3d getLocalMapSize() const { return local_map_size_; }

    /**
     * @brief 设置局部坐标系的边界尺寸
     * @param size 边界尺寸 (x_size, y_size, z_size)
     */
    void setLocalMapSize(const Eigen::Vector3d &size) { local_map_size_ = size; }

    /**
     * @brief 启用/禁用局部坐标系
     * @param enable true表示启用
     */
    void setEnable(bool enable) { enable_local_frame_ = enable; }

    /**
     * @brief 检查局部坐标系是否已启用
     * @return true 如果已启用
     */
    bool isEnabled() const { return enable_local_frame_; }

    /**
     * @brief 设置自动更新原点
     * @param auto_update true表示自动更新
     */
    void setAutoUpdate(bool auto_update) { auto_update_origin_ = auto_update; }

    /**
     * @brief 设置原点更新距离阈值
     * @param thresh 阈值（米）
     */
    void setUpdateThreshold(double thresh) { update_distance_thresh_ = thresh; }

    /**
     * @brief 获取到原点的距离
     * @param global_pos 全局位置
     * @return 距离
     */
    double getDistanceToOrigin(const Eigen::Vector3d &global_pos) const;

    /**
     * @brief 打印调试信息
     */
    void printDebugInfo() const;

private:
    bool initialized_;              // 是否已初始化
    bool enable_local_frame_;       // 是否启用局部坐标系
    bool auto_update_origin_;       // 是否自动更新原点
    double update_distance_thresh_; // 原点更新距离阈值

    Eigen::Vector3d local_origin_;  // 局部坐标系原点（全局坐标）
    Eigen::Vector3d local_map_size_; // 局部地图尺寸 (半径)
    
    // 用于记录原点更新历史
    std::vector<Eigen::Vector3d> origin_history_;
    int max_history_size_;

    ros::NodeHandle node_;
};

} // namespace ego_planner

#endif // _LOCAL_COORDINATE_SYSTEM_H_
