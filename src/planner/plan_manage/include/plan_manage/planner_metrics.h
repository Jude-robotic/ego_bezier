/**
 * @file planner_metrics.h
 * @brief 规划器性能指标收集与评估系统
 * 
 * 提供以下指标:
 * 1. 任务完成率 (成功到达目标点的比率)
 * 2. 轨迹平滑度 (jerk/snap指标)
 * 3. 碰撞安全性 (最小安全距离)
 * 4. 计算效率 (优化时间、迭代次数)
 * 5. 路径质量 (路径长度、时间)
 * 6. 动力学可行性 (速度/加速度违规)
 */

#ifndef _PLANNER_METRICS_H_
#define _PLANNER_METRICS_H_

#include <ros/ros.h>
#include <Eigen/Eigen>
#include <fstream>
#include <vector>
#include <deque>
#include <numeric>
#include <visualization_msgs/Marker.h>
#include <std_msgs/Float64MultiArray.h>

namespace ego_planner
{

struct PlanningMetrics
{
    // 任务执行指标
    double mission_success_rate;      // 任务成功率 [0,1]
    int total_attempts;                // 总尝试次数
    int successful_plans;              // 成功规划次数
    double goal_reached_count;         // 到达目标次数
    
    // 轨迹质量指标
    double avg_smoothness;             // 平均平滑度 (snap积分)
    double max_jerk;                   // 最大jerk值
    double avg_path_length;            // 平均路径长度
    double avg_trajectory_time;        // 平均轨迹时间
    
    // 安全性指标
    double min_obstacle_distance;      // 最小障碍物距离
    double avg_clearance;              // 平均安全余量
    int collision_count;               // 碰撞次数
    double safety_violation_rate;      // 安全违规率
    
    // 计算效率指标
    double avg_optimization_time;      // 平均优化时间 (ms)
    double max_optimization_time;      // 最大优化时间 (ms)
    double avg_iterations;             // 平均迭代次数
    double avg_replan_frequency;       // 平均重规划频率 (Hz)
    
    // 动力学可行性指标
    double max_velocity;               // 最大速度
    double max_acceleration;           // 最大加速度
    double velocity_violation_rate;    // 速度违规率
    double acceleration_violation_rate;// 加速度违规率
    
    // 优化收敛指标
    double final_cost;                 // 最终代价值
    double cost_reduction_ratio;       // 代价下降率
    int early_termination_count;       // 提前终止次数
    
    PlanningMetrics() { reset(); }
    
    void reset() {
        mission_success_rate = 0.0;
        total_attempts = 0;
        successful_plans = 0;
        goal_reached_count = 0;
        avg_smoothness = 0.0;
        max_jerk = 0.0;
        avg_path_length = 0.0;
        avg_trajectory_time = 0.0;
        min_obstacle_distance = 1e10;
        avg_clearance = 0.0;
        collision_count = 0;
        safety_violation_rate = 0.0;
        avg_optimization_time = 0.0;
        max_optimization_time = 0.0;
        avg_iterations = 0.0;
        avg_replan_frequency = 0.0;
        max_velocity = 0.0;
        max_acceleration = 0.0;
        velocity_violation_rate = 0.0;
        acceleration_violation_rate = 0.0;
        final_cost = 0.0;
        cost_reduction_ratio = 0.0;
        early_termination_count = 0;
    }
};

class PlannerMetricsRecorder
{
public:
    PlannerMetricsRecorder(ros::NodeHandle& nh);
    ~PlannerMetricsRecorder();
    
    // 记录单次规划事件
    void recordPlanningAttempt(bool success);
    void recordOptimizationTime(double time_ms);
    void recordIterations(int iters);
    void recordFinalCost(double cost, double initial_cost);
    void recordTrajectorySmooth(const Eigen::MatrixXd& control_points, double dt);
    void recordSafety(double min_dist, double avg_clear);
    void recordDynamics(double max_vel, double max_acc, bool vel_violated, bool acc_violated);
    void recordPathLength(double length);
    void recordGoalReached();
    
    // 获取统计指标
    PlanningMetrics getMetrics() const { return metrics_; }
    
    
    // 重置所有指标
    void resetMetrics();
    // 保存和显示
    void saveMetricsToFile(const std::string& filename);
    void publishMetricsVisualization();
    void printMetricsSummary();
    
private:
    ros::NodeHandle nh_;
    ros::Publisher metrics_pub_;
    ros::Publisher text_marker_pub_;
    
    PlanningMetrics metrics_;
    
    // 滑动窗口统计
    std::deque<double> opt_time_window_;
    std::deque<int> iter_window_;
    std::deque<double> smoothness_window_;
    std::deque<double> path_length_window_;
    std::deque<double> clearance_window_;
    
    const int window_size_ = 50;  // 窗口大小
    
    // 时间戳
    ros::Time last_replan_time_;
    std::deque<double> replan_intervals_;
    
    // 文件输出
    std::ofstream metrics_file_;
    std::string log_dir_;
    
    // 辅助函数
    double computeAverage(const std::deque<double>& data);
    void updateSlidingWindow(std::deque<double>& window, double value);
    void updateSlidingWindow(std::deque<int>& window, int value);
    void publishTextMarker(const std::string& text, int id);
};

} // namespace ego_planner

#endif
