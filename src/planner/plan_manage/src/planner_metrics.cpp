/**
 * @file planner_metrics.cpp
 * @brief 规划器性能指标实现
 */

#include <plan_manage/planner_metrics.h>
#include <iomanip>
#include <sstream>
#include <sys/stat.h>
#include <ctime>

namespace ego_planner
{

PlannerMetricsRecorder::PlannerMetricsRecorder(ros::NodeHandle& nh) : nh_(nh)
{
    // 初始化发布器
    metrics_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("/planner_metrics", 10);
    text_marker_pub_ = nh_.advertise<visualization_msgs::Marker>("/planner_metrics_text", 10);
    
    // 创建日志目录
    log_dir_ = "/tmp/ego_planner_metrics/";
    mkdir(log_dir_.c_str(), 0777);
    
    // 生成时间戳文件名
    std::time_t now = std::time(nullptr);
    char timestamp[100];
    std::strftime(timestamp, sizeof(timestamp), "%Y%m%d_%H%M%S", std::localtime(&now));
    
    std::string filename = log_dir_ + "metrics_" + std::string(timestamp) + ".csv";
    metrics_file_.open(filename);
    
    // 写CSV头
    metrics_file_ << "timestamp,success_rate,avg_opt_time_ms,avg_iterations,"
                  << "avg_smoothness,min_clearance,max_velocity,max_acceleration,"
                  << "path_length,final_cost,collision_count,replan_freq_hz\n";
    
    ROS_INFO("\033[32m[Metrics] Logging to: %s\033[0m", filename.c_str());
    
    last_replan_time_ = ros::Time::now();
}

PlannerMetricsRecorder::~PlannerMetricsRecorder()
{
    if (metrics_file_.is_open()) {
        metrics_file_.close();
    }
    printMetricsSummary();
}

void PlannerMetricsRecorder::recordPlanningAttempt(bool success)
{
    metrics_.total_attempts++;
    if (success) {
        metrics_.successful_plans++;
    }
    metrics_.mission_success_rate = 
        static_cast<double>(metrics_.successful_plans) / metrics_.total_attempts;
    
    // 记录重规划频率
    ros::Time now = ros::Time::now();
    double interval = (now - last_replan_time_).toSec();
    if (interval > 0.001) {  // 避免除零
        updateSlidingWindow(replan_intervals_, interval);
        metrics_.avg_replan_frequency = 1.0 / computeAverage(replan_intervals_);
    }
    last_replan_time_ = now;
}

void PlannerMetricsRecorder::recordOptimizationTime(double time_ms)
{
    updateSlidingWindow(opt_time_window_, time_ms);
    metrics_.avg_optimization_time = computeAverage(opt_time_window_);
    metrics_.max_optimization_time = std::max(metrics_.max_optimization_time, time_ms);
}

void PlannerMetricsRecorder::recordIterations(int iters)
{
    updateSlidingWindow(iter_window_, iters);
    
    // 计算平均迭代次数
    double sum = 0;
    for (int val : iter_window_) sum += val;
    metrics_.avg_iterations = sum / iter_window_.size();
}

void PlannerMetricsRecorder::recordFinalCost(double cost, double initial_cost)
{
    metrics_.final_cost = cost;
    if (initial_cost > 1e-6) {
        metrics_.cost_reduction_ratio = (initial_cost - cost) / initial_cost;
    }
}

void PlannerMetricsRecorder::recordTrajectorySmooth(
    const Eigen::MatrixXd& control_points, double dt)
{
    // 计算轨迹的平滑度 (基于控制点的四阶差分近似snap)
    int n = control_points.cols();
    if (n < 5) return;
    
    double smoothness = 0.0;
    double max_jerk = 0.0;
    
    // 计算相邻控制点的三阶差分 (近似jerk)
    for (int i = 0; i < n - 3; ++i) {
        Eigen::Vector3d diff1 = control_points.col(i+1) - control_points.col(i);
        Eigen::Vector3d diff2 = control_points.col(i+2) - control_points.col(i+1);
        Eigen::Vector3d diff3 = control_points.col(i+3) - control_points.col(i+2);
        
        Eigen::Vector3d jerk = (diff3 - diff2) - (diff2 - diff1);
        double jerk_norm = jerk.norm() / (dt * dt * dt);
        
        max_jerk = std::max(max_jerk, jerk_norm);
        smoothness += jerk_norm * jerk_norm * dt;
    }
    
    updateSlidingWindow(smoothness_window_, smoothness);
    metrics_.avg_smoothness = computeAverage(smoothness_window_);
    metrics_.max_jerk = max_jerk;
}

void PlannerMetricsRecorder::recordSafety(double min_dist, double avg_clear)
{
    metrics_.min_obstacle_distance = std::min(metrics_.min_obstacle_distance, min_dist);
    updateSlidingWindow(clearance_window_, avg_clear);
    metrics_.avg_clearance = computeAverage(clearance_window_);
    
    // 记录碰撞
    if (min_dist < 0.1) {  // 碰撞阈值
        metrics_.collision_count++;
    }
    
    // 计算安全违规率
    if (metrics_.total_attempts > 0) {
        metrics_.safety_violation_rate = 
            static_cast<double>(metrics_.collision_count) / metrics_.total_attempts;
    }
}

void PlannerMetricsRecorder::recordDynamics(
    double max_vel, double max_acc, bool vel_violated, bool acc_violated)
{
    metrics_.max_velocity = std::max(metrics_.max_velocity, max_vel);
    metrics_.max_acceleration = std::max(metrics_.max_acceleration, max_acc);
    
    static int vel_violations = 0;
    static int acc_violations = 0;
    
    if (vel_violated) vel_violations++;
    if (acc_violated) acc_violations++;
    
    if (metrics_.total_attempts > 0) {
        metrics_.velocity_violation_rate = 
            static_cast<double>(vel_violations) / metrics_.total_attempts;
        metrics_.acceleration_violation_rate = 
            static_cast<double>(acc_violations) / metrics_.total_attempts;
    }
}

void PlannerMetricsRecorder::recordPathLength(double length)
{
    updateSlidingWindow(path_length_window_, length);
    metrics_.avg_path_length = computeAverage(path_length_window_);
}

void PlannerMetricsRecorder::recordGoalReached()
{
    metrics_.goal_reached_count++;
}

void PlannerMetricsRecorder::saveMetricsToFile(const std::string& filename)
{
    if (!metrics_file_.is_open()) return;
    
    // 写入当前时间戳和所有指标
    metrics_file_ << ros::Time::now().toSec() << ","
                  << metrics_.mission_success_rate << ","
                  << metrics_.avg_optimization_time << ","
                  << metrics_.avg_iterations << ","
                  << metrics_.avg_smoothness << ","
                  << metrics_.min_obstacle_distance << ","
                  << metrics_.max_velocity << ","
                  << metrics_.max_acceleration << ","
                  << metrics_.avg_path_length << ","
                  << metrics_.final_cost << ","
                  << metrics_.collision_count << ","
                  << metrics_.avg_replan_frequency << "\n";
    
    metrics_file_.flush();
}

void PlannerMetricsRecorder::publishMetricsVisualization()
{
    // 发布数值数据
    std_msgs::Float64MultiArray msg;
    msg.data.push_back(metrics_.mission_success_rate);
    msg.data.push_back(metrics_.avg_optimization_time);
    msg.data.push_back(metrics_.avg_iterations);
    msg.data.push_back(metrics_.avg_smoothness);
    msg.data.push_back(metrics_.min_obstacle_distance);
    msg.data.push_back(metrics_.avg_clearance);
    msg.data.push_back(metrics_.max_velocity);
    msg.data.push_back(metrics_.max_acceleration);
    msg.data.push_back(metrics_.avg_path_length);
    msg.data.push_back(metrics_.final_cost);
    msg.data.push_back(metrics_.avg_replan_frequency);
    metrics_pub_.publish(msg);
    
    // 发布文本标记
    std::stringstream ss;
    ss << std::fixed << std::setprecision(2);
    ss << "=== Planner Metrics ===\n";
    ss << "Success Rate: " << (metrics_.mission_success_rate * 100) << "%\n";
    ss << "Opt Time: " << metrics_.avg_optimization_time << " ms\n";
    ss << "Iterations: " << metrics_.avg_iterations << "\n";
    ss << "Smoothness: " << metrics_.avg_smoothness << "\n";
    ss << "Min Clearance: " << metrics_.min_obstacle_distance << " m\n";
    ss << "Replan Freq: " << metrics_.avg_replan_frequency << " Hz\n";
    ss << "Collisions: " << metrics_.collision_count;
    
    publishTextMarker(ss.str(), 0);
}

void PlannerMetricsRecorder::printMetricsSummary()
{
    ROS_INFO("\033[1;36m========================================\033[0m");
    ROS_INFO("\033[1;36m       PLANNER PERFORMANCE METRICS      \033[0m");
    ROS_INFO("\033[1;36m========================================\033[0m");
    
    ROS_INFO("\033[1;33m[Task Completion]\033[0m");
    ROS_INFO("  Success Rate:      \033[1;32m%.1f%%\033[0m (%d/%d)", 
             metrics_.mission_success_rate * 100, 
             metrics_.successful_plans, metrics_.total_attempts);
    ROS_INFO("  Goals Reached:     %d", (int)metrics_.goal_reached_count);
    
    ROS_INFO("\033[1;33m[Trajectory Quality]\033[0m");
    ROS_INFO("  Avg Smoothness:    %.4f", metrics_.avg_smoothness);
    ROS_INFO("  Max Jerk:          %.2f m/s³", metrics_.max_jerk);
    ROS_INFO("  Avg Path Length:   %.2f m", metrics_.avg_path_length);
    
    ROS_INFO("\033[1;33m[Safety]\033[0m");
    ROS_INFO("  Min Obstacle Dist: \033[1;%dm%.3f m\033[0m", 
             metrics_.min_obstacle_distance > 0.3 ? 32 : 31,
             metrics_.min_obstacle_distance);
    ROS_INFO("  Avg Clearance:     %.3f m", metrics_.avg_clearance);
    ROS_INFO("  Collision Count:   \033[1;%dm%d\033[0m", 
             metrics_.collision_count == 0 ? 32 : 31,
             metrics_.collision_count);
    ROS_INFO("  Safety Violation:  %.1f%%", metrics_.safety_violation_rate * 100);
    
    ROS_INFO("\033[1;33m[Computational Efficiency]\033[0m");
    ROS_INFO("  Avg Opt Time:      %.2f ms", metrics_.avg_optimization_time);
    ROS_INFO("  Max Opt Time:      %.2f ms", metrics_.max_optimization_time);
    ROS_INFO("  Avg Iterations:    %.1f", metrics_.avg_iterations);
    ROS_INFO("  Replan Frequency:  %.2f Hz", metrics_.avg_replan_frequency);
    
    ROS_INFO("\033[1;33m[Dynamics Feasibility]\033[0m");
    ROS_INFO("  Max Velocity:      %.2f m/s", metrics_.max_velocity);
    ROS_INFO("  Max Acceleration:  %.2f m/s²", metrics_.max_acceleration);
    ROS_INFO("  Vel Violations:    %.1f%%", metrics_.velocity_violation_rate * 100);
    ROS_INFO("  Acc Violations:    %.1f%%", metrics_.acceleration_violation_rate * 100);
    
    ROS_INFO("\033[1;33m[Optimization Convergence]\033[0m");
    ROS_INFO("  Final Cost:        %.4f", metrics_.final_cost);
    ROS_INFO("  Cost Reduction:    %.1f%%", metrics_.cost_reduction_ratio * 100);
    
    ROS_INFO("\033[1;36m========================================\033[0m");
}

void PlannerMetricsRecorder::publishTextMarker(const std::string& text, int id)
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = "world";
    marker.header.stamp = ros::Time::now();
    marker.ns = "metrics_text";
    marker.id = id;
    marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker.action = visualization_msgs::Marker::ADD;
    
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 3.0;
    marker.pose.orientation.w = 1.0;
    
    marker.scale.z = 0.3;  // 文字高度
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 1.0;
    marker.color.a = 1.0;
    
    marker.text = text;
    marker.lifetime = ros::Duration(0.5);
    
    text_marker_pub_.publish(marker);
}

double PlannerMetricsRecorder::computeAverage(const std::deque<double>& data)
{
    if (data.empty()) return 0.0;
    return std::accumulate(data.begin(), data.end(), 0.0) / data.size();
}


void PlannerMetricsRecorder::updateSlidingWindow(std::deque<double>& window, double value)
{
    window.push_back(value);
    if (window.size() > window_size_) {
        window.pop_front();
    }
}

void PlannerMetricsRecorder::updateSlidingWindow(std::deque<int>& window, int value)
{
    window.push_back(value);
    if (window.size() > window_size_) {
        window.pop_front();
    }
}

void PlannerMetricsRecorder::resetMetrics()
{
    metrics_.reset();
    opt_time_window_.clear();
    iter_window_.clear();
    smoothness_window_.clear();
    clearance_window_.clear();
    path_length_window_.clear();
    replan_intervals_.clear();
    
    last_replan_time_ = ros::Time::now();
    
    ROS_INFO("\033[1;33m[Metrics] Metrics reset for new flight\033[0m");
}

} // namespace ego_planner
