/**
 * @file metrics_recorder_node.cpp
 * @brief 改进的性能指标记录节点 - 只记录起飞到到达终点的数据
 * 
 * 改进点:
 * 1. 添加飞行状态检测 (起飞/飞行中/到达)
 * 2. 只在飞行过程中记录数据
 * 3. 检测无人机位置变化判断是否起飞
 */

#include <ros/ros.h>
#include <plan_manage/planner_metrics.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <quadrotor_msgs/PositionCommand.h>

using namespace ego_planner;

enum FlightState {
    IDLE,           // 空闲状态（未起飞）
    TAKING_OFF,     // 起飞中
    FLYING,         // 飞行中
    ARRIVED         // 已到达目标
};

class MetricsRecorderNode {
private:
    ros::NodeHandle nh_;
    
    std::shared_ptr<PlannerMetricsRecorder> metrics_recorder_;
    
    // 订阅器
    ros::Subscriber odom_sub_;
    ros::Subscriber traj_sub_;
    ros::Subscriber cmd_sub_;
    ros::Subscriber goal_sub_;
    
    // 状态变量
    Eigen::Vector3d current_pos_, last_pos_, start_pos_;
    Eigen::Vector3d current_vel_;
    Eigen::Vector3d goal_pos_;
    ros::Time last_update_time_;
    ros::Time last_traj_time_;
    ros::Time flight_start_time_;
    
    FlightState flight_state_;
    bool has_trajectory_ = false;
    bool has_goal_ = false;
    double total_path_length_ = 0.0;
    
    // 参数
    double takeoff_height_threshold_;  // 起飞高度阈值
    double goal_reach_threshold_;      // 到达目标阈值
    double velocity_threshold_;        // 速度阈值（判断是否在飞）
    
    // 定时器
    ros::Timer metrics_timer_;
    
public:
    MetricsRecorderNode(ros::NodeHandle& nh) : nh_(nh), flight_state_(IDLE) {
        // 读取参数
        nh_.param("takeoff_height_threshold", takeoff_height_threshold_, 0.3);
        nh_.param("goal_reach_threshold", goal_reach_threshold_, 0.5);
        nh_.param("velocity_threshold", velocity_threshold_, 0.1);
        
        // 初始化metrics recorder
        metrics_recorder_.reset(new PlannerMetricsRecorder(nh));
        
        // 订阅话题
        odom_sub_ = nh_.subscribe("/odom_world", 10, &MetricsRecorderNode::odomCallback, this);
        traj_sub_ = nh_.subscribe("/planning/pos_cmd", 10, &MetricsRecorderNode::trajCallback, this);
        goal_sub_ = nh_.subscribe("/move_base_simple/goal", 1, &MetricsRecorderNode::goalCallback, this);
        
        // 定时发布指标 (2Hz) - 只在飞行时记录
        metrics_timer_ = nh_.createTimer(ros::Duration(0.5), &MetricsRecorderNode::metricsTimerCallback, this);
        
        last_update_time_ = ros::Time::now();
        last_traj_time_ = ros::Time::now();
        
        ROS_INFO("\033[1;32m[MetricsRecorder] Improved node started!\033[0m");
        ROS_INFO("[MetricsRecorder] Parameters:");
        ROS_INFO("  Takeoff height: %.2f m", takeoff_height_threshold_);
        ROS_INFO("  Goal reach threshold: %.2f m", goal_reach_threshold_);
        ROS_INFO("  Velocity threshold: %.2f m/s", velocity_threshold_);
    }
    
    void updateFlightState() {
        double height_diff = current_pos_.z() - start_pos_.z();
        double vel_norm = current_vel_.norm();
        double dist_to_goal = has_goal_ ? (current_pos_ - goal_pos_).norm() : 1e10;
        
        switch (flight_state_) {
            case IDLE:
                // 检测起飞：高度变化或速度增加
                if (has_goal_ && (height_diff > takeoff_height_threshold_ || vel_norm > velocity_threshold_)) {
                    flight_state_ = TAKING_OFF;
                    flight_start_time_ = ros::Time::now();
                    start_pos_ = current_pos_;
                    total_path_length_ = 0.0;
                    metrics_recorder_->resetMetrics();  // 重置指标
                    ROS_INFO("\033[1;33m[Flight] State: IDLE -> TAKING_OFF (height: %.2f m, vel: %.2f m/s)\033[0m", 
                             height_diff, vel_norm);
                }
                break;
                
            case TAKING_OFF:
                // 起飞完成，进入飞行状态
                if (vel_norm > velocity_threshold_) {
                    flight_state_ = FLYING;
                    ROS_INFO("\033[1;32m[Flight] State: TAKING_OFF -> FLYING\033[0m");
                }
                break;
                
            case FLYING:
                // 检测到达目标
                if (dist_to_goal < goal_reach_threshold_ && vel_norm < velocity_threshold_) {
                    flight_state_ = ARRIVED;
                    double flight_time = (ros::Time::now() - flight_start_time_).toSec();
                    metrics_recorder_->recordGoalReached();
                    ROS_INFO("\033[1;32m[Flight] State: FLYING -> ARRIVED (time: %.2f s, distance: %.2f m)\033[0m", 
                             flight_time, total_path_length_);
                    
                    // 打印最终统计
                    metrics_recorder_->printMetricsSummary();
                }
                break;
                
            case ARRIVED:
                // 保持在到达状态，直到收到新目标
                break;
        }
    }
    
    bool isRecording() const {
        return (flight_state_ == TAKING_OFF || flight_state_ == FLYING);
    }
    
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        current_pos_ << msg->pose.pose.position.x, 
                        msg->pose.pose.position.y, 
                        msg->pose.pose.position.z;
        current_vel_ << msg->twist.twist.linear.x,
                        msg->twist.twist.linear.y,
                        msg->twist.twist.linear.z;
        
        // 累计路径长度 (只在飞行时)
        ros::Time now = ros::Time::now();
        double dt = (now - last_update_time_).toSec();
        
        if (isRecording() && dt > 0.001) {
            double dist = (current_pos_ - last_pos_).norm();
            if (dist < 1.0) {  // 过滤异常值
                total_path_length_ += dist;
            }
        }
        
        last_pos_ = current_pos_;
        last_update_time_ = now;
        
        // 更新飞行状态
        updateFlightState();
    }
    
    void trajCallback(const quadrotor_msgs::PositionCommand::ConstPtr& msg) {
        if (!isRecording()) {
            return;  // 不在飞行状态，不记录
        }
        
        // 记录收到新轨迹
        ros::Time now = ros::Time::now();
        double interval = (now - last_traj_time_).toSec();
        
        if (interval > 0.05) {  // 新的规划
            metrics_recorder_->recordPlanningAttempt(true);
            has_trajectory_ = true;
            
            // 记录路径长度
            if (total_path_length_ > 0.1) {
                metrics_recorder_->recordPathLength(total_path_length_);
            }
        }
        
        // 记录动力学
        Eigen::Vector3d vel(msg->velocity.x, msg->velocity.y, msg->velocity.z);
        Eigen::Vector3d acc(msg->acceleration.x, msg->acceleration.y, msg->acceleration.z);
        
        double max_vel = vel.norm();
        double max_acc = acc.norm();
        
        double vel_limit = 2.0;  // TODO: 从参数读取
        double acc_limit = 2.0;
        
        metrics_recorder_->recordDynamics(max_vel, max_acc, 
                                         max_vel > vel_limit, 
                                         max_acc > acc_limit);
        
        last_traj_time_ = now;
    }
    
    void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        goal_pos_ << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
        has_goal_ = true;
        total_path_length_ = 0.0;
        
        // 重置状态为IDLE，准备新任务
        if (flight_state_ == ARRIVED) {
            flight_state_ = IDLE;
            start_pos_ = current_pos_;
            ROS_INFO("\033[1;36m[Flight] New goal set, state: ARRIVED -> IDLE\033[0m");
        } else if (flight_state_ == IDLE) {
            start_pos_ = current_pos_;
            ROS_INFO("\033[1;36m[Flight] New goal set at IDLE state\033[0m");
        }
        
        ROS_INFO("[Flight] Goal: (%.2f, %.2f, %.2f)", goal_pos_.x(), goal_pos_.y(), goal_pos_.z());
    }
    
    void metricsTimerCallback(const ros::TimerEvent&) {
        // 只在飞行时发布和保存
        if (isRecording()) {
            metrics_recorder_->publishMetricsVisualization();
            metrics_recorder_->saveMetricsToFile("");
        }
    }
    
    std::string getFlightStateString() const {
        switch (flight_state_) {
            case IDLE: return "IDLE";
            case TAKING_OFF: return "TAKING_OFF";
            case FLYING: return "FLYING";
            case ARRIVED: return "ARRIVED";
            default: return "UNKNOWN";
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "metrics_recorder_node");
    ros::NodeHandle nh("~");
    
    MetricsRecorderNode node(nh);
    
    ros::spin();
    
    return 0;
}
