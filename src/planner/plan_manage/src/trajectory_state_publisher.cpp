/**
 * @file trajectory_state_publisher.cpp
 * @brief 实时轨迹状态发布节点
 * 
 * 功能：
 * 1. 订阅贝塞尔轨迹和里程计
 * 2. 实时计算并输出 xyz、yaw、速度、加速度、时间戳
 * 3. 发布轨迹状态消息供外部系统使用
 */

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/Float64MultiArray.h>
#include <tf/tf.h>
#include <Eigen/Eigen>
#include <deque>
#include <fstream>

#include "bezier_opt/piecewise_bezier.h"
#include "ego_planner/Bezier.h"

using namespace std;
using namespace ego_planner;

class TrajectoryStatePublisher
{
private:
    ros::NodeHandle nh_;
    
    // 订阅器
    ros::Subscriber odom_sub_;
    ros::Subscriber bezier_sub_;
    
    // 发布器
    ros::Publisher state_pub_;           // 完整状态发布
    ros::Publisher pose_pub_;            // 位姿发布
    ros::Publisher velocity_pub_;        // 速度发布
    
    // 定时器
    ros::Timer state_timer_;
    
    // 轨迹数据
    vector<PiecewiseBezier> traj_;
    ros::Time traj_start_time_;
    double traj_duration_;
    bool has_traj_;
    int traj_id_;
    
    // 里程计数据
    Eigen::Vector3d current_pos_;
    Eigen::Vector3d current_vel_;
    double current_yaw_;
    bool has_odom_;
    ros::Time last_odom_time_;
    
    // 历史状态存储 (用于外部访问)
    struct TrajectoryState {
        double timestamp;
        Eigen::Vector3d position;
        double yaw;
        Eigen::Vector3d velocity;
        Eigen::Vector3d acceleration;
    };
    deque<TrajectoryState> state_history_;
    int max_history_size_;
    
    // 参数
    bool enable_console_output_;
    bool enable_file_logging_;
    string log_file_path_;
    ofstream log_file_;
    double publish_rate_;
    
public:
    TrajectoryStatePublisher() : has_traj_(false), has_odom_(false), max_history_size_(1000)
    {
        // 读取参数
        nh_.param("trajectory_state/enable_console_output", enable_console_output_, true);
        nh_.param("trajectory_state/enable_file_logging", enable_file_logging_, false);
        nh_.param<string>("trajectory_state/log_file_path", log_file_path_, "/tmp/trajectory_log.csv");
        nh_.param("trajectory_state/publish_rate", publish_rate_, 50.0);
        nh_.param("trajectory_state/max_history_size", max_history_size_, 1000);
        
        // 订阅器
        odom_sub_ = nh_.subscribe("/odom_world", 1, &TrajectoryStatePublisher::odomCallback, this);
        bezier_sub_ = nh_.subscribe("/planning/bezier", 1, &TrajectoryStatePublisher::bezierCallback, this);
        
        // 发布器
        state_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("/trajectory_state/full_state", 10);
        pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/trajectory_state/pose", 10);
        velocity_pub_ = nh_.advertise<geometry_msgs::TwistStamped>("/trajectory_state/velocity", 10);
        
        // 定时器
        state_timer_ = nh_.createTimer(ros::Duration(1.0 / publish_rate_), 
                                        &TrajectoryStatePublisher::stateTimerCallback, this);
        
        // 初始化日志文件
        if (enable_file_logging_)
        {
            log_file_.open(log_file_path_);
            if (log_file_.is_open())
            {
                log_file_ << "timestamp,x,y,z,yaw,vx,vy,vz,ax,ay,az" << endl;
                ROS_INFO("[TrajState] Logging to: %s", log_file_path_.c_str());
            }
            else
            {
                ROS_WARN("[TrajState] Failed to open log file: %s", log_file_path_.c_str());
            }
        }
        
        ROS_INFO("[TrajState] Trajectory State Publisher initialized");
        ROS_INFO("[TrajState] Console output: %s, File logging: %s", 
                 enable_console_output_ ? "ON" : "OFF",
                 enable_file_logging_ ? "ON" : "OFF");
    }
    
    ~TrajectoryStatePublisher()
    {
        if (log_file_.is_open())
        {
            log_file_.close();
        }
    }
    
    void odomCallback(const nav_msgs::OdometryConstPtr& msg)
    {
        current_pos_(0) = msg->pose.pose.position.x;
        current_pos_(1) = msg->pose.pose.position.y;
        current_pos_(2) = msg->pose.pose.position.z;
        
        current_vel_(0) = msg->twist.twist.linear.x;
        current_vel_(1) = msg->twist.twist.linear.y;
        current_vel_(2) = msg->twist.twist.linear.z;
        
        // 提取 yaw 角度
        tf::Quaternion q(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w
        );
        double roll, pitch, yaw;
        tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
        current_yaw_ = yaw;
        
        last_odom_time_ = msg->header.stamp;
        has_odom_ = true;
    }
    
    void bezierCallback(const ego_planner::BezierConstPtr& msg)
    {
        // 解析控制点
        Eigen::MatrixXd pos_pts(3, msg->pos_pts.size());
        for (size_t i = 0; i < msg->pos_pts.size(); ++i)
        {
            pos_pts(0, i) = msg->pos_pts[i].x;
            pos_pts(1, i) = msg->pos_pts[i].y;
            pos_pts(2, i) = msg->pos_pts[i].z;
        }
        
        // 获取段时长
        double segment_duration = 0.1;
        if (!msg->segment_durations.empty())
        {
            segment_duration = msg->segment_durations[0];
        }
        
        // 构建贝塞尔曲线
        PiecewiseBezier pos_traj(pos_pts, msg->order, segment_duration);
        
        if (!msg->segment_durations.empty())
        {
            Eigen::VectorXd durations(msg->segment_durations.size());
            for (size_t i = 0; i < msg->segment_durations.size(); ++i)
            {
                durations(i) = msg->segment_durations[i];
            }
            pos_traj.setKnot(durations);
        }
        
        traj_.clear();
        traj_.push_back(pos_traj);
        traj_.push_back(traj_[0].getDerivative());   // 速度
        traj_.push_back(traj_[1].getDerivative());   // 加速度
        
        traj_start_time_ = msg->start_time;
        traj_id_ = msg->traj_id;
        traj_duration_ = traj_[0].getTimeSum();
        has_traj_ = true;
        
        ROS_INFO("[TrajState] New trajectory received, ID=%d, duration=%.2fs", traj_id_, traj_duration_);
    }
    
    void stateTimerCallback(const ros::TimerEvent& e)
    {
        if (!has_odom_)
            return;
        
        ros::Time now = ros::Time::now();
        
        Eigen::Vector3d pos, vel, acc;
        double yaw;
        
        if (has_traj_)
        {
            double t_cur = (now - traj_start_time_).toSec();
            
            if (t_cur >= 0.0 && t_cur < traj_duration_)
            {
                // 从轨迹获取期望状态
                pos = traj_[0].evaluate(t_cur);
                vel = traj_[1].evaluate(t_cur);
                acc = traj_[2].evaluate(t_cur);
                
                // 根据速度方向计算 yaw
                if (vel.head<2>().norm() > 0.1)
                {
                    yaw = atan2(vel(1), vel(0));
                }
                else
                {
                    yaw = current_yaw_;
                }
            }
            else
            {
                // 轨迹结束，使用当前里程计数据
                pos = current_pos_;
                vel = current_vel_;
                acc = Eigen::Vector3d::Zero();
                yaw = current_yaw_;
            }
        }
        else
        {
            // 无轨迹，使用当前里程计数据
            pos = current_pos_;
            vel = current_vel_;
            acc = Eigen::Vector3d::Zero();
            yaw = current_yaw_;
        }
        
        // 存储到历史记录
        TrajectoryState state;
        state.timestamp = now.toSec();
        state.position = pos;
        state.yaw = yaw;
        state.velocity = vel;
        state.acceleration = acc;
        
        state_history_.push_back(state);
        while ((int)state_history_.size() > max_history_size_)
        {
            state_history_.pop_front();
        }
        
        // 控制台输出
        if (enable_console_output_)
        {
            ROS_INFO_THROTTLE(0.5, 
                "[TrajState] t=%.3f | pos=[%.2f, %.2f, %.2f] | yaw=%.2f° | vel=[%.2f, %.2f, %.2f] | acc=[%.2f, %.2f, %.2f]",
                now.toSec(),
                pos(0), pos(1), pos(2),
                yaw * 180.0 / M_PI,
                vel(0), vel(1), vel(2),
                acc(0), acc(1), acc(2));
        }
        
        // 文件记录
        if (enable_file_logging_ && log_file_.is_open())
        {
            log_file_ << fixed << setprecision(6)
                      << now.toSec() << ","
                      << pos(0) << "," << pos(1) << "," << pos(2) << ","
                      << yaw << ","
                      << vel(0) << "," << vel(1) << "," << vel(2) << ","
                      << acc(0) << "," << acc(1) << "," << acc(2) << endl;
        }
        
        // 发布完整状态 (Float64MultiArray: [t, x, y, z, yaw, vx, vy, vz, ax, ay, az])
        std_msgs::Float64MultiArray state_msg;
        state_msg.data.resize(11);
        state_msg.data[0] = now.toSec();
        state_msg.data[1] = pos(0);
        state_msg.data[2] = pos(1);
        state_msg.data[3] = pos(2);
        state_msg.data[4] = yaw;
        state_msg.data[5] = vel(0);
        state_msg.data[6] = vel(1);
        state_msg.data[7] = vel(2);
        state_msg.data[8] = acc(0);
        state_msg.data[9] = acc(1);
        state_msg.data[10] = acc(2);
        state_pub_.publish(state_msg);
        
        // 发布位姿
        geometry_msgs::PoseStamped pose_msg;
        pose_msg.header.stamp = now;
        pose_msg.header.frame_id = "world";
        pose_msg.pose.position.x = pos(0);
        pose_msg.pose.position.y = pos(1);
        pose_msg.pose.position.z = pos(2);
        tf::Quaternion q;
        q.setRPY(0, 0, yaw);
        pose_msg.pose.orientation.x = q.x();
        pose_msg.pose.orientation.y = q.y();
        pose_msg.pose.orientation.z = q.z();
        pose_msg.pose.orientation.w = q.w();
        pose_pub_.publish(pose_msg);
        
        // 发布速度
        geometry_msgs::TwistStamped vel_msg;
        vel_msg.header.stamp = now;
        vel_msg.header.frame_id = "world";
        vel_msg.twist.linear.x = vel(0);
        vel_msg.twist.linear.y = vel(1);
        vel_msg.twist.linear.z = vel(2);
        velocity_pub_.publish(vel_msg);
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "trajectory_state_publisher");
    TrajectoryStatePublisher node;
    ros::spin();
    return 0;
}
