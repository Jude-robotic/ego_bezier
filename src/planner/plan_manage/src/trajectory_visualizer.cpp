/**
 * @file trajectory_visualizer.cpp
 * @brief Enhanced trajectory visualization node
 * 
 * Features:
 * - Bezier control points (RED spheres)
 * - Planned trajectory (GREEN line)  
 * - Executed trajectory (BLUE line)
 * - Minimal console output (trajectory state only)
 */

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/Float64MultiArray.h>
#include <bezier_opt/piecewise_bezier.h>
#include <ego_planner/Bezier.h>
#include <Eigen/Dense>
#include <fstream>
#include <iomanip>
#include <deque>

class TrajectoryVisualizer {
public:
    TrajectoryVisualizer(ros::NodeHandle& nh) : nh_(nh) {
        // Parameters
        nh_.param("trajectory_vis/enable_console_output", enable_console_output_, true);
        nh_.param("trajectory_vis/enable_file_logging", enable_file_logging_, true);
        nh_.param("trajectory_vis/log_file_path", log_file_path_, std::string("/tmp/ego_trajectory_log.csv"));
        nh_.param("trajectory_vis/output_rate", output_rate_, 10.0);
        nh_.param("trajectory_vis/max_executed_points", max_executed_points_, 5000);
        nh_.param("trajectory_vis/control_point_size", control_point_size_, 0.15);
        nh_.param("trajectory_vis/planned_line_width", planned_line_width_, 0.05);
        nh_.param("trajectory_vis/executed_line_width", executed_line_width_, 0.03);
        
        // Subscribers
        odom_sub_ = nh_.subscribe("/odom_world", 1, &TrajectoryVisualizer::odomCallback, this);
        bezier_sub_ = nh_.subscribe("/planning/bezier", 1, &TrajectoryVisualizer::bezierCallback, this);
        
        // Publishers
        control_points_pub_ = nh_.advertise<visualization_msgs::Marker>("/trajectory_vis/control_points", 10);
        planned_traj_pub_ = nh_.advertise<visualization_msgs::Marker>("/trajectory_vis/planned_trajectory", 10);
        executed_traj_pub_ = nh_.advertise<visualization_msgs::Marker>("/trajectory_vis/executed_trajectory", 10);
        executed_path_pub_ = nh_.advertise<nav_msgs::Path>("/trajectory_vis/executed_path", 10);
        state_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("/trajectory_vis/state", 10);
        
        // Initialize file logging
        if (enable_file_logging_) {
            log_file_.open(log_file_path_);
            if (log_file_.is_open()) {
                log_file_ << "timestamp,x,y,z,yaw,vx,vy,vz,ax,ay,az" << std::endl;
                ROS_INFO("[TrajectoryVis] Logging to: %s", log_file_path_.c_str());
            }
        }
        
        // Timer for console output
        last_output_time_ = ros::Time::now();
        output_interval_ = ros::Duration(1.0 / output_rate_);
        
        has_odom_ = false;
        has_traj_ = false;
        
        ROS_INFO("[TrajectoryVis] Initialized - Control points: RED, Planned: GREEN, Executed: BLUE");
    }
    
    ~TrajectoryVisualizer() {
        if (log_file_.is_open()) {
            log_file_.close();
        }
    }

private:
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        current_pos_ = Eigen::Vector3d(
            msg->pose.pose.position.x,
            msg->pose.pose.position.y,
            msg->pose.pose.position.z
        );
        
        current_vel_ = Eigen::Vector3d(
            msg->twist.twist.linear.x,
            msg->twist.twist.linear.y,
            msg->twist.twist.linear.z
        );
        
        // Extract yaw from quaternion
        double qw = msg->pose.pose.orientation.w;
        double qx = msg->pose.pose.orientation.x;
        double qy = msg->pose.pose.orientation.y;
        double qz = msg->pose.pose.orientation.z;
        current_yaw_ = std::atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));
        
        has_odom_ = true;
        
        // Update executed trajectory
        geometry_msgs::PoseStamped pose;
        pose.header = msg->header;
        pose.header.frame_id = "world";
        pose.pose = msg->pose.pose;
        
        executed_poses_.push_back(pose);
        while (executed_poses_.size() > static_cast<size_t>(max_executed_points_)) {
            executed_poses_.pop_front();
        }
        
        // Publish executed trajectory
        publishExecutedTrajectory();
        
        // Console output (rate limited)
        ros::Time now = ros::Time::now();
        if (enable_console_output_ && (now - last_output_time_) >= output_interval_) {
            last_output_time_ = now;
            
            // Calculate acceleration from velocity derivative (simplified)
            Eigen::Vector3d acc = Eigen::Vector3d::Zero();
            if (has_traj_) {
                double t = (now - traj_start_time_).toSec();
                if (t >= 0 && t <= traj_duration_) {
                    // Get derivative for acceleration
                    ego_planner::PiecewiseBezier vel_traj = bezier_traj_.getDerivative();
                    acc = vel_traj.evaluate(t);
                }
            }
            
            // Clean console output - only trajectory state
            printf("\033[2K\r[Traj] t=%.2f | pos=[%.2f, %.2f, %.2f] | yaw=%.1f° | vel=[%.2f, %.2f, %.2f] | acc=[%.2f, %.2f, %.2f]",
                   now.toSec(),
                   current_pos_.x(), current_pos_.y(), current_pos_.z(),
                   current_yaw_ * 180.0 / M_PI,
                   current_vel_.x(), current_vel_.y(), current_vel_.z(),
                   acc.x(), acc.y(), acc.z());
            fflush(stdout);
            
            // File logging
            if (enable_file_logging_ && log_file_.is_open()) {
                log_file_ << std::fixed << std::setprecision(6)
                          << now.toSec() << ","
                          << current_pos_.x() << "," << current_pos_.y() << "," << current_pos_.z() << ","
                          << current_yaw_ << ","
                          << current_vel_.x() << "," << current_vel_.y() << "," << current_vel_.z() << ","
                          << acc.x() << "," << acc.y() << "," << acc.z()
                          << std::endl;
            }
            
            // Publish state message
            std_msgs::Float64MultiArray state_msg;
            state_msg.data = {
                now.toSec(),
                current_pos_.x(), current_pos_.y(), current_pos_.z(),
                current_yaw_,
                current_vel_.x(), current_vel_.y(), current_vel_.z(),
                acc.x(), acc.y(), acc.z()
            };
            state_pub_.publish(state_msg);
        }
    }
    
    void bezierCallback(const ego_planner::Bezier::ConstPtr& msg) {
        if (msg->order < 3 || msg->traj_id < 0) {
            return;
        }
        
        // Get piece number from segment_durations
        int piece_num = msg->segment_durations.size();
        if (piece_num == 0) return;
        
        // Compute average duration for each segment
        double total_duration = 0;
        for (int i = 0; i < piece_num; i++) {
            total_duration += msg->segment_durations[i];
        }
        double avg_interval = total_duration / piece_num;
        
        // Build control points matrix from pos_pts
        int num_pts = msg->pos_pts.size();
        Eigen::MatrixXd ctrl_pts(3, num_pts);
        for (int i = 0; i < num_pts; i++) {
            ctrl_pts(0, i) = msg->pos_pts[i].x;
            ctrl_pts(1, i) = msg->pos_pts[i].y;
            ctrl_pts(2, i) = msg->pos_pts[i].z;
        }
        
        // Create Bezier trajectory
        bezier_traj_.setBezierCurve(ctrl_pts, msg->order, avg_interval);
        traj_start_time_ = msg->start_time;
        
        double t_min, t_max;
        bezier_traj_.getTimeSpan(t_min, t_max);
        traj_duration_ = t_max - t_min;
        
        has_traj_ = true;
        
        // Publish control points (RED)
        publishControlPoints(ctrl_pts);
        
        // Publish planned trajectory (GREEN)
        publishPlannedTrajectory();
    }
    
    void publishControlPoints(const Eigen::MatrixXd& ctrl_pts) {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "world";
        marker.header.stamp = ros::Time::now();
        marker.ns = "bezier_control_points";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::SPHERE_LIST;
        marker.action = visualization_msgs::Marker::ADD;
        
        marker.pose.orientation.w = 1.0;
        
        // RED for control points
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0;
        
        marker.scale.x = control_point_size_;
        marker.scale.y = control_point_size_;
        marker.scale.z = control_point_size_;
        
        for (int i = 0; i < ctrl_pts.cols(); i++) {
            geometry_msgs::Point pt;
            pt.x = ctrl_pts(0, i);
            pt.y = ctrl_pts(1, i);
            pt.z = ctrl_pts(2, i);
            marker.points.push_back(pt);
        }
        
        control_points_pub_.publish(marker);
        
        // Also publish connecting lines between control points
        visualization_msgs::Marker line_marker;
        line_marker.header.frame_id = "world";
        line_marker.header.stamp = ros::Time::now();
        line_marker.ns = "bezier_control_lines";
        line_marker.id = 1;
        line_marker.type = visualization_msgs::Marker::LINE_STRIP;
        line_marker.action = visualization_msgs::Marker::ADD;
        
        line_marker.pose.orientation.w = 1.0;
        
        // Light red for control polygon
        line_marker.color.r = 1.0;
        line_marker.color.g = 0.3;
        line_marker.color.b = 0.3;
        line_marker.color.a = 0.5;
        
        line_marker.scale.x = 0.02;
        
        for (int i = 0; i < ctrl_pts.cols(); i++) {
            geometry_msgs::Point pt;
            pt.x = ctrl_pts(0, i);
            pt.y = ctrl_pts(1, i);
            pt.z = ctrl_pts(2, i);
            line_marker.points.push_back(pt);
        }
        
        control_points_pub_.publish(line_marker);
    }
    
    void publishPlannedTrajectory() {
        if (!has_traj_) return;
        
        visualization_msgs::Marker marker;
        marker.header.frame_id = "world";
        marker.header.stamp = ros::Time::now();
        marker.ns = "planned_trajectory";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::LINE_STRIP;
        marker.action = visualization_msgs::Marker::ADD;
        
        marker.pose.orientation.w = 1.0;
        
        // GREEN for planned trajectory
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0;
        
        marker.scale.x = planned_line_width_;
        
        double t_min, t_max;
        bezier_traj_.getTimeSpan(t_min, t_max);
        
        for (double t = t_min; t <= t_max; t += 0.02) {
            Eigen::VectorXd pt = bezier_traj_.evaluate(t);
            geometry_msgs::Point p;
            p.x = pt.x();
            p.y = pt.y();
            p.z = pt.z();
            marker.points.push_back(p);
        }
        
        planned_traj_pub_.publish(marker);
    }
    
    void publishExecutedTrajectory() {
        if (executed_poses_.empty()) return;
        
        // Publish as Marker (BLUE)
        visualization_msgs::Marker marker;
        marker.header.frame_id = "world";
        marker.header.stamp = ros::Time::now();
        marker.ns = "executed_trajectory";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::LINE_STRIP;
        marker.action = visualization_msgs::Marker::ADD;
        
        marker.pose.orientation.w = 1.0;
        
        // BLUE for executed trajectory
        marker.color.r = 0.0;
        marker.color.g = 0.0;
        marker.color.b = 1.0;
        marker.color.a = 1.0;
        
        marker.scale.x = executed_line_width_;
        
        for (const auto& pose : executed_poses_) {
            geometry_msgs::Point p;
            p.x = pose.pose.position.x;
            p.y = pose.pose.position.y;
            p.z = pose.pose.position.z;
            marker.points.push_back(p);
        }
        
        executed_traj_pub_.publish(marker);
        
        // Also publish as Path
        nav_msgs::Path path;
        path.header.frame_id = "world";
        path.header.stamp = ros::Time::now();
        path.poses.assign(executed_poses_.begin(), executed_poses_.end());
        executed_path_pub_.publish(path);
    }

    ros::NodeHandle nh_;
    
    // Subscribers
    ros::Subscriber odom_sub_;
    ros::Subscriber bezier_sub_;
    
    // Publishers
    ros::Publisher control_points_pub_;
    ros::Publisher planned_traj_pub_;
    ros::Publisher executed_traj_pub_;
    ros::Publisher executed_path_pub_;
    ros::Publisher state_pub_;
    
    // State
    Eigen::Vector3d current_pos_;
    Eigen::Vector3d current_vel_;
    double current_yaw_;
    bool has_odom_;
    bool has_traj_;
    
    // Bezier trajectory
    ego_planner::PiecewiseBezier bezier_traj_;
    ros::Time traj_start_time_;
    double traj_duration_;
    
    // Executed trajectory history
    std::deque<geometry_msgs::PoseStamped> executed_poses_;
    int max_executed_points_;
    
    // Parameters
    bool enable_console_output_;
    bool enable_file_logging_;
    std::string log_file_path_;
    double output_rate_;
    double control_point_size_;
    double planned_line_width_;
    double executed_line_width_;
    
    // Timing
    ros::Time last_output_time_;
    ros::Duration output_interval_;
    
    // File logging
    std::ofstream log_file_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "trajectory_visualizer");
    ros::NodeHandle nh("~");
    
    TrajectoryVisualizer visualizer(nh);
    
    ros::spin();
    
    return 0;
}
