/**
 * @file local_map_visualizer.cpp
 * @brief 局部地图可视化节点
 * 
 * 功能：
 * 1. 订阅 grid_map 的局部障碍地图
 * 2. 使用不同于全局地图的颜色显示局部感知区域
 * 3. 显示感知范围边界
 * 4. 区分已探索区域和未知区域
 */

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/Odometry.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <Eigen/Eigen>

using namespace std;

class LocalMapVisualizer
{
private:
    ros::NodeHandle nh_;
    
    // 订阅器
    ros::Subscriber occ_sub_;       // 占用地图
    ros::Subscriber odom_sub_;      // 里程计
    
    // 发布器
    ros::Publisher local_map_pub_;      // 局部地图 (不同颜色)
    ros::Publisher sensing_range_pub_;  // 感知范围可视化
    ros::Publisher explored_pub_;       // 已探索区域
    
    // 定时器
    ros::Timer vis_timer_;
    
    // 状态变量
    Eigen::Vector3d current_pos_;
    bool has_odom_;
    
    // 参数
    double sensing_range_;
    double map_resolution_;
    string frame_id_;
    
    // 颜色设置
    double local_map_r_, local_map_g_, local_map_b_;
    double sensing_range_r_, sensing_range_g_, sensing_range_b_;
    
public:
    LocalMapVisualizer() : has_odom_(false)
    {
        // 读取参数
        nh_.param("local_map_vis/sensing_range", sensing_range_, 5.0);
        nh_.param("local_map_vis/map_resolution", map_resolution_, 0.1);
        nh_.param<string>("local_map_vis/frame_id", frame_id_, "world");
        
        // 局部地图颜色 (默认橙色)
        nh_.param("local_map_vis/local_map_r", local_map_r_, 1.0);
        nh_.param("local_map_vis/local_map_g", local_map_g_, 0.5);
        nh_.param("local_map_vis/local_map_b", local_map_b_, 0.0);
        
        // 感知范围颜色 (默认青色)
        nh_.param("local_map_vis/sensing_range_r", sensing_range_r_, 0.0);
        nh_.param("local_map_vis/sensing_range_g", sensing_range_g_, 1.0);
        nh_.param("local_map_vis/sensing_range_b", sensing_range_b_, 1.0);
        
        // 订阅器
        occ_sub_ = nh_.subscribe("/grid_map/occupancy", 1, &LocalMapVisualizer::occCallback, this);
        odom_sub_ = nh_.subscribe("/odom_world", 1, &LocalMapVisualizer::odomCallback, this);
        
        // 发布器
        local_map_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/local_map_vis/local_obstacles", 10);
        sensing_range_pub_ = nh_.advertise<visualization_msgs::Marker>("/local_map_vis/sensing_range", 10);
        explored_pub_ = nh_.advertise<visualization_msgs::Marker>("/local_map_vis/explored_area", 10);
        
        // 定时器发布感知范围
        vis_timer_ = nh_.createTimer(ros::Duration(0.1), &LocalMapVisualizer::visTimerCallback, this);
        
        ROS_INFO("[LocalMapVis] Local Map Visualizer initialized");
        ROS_INFO("[LocalMapVis] Sensing range: %.1f m, Local map color: RGB(%.1f,%.1f,%.1f)",
                 sensing_range_, local_map_r_, local_map_g_, local_map_b_);
    }
    
    void odomCallback(const nav_msgs::OdometryConstPtr& msg)
    {
        current_pos_(0) = msg->pose.pose.position.x;
        current_pos_(1) = msg->pose.pose.position.y;
        current_pos_(2) = msg->pose.pose.position.z;
        has_odom_ = true;
    }
    
    void occCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
    {
        if (!has_odom_)
            return;
        
        // 转换点云
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *cloud_in);
        
        // 创建带颜色的局部点云
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr local_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        
        for (const auto& pt : cloud_in->points)
        {
            // 计算到无人机的距离
            double dist = sqrt(pow(pt.x - current_pos_(0), 2) + 
                              pow(pt.y - current_pos_(1), 2) + 
                              pow(pt.z - current_pos_(2), 2));
            
            // 只显示感知范围内的障碍物
            if (dist <= sensing_range_)
            {
                pcl::PointXYZRGB pt_rgb;
                pt_rgb.x = pt.x;
                pt_rgb.y = pt.y;
                pt_rgb.z = pt.z;
                
                // 根据距离调整颜色亮度
                double ratio = 1.0 - (dist / sensing_range_) * 0.5;
                pt_rgb.r = (uint8_t)(local_map_r_ * ratio * 255);
                pt_rgb.g = (uint8_t)(local_map_g_ * ratio * 255);
                pt_rgb.b = (uint8_t)(local_map_b_ * ratio * 255);
                
                local_cloud->points.push_back(pt_rgb);
            }
        }
        
        // 发布局部地图
        if (!local_cloud->empty())
        {
            sensor_msgs::PointCloud2 local_msg;
            pcl::toROSMsg(*local_cloud, local_msg);
            local_msg.header.frame_id = frame_id_;
            local_msg.header.stamp = ros::Time::now();
            local_map_pub_.publish(local_msg);
        }
    }
    
    void visTimerCallback(const ros::TimerEvent& e)
    {
        if (!has_odom_)
            return;
        
        // 发布感知范围球体
        publishSensingRange();
        
        // 发布已探索区域标记
        publishExploredArea();
    }
    
    void publishSensingRange()
    {
        visualization_msgs::Marker marker;
        marker.header.frame_id = frame_id_;
        marker.header.stamp = ros::Time::now();
        marker.ns = "sensing_range";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        
        marker.pose.position.x = current_pos_(0);
        marker.pose.position.y = current_pos_(1);
        marker.pose.position.z = current_pos_(2);
        marker.pose.orientation.w = 1.0;
        
        marker.scale.x = sensing_range_ * 2;
        marker.scale.y = sensing_range_ * 2;
        marker.scale.z = sensing_range_ * 2;
        
        marker.color.r = sensing_range_r_;
        marker.color.g = sensing_range_g_;
        marker.color.b = sensing_range_b_;
        marker.color.a = 0.1;  // 半透明
        
        marker.lifetime = ros::Duration(0.2);
        
        sensing_range_pub_.publish(marker);
    }
    
    void publishExploredArea()
    {
        visualization_msgs::Marker marker;
        marker.header.frame_id = frame_id_;
        marker.header.stamp = ros::Time::now();
        marker.ns = "explored_boundary";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::LINE_STRIP;
        marker.action = visualization_msgs::Marker::ADD;
        
        marker.pose.orientation.w = 1.0;
        
        marker.scale.x = 0.05;  // 线宽
        
        marker.color.r = sensing_range_r_;
        marker.color.g = sensing_range_g_;
        marker.color.b = sensing_range_b_;
        marker.color.a = 0.8;
        
        // 绘制感知范围圆形边界
        int num_points = 36;
        for (int i = 0; i <= num_points; ++i)
        {
            double angle = 2 * M_PI * i / num_points;
            geometry_msgs::Point p;
            p.x = current_pos_(0) + sensing_range_ * cos(angle);
            p.y = current_pos_(1) + sensing_range_ * sin(angle);
            p.z = current_pos_(2);
            marker.points.push_back(p);
        }
        
        marker.lifetime = ros::Duration(0.2);
        
        explored_pub_.publish(marker);
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "local_map_visualizer");
    LocalMapVisualizer node;
    ros::spin();
    return 0;
}
