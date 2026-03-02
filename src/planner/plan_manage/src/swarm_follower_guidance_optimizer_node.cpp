#include <bezier_opt/bezier_optimizer.h>
#include <ego_planner/Bezier.h>
#include <plan_env/grid_map.h>
#include <ros/ros.h>

class SwarmFollowerGuidanceOptimizer
{
public:
  void init(ros::NodeHandle &nh)
  {
    nh_ = nh;

    nh_.param("enable_local_refine", enable_local_refine_, true);
    nh_.param("enable_collision_rebound", enable_collision_rebound_, false);
    nh_.param("min_start_time_delay", min_start_time_delay_, 0.03);
    nh_.param("fallback_segment_dt", fallback_segment_dt_, 0.1);

    nh_.param("use_local_map", use_local_map_, false);
    if (use_local_map_)
    {
      grid_map_.reset(new GridMap);
      grid_map_->initMap(nh_);
      ROS_INFO("[FollowerGuidance] local GridMap enabled.");
    }

    optimizer_.reset(new ego_planner::BezierOptimizer);
    optimizer_->setParam(nh_);
    if (grid_map_)
    {
      optimizer_->setEnvironment(grid_map_);
      optimizer_->a_star_.reset(new AStar);
      optimizer_->a_star_->initGridMap(grid_map_, Eigen::Vector3i(100, 100, 100));
    }

    sub_ = nh_.subscribe("guidance_in", 10, &SwarmFollowerGuidanceOptimizer::guidanceCallback, this);
    pub_ = nh_.advertise<ego_planner::Bezier>("planning/bezier", 10);

    ROS_INFO("[FollowerGuidance] initialized. refine=%d rebound=%d local_map=%d",
             static_cast<int>(enable_local_refine_),
             static_cast<int>(enable_collision_rebound_),
             static_cast<int>(use_local_map_));
  }

private:
  static Eigen::MatrixXd toCtrlPts(const ego_planner::Bezier &msg)
  {
    Eigen::MatrixXd ctrl(3, msg.pos_pts.size());
    for (size_t i = 0; i < msg.pos_pts.size(); ++i)
    {
      ctrl(0, i) = msg.pos_pts[i].x;
      ctrl(1, i) = msg.pos_pts[i].y;
      ctrl(2, i) = msg.pos_pts[i].z;
    }
    return ctrl;
  }

  static void fillCtrlPts(ego_planner::Bezier &msg, const Eigen::MatrixXd &ctrl)
  {
    msg.pos_pts.clear();
    msg.pos_pts.reserve(ctrl.cols());
    for (int i = 0; i < ctrl.cols(); ++i)
    {
      geometry_msgs::Point p;
      p.x = ctrl(0, i);
      p.y = ctrl(1, i);
      p.z = ctrl(2, i);
      msg.pos_pts.push_back(p);
    }
  }

  void guidanceCallback(const ego_planner::BezierConstPtr &msg)
  {

    if (msg->pos_pts.size() < 4)
    {
      ego_planner::Bezier out = *msg;
      // 直接传递主机的绝对时间戳，保持主从机全局时钟同步（用于传感器融合）。
      // 不做任何偏移：leader 的 start_time 通常略晚于 now（刚刚 replan），
      // 从机 traj_server 收到时 t_cur ≈ 几ms > 0，可立即执行。
      out.start_time = msg->start_time;
      pub_.publish(out);
      return;
    }

    ego_planner::Bezier out = *msg;

    const double ts = (!msg->segment_durations.empty()) ? msg->segment_durations[0] : fallback_segment_dt_;

    if (enable_local_refine_)
    {
      Eigen::MatrixXd init_ctrl = toCtrlPts(*msg);
      Eigen::MatrixXd opt_ctrl = init_ctrl;
      bool ok = false;

      if (enable_collision_rebound_ && grid_map_)
      {
        optimizer_->initControlPoints(opt_ctrl, true);
        ok = optimizer_->BezierOptimizeTrajRebound(opt_ctrl, ts);
      }

      if (!ok)
      {
        ok = optimizer_->BezierOptimizeTrajRefine(init_ctrl, ts, opt_ctrl);
      }

      if (ok)
      {
        fillCtrlPts(out, opt_ctrl);
      }
      else
      {
        ROS_WARN_THROTTLE(1.0, "[FollowerGuidance] local refine failed, fallback to guidance control points.");
      }
    }

    // 直接传递主机的绝对时间戳，保持全局时钟同步（用于传感器融合）。
    out.start_time = msg->start_time;
    pub_.publish(out);
  }

private:
  ros::NodeHandle nh_;
  ros::Subscriber sub_;
  ros::Publisher pub_;

  bool enable_local_refine_{true};
  bool enable_collision_rebound_{false};
  bool use_local_map_{false};
  double min_start_time_delay_{0.03};
  double fallback_segment_dt_{0.1};

  GridMap::Ptr grid_map_;
  ego_planner::BezierOptimizer::Ptr optimizer_;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "swarm_follower_guidance_optimizer_node");
  ros::NodeHandle nh("~");

  SwarmFollowerGuidanceOptimizer node;
  node.init(nh);

  ros::spin();
  return 0;
}
