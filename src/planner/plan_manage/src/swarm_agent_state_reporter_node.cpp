#include <ego_planner/SwarmAgentState.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>

class SwarmAgentStateReporter
{
public:
  void init(ros::NodeHandle &nh)
  {
    nh_ = nh;

    nh_.param("agent_id", agent_id_, 1);
    nh_.param("publish_rate", publish_rate_, 30.0);
    nh_.param("state_timeout", state_timeout_, 0.2);

    odom_sub_ = nh_.subscribe("odom", 20, &SwarmAgentStateReporter::odomCallback, this);
    state_pub_ = nh_.advertise<ego_planner::SwarmAgentState>("state_out", 20);

    pub_timer_ = nh_.createTimer(ros::Duration(1.0 / std::max(1.0, publish_rate_)),
                                 &SwarmAgentStateReporter::timerCallback, this);

    ROS_INFO("[SwarmStateReporter] ready. agent_id=%d", agent_id_);
  }

private:
  void odomCallback(const nav_msgs::OdometryConstPtr &msg)
  {
    last_odom_ = *msg;
    have_odom_ = true;
    last_odom_time_ = msg->header.stamp;
  }

  void timerCallback(const ros::TimerEvent & /*e*/)
  {
    ego_planner::SwarmAgentState out;
    out.header.stamp = ros::Time::now();
    out.header.frame_id = "world";
    out.agent_id = agent_id_;

    if (have_odom_)
    {
      out.position.x = last_odom_.pose.pose.position.x;
      out.position.y = last_odom_.pose.pose.position.y;
      out.position.z = last_odom_.pose.pose.position.z;

      out.velocity.x = last_odom_.twist.twist.linear.x;
      out.velocity.y = last_odom_.twist.twist.linear.y;
      out.velocity.z = last_odom_.twist.twist.linear.z;

      const double age = (ros::Time::now() - last_odom_time_).toSec();
      out.is_valid = (age <= state_timeout_);
    }
    else
    {
      out.position = geometry_msgs::Point();
      out.velocity = geometry_msgs::Vector3();
      out.is_valid = false;
    }

    state_pub_.publish(out);
  }

private:
  ros::NodeHandle nh_;
  ros::Subscriber odom_sub_;
  ros::Publisher state_pub_;
  ros::Timer pub_timer_;

  int agent_id_{1};
  double publish_rate_{30.0};
  double state_timeout_{0.2};

  bool have_odom_{false};
  nav_msgs::Odometry last_odom_;
  ros::Time last_odom_time_;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "swarm_agent_state_reporter_node");
  ros::NodeHandle nh("~");

  SwarmAgentStateReporter node;
  node.init(nh);

  ros::spin();
  return 0;
}
