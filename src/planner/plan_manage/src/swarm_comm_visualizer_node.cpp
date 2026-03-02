#include <ego_planner/Bezier.h>
#include <ego_planner/SwarmAgentState.h>

#include <geometry_msgs/Point.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <algorithm>
#include <cstdio>
#include <iomanip>
#include <map>
#include <sstream>
#include <string>
#include <vector>

namespace
{
struct AgentStateCache
{
  bool received = false;
  ros::Time stamp;
  geometry_msgs::Point pos;
};

struct GuidanceCache
{
  bool received = false;
  ros::Time recv_stamp;
  ros::Time start_time;
};

std::string formatTopic(const std::string &topic_template, int agent_id)
{
  std::string topic = topic_template;
  const std::string key = "{id}";
  const std::size_t pos = topic.find(key);
  if (pos != std::string::npos)
  {
    topic.replace(pos, key.size(), std::to_string(agent_id));
  }
  return topic;
}

visualization_msgs::Marker makeTextMarker(int id, const std::string &ns, const geometry_msgs::Point &p,
                                          const std::string &text, float r, float g, float b)
{
  visualization_msgs::Marker mk;
  mk.header.frame_id = "world";
  mk.header.stamp = ros::Time::now();
  mk.ns = ns;
  mk.id = id;
  mk.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  mk.action = visualization_msgs::Marker::ADD;
  mk.pose.position = p;
  mk.pose.orientation.w = 1.0;
  mk.scale.z = 0.28;
  mk.color.a = 1.0;
  mk.color.r = r;
  mk.color.g = g;
  mk.color.b = b;
  mk.text = text;
  return mk;
}

visualization_msgs::Marker makeSphereMarker(int id, const std::string &ns, const geometry_msgs::Point &p,
                                            double scale, float r, float g, float b, float a)
{
  visualization_msgs::Marker mk;
  mk.header.frame_id = "world";
  mk.header.stamp = ros::Time::now();
  mk.ns = ns;
  mk.id = id;
  mk.type = visualization_msgs::Marker::SPHERE;
  mk.action = visualization_msgs::Marker::ADD;
  mk.pose.position = p;
  mk.pose.orientation.w = 1.0;
  mk.scale.x = scale;
  mk.scale.y = scale;
  mk.scale.z = scale;
  mk.color.a = a;
  mk.color.r = r;
  mk.color.g = g;
  mk.color.b = b;
  return mk;
}

visualization_msgs::Marker makeLineMarker(int id, const std::string &ns, const geometry_msgs::Point &p0,
                                          const geometry_msgs::Point &p1, double width,
                                          float r, float g, float b, float a)
{
  visualization_msgs::Marker mk;
  mk.header.frame_id = "world";
  mk.header.stamp = ros::Time::now();
  mk.ns = ns;
  mk.id = id;
  mk.type = visualization_msgs::Marker::LINE_STRIP;
  mk.action = visualization_msgs::Marker::ADD;
  mk.pose.orientation.w = 1.0;
  mk.scale.x = width;
  mk.color.a = a;
  mk.color.r = r;
  mk.color.g = g;
  mk.color.b = b;
  mk.points.push_back(p0);
  mk.points.push_back(p1);
  return mk;
}
} // namespace

class SwarmCommVisualizer
{
public:
  SwarmCommVisualizer()
      : nh_(), pnh_("~")
  {
    pnh_.param("leader_bezier_topic", leader_bezier_topic_, std::string("/planning/bezier"));
    pnh_.param("state_topic_template", state_topic_template_, std::string("/swarm/agent_{id}/state"));
    pnh_.param("guidance_topic_template", guidance_topic_template_, std::string("/swarm/agent_{id}/guidance_bezier"));
    pnh_.param("marker_topic", marker_topic_, std::string("/swarm/comm/markers"));

    pnh_.param("publish_rate", publish_rate_, 15.0);
    pnh_.param("state_timeout", state_timeout_, 0.30);
    pnh_.param("guidance_timeout", guidance_timeout_, 0.30);
    pnh_.param("text_z_offset", text_z_offset_, 0.50);

    if (!pnh_.getParam("agent_ids", agent_ids_) || agent_ids_.empty())
    {
      agent_ids_.push_back(1);
      agent_ids_.push_back(2);
      ROS_WARN("[SwarmCommViz] ~agent_ids not found, fallback to [1,2]");
    }

    marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(marker_topic_, 1);
    leader_sub_ = nh_.subscribe(leader_bezier_topic_, 10, &SwarmCommVisualizer::leaderBezierCallback, this);

    for (const int id : agent_ids_)
    {
      const std::string st_topic = formatTopic(state_topic_template_, id);
      const std::string gd_topic = formatTopic(guidance_topic_template_, id);

      state_subs_.push_back(
          nh_.subscribe<ego_planner::SwarmAgentState>(
              st_topic, 10,
              [this, id](const ego_planner::SwarmAgentStateConstPtr &msg)
              {
                AgentStateCache &cache = agent_states_[id];
                cache.received = msg->is_valid;
                cache.stamp = msg->header.stamp;
                cache.pos = msg->position;
              }));

      guidance_subs_.push_back(
          nh_.subscribe<ego_planner::Bezier>(
              gd_topic, 10,
              [this, id](const ego_planner::BezierConstPtr &msg)
              {
                GuidanceCache &cache = guidance_states_[id];
                cache.received = true;
                cache.recv_stamp = ros::Time::now();
                cache.start_time = msg->start_time;
              }));

      ROS_INFO("[SwarmCommViz] agent=%d, state=%s, guidance=%s", id, st_topic.c_str(), gd_topic.c_str());
    }

    timer_ = nh_.createTimer(ros::Duration(1.0 / std::max(1.0, publish_rate_)), &SwarmCommVisualizer::onTimer, this);
    ROS_INFO("[SwarmCommViz] started. marker_topic=%s", marker_topic_.c_str());
  }

private:
  void leaderBezierCallback(const ego_planner::BezierConstPtr &msg)
  {
    if (msg->pos_pts.empty())
      return;

    master_anchor_ = msg->pos_pts.front();
    have_master_anchor_ = true;
  }

  void onTimer(const ros::TimerEvent &)
  {
    visualization_msgs::MarkerArray array_msg;

    visualization_msgs::Marker clear_all;
    clear_all.header.frame_id = "world";
    clear_all.header.stamp = ros::Time::now();
    clear_all.action = visualization_msgs::Marker::DELETEALL;
    array_msg.markers.push_back(clear_all);

    const ros::Time now = ros::Time::now();

    geometry_msgs::Point anchor = master_anchor_;
    if (!have_master_anchor_)
    {
      anchor.x = 0.0;
      anchor.y = 0.0;
      anchor.z = 1.0;
    }

    array_msg.markers.push_back(makeSphereMarker(1, "master_anchor", anchor, 0.22, 1.0f, 0.9f, 0.1f, 0.95f));

    geometry_msgs::Point master_text_pos = anchor;
    master_text_pos.z += 0.6;
    array_msg.markers.push_back(makeTextMarker(2, "master_anchor", master_text_pos,
                                               have_master_anchor_ ? "MASTER" : "MASTER(no leader traj)",
                                               1.0f, 1.0f, 0.2f));

    for (const int id : agent_ids_)
    {
      const auto sit = agent_states_.find(id);
      const auto git = guidance_states_.find(id);

      const bool has_state = (sit != agent_states_.end() && sit->second.received);
      const bool has_guidance = (git != guidance_states_.end() && git->second.received);

      const double state_age = has_state ? (now - sit->second.stamp).toSec() : 1e9;
      const double guidance_age = has_guidance ? (now - git->second.recv_stamp).toSec() : 1e9;

      const bool state_fresh = has_state && state_age <= state_timeout_;
      const bool guidance_fresh = has_guidance && guidance_age <= guidance_timeout_;

      geometry_msgs::Point p;
      if (has_state)
      {
        p = sit->second.pos;
      }
      else
      {
        p = anchor;
        p.y += 0.8 * static_cast<double>(id);
      }

      const float line_r = guidance_fresh ? 0.2f : 1.0f;
      const float line_g = guidance_fresh ? 0.9f : 0.2f;
      const float line_b = guidance_fresh ? 1.0f : 0.2f;
      array_msg.markers.push_back(makeLineMarker(1000 + id, "comm_link", anchor, p, 0.05, line_r, line_g, line_b, 0.95f));

      if (has_state)
      {
        const float sphere_r = state_fresh ? 0.2f : 1.0f;
        const float sphere_g = state_fresh ? 1.0f : 0.2f;
        const float sphere_b = 0.2f;
        array_msg.markers.push_back(makeSphereMarker(2000 + id, "agent_state", p, 0.24, sphere_r, sphere_g, sphere_b, 0.95f));
      }

      geometry_msgs::Point text_p = p;
      text_p.z += text_z_offset_;

      const bool state_timeout_flag = !state_fresh;
      const bool guidance_timeout_flag = !guidance_fresh;

      std::ostringstream ss;
      ss << std::fixed << std::setprecision(2)
         << "agent_id=" << id
         << " | state_age=" << (has_state ? state_age : -1.0) << "s"
         << " | guidance_age=" << (has_guidance ? guidance_age : -1.0) << "s"
         << " | timeout_state=" << (state_timeout_flag ? "Y" : "N")
         << " | timeout_guidance=" << (guidance_timeout_flag ? "Y" : "N");

      const float text_r = (state_fresh && guidance_fresh) ? 0.2f : 1.0f;
      const float text_g = (state_fresh && guidance_fresh) ? 1.0f : 0.3f;
      const float text_b = 0.2f;
      array_msg.markers.push_back(makeTextMarker(3000 + id, "agent_text", text_p, ss.str(), text_r, text_g, text_b));
    }

    marker_pub_.publish(array_msg);
  }

private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  std::vector<int> agent_ids_;

  std::string leader_bezier_topic_;
  std::string state_topic_template_;
  std::string guidance_topic_template_;
  std::string marker_topic_;

  double publish_rate_ = 15.0;
  double state_timeout_ = 0.30;
  double guidance_timeout_ = 0.30;
  double text_z_offset_ = 0.50;

  ros::Publisher marker_pub_;
  ros::Subscriber leader_sub_;
  std::vector<ros::Subscriber> state_subs_;
  std::vector<ros::Subscriber> guidance_subs_;
  ros::Timer timer_;

  geometry_msgs::Point master_anchor_;
  bool have_master_anchor_ = false;

  std::map<int, AgentStateCache> agent_states_;
  std::map<int, GuidanceCache> guidance_states_;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "swarm_comm_visualizer");
  SwarmCommVisualizer node;
  ros::spin();
  return 0;
}
