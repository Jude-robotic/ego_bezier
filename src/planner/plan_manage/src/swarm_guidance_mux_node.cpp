#include <ego_planner/Bezier.h>
#include <ros/ros.h>
#include <std_msgs/String.h>

#include <algorithm>
#include <boost/bind.hpp>
#include <cctype>
#include <string>
#include <unordered_map>
#include <vector>

namespace
{

enum class ModeKind
{
  NORMAL = 0,
  PROGRAM_A,
  PROGRAM_B
};

std::string toLowerCopy(std::string text)
{
  std::transform(text.begin(), text.end(), text.begin(),
                 [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return text;
}

ModeKind modeKindFromString(const std::string &mode_raw)
{
  const std::string mode = toLowerCopy(mode_raw);
  if (mode == "program_a" || mode == "a" || mode == "line" || mode == "custom_a")
    return ModeKind::PROGRAM_A;
  if (mode == "program_b" || mode == "b" || mode == "compact_v" ||
      mode == "tight_v" || mode == "compact")
    return ModeKind::PROGRAM_B;
  return ModeKind::NORMAL;
}

std::string formatTopic(const std::string &topic_template, const int agent_id)
{
  std::string topic = topic_template;
  const std::string key = "{id}";
  const std::string value = std::to_string(agent_id);
  std::string::size_type pos = 0;
  while ((pos = topic.find(key, pos)) != std::string::npos)
  {
    topic.replace(pos, key.size(), value);
    pos += value.size();
  }
  return topic;
}

struct CachedBezier
{
  ego_planner::Bezier msg;
  bool valid{false};
};

struct AgentChannel
{
  CachedBezier normal;
  CachedBezier program_a;
  CachedBezier program_b;
  ros::Publisher pub;
};

class SwarmGuidanceMux
{
public:
  void init()
  {
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    pnh.param<std::string>("mode_topic", mode_topic_, std::string("/swarm/formation_mode"));
    pnh.param<std::string>("normal_guidance_topic_template", normal_guidance_topic_template_,
                           std::string("/swarm/master/agent_{id}/guidance_bezier"));
    pnh.param<std::string>("program_a_guidance_topic_template", program_a_guidance_topic_template_,
                           std::string("/swarm/program_a/agent_{id}/guidance_bezier"));
    if (pnh.hasParam("custom_guidance_topic_template"))
      pnh.getParam("custom_guidance_topic_template", program_a_guidance_topic_template_);
    pnh.param<std::string>("program_b_guidance_topic_template", program_b_guidance_topic_template_,
                           std::string("/swarm/program_b/agent_{id}/guidance_bezier"));
    pnh.param<std::string>("output_guidance_topic_template", output_guidance_topic_template_,
                           std::string("/swarm/agent_{id}/guidance_bezier"));

    if (!pnh.getParam("agent_ids", agent_ids_) || agent_ids_.empty())
    {
      agent_ids_.push_back(1);
      agent_ids_.push_back(2);
      ROS_WARN("[GuidanceMux] agent_ids not provided, fallback to [1, 2].");
    }

    mode_sub_ = nh.subscribe(mode_topic_, 5, &SwarmGuidanceMux::modeCallback, this);

    for (const int agent_id : agent_ids_)
    {
      AgentChannel channel;
      channel.pub = nh.advertise<ego_planner::Bezier>(
          formatTopic(output_guidance_topic_template_, agent_id), 10);
      channels_[agent_id] = channel;

      normal_subs_.push_back(
          nh.subscribe<ego_planner::Bezier>(
              formatTopic(normal_guidance_topic_template_, agent_id), 10,
              boost::bind(&SwarmGuidanceMux::normalCallback, this, _1, agent_id)));
      program_a_subs_.push_back(
          nh.subscribe<ego_planner::Bezier>(
              formatTopic(program_a_guidance_topic_template_, agent_id), 10,
              boost::bind(&SwarmGuidanceMux::programACallback, this, _1, agent_id)));
      program_b_subs_.push_back(
          nh.subscribe<ego_planner::Bezier>(
              formatTopic(program_b_guidance_topic_template_, agent_id), 10,
              boost::bind(&SwarmGuidanceMux::programBCallback, this, _1, agent_id)));

      ROS_INFO("[GuidanceMux] agent=%d normal=%s program_a=%s program_b=%s output=%s",
               agent_id,
               formatTopic(normal_guidance_topic_template_, agent_id).c_str(),
               formatTopic(program_a_guidance_topic_template_, agent_id).c_str(),
               formatTopic(program_b_guidance_topic_template_, agent_id).c_str(),
               formatTopic(output_guidance_topic_template_, agent_id).c_str());
    }

    ROS_INFO("[GuidanceMux] ready. mode_topic=%s default_mode=%s",
             mode_topic_.c_str(), active_mode_.c_str());
  }

private:
  void modeCallback(const std_msgs::StringConstPtr &msg)
  {
    active_mode_ = msg->data;
    ROS_INFO("[GuidanceMux] mode switched to '%s'.", active_mode_.c_str());

    for (const int agent_id : agent_ids_)
      publishSelected(agent_id, true);
  }

  void normalCallback(const ego_planner::BezierConstPtr &msg, const int agent_id)
  {
    channels_[agent_id].normal.msg = *msg;
    channels_[agent_id].normal.valid = true;

    if (modeKindFromString(active_mode_) == ModeKind::NORMAL)
      publishSelected(agent_id, false);
  }

  void programACallback(const ego_planner::BezierConstPtr &msg, const int agent_id)
  {
    channels_[agent_id].program_a.msg = *msg;
    channels_[agent_id].program_a.valid = true;

    if (modeKindFromString(active_mode_) == ModeKind::PROGRAM_A)
      publishSelected(agent_id, false);
  }

  void programBCallback(const ego_planner::BezierConstPtr &msg, const int agent_id)
  {
    channels_[agent_id].program_b.msg = *msg;
    channels_[agent_id].program_b.valid = true;

    if (modeKindFromString(active_mode_) == ModeKind::PROGRAM_B)
      publishSelected(agent_id, false);
  }

  void publishSelected(const int agent_id, const bool force_log)
  {
    auto it = channels_.find(agent_id);
    if (it == channels_.end())
      return;

    AgentChannel &channel = it->second;
    const CachedBezier *selected = nullptr;
    const char *selected_name = "normal";
    const ModeKind mode = modeKindFromString(active_mode_);

    if (mode == ModeKind::PROGRAM_A && channel.program_a.valid)
    {
      selected = &channel.program_a;
      selected_name = "program_a";
    }
    else if (mode == ModeKind::PROGRAM_B && channel.program_b.valid)
    {
      selected = &channel.program_b;
      selected_name = "program_b";
    }
    else if (channel.normal.valid)
    {
      selected = &channel.normal;
      selected_name = "normal";
    }
    else if (mode == ModeKind::PROGRAM_A && channel.program_a.valid)
    {
      selected = &channel.program_a;
      selected_name = "program_a(fallback)";
    }
    else if (mode == ModeKind::PROGRAM_B && channel.program_b.valid)
    {
      selected = &channel.program_b;
      selected_name = "program_b(fallback)";
    }
    else if (channel.program_a.valid)
    {
      selected = &channel.program_a;
      selected_name = "program_a(fallback-no-normal)";
    }
    else if (channel.program_b.valid)
    {
      selected = &channel.program_b;
      selected_name = "program_b(fallback-no-normal)";
    }

    if (!selected)
    {
      ROS_WARN_THROTTLE(1.0, "[GuidanceMux] agent=%d has no guidance to publish yet.", agent_id);
      return;
    }

    channel.pub.publish(selected->msg);

    if (force_log)
    {
      ROS_INFO("[GuidanceMux] agent=%d now forwarding %s guidance (traj_id=%ld).",
               agent_id, selected_name, static_cast<long>(selected->msg.traj_id));
    }
  }

private:
  std::vector<int> agent_ids_;
  std::unordered_map<int, AgentChannel> channels_;

  std::string active_mode_{"normal"};
  std::string mode_topic_;
  std::string normal_guidance_topic_template_;
  std::string program_a_guidance_topic_template_;
  std::string program_b_guidance_topic_template_;
  std::string output_guidance_topic_template_;

  ros::Subscriber mode_sub_;
  std::vector<ros::Subscriber> normal_subs_;
  std::vector<ros::Subscriber> program_a_subs_;
  std::vector<ros::Subscriber> program_b_subs_;
};

}  // namespace

int main(int argc, char **argv)
{
  ros::init(argc, argv, "swarm_guidance_mux_node");

  SwarmGuidanceMux node;
  node.init();
  ros::spin();
  return 0;
}
