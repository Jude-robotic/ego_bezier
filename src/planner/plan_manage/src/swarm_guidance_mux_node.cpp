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
  PROGRAM_RING,
  PROGRAM_NARROW,
  PROGRAM_DOOR,
  PROGRAM_SLIT
};

std::string toLowerCopy(std::string text)
{
  std::transform(text.begin(), text.end(), text.begin(),
                 [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return text;
}

std::string normalizeModeToken(std::string mode_raw)
{
  const auto not_space = [](unsigned char c) {
    return !std::isspace(c);
  };

  const auto begin_it = std::find_if(mode_raw.begin(), mode_raw.end(), not_space);
  const auto end_it = std::find_if(mode_raw.rbegin(), mode_raw.rend(), not_space).base();
  if (begin_it >= end_it)
    return std::string();

  std::string mode(begin_it, end_it);
  if (mode.size() >= 2)
  {
    const char first = mode.front();
    const char last = mode.back();
    if ((first == '\'' && last == '\'') || (first == '"' && last == '"'))
      mode = mode.substr(1, mode.size() - 2);
  }

  return toLowerCopy(mode);
}

ModeKind modeKindFromString(const std::string &mode_raw)
{
  const std::string mode = normalizeModeToken(mode_raw);
  if (mode == "normal" || mode == "program_normal")
    return ModeKind::NORMAL;
  if (mode == "program_ring" || mode == "ring" || mode == "line" || mode == "custom_ring")
    return ModeKind::PROGRAM_RING;
  if (mode == "program_narrow" || mode == "narrow" || mode == "narrow_v" ||
      mode == "tight_narrow" || mode == "compact")
    return ModeKind::PROGRAM_NARROW;
  if (mode == "program_door" || mode == "door" || mode == "door_frame")
    return ModeKind::PROGRAM_DOOR;
  if (mode == "program_slit" || mode == "slit" || mode == "z_slit" || mode == "low_profile")
    return ModeKind::PROGRAM_SLIT;
  return ModeKind::NORMAL;
}

bool isRecognizedModeString(const std::string &mode_raw)
{
  const std::string mode = normalizeModeToken(mode_raw);
  return mode == "normal" || mode == "program_normal" ||
         mode == "program_ring" || mode == "ring" || mode == "line" || mode == "custom_ring" ||
         mode == "program_narrow" || mode == "narrow" || mode == "narrow_v" ||
             mode == "tight_narrow" || mode == "compact" ||
         mode == "program_door" || mode == "door" || mode == "door_frame" ||
         mode == "program_slit" || mode == "slit" || mode == "z_slit" || mode == "low_profile";
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
  CachedBezier program_ring;
  CachedBezier program_narrow;
  CachedBezier program_door;
  CachedBezier program_slit;
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
    pnh.param<std::string>("program_ring_guidance_topic_template", program_ring_guidance_topic_template_,
                           std::string("/swarm/program_ring/agent_{id}/guidance_bezier"));
    if (pnh.hasParam("custom_ring_guidance_topic_template"))
    {
      pnh.getParam("custom_ring_guidance_topic_template", program_ring_guidance_topic_template_);
      ROS_WARN("[GuidanceMux] param custom_ring_guidance_topic_template is deprecated; use program_ring_guidance_topic_template.");
    }
    else if (pnh.hasParam("custom_guidance_topic_template"))
    {
      pnh.getParam("custom_guidance_topic_template", program_ring_guidance_topic_template_);
      ROS_WARN("[GuidanceMux] param custom_guidance_topic_template is deprecated; use program_ring_guidance_topic_template.");
    }
    pnh.param<std::string>("program_narrow_guidance_topic_template", program_narrow_guidance_topic_template_,
                           std::string("/swarm/program_narrow/agent_{id}/guidance_bezier"));
    pnh.param<std::string>("program_door_guidance_topic_template", program_door_guidance_topic_template_,
                 std::string("/swarm/program_door/agent_{id}/guidance_bezier"));
    pnh.param<std::string>("program_slit_guidance_topic_template", program_slit_guidance_topic_template_,
                 std::string("/swarm/program_slit/agent_{id}/guidance_bezier"));
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
      program_ring_subs_.push_back(
          nh.subscribe<ego_planner::Bezier>(
              formatTopic(program_ring_guidance_topic_template_, agent_id), 10,
            boost::bind(&SwarmGuidanceMux::programRingCallback, this, _1, agent_id)));
      program_narrow_subs_.push_back(
          nh.subscribe<ego_planner::Bezier>(
              formatTopic(program_narrow_guidance_topic_template_, agent_id), 10,
            boost::bind(&SwarmGuidanceMux::programNarrowCallback, this, _1, agent_id)));
        program_door_subs_.push_back(
          nh.subscribe<ego_planner::Bezier>(
            formatTopic(program_door_guidance_topic_template_, agent_id), 10,
            boost::bind(&SwarmGuidanceMux::programDoorCallback, this, _1, agent_id)));
        program_slit_subs_.push_back(
          nh.subscribe<ego_planner::Bezier>(
            formatTopic(program_slit_guidance_topic_template_, agent_id), 10,
            boost::bind(&SwarmGuidanceMux::programSlitCallback, this, _1, agent_id)));

        ROS_INFO("[GuidanceMux] agent=%d normal=%s program_ring=%s program_narrow=%s program_door=%s program_slit=%s output=%s",
               agent_id,
               formatTopic(normal_guidance_topic_template_, agent_id).c_str(),
               formatTopic(program_ring_guidance_topic_template_, agent_id).c_str(),
               formatTopic(program_narrow_guidance_topic_template_, agent_id).c_str(),
             formatTopic(program_door_guidance_topic_template_, agent_id).c_str(),
             formatTopic(program_slit_guidance_topic_template_, agent_id).c_str(),
               formatTopic(output_guidance_topic_template_, agent_id).c_str());
    }

    ROS_INFO("[GuidanceMux] ready. mode_topic=%s default_mode=%s",
             mode_topic_.c_str(), active_mode_.c_str());
  }

private:
  void modeCallback(const std_msgs::StringConstPtr &msg)
  {
    const std::string normalized = normalizeModeToken(msg->data);
    if (!isRecognizedModeString(msg->data))
    {
      ROS_WARN("[GuidanceMux] unknown mode raw='%s' normalized='%s'. Supported: normal/program_normal, program_ring, program_narrow, program_door, program_slit. Falling back to normal routing.",
               msg->data.c_str(), normalized.c_str());
    }

    active_mode_ = normalized;
    ROS_INFO("[GuidanceMux] mode switched raw='%s' normalized='%s'.",
             msg->data.c_str(), active_mode_.c_str());

    for (const int agent_id : agent_ids_)
      publishSelected(agent_id, true);
  }

  void normalCallback(const ego_planner::BezierConstPtr &msg, const int agent_id)
  {
    channels_[agent_id].normal.msg = *msg;
    channels_[agent_id].normal.valid = true;
    publishSelected(agent_id, false);
  }

  void programRingCallback(const ego_planner::BezierConstPtr &msg, const int agent_id)
  {
    channels_[agent_id].program_ring.msg = *msg;
    channels_[agent_id].program_ring.valid = true;
    publishSelected(agent_id, false);
  }

  void programNarrowCallback(const ego_planner::BezierConstPtr &msg, const int agent_id)
  {
    channels_[agent_id].program_narrow.msg = *msg;
    channels_[agent_id].program_narrow.valid = true;
    publishSelected(agent_id, false);
  }

  void programDoorCallback(const ego_planner::BezierConstPtr &msg, const int agent_id)
  {
    channels_[agent_id].program_door.msg = *msg;
    channels_[agent_id].program_door.valid = true;
    publishSelected(agent_id, false);
  }

  void programSlitCallback(const ego_planner::BezierConstPtr &msg, const int agent_id)
  {
    channels_[agent_id].program_slit.msg = *msg;
    channels_[agent_id].program_slit.valid = true;
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

    auto trySelect = [&](const CachedBezier &cache, const char *name) {
      if (selected || !cache.valid)
        return;
      selected = &cache;
      selected_name = name;
    };

    switch (mode)
    {
      case ModeKind::PROGRAM_RING:
        trySelect(channel.program_ring, "program_ring");
        trySelect(channel.normal, "normal(fallback)");
        trySelect(channel.program_narrow, "program_narrow(fallback)");
        trySelect(channel.program_door, "program_door(fallback)");
        trySelect(channel.program_slit, "program_slit(fallback)");
        break;
      case ModeKind::PROGRAM_NARROW:
        trySelect(channel.program_narrow, "program_narrow");
        trySelect(channel.normal, "normal(fallback)");
        trySelect(channel.program_ring, "program_ring(fallback)");
        trySelect(channel.program_door, "program_door(fallback)");
        trySelect(channel.program_slit, "program_slit(fallback)");
        break;
      case ModeKind::PROGRAM_DOOR:
        trySelect(channel.program_door, "program_door");
        trySelect(channel.normal, "normal(fallback)");
        trySelect(channel.program_ring, "program_ring(fallback)");
        trySelect(channel.program_narrow, "program_narrow(fallback)");
        trySelect(channel.program_slit, "program_slit(fallback)");
        break;
      case ModeKind::PROGRAM_SLIT:
        trySelect(channel.program_slit, "program_slit");
        trySelect(channel.normal, "normal(fallback)");
        trySelect(channel.program_ring, "program_ring(fallback)");
        trySelect(channel.program_narrow, "program_narrow(fallback)");
        trySelect(channel.program_door, "program_door(fallback)");
        break;
      case ModeKind::NORMAL:
      default:
        trySelect(channel.normal, "normal");
        trySelect(channel.program_ring, "program_ring(fallback-no-normal)");
        trySelect(channel.program_narrow, "program_narrow(fallback-no-normal)");
        trySelect(channel.program_door, "program_door(fallback-no-normal)");
        trySelect(channel.program_slit, "program_slit(fallback-no-normal)");
        break;
    }

    if (!selected)
    {
      ROS_WARN_THROTTLE(1.0,
                        "[GuidanceMux] agent=%d mode=%s has no guidance. valid channels: normal=%d ring=%d narrow=%d door=%d slit=%d",
                        agent_id,
                        active_mode_.c_str(),
                        static_cast<int>(channel.normal.valid),
                        static_cast<int>(channel.program_ring.valid),
                        static_cast<int>(channel.program_narrow.valid),
                        static_cast<int>(channel.program_door.valid),
                        static_cast<int>(channel.program_slit.valid));
      return;
    }

    channel.pub.publish(selected->msg);

    const bool fallback_route =
        std::string(selected_name).find("fallback") != std::string::npos;
    if (fallback_route)
    {
      ROS_WARN_THROTTLE(1.0,
                        "[GuidanceMux] agent=%d mode=%s requested but forwarding %s (traj_id=%ld).",
                        agent_id,
                        active_mode_.c_str(),
                        selected_name,
                        static_cast<long>(selected->msg.traj_id));
    }

    if (force_log)
    {
      ROS_INFO("[GuidanceMux] agent=%d now forwarding %s guidance (traj_id=%ld).",
               agent_id, selected_name, static_cast<long>(selected->msg.traj_id));
    }
    else if (fallback_route)
    {
      ROS_INFO_THROTTLE(1.0,
                        "[GuidanceMux] agent=%d now forwarding %s guidance (traj_id=%ld).",
                        agent_id, selected_name, static_cast<long>(selected->msg.traj_id));
    }
  }

private:
  std::vector<int> agent_ids_;
  std::unordered_map<int, AgentChannel> channels_;

  std::string active_mode_{"normal"};
  std::string mode_topic_;
  std::string normal_guidance_topic_template_;
  std::string program_ring_guidance_topic_template_;
  std::string program_narrow_guidance_topic_template_;
  std::string program_door_guidance_topic_template_;
  std::string program_slit_guidance_topic_template_;
  std::string output_guidance_topic_template_;

  ros::Subscriber mode_sub_;
  std::vector<ros::Subscriber> normal_subs_;
  std::vector<ros::Subscriber> program_ring_subs_;
  std::vector<ros::Subscriber> program_narrow_subs_;
  std::vector<ros::Subscriber> program_door_subs_;
  std::vector<ros::Subscriber> program_slit_subs_;
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
