#include <ego_planner/Bezier.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <std_msgs/String.h>

#include <Eigen/Eigen>

#include <algorithm>
#include <boost/bind.hpp>
#include <cctype>
#include <cmath>
#include <limits>
#include <string>
#include <unordered_map>
#include <vector>

namespace
{

constexpr double kEps = 1e-6;

std::string toLowerCopy(std::string text)
{
  std::transform(text.begin(), text.end(), text.begin(),
                 [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return text;
}

bool useProgramA(const std::string &mode_raw)
{
  const std::string mode = toLowerCopy(mode_raw);
  return mode == "program_a" || mode == "a" || mode == "line" || mode == "custom_a";
}

double clamp01(const double value)
{
  return std::max(0.0, std::min(1.0, value));
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

Eigen::Vector3d normalizedOrFallback(const Eigen::Vector3d &vec,
                                     const Eigen::Vector3d &fallback)
{
  const double norm = vec.norm();
  if (norm < kEps)
    return fallback;
  return vec / norm;
}

Eigen::MatrixXd toCtrlPts(const ego_planner::Bezier &msg)
{
  Eigen::MatrixXd ctrl(3, static_cast<int>(msg.pos_pts.size()));
  for (size_t i = 0; i < msg.pos_pts.size(); ++i)
  {
    ctrl(0, static_cast<int>(i)) = msg.pos_pts[i].x;
    ctrl(1, static_cast<int>(i)) = msg.pos_pts[i].y;
    ctrl(2, static_cast<int>(i)) = msg.pos_pts[i].z;
  }
  return ctrl;
}

ego_planner::Bezier buildBezierMsg(const ego_planner::Bezier &meta,
                                   const Eigen::MatrixXd &ctrl,
                                   const ros::Time &start_time,
                                   const int64_t traj_id)
{
  ego_planner::Bezier out = meta;
  out.start_time = start_time;
  out.traj_id = traj_id;
  out.pos_pts.clear();
  out.pos_pts.reserve(static_cast<size_t>(ctrl.cols()));
  for (int i = 0; i < ctrl.cols(); ++i)
  {
    geometry_msgs::Point p;
    p.x = ctrl(0, i);
    p.y = ctrl(1, i);
    p.z = ctrl(2, i);
    out.pos_pts.push_back(p);
  }
  return out;
}

struct CachedBezier
{
  ego_planner::Bezier msg;
  bool valid{false};
};

struct PublishRecord
{
  int64_t leader_traj_id{-1};
  ros::Time leader_start_time;
  int64_t base_traj_id{-1};
  ros::Time base_start_time;
  double alpha{std::numeric_limits<double>::quiet_NaN()};
  ros::Time stamp;
};

class SwarmProgramA
{
public:
  void init()
  {
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    pnh.param<std::string>("mode_topic", mode_topic_, std::string("/swarm/formation_mode"));
    pnh.param<std::string>("leader_guidance_topic", leader_guidance_topic_,
                           std::string("/swarm/leader/corrected_bezier"));
    pnh.param<std::string>("leader_state_topic", leader_state_topic_,
                           std::string("/visual_slam/odom"));
    pnh.param<std::string>("normal_guidance_topic_template", normal_guidance_topic_template_,
                           std::string("/swarm/master/agent_{id}/guidance_bezier"));
    pnh.param<std::string>("output_guidance_topic_template", output_guidance_topic_template_,
                           std::string("/swarm/program_a/agent_{id}/guidance_bezier"));
    pnh.param("line_spacing", line_spacing_, 1.0);
    pnh.param("transition_duration", transition_duration_, 2.4);
    pnh.param("agent2_phase_delay", agent2_phase_delay_, 0.35);
    pnh.param("same_traj_start_delay", same_traj_start_delay_, 0.03);
    pnh.param("refresh_min_interval", refresh_min_interval_, 0.20);
    pnh.param("refresh_min_alpha_delta", refresh_min_alpha_delta_, 0.08);
    pnh.param("publish_rate", publish_rate_, 10.0);

    if (!pnh.getParam("agent_ids", agent_ids_) || agent_ids_.empty())
    {
      agent_ids_.push_back(1);
      agent_ids_.push_back(2);
      ROS_WARN("[ProgramA] agent_ids not provided, fallback to [1, 2].");
    }

    mode_sub_ = nh.subscribe(mode_topic_, 5, &SwarmProgramA::modeCallback, this);
    leader_guidance_sub_ = nh.subscribe(leader_guidance_topic_, 10,
                                        &SwarmProgramA::leaderGuidanceCallback, this);
    leader_state_sub_ = nh.subscribe(leader_state_topic_, 20,
                                     &SwarmProgramA::leaderStateCallback, this);

    for (size_t i = 0; i < agent_ids_.size(); ++i)
    {
      const int agent_id = agent_ids_[i];
      base_guidance_subs_.push_back(
          nh.subscribe<ego_planner::Bezier>(
              formatTopic(normal_guidance_topic_template_, agent_id), 10,
              boost::bind(&SwarmProgramA::baseGuidanceCallback, this, _1, agent_id)));
      output_pubs_[agent_id] = nh.advertise<ego_planner::Bezier>(
          formatTopic(output_guidance_topic_template_, agent_id), 10);
      agent_rank_[agent_id] = static_cast<int>(i) + 1;

      ROS_INFO("[ProgramA] agent=%d normal=%s output=%s rank=%d",
               agent_id,
               formatTopic(normal_guidance_topic_template_, agent_id).c_str(),
               formatTopic(output_guidance_topic_template_, agent_id).c_str(),
               agent_rank_[agent_id]);
    }

    timer_ = nh.createTimer(ros::Duration(1.0 / std::max(1.0, publish_rate_)),
                            &SwarmProgramA::timerCallback, this);

    ROS_INFO("[ProgramA] ready. mode_topic=%s line_spacing=%.2f transition_duration=%.2fs",
             mode_topic_.c_str(), line_spacing_, transition_duration_);
  }

private:
  void modeCallback(const std_msgs::StringConstPtr &msg)
  {
    target_active_ = useProgramA(msg->data);
    ROS_INFO("[ProgramA] mode command '%s' -> target_active=%d",
             msg->data.c_str(), static_cast<int>(target_active_));
  }

  void leaderGuidanceCallback(const ego_planner::BezierConstPtr &msg)
  {
    leader_guidance_.msg = *msg;
    leader_guidance_.valid = true;
    source_dirty_ = true;
  }

  void leaderStateCallback(const nav_msgs::OdometryConstPtr &msg)
  {
    leader_vel_(0) = msg->twist.twist.linear.x;
    leader_vel_(1) = msg->twist.twist.linear.y;
    leader_vel_(2) = msg->twist.twist.linear.z;
    have_leader_state_ = true;
  }

  void baseGuidanceCallback(const ego_planner::BezierConstPtr &msg, const int agent_id)
  {
    base_guidance_[agent_id].msg = *msg;
    base_guidance_[agent_id].valid = true;
    source_dirty_ = true;
  }

  Eigen::Vector3d fallbackHeading() const
  {
    Eigen::Vector3d fallback = Eigen::Vector3d::UnitX();
    if (have_leader_state_)
    {
      Eigen::Vector3d vel_xy = leader_vel_;
      vel_xy.z() = 0.0;
      if (vel_xy.norm() > 0.1)
        fallback = vel_xy.normalized();
    }
    return fallback;
  }

  std::vector<Eigen::Vector3d> computeHeadings(const Eigen::MatrixXd &ctrl) const
  {
    const int cols = static_cast<int>(ctrl.cols());
    std::vector<Eigen::Vector3d> headings(static_cast<size_t>(cols), fallbackHeading());
    if (cols <= 0)
      return headings;

    for (int i = 0; i < cols; ++i)
    {
      const int left = std::max(0, i - 1);
      const int right = std::min(cols - 1, i + 1);
      Eigen::Vector3d diff = ctrl.col(right) - ctrl.col(left);
      diff.z() = 0.0;
      const Eigen::Vector3d fallback = (i > 0) ? headings[static_cast<size_t>(i - 1)] : fallbackHeading();
      headings[static_cast<size_t>(i)] = normalizedOrFallback(diff, fallback);
    }

    if (cols >= 3)
    {
      std::vector<Eigen::Vector3d> smoothed = headings;
      for (int i = 1; i < cols - 1; ++i)
      {
        Eigen::Vector3d avg =
            headings[static_cast<size_t>(i - 1)] +
            2.0 * headings[static_cast<size_t>(i)] +
            headings[static_cast<size_t>(i + 1)];
        avg.z() = 0.0;
        smoothed[static_cast<size_t>(i)] =
            normalizedOrFallback(avg, headings[static_cast<size_t>(i)]);
      }
      headings.swap(smoothed);
    }

    return headings;
  }

  Eigen::MatrixXd buildLineCtrl(const Eigen::MatrixXd &leader_ctrl,
                                const double trailing_distance) const
  {
    Eigen::MatrixXd line_ctrl = leader_ctrl;
    const std::vector<Eigen::Vector3d> headings = computeHeadings(leader_ctrl);

    for (int i = 0; i < leader_ctrl.cols(); ++i)
    {
      const Eigen::Vector3d back_shift =
          headings[static_cast<size_t>(i)] * trailing_distance;
      line_ctrl.col(i) = leader_ctrl.col(i) - back_shift;
      line_ctrl(2, i) = leader_ctrl(2, i);
    }
    return line_ctrl;
  }

  double agentAlpha(const int agent_id) const
  {
    const auto rank_it = agent_rank_.find(agent_id);
    const int rank = (rank_it == agent_rank_.end()) ? 1 : rank_it->second;
    if (rank <= 1)
      return line_alpha_;

    const double delayed =
        (line_alpha_ - agent2_phase_delay_) / std::max(1e-3, 1.0 - agent2_phase_delay_);
    return clamp01(delayed);
  }

  bool shouldPublish(const int agent_id, const double alpha,
                     const ros::Time &now) const
  {
    auto record_it = publish_records_.find(agent_id);
    if (record_it == publish_records_.end())
      return true;

    const PublishRecord &record = record_it->second;
    const auto base_it = base_guidance_.find(agent_id);
    if (base_it == base_guidance_.end() || !base_it->second.valid || !leader_guidance_.valid)
      return false;

    const bool leader_changed =
        record.leader_traj_id != leader_guidance_.msg.traj_id ||
        std::fabs((record.leader_start_time - leader_guidance_.msg.start_time).toSec()) > 1e-4;
    const bool base_changed =
        record.base_traj_id != base_it->second.msg.traj_id ||
        std::fabs((record.base_start_time - base_it->second.msg.start_time).toSec()) > 1e-4;
    if (leader_changed || base_changed)
      return true;

    if (!std::isfinite(record.alpha))
      return true;

    const bool alpha_changed = std::fabs(alpha - record.alpha) >= refresh_min_alpha_delta_;
    const bool interval_ready =
        (now - record.stamp).toSec() >= refresh_min_interval_;
    return alpha_changed && interval_ready;
  }

  void publishAgentGuidance(const int agent_id, const ros::Time &now)
  {
    const auto base_it = base_guidance_.find(agent_id);
    const auto pub_it = output_pubs_.find(agent_id);
    const auto rank_it = agent_rank_.find(agent_id);
    if (base_it == base_guidance_.end() || !base_it->second.valid ||
        pub_it == output_pubs_.end() || rank_it == agent_rank_.end() ||
        !leader_guidance_.valid)
      return;

    const ego_planner::Bezier &base_msg = base_it->second.msg;
    const ego_planner::Bezier &leader_msg = leader_guidance_.msg;
    const Eigen::MatrixXd base_ctrl = toCtrlPts(base_msg);
    const Eigen::MatrixXd leader_ctrl = toCtrlPts(leader_msg);

    if (base_ctrl.cols() <= 0 || leader_ctrl.cols() <= 0)
      return;

    if (base_ctrl.cols() != leader_ctrl.cols())
    {
      ROS_WARN_THROTTLE(1.0,
                        "[ProgramA] agent=%d ctrl size mismatch: base=%d leader=%d, skip.",
                        agent_id, static_cast<int>(base_ctrl.cols()),
                        static_cast<int>(leader_ctrl.cols()));
      return;
    }

    const double alpha = agentAlpha(agent_id);
    if (!shouldPublish(agent_id, alpha, now))
      return;

    const double trailing_distance =
        static_cast<double>(rank_it->second) * line_spacing_;
    const Eigen::MatrixXd line_ctrl = buildLineCtrl(leader_ctrl, trailing_distance);
    const Eigen::MatrixXd out_ctrl =
        (1.0 - alpha) * base_ctrl + alpha * line_ctrl;

    const auto record_it = publish_records_.find(agent_id);
    const bool same_leader_traj =
        record_it != publish_records_.end() &&
        record_it->second.leader_traj_id == leader_msg.traj_id &&
        std::fabs((record_it->second.leader_start_time - leader_msg.start_time).toSec()) < 1e-4 &&
        record_it->second.base_traj_id == base_msg.traj_id &&
        std::fabs((record_it->second.base_start_time - base_msg.start_time).toSec()) < 1e-4;

    ros::Time publish_start_time = base_msg.start_time;
    if (same_leader_traj && alpha > 1e-3)
      publish_start_time = now + ros::Duration(same_traj_start_delay_);

    ego_planner::Bezier out =
        buildBezierMsg(base_msg, out_ctrl, publish_start_time, ++custom_traj_id_);
    pub_it->second.publish(out);

    PublishRecord &record = publish_records_[agent_id];
    record.leader_traj_id = leader_msg.traj_id;
    record.leader_start_time = leader_msg.start_time;
    record.base_traj_id = base_msg.traj_id;
    record.base_start_time = base_msg.start_time;
    record.alpha = alpha;
    record.stamp = now;

    ROS_INFO_THROTTLE(0.5,
                      "[ProgramA] agent=%d publish alpha=%.2f trailing=%.2f traj_id=%ld start_in=%.3f",
                      agent_id, alpha, trailing_distance, static_cast<long>(out.traj_id),
                      (out.start_time - now).toSec());
  }

  void timerCallback(const ros::TimerEvent &event)
  {
    const ros::Time now = event.current_real;
    if (!have_timer_stamp_)
    {
      last_timer_stamp_ = now;
      have_timer_stamp_ = true;
    }

    const double dt = std::max(0.0, (now - last_timer_stamp_).toSec());
    last_timer_stamp_ = now;

    const double old_alpha = line_alpha_;
    if (target_active_)
      line_alpha_ = clamp01(line_alpha_ + dt / std::max(1e-3, transition_duration_));
    else
      line_alpha_ = clamp01(line_alpha_ - dt / std::max(1e-3, transition_duration_));

    const bool alpha_moved = std::fabs(line_alpha_ - old_alpha) > 1e-4;
    if (!leader_guidance_.valid)
      return;

    if (!source_dirty_ && !alpha_moved)
      return;

    for (const int agent_id : agent_ids_)
      publishAgentGuidance(agent_id, now);

    source_dirty_ = false;
  }

private:
  std::vector<int> agent_ids_;
  std::unordered_map<int, int> agent_rank_;
  std::unordered_map<int, CachedBezier> base_guidance_;
  CachedBezier leader_guidance_;
  std::unordered_map<int, ros::Publisher> output_pubs_;
  std::unordered_map<int, PublishRecord> publish_records_;

  std::string mode_topic_;
  std::string leader_guidance_topic_;
  std::string leader_state_topic_;
  std::string normal_guidance_topic_template_;
  std::string output_guidance_topic_template_;

  double line_spacing_{1.0};
  double transition_duration_{2.4};
  double agent2_phase_delay_{0.35};
  double same_traj_start_delay_{0.03};
  double refresh_min_interval_{0.20};
  double refresh_min_alpha_delta_{0.08};
  double publish_rate_{10.0};

  bool target_active_{false};
  bool source_dirty_{false};
  double line_alpha_{0.0};
  int64_t custom_traj_id_{100000};

  bool have_timer_stamp_{false};
  ros::Time last_timer_stamp_;

  bool have_leader_state_{false};
  Eigen::Vector3d leader_vel_{Eigen::Vector3d::Zero()};

  ros::Subscriber mode_sub_;
  ros::Subscriber leader_guidance_sub_;
  ros::Subscriber leader_state_sub_;
  std::vector<ros::Subscriber> base_guidance_subs_;
  ros::Timer timer_;
};

}  // namespace

int main(int argc, char **argv)
{
  ros::init(argc, argv, "swarm_program_a_node");

  SwarmProgramA node;
  node.init();
  ros::spin();
  return 0;
}
