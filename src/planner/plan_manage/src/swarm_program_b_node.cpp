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

bool useProgramB(const std::string &mode_raw)
{
  const std::string mode = toLowerCopy(mode_raw);
  return mode == "program_b" || mode == "b" || mode == "compact_v" ||
         mode == "tight_v" || mode == "compact";
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

void loadDoubleParam(ros::NodeHandle &pnh, ros::NodeHandle &nh,
                     const std::string &private_key,
                     const std::string &global_key,
                     const double fallback,
                     double &value)
{
  if (pnh.getParam(private_key, value))
    return;

  if (!global_key.empty() && nh.getParam(global_key, value))
    return;

  value = fallback;
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

class SwarmProgramB
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
                           std::string("/swarm/program_b/agent_{id}/guidance_bezier"));
    pnh.param("transition_duration", transition_duration_, 2.4);
    pnh.param("same_traj_start_delay", same_traj_start_delay_, 0.03);
    pnh.param("refresh_min_interval", refresh_min_interval_, 0.20);
    pnh.param("refresh_min_alpha_delta", refresh_min_alpha_delta_, 0.08);
    pnh.param("publish_rate", publish_rate_, 10.0);

    loadDoubleParam(pnh, nh, "compact_spacing",
                    "",
                    -1.0, compact_spacing_cmd_);
    loadDoubleParam(pnh, nh, "formation_spacing",
                    "/swarm_master_node/swarm_master/formation/spacing",
                    1.6, normal_spacing_);
    loadDoubleParam(pnh, nh, "formation_angle_deg",
                    "/swarm_master_node/swarm_master/formation/angle_deg",
                    35.0, formation_angle_deg_);
    loadDoubleParam(pnh, nh, "formation_z_offset",
                    "/swarm_master_node/swarm_master/formation/z_offset",
                    0.0, formation_z_offset_);
    loadDoubleParam(pnh, nh, "min_pair_distance",
                    "",
                    -1.0, min_pair_distance_cmd_);
    loadDoubleParam(pnh, nh, "safe_radius",
                    "/swarm_master_node/swarm_master/safe_radius",
                    1.2, safe_radius_);
    loadDoubleParam(pnh, nh, "inter_uav_sep_min",
                    "/swarm_master_node/swarm_master/payload/inter_uav_sep_min",
                    0.8, inter_uav_sep_min_);

    if (!pnh.getParam("agent_ids", agent_ids_) || agent_ids_.empty())
    {
      agent_ids_.push_back(1);
      agent_ids_.push_back(2);
      ROS_WARN("[ProgramB] agent_ids not provided, fallback to [1, 2].");
    }

    resolveCompactSpacing();

    mode_sub_ = nh.subscribe(mode_topic_, 5, &SwarmProgramB::modeCallback, this);
    leader_guidance_sub_ = nh.subscribe(leader_guidance_topic_, 10,
                                        &SwarmProgramB::leaderGuidanceCallback, this);
    leader_state_sub_ = nh.subscribe(leader_state_topic_, 20,
                                     &SwarmProgramB::leaderStateCallback, this);

    for (size_t i = 0; i < agent_ids_.size(); ++i)
    {
      const int agent_id = agent_ids_[i];
      base_guidance_subs_.push_back(
          nh.subscribe<ego_planner::Bezier>(
              formatTopic(normal_guidance_topic_template_, agent_id), 10,
              boost::bind(&SwarmProgramB::baseGuidanceCallback, this, _1, agent_id)));
      output_pubs_[agent_id] = nh.advertise<ego_planner::Bezier>(
          formatTopic(output_guidance_topic_template_, agent_id), 10);
      agent_rank_[agent_id] = static_cast<int>(i) + 1;

      ROS_INFO("[ProgramB] agent=%d normal=%s output=%s rank=%d",
               agent_id,
               formatTopic(normal_guidance_topic_template_, agent_id).c_str(),
               formatTopic(output_guidance_topic_template_, agent_id).c_str(),
               agent_rank_[agent_id]);
    }

    timer_ = nh.createTimer(ros::Duration(1.0 / std::max(1.0, publish_rate_)),
                            &SwarmProgramB::timerCallback, this);

    ROS_INFO("[ProgramB] ready. mode_topic=%s compact_spacing=%.3f normal_spacing=%.3f angle=%.1fdeg pair_min=%.3f",
             mode_topic_.c_str(), compact_spacing_, normal_spacing_, formation_angle_deg_,
             pair_distance_min_);
  }

private:
  void resolveCompactSpacing()
  {
    const double angle_rad = formation_angle_deg_ * M_PI / 180.0;
    const double lateral_factor = 2.0 * std::abs(std::sin(angle_rad));
    const double base_pair_min = std::max(safe_radius_, inter_uav_sep_min_);
    if (min_pair_distance_cmd_ > 0.0 && min_pair_distance_cmd_ + 1e-3 < base_pair_min)
    {
      ROS_WARN("[ProgramB] min_pair_distance=%.3f is below base safety %.3f. "
               "The stricter safety threshold will be kept.",
               min_pair_distance_cmd_, base_pair_min);
    }
    pair_distance_min_ = (min_pair_distance_cmd_ > 0.0)
                             ? std::max(min_pair_distance_cmd_, base_pair_min)
                             : base_pair_min;

    double required_spacing = pair_distance_min_;
    if (lateral_factor > kEps)
      required_spacing = std::max(required_spacing, pair_distance_min_ / lateral_factor);

    compact_spacing_ = (compact_spacing_cmd_ > 0.0) ? compact_spacing_cmd_ : required_spacing;
    if (compact_spacing_ < required_spacing)
    {
      ROS_WARN("[ProgramB] compact_spacing=%.3f is below required safe spacing %.3f, clamp applied.",
               compact_spacing_, required_spacing);
      compact_spacing_ = required_spacing;
    }

    if (compact_spacing_ > normal_spacing_ + 1e-3 && required_spacing <= normal_spacing_ + 1e-3)
    {
      ROS_WARN("[ProgramB] compact_spacing=%.3f is above normal spacing %.3f. "
               "Program B keeps the tighter normal-spacing upper bound.",
               compact_spacing_, normal_spacing_);
      compact_spacing_ = normal_spacing_;
    }

    if (compact_spacing_ > normal_spacing_ + 1e-3)
    {
      ROS_WARN("[ProgramB] computed compact spacing %.3f exceeds normal spacing %.3f. "
               "Program B will not compress under current constraints.",
               compact_spacing_, normal_spacing_);
    }

    ROS_INFO("[ProgramB] spacing resolution: safe_radius=%.3f inter_uav_sep_min=%.3f pair_min=%.3f required=%.3f final=%.3f",
             safe_radius_, inter_uav_sep_min_, pair_distance_min_, required_spacing, compact_spacing_);
  }

  void modeCallback(const std_msgs::StringConstPtr &msg)
  {
    target_active_ = useProgramB(msg->data);
    ROS_INFO("[ProgramB] mode command '%s' -> target_active=%d",
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

  Eigen::MatrixXd buildCompactVCtrl(const Eigen::MatrixXd &leader_ctrl,
                                    const int rank) const
  {
    Eigen::MatrixXd compact_ctrl = leader_ctrl;
    const std::vector<Eigen::Vector3d> headings = computeHeadings(leader_ctrl);
    const double angle_rad = formation_angle_deg_ * M_PI / 180.0;
    const int level = std::max(1, (rank + 1) / 2);
    const int side = (rank % 2 == 1) ? 1 : -1;

    const double back_dist = static_cast<double>(level) * compact_spacing_ * std::cos(angle_rad);
    const double lateral_dist = static_cast<double>(side * level) * compact_spacing_ * std::sin(angle_rad);

    for (int i = 0; i < leader_ctrl.cols(); ++i)
    {
      const Eigen::Vector3d &forward = headings[static_cast<size_t>(i)];
      Eigen::Vector3d left_dir(-forward.y(), forward.x(), 0.0);
      if (left_dir.norm() < kEps)
        left_dir = Eigen::Vector3d::UnitY();
      else
        left_dir.normalize();

      compact_ctrl.col(i) =
          leader_ctrl.col(i) -
          forward * back_dist +
          left_dir * lateral_dist +
          Eigen::Vector3d(0.0, 0.0, formation_z_offset_);
    }

    return compact_ctrl;
  }

  double agentAlpha(const int /*agent_id*/) const
  {
    return compact_alpha_;
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
                        "[ProgramB] agent=%d ctrl size mismatch: base=%d leader=%d, skip.",
                        agent_id, static_cast<int>(base_ctrl.cols()),
                        static_cast<int>(leader_ctrl.cols()));
      return;
    }

    const double alpha = agentAlpha(agent_id);
    if (!shouldPublish(agent_id, alpha, now))
      return;

    const Eigen::MatrixXd compact_ctrl = buildCompactVCtrl(leader_ctrl, rank_it->second);
    const Eigen::MatrixXd out_ctrl =
        (1.0 - alpha) * base_ctrl + alpha * compact_ctrl;

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
                      "[ProgramB] agent=%d publish alpha=%.2f compact_spacing=%.3f traj_id=%ld start_in=%.3f",
                      agent_id, alpha, compact_spacing_, static_cast<long>(out.traj_id),
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

    const double old_alpha = compact_alpha_;
    if (target_active_)
      compact_alpha_ = clamp01(compact_alpha_ + dt / std::max(1e-3, transition_duration_));
    else
      compact_alpha_ = clamp01(compact_alpha_ - dt / std::max(1e-3, transition_duration_));

    const bool alpha_moved = std::fabs(compact_alpha_ - old_alpha) > 1e-4;
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

  double transition_duration_{2.4};
  double same_traj_start_delay_{0.03};
  double refresh_min_interval_{0.20};
  double refresh_min_alpha_delta_{0.08};
  double publish_rate_{10.0};

  double compact_spacing_cmd_{-1.0};
  double min_pair_distance_cmd_{-1.0};
  double normal_spacing_{1.6};
  double formation_angle_deg_{35.0};
  double formation_z_offset_{0.0};
  double safe_radius_{1.2};
  double inter_uav_sep_min_{0.8};
  double pair_distance_min_{1.2};
  double compact_spacing_{1.2};

  bool target_active_{false};
  bool source_dirty_{false};
  double compact_alpha_{0.0};
  int64_t custom_traj_id_{200000};

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
  ros::init(argc, argv, "swarm_program_b_node");

  SwarmProgramB node;
  node.init();
  ros::spin();
  return 0;
}
