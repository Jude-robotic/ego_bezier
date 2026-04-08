#include <ego_planner/Bezier.h>
#include <ego_planner/SwarmAgentState.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <bezier_opt/payload_geometry.h>

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <map>
#include <mutex>
#include <sstream>
#include <string>
#include <vector>

namespace payload_geometry = ego_planner::payload_geometry;

namespace
{
std::string formatTopic(const std::string &topic_template, int agent_id)
{
  std::string topic = topic_template;
  const std::string key = "{id}";
  const std::size_t pos = topic.find(key);
  if (pos != std::string::npos)
    topic.replace(pos, key.size(), std::to_string(agent_id));
  return topic;
}

bool evalBezier(const ego_planner::Bezier &msg, double t_query, geometry_msgs::Point &result)
{
  const int n_pts = static_cast<int>(msg.pos_pts.size());
  const int n_seg = static_cast<int>(msg.segment_durations.size());
  const int order = msg.order > 0 ? msg.order : 3;
  if (n_pts <= 0 || n_seg <= 0 || order < 1)
    return false;

  double t_local = std::max(0.0, t_query);
  int seg_idx = n_seg - 1;
  for (int i = 0; i < n_seg; ++i)
  {
    if (t_local <= msg.segment_durations[i] || i == n_seg - 1)
    {
      seg_idx = i;
      break;
    }
    t_local -= msg.segment_durations[i];
  }

  const double seg_dt = msg.segment_durations[seg_idx];
  const double u = (seg_dt < 1e-9) ? 1.0 : std::max(0.0, std::min(1.0, t_local / seg_dt));
  const int base = seg_idx * order;

  struct P3
  {
    double x;
    double y;
    double z;
  };

  std::vector<P3> pts;
  pts.reserve(static_cast<size_t>(order + 1));
  for (int k = 0; k <= order; ++k)
  {
    const int idx = std::min(base + k, n_pts - 1);
    pts.push_back({msg.pos_pts[idx].x, msg.pos_pts[idx].y, msg.pos_pts[idx].z});
  }

  int sz = static_cast<int>(pts.size());
  while (sz > 1)
  {
    for (int i = 0; i < sz - 1; ++i)
    {
      pts[i].x = (1.0 - u) * pts[i].x + u * pts[i + 1].x;
      pts[i].y = (1.0 - u) * pts[i].y + u * pts[i + 1].y;
      pts[i].z = (1.0 - u) * pts[i].z + u * pts[i + 1].z;
    }
    --sz;
  }

  result.x = pts[0].x;
  result.y = pts[0].y;
  result.z = pts[0].z;
  return true;
}

double bezierDuration(const ego_planner::Bezier &msg)
{
  double out = 0.0;
  for (const double dt : msg.segment_durations)
    out += dt;
  return out;
}

visualization_msgs::Marker makeSphere(int id, const std::string &ns, const geometry_msgs::Point &p,
                                      const double scale, const float r, const float g,
                                      const float b, const float a)
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
  mk.color.r = r;
  mk.color.g = g;
  mk.color.b = b;
  mk.color.a = a;
  mk.lifetime = ros::Duration(0.25);
  return mk;
}

visualization_msgs::Marker makeText(int id, const std::string &ns, const geometry_msgs::Point &p,
                                    const std::string &text, const float r, const float g,
                                    const float b, const double scale_z)
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
  mk.scale.z = scale_z;
  mk.color.r = r;
  mk.color.g = g;
  mk.color.b = b;
  mk.color.a = 0.98f;
  mk.text = text;
  mk.lifetime = ros::Duration(0.25);
  return mk;
}

visualization_msgs::Marker makeLineList(int id, const std::string &ns, const double width,
                                        const float r, const float g, const float b, const float a)
{
  visualization_msgs::Marker mk;
  mk.header.frame_id = "world";
  mk.header.stamp = ros::Time::now();
  mk.ns = ns;
  mk.id = id;
  mk.type = visualization_msgs::Marker::LINE_LIST;
  mk.action = visualization_msgs::Marker::ADD;
  mk.pose.orientation.w = 1.0;
  mk.scale.x = width;
  mk.color.r = r;
  mk.color.g = g;
  mk.color.b = b;
  mk.color.a = a;
  mk.lifetime = ros::Duration(0.25);
  return mk;
}

visualization_msgs::Marker makeLineStrip(int id, const std::string &ns, const double width,
                                         const float r, const float g, const float b, const float a)
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
  mk.color.r = r;
  mk.color.g = g;
  mk.color.b = b;
  mk.color.a = a;
  mk.lifetime = ros::Duration(0.25);
  return mk;
}

visualization_msgs::Marker makeSphereList(int id, const std::string &ns, const double scale,
                                          const float r, const float g, const float b, const float a)
{
  visualization_msgs::Marker mk;
  mk.header.frame_id = "world";
  mk.header.stamp = ros::Time::now();
  mk.ns = ns;
  mk.id = id;
  mk.type = visualization_msgs::Marker::SPHERE_LIST;
  mk.action = visualization_msgs::Marker::ADD;
  mk.pose.orientation.w = 1.0;
  mk.scale.x = scale;
  mk.scale.y = scale;
  mk.scale.z = scale;
  mk.color.r = r;
  mk.color.g = g;
  mk.color.b = b;
  mk.color.a = a;
  mk.lifetime = ros::Duration(0.25);
  return mk;
}
}  // namespace

class SwarmPayloadVisualizer
{
public:
  SwarmPayloadVisualizer()
      : nh_(), pnh_("~")
  {
    pnh_.param("publish_rate", publish_rate_, 20.0);
    pnh_.param("marker_topic", marker_topic_, std::string("/swarm/payload_viz/markers"));
    pnh_.param("leader_state_topic", leader_state_topic_, std::string("/visual_slam/odom"));
    pnh_.param("leader_traj_topic", leader_traj_topic_, std::string("/swarm/leader/corrected_bezier"));
    pnh_.param("state_topic_template", state_topic_template_, std::string("/swarm/agent_{id}/state"));
    pnh_.param("guidance_topic_template", guidance_topic_template_, std::string("/swarm/agent_{id}/guidance_bezier"));
    pnh_.param("rope_length", rope_length_, 1.0);
    pnh_.param("payload_radius", payload_radius_, 0.2);
    pnh_.param("payload_extra_margin", payload_extra_margin_, 0.05);
    pnh_.param("state_timeout", state_timeout_, 0.3);
    pnh_.param("sample_dt", sample_dt_, 0.05);
    pnh_.param("plan_horizon", plan_horizon_, 3.0);

    pnh_.param("actual_center_size", actual_center_size_, 0.20);
    pnh_.param("planned_center_size", planned_center_size_, 0.18);
    pnh_.param("actual_rope_width", actual_rope_width_, 0.03);
    pnh_.param("planned_traj_width", planned_traj_width_, 0.035);
    pnh_.param("invalid_sample_size", invalid_sample_size_, 0.10);
    pnh_.param("warning_text_size", warning_text_size_, 0.24);

    pnh_.param("actual_center_color_r", actual_center_color_r_, 1.0);
    pnh_.param("actual_center_color_g", actual_center_color_g_, 0.45);
    pnh_.param("actual_center_color_b", actual_center_color_b_, 0.10);
    pnh_.param("actual_center_color_a", actual_center_color_a_, 0.95);

    pnh_.param("actual_safety_color_r", actual_safety_color_r_, 1.0);
    pnh_.param("actual_safety_color_g", actual_safety_color_g_, 0.55);
    pnh_.param("actual_safety_color_b", actual_safety_color_b_, 0.15);
    pnh_.param("actual_safety_color_a", actual_safety_color_a_, 0.22);

    pnh_.param("planned_traj_color_r", planned_traj_color_r_, 0.2);
    pnh_.param("planned_traj_color_g", planned_traj_color_g_, 0.95);
    pnh_.param("planned_traj_color_b", planned_traj_color_b_, 0.95);
    pnh_.param("planned_traj_color_a", planned_traj_color_a_, 0.95);

    pnh_.param("planned_center_color_r", planned_center_color_r_, 0.1);
    pnh_.param("planned_center_color_g", planned_center_color_g_, 0.95);
    pnh_.param("planned_center_color_b", planned_center_color_b_, 0.95);
    pnh_.param("planned_center_color_a", planned_center_color_a_, 0.95);

    pnh_.param("rope_color_r", rope_color_r_, 0.95);
    pnh_.param("rope_color_g", rope_color_g_, 0.65);
    pnh_.param("rope_color_b", rope_color_b_, 0.25);
    pnh_.param("rope_color_a", rope_color_a_, 0.90);

    pnh_.param("invalid_color_r", invalid_color_r_, 1.0);
    pnh_.param("invalid_color_g", invalid_color_g_, 0.12);
    pnh_.param("invalid_color_b", invalid_color_b_, 0.12);
    pnh_.param("invalid_color_a", invalid_color_a_, 0.92);

    if (!pnh_.getParam("agent_ids", agent_ids_) || agent_ids_.size() < 2)
    {
      agent_ids_.clear();
      agent_ids_.push_back(1);
      agent_ids_.push_back(2);
      ROS_WARN("[PayloadViz] agent_ids missing, fallback to [1, 2].");
    }

    marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(marker_topic_, 1);

    leader_state_sub_ = nh_.subscribe<nav_msgs::Odometry>(
        leader_state_topic_, 20,
        [this](const nav_msgs::OdometryConstPtr &msg)
        {
          std::lock_guard<std::mutex> lk(mutex_);
          leader_state_received_ = true;
          leader_stamp_ = msg->header.stamp;
          leader_pos_.x = msg->pose.pose.position.x;
          leader_pos_.y = msg->pose.pose.position.y;
          leader_pos_.z = msg->pose.pose.position.z;
        });

    leader_traj_sub_ = nh_.subscribe<ego_planner::Bezier>(
        leader_traj_topic_, 10,
        [this](const ego_planner::BezierConstPtr &msg)
        {
          std::lock_guard<std::mutex> lk(mutex_);
          leader_traj_ = *msg;
          have_leader_traj_ = true;
        });

    for (const int id : agent_ids_)
    {
      const std::string st_topic = formatTopic(state_topic_template_, id);
      const std::string gd_topic = formatTopic(guidance_topic_template_, id);

      state_subs_.push_back(nh_.subscribe<ego_planner::SwarmAgentState>(
          st_topic, 20,
          [this, id](const ego_planner::SwarmAgentStateConstPtr &msg)
          {
            std::lock_guard<std::mutex> lk(mutex_);
            AgentStateCache &cache = states_[id];
            cache.received = true;
            cache.valid = msg->is_valid;
            cache.stamp = msg->header.stamp;
            cache.pos = msg->position;
          }));

      guidance_subs_.push_back(nh_.subscribe<ego_planner::Bezier>(
          gd_topic, 10,
          [this, id](const ego_planner::BezierConstPtr &msg)
          {
            std::lock_guard<std::mutex> lk(mutex_);
            guidance_[id] = *msg;
          }));

      ROS_INFO("[PayloadViz] agent=%d state=%s guidance=%s", id, st_topic.c_str(), gd_topic.c_str());
    }

    timer_ = nh_.createTimer(ros::Duration(1.0 / std::max(1.0, publish_rate_)),
                             &SwarmPayloadVisualizer::onTimer, this);

    ROS_INFO("[PayloadViz] started. marker_topic=%s", marker_topic_.c_str());
  }

private:
  struct AgentStateCache
  {
    bool received{false};
    bool valid{false};
    ros::Time stamp;
    geometry_msgs::Point pos;
  };

  bool getActualPayload(payload_geometry::PayloadSolution &payload,
                        geometry_msgs::Point &leader,
                        geometry_msgs::Point &f1,
                        geometry_msgs::Point &f2,
                        bool *geometry_invalid) const
  {
    if (geometry_invalid != nullptr)
      *geometry_invalid = false;

    if (!leader_state_received_)
      return false;

    const int id1 = agent_ids_[0];
    const int id2 = agent_ids_[1];
    auto it1 = states_.find(id1);
    auto it2 = states_.find(id2);
    if (it1 == states_.end() || it2 == states_.end())
      return false;

    const ros::Time now = ros::Time::now();
    const bool s1_ok = it1->second.received && it1->second.valid && (now - it1->second.stamp).toSec() <= state_timeout_;
    const bool s2_ok = it2->second.received && it2->second.valid && (now - it2->second.stamp).toSec() <= state_timeout_;
    if (!s1_ok || !s2_ok)
      return false;

    leader = leader_pos_;
    f1 = it1->second.pos;
    f2 = it2->second.pos;

    payload = payload_geometry::solvePayloadCenterLowerBranch(
        Eigen::Vector3d(leader.x, leader.y, leader.z),
        Eigen::Vector3d(f1.x, f1.y, f1.z),
        Eigen::Vector3d(f2.x, f2.y, f2.z),
        rope_length_);

    if (!payload.valid && geometry_invalid != nullptr)
      *geometry_invalid = true;

    return payload.valid;
  }

  bool getPlannedPayloadNow(payload_geometry::PayloadSolution &payload,
                            geometry_msgs::Point &center,
                            std::vector<geometry_msgs::Point> *traj_points,
                            std::vector<geometry_msgs::Point> *invalid_points,
                            bool *current_invalid,
                            int *invalid_samples) const
  {
    if (current_invalid != nullptr)
      *current_invalid = false;
    if (invalid_samples != nullptr)
      *invalid_samples = 0;

    if (!have_leader_traj_)
      return false;

    const int id1 = agent_ids_[0];
    const int id2 = agent_ids_[1];
    auto it1 = guidance_.find(id1);
    auto it2 = guidance_.find(id2);
    if (it1 == guidance_.end() || it2 == guidance_.end())
      return false;

    const ego_planner::Bezier &g1 = it1->second;
    const ego_planner::Bezier &g2 = it2->second;

    geometry_msgs::Point p0;
    geometry_msgs::Point p1;
    geometry_msgs::Point p2;

    const ros::Time now = ros::Time::now();
    const double t_now = (now - leader_traj_.start_time).toSec();
    if (!evalBezier(leader_traj_, t_now, p0) || !evalBezier(g1, t_now, p1) || !evalBezier(g2, t_now, p2))
      return false;

    payload = payload_geometry::solvePayloadCenterLowerBranch(
        Eigen::Vector3d(p0.x, p0.y, p0.z),
        Eigen::Vector3d(p1.x, p1.y, p1.z),
        Eigen::Vector3d(p2.x, p2.y, p2.z),
        rope_length_);

    if (!payload.valid)
    {
      if (current_invalid != nullptr)
        *current_invalid = true;
    }
    else
    {
      center.x = payload.selected_center.x();
      center.y = payload.selected_center.y();
      center.z = payload.selected_center.z();
    }

    if (traj_points != nullptr || invalid_points != nullptr)
    {
      if (traj_points != nullptr)
        traj_points->clear();
      if (invalid_points != nullptr)
        invalid_points->clear();

      int invalid_count_local = 0;
      const double t_begin = std::max(0.0, t_now);
      const double t_end = std::min(bezierDuration(leader_traj_), t_begin + plan_horizon_);
      for (double t = t_begin; t <= t_end + 0.5 * sample_dt_; t += sample_dt_)
      {
        geometry_msgs::Point q0;
        geometry_msgs::Point q1;
        geometry_msgs::Point q2;
        if (!evalBezier(leader_traj_, t, q0) || !evalBezier(g1, t, q1) || !evalBezier(g2, t, q2))
          continue;

        const payload_geometry::PayloadSolution q_payload = payload_geometry::solvePayloadCenterLowerBranch(
            Eigen::Vector3d(q0.x, q0.y, q0.z),
            Eigen::Vector3d(q1.x, q1.y, q1.z),
            Eigen::Vector3d(q2.x, q2.y, q2.z),
            rope_length_);
        if (!q_payload.valid)
        {
          ++invalid_count_local;
          if (invalid_points != nullptr)
            invalid_points->push_back(q0);
          continue;
        }

        geometry_msgs::Point p;
        p.x = q_payload.selected_center.x();
        p.y = q_payload.selected_center.y();
        p.z = q_payload.selected_center.z();
        if (traj_points != nullptr)
          traj_points->push_back(p);
      }

      if (invalid_samples != nullptr)
        *invalid_samples = invalid_count_local;
    }

    return payload.valid;
  }

  void onTimer(const ros::TimerEvent &)
  {
    if (marker_pub_.getNumSubscribers() == 0)
      return;

    visualization_msgs::MarkerArray out;
    visualization_msgs::Marker clr;
    clr.header.frame_id = "world";
    clr.header.stamp = ros::Time::now();
    clr.action = visualization_msgs::Marker::DELETEALL;
    out.markers.push_back(clr);

    std::lock_guard<std::mutex> lk(mutex_);

    payload_geometry::PayloadSolution actual_payload;
    geometry_msgs::Point leader;
    geometry_msgs::Point f1;
    geometry_msgs::Point f2;
    bool actual_geometry_invalid = false;
    const bool have_actual = getActualPayload(actual_payload, leader, f1, f2, &actual_geometry_invalid);

    geometry_msgs::Point planned_center;
    payload_geometry::PayloadSolution planned_payload;
    std::vector<geometry_msgs::Point> planned_traj;
    std::vector<geometry_msgs::Point> planned_invalid_pts;
    bool planned_current_invalid = false;
    int planned_invalid_samples = 0;
    const bool have_plan = getPlannedPayloadNow(planned_payload, planned_center, &planned_traj,
                                                &planned_invalid_pts, &planned_current_invalid,
                                                &planned_invalid_samples);

    if (have_actual)
    {
      geometry_msgs::Point center;
      center.x = actual_payload.selected_center.x();
      center.y = actual_payload.selected_center.y();
      center.z = actual_payload.selected_center.z();

      out.markers.push_back(makeSphere(1, "payload_actual_center", center, actual_center_size_,
                                       static_cast<float>(actual_center_color_r_),
                                       static_cast<float>(actual_center_color_g_),
                                       static_cast<float>(actual_center_color_b_),
                                       static_cast<float>(actual_center_color_a_)));
      out.markers.push_back(makeSphere(2, "payload_actual_safety", center,
                                       2.0 * (payload_radius_ + payload_extra_margin_),
                                       static_cast<float>(actual_safety_color_r_),
                                       static_cast<float>(actual_safety_color_g_),
                                       static_cast<float>(actual_safety_color_b_),
                                       static_cast<float>(actual_safety_color_a_)));

      visualization_msgs::Marker rope_mk = makeLineList(1, "payload_actual_rope", actual_rope_width_,
                                                        static_cast<float>(rope_color_r_),
                                                        static_cast<float>(rope_color_g_),
                                                        static_cast<float>(rope_color_b_),
                                                        static_cast<float>(rope_color_a_));
      rope_mk.points.push_back(leader);
      rope_mk.points.push_back(center);
      rope_mk.points.push_back(f1);
      rope_mk.points.push_back(center);
      rope_mk.points.push_back(f2);
      rope_mk.points.push_back(center);
      out.markers.push_back(rope_mk);

      std::ostringstream ss;
      ss << std::fixed << std::setprecision(2)
         << "Payload(actual): R=" << actual_payload.circumradius
         << " L=" << rope_length_
         << " A=" << actual_payload.area;
      geometry_msgs::Point txt_p = center;
      txt_p.z += 0.45;
      out.markers.push_back(makeText(1, "payload_actual_text", txt_p, ss.str(),
                                     static_cast<float>(rope_color_r_),
                                     static_cast<float>(rope_color_g_),
                                     static_cast<float>(rope_color_b_), warning_text_size_));
    }
    else if (actual_geometry_invalid)
    {
      geometry_msgs::Point warn_p = leader_pos_;
      warn_p.z += 0.8;
      out.markers.push_back(makeText(10, "payload_actual_warning", warn_p,
                                     "Payload(actual) INVALID geometry",
                                     static_cast<float>(invalid_color_r_),
                                     static_cast<float>(invalid_color_g_),
                                     static_cast<float>(invalid_color_b_), warning_text_size_));
    }

    if (have_plan)
    {
      if (!planned_traj.empty())
      {
        visualization_msgs::Marker traj_mk = makeLineStrip(1, "payload_plan_traj", planned_traj_width_,
                                                           static_cast<float>(planned_traj_color_r_),
                                                           static_cast<float>(planned_traj_color_g_),
                                                           static_cast<float>(planned_traj_color_b_),
                                                           static_cast<float>(planned_traj_color_a_));
        traj_mk.points = planned_traj;
        out.markers.push_back(traj_mk);
      }

      out.markers.push_back(makeSphere(1, "payload_plan_now", planned_center, planned_center_size_,
                                       static_cast<float>(planned_center_color_r_),
                                       static_cast<float>(planned_center_color_g_),
                                       static_cast<float>(planned_center_color_b_),
                                       static_cast<float>(planned_center_color_a_)));

      std::ostringstream ss;
      ss << std::fixed << std::setprecision(2)
         << "Payload(plan): R=" << planned_payload.circumradius
         << " L=" << rope_length_
         << " A=" << planned_payload.area;
      geometry_msgs::Point txt_p = planned_center;
      txt_p.z += 0.38;
      out.markers.push_back(makeText(1, "payload_plan_text", txt_p, ss.str(),
                                     static_cast<float>(planned_center_color_r_),
                                     static_cast<float>(planned_center_color_g_),
                                     static_cast<float>(planned_center_color_b_), warning_text_size_));
    }

    if (planned_invalid_samples > 0)
    {
      if (!planned_invalid_pts.empty())
      {
        visualization_msgs::Marker invalid_mk = makeSphereList(
            1, "payload_plan_invalid_samples", invalid_sample_size_,
            static_cast<float>(invalid_color_r_),
            static_cast<float>(invalid_color_g_),
            static_cast<float>(invalid_color_b_),
            static_cast<float>(invalid_color_a_));
        invalid_mk.points = planned_invalid_pts;
        out.markers.push_back(invalid_mk);
      }

      geometry_msgs::Point warn_p = have_plan ? planned_center : leader_pos_;
      warn_p.z += 0.55;
      std::ostringstream warn_ss;
      warn_ss << "Payload(plan) invalid samples: " << planned_invalid_samples;
      out.markers.push_back(makeText(2, "payload_plan_warning", warn_p, warn_ss.str(),
                                     static_cast<float>(invalid_color_r_),
                                     static_cast<float>(invalid_color_g_),
                                     static_cast<float>(invalid_color_b_), warning_text_size_));
    }

    if (!have_plan && planned_current_invalid)
    {
      geometry_msgs::Point warn_p = leader_pos_;
      warn_p.z += 1.1;
      out.markers.push_back(makeText(3, "payload_plan_now_invalid", warn_p,
                                     "Payload(plan now) INVALID geometry",
                                     static_cast<float>(invalid_color_r_),
                                     static_cast<float>(invalid_color_g_),
                                     static_cast<float>(invalid_color_b_), warning_text_size_));
    }

    if (have_actual && have_plan)
    {
      geometry_msgs::Point actual_center;
      actual_center.x = actual_payload.selected_center.x();
      actual_center.y = actual_payload.selected_center.y();
      actual_center.z = actual_payload.selected_center.z();

      visualization_msgs::Marker err_line = makeLineList(1, "payload_error_line", 0.025,
                                                         1.0f, 0.25f, 0.25f, 0.92f);
      err_line.points.push_back(planned_center);
      err_line.points.push_back(actual_center);
      out.markers.push_back(err_line);

      const double err = std::sqrt(
          (planned_center.x - actual_center.x) * (planned_center.x - actual_center.x) +
          (planned_center.y - actual_center.y) * (planned_center.y - actual_center.y) +
          (planned_center.z - actual_center.z) * (planned_center.z - actual_center.z));
      out.markers.push_back(makeSphere(1, "payload_error_envelope", planned_center,
                                       2.0 * std::max(0.05, err),
                                       1.0f, 0.2f, 0.2f, 0.15f));
    }

    marker_pub_.publish(out);
  }

private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  ros::Publisher marker_pub_;
  ros::Subscriber leader_state_sub_;
  ros::Subscriber leader_traj_sub_;
  std::vector<ros::Subscriber> state_subs_;
  std::vector<ros::Subscriber> guidance_subs_;
  ros::Timer timer_;

  std::string marker_topic_;
  std::string leader_state_topic_;
  std::string leader_traj_topic_;
  std::string state_topic_template_;
  std::string guidance_topic_template_;

  std::vector<int> agent_ids_;

  double publish_rate_{20.0};
  double rope_length_{1.0};
  double payload_radius_{0.2};
  double payload_extra_margin_{0.05};
  double state_timeout_{0.3};
  double sample_dt_{0.05};
  double plan_horizon_{3.0};
  double actual_center_size_{0.20};
  double planned_center_size_{0.18};
  double actual_rope_width_{0.03};
  double planned_traj_width_{0.035};
  double invalid_sample_size_{0.10};
  double warning_text_size_{0.24};

  double actual_center_color_r_{1.0};
  double actual_center_color_g_{0.45};
  double actual_center_color_b_{0.10};
  double actual_center_color_a_{0.95};

  double actual_safety_color_r_{1.0};
  double actual_safety_color_g_{0.55};
  double actual_safety_color_b_{0.15};
  double actual_safety_color_a_{0.22};

  double planned_traj_color_r_{0.2};
  double planned_traj_color_g_{0.95};
  double planned_traj_color_b_{0.95};
  double planned_traj_color_a_{0.95};

  double planned_center_color_r_{0.1};
  double planned_center_color_g_{0.95};
  double planned_center_color_b_{0.95};
  double planned_center_color_a_{0.95};

  double rope_color_r_{0.95};
  double rope_color_g_{0.65};
  double rope_color_b_{0.25};
  double rope_color_a_{0.90};

  double invalid_color_r_{1.0};
  double invalid_color_g_{0.12};
  double invalid_color_b_{0.12};
  double invalid_color_a_{0.92};

  mutable std::mutex mutex_;

  bool leader_state_received_{false};
  ros::Time leader_stamp_;
  geometry_msgs::Point leader_pos_;

  bool have_leader_traj_{false};
  ego_planner::Bezier leader_traj_;

  std::map<int, AgentStateCache> states_;
  std::map<int, ego_planner::Bezier> guidance_;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "swarm_payload_visualizer");
  SwarmPayloadVisualizer node;
  ros::spin();
  return 0;
}
