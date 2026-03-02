#include <ego_planner/Bezier.h>
#include <ego_planner/SwarmAgentState.h>
#include <boost/bind.hpp>
#include <ros/master.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <std_srvs/Trigger.h>
#include <xmlrpcpp/XmlRpcValue.h>

#include <string>
#include <unordered_map>
#include <vector>

class SwarmPreflightCheck
{
public:
  void init(ros::NodeHandle &nh)
  {
    nh_ = nh;

    nh_.param("check_rate", check_rate_, 2.0);
    nh_.param("state_timeout_sec", state_timeout_sec_, 0.2);
    nh_.param("max_topic_silence_sec", max_topic_silence_sec_, 0.5);
    nh_.param("min_start_time_margin_sec", min_start_time_margin_sec_, 0.05);
    nh_.param("max_stamp_delay_sec", max_stamp_delay_sec_, 0.2);
    nh_.param("max_warn_for_ok", max_warn_for_ok_, 0);

    loadStringListParam("expected_topics", expected_topics_);
    loadStringListParam("state_topics", state_topics_);
    loadStringListParam("bezier_topics", bezier_topics_);

    for (const auto &tp : state_topics_)
    {
      state_subs_.push_back(
          nh_.subscribe<ego_planner::SwarmAgentState>(tp, 20,
                                                      boost::bind(&SwarmPreflightCheck::stateCb, this, _1, tp)));
      state_status_[tp] = Status();
    }

    for (const auto &tp : bezier_topics_)
    {
      bezier_subs_.push_back(
          nh_.subscribe<ego_planner::Bezier>(tp, 20,
                                             boost::bind(&SwarmPreflightCheck::bezierCb, this, _1, tp)));
      bezier_status_[tp] = Status();
    }

    summary_pub_ = nh_.advertise<std_msgs::String>("/swarm/preflight/summary", 5, true);
    ok_pub_ = nh_.advertise<std_msgs::Bool>("/swarm/preflight/ok", 1, true);
    can_arm_srv_ = nh_.advertiseService("/swarm/preflight/can_arm", &SwarmPreflightCheck::canArmCb, this);
    timer_ = nh_.createTimer(ros::Duration(1.0 / std::max(1.0, check_rate_)),
                             &SwarmPreflightCheck::timerCb, this);

    ROS_INFO("[Preflight] initialized. state_topics=%zu bezier_topics=%zu expected_topics=%zu",
             state_topics_.size(), bezier_topics_.size(), expected_topics_.size());
  }

private:
  struct Status
  {
    bool ever_received{false};
    ros::Time last_recv;
    double last_stamp_delay{0.0};
    double last_start_margin{0.0};
    bool last_state_valid{true};
  };

  void loadStringListParam(const std::string &key, std::vector<std::string> &out)
  {
    out.clear();
    XmlRpc::XmlRpcValue v;
    if (!nh_.getParam(key, v) || v.getType() != XmlRpc::XmlRpcValue::TypeArray)
      return;

    for (int i = 0; i < v.size(); ++i)
    {
      if (v[i].getType() == XmlRpc::XmlRpcValue::TypeString)
      {
        out.push_back(static_cast<std::string>(v[i]));
      }
    }
  }

  void stateCb(const ego_planner::SwarmAgentStateConstPtr &msg, const std::string &topic)
  {
    auto &s = state_status_[topic];
    s.ever_received = true;
    s.last_recv = ros::Time::now();
    s.last_stamp_delay = (s.last_recv - msg->header.stamp).toSec();
    s.last_state_valid = msg->is_valid;
  }

  void bezierCb(const ego_planner::BezierConstPtr &msg, const std::string &topic)
  {
    auto &s = bezier_status_[topic];
    s.ever_received = true;
    s.last_recv = ros::Time::now();
    s.last_start_margin = (msg->start_time - s.last_recv).toSec();
  }

  bool checkTopicAdvertised(const std::string &topic) const
  {
    ros::master::V_TopicInfo infos;
    ros::master::getTopics(infos);
    for (const auto &it : infos)
    {
      if (it.name == topic)
        return true;
    }
    return false;
  }

  bool canArmCb(std_srvs::Trigger::Request &, std_srvs::Trigger::Response &res)
  {
    res.success = preflight_ok_;
    res.message = preflight_ok_ ? "PRECHECK_OK: can arm/takeoff" : "PRECHECK_FAIL: arm/takeoff blocked";
    return true;
  }

  void timerCb(const ros::TimerEvent &)
  {
    const ros::Time now = ros::Time::now();
    int err_cnt = 0;
    int warn_cnt = 0;

    for (const auto &tp : expected_topics_)
    {
      if (!checkTopicAdvertised(tp))
      {
        ROS_ERROR_THROTTLE(1.0, "[Preflight] Missing advertised topic: %s", tp.c_str());
        ++err_cnt;
      }
    }

    for (const auto &kv : state_status_)
    {
      const auto &tp = kv.first;
      const auto &s = kv.second;

      if (!s.ever_received)
      {
        ROS_WARN_THROTTLE(1.0, "[Preflight] No state message yet: %s", tp.c_str());
        ++warn_cnt;
        continue;
      }

      const double silence = (now - s.last_recv).toSec();
      if (silence > max_topic_silence_sec_)
      {
        ROS_ERROR_THROTTLE(1.0, "[Preflight] State topic timeout: %s silence=%.3f", tp.c_str(), silence);
        ++err_cnt;
      }

      if (s.last_stamp_delay > max_stamp_delay_sec_)
      {
        ROS_WARN_THROTTLE(1.0, "[Preflight] State stamp delay too high: %s delay=%.3f", tp.c_str(), s.last_stamp_delay);
        ++warn_cnt;
      }

      if (!s.last_state_valid)
      {
        ROS_ERROR_THROTTLE(1.0, "[Preflight] State reports invalid: %s", tp.c_str());
        ++err_cnt;
      }
    }

    for (const auto &kv : bezier_status_)
    {
      const auto &tp = kv.first;
      const auto &s = kv.second;

      if (!s.ever_received)
      {
        ROS_WARN_THROTTLE(1.0, "[Preflight] No bezier message yet: %s", tp.c_str());
        ++warn_cnt;
        continue;
      }

      const double silence = (now - s.last_recv).toSec();
      if (silence > max_topic_silence_sec_)
      {
        ROS_ERROR_THROTTLE(1.0, "[Preflight] Bezier topic timeout: %s silence=%.3f", tp.c_str(), silence);
        ++err_cnt;
      }

      if (s.last_start_margin < min_start_time_margin_sec_)
      {
        ROS_WARN_THROTTLE(1.0, "[Preflight] start_time margin low: %s margin=%.3f", tp.c_str(), s.last_start_margin);
        ++warn_cnt;
      }
    }

    preflight_ok_ = (err_cnt == 0 && warn_cnt <= max_warn_for_ok_);

    std_msgs::String summary;
    std_msgs::Bool ok_msg;
    ok_msg.data = preflight_ok_;

    if (preflight_ok_)
    {
      summary.data = "PRECHECK_OK";
      ROS_INFO_THROTTLE(1.0, "[Preflight] OK");
    }
    else
    {
      summary.data = "PRECHECK_WARN_ERR: err=" + std::to_string(err_cnt) + " warn=" + std::to_string(warn_cnt);
      ROS_WARN_THROTTLE(1.0, "[Preflight] %s", summary.data.c_str());
    }

    summary_pub_.publish(summary);
    ok_pub_.publish(ok_msg);
  }

private:
  ros::NodeHandle nh_;
  double check_rate_{2.0};
  double state_timeout_sec_{0.2};
  double max_topic_silence_sec_{0.5};
  double min_start_time_margin_sec_{0.05};
  double max_stamp_delay_sec_{0.2};

  std::vector<std::string> expected_topics_;
  std::vector<std::string> state_topics_;
  std::vector<std::string> bezier_topics_;

  std::vector<ros::Subscriber> state_subs_;
  std::vector<ros::Subscriber> bezier_subs_;
  std::unordered_map<std::string, Status> state_status_;
  std::unordered_map<std::string, Status> bezier_status_;

  bool preflight_ok_{false};
  int max_warn_for_ok_{0};

  ros::Publisher summary_pub_;
  ros::Publisher ok_pub_;
  ros::ServiceServer can_arm_srv_;
  ros::Timer timer_;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "swarm_preflight_check_node");
  ros::NodeHandle nh("~");

  SwarmPreflightCheck checker;
  checker.init(nh);

  ros::spin();
  return 0;
}
