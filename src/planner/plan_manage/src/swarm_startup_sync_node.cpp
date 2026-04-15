#include <ego_planner/Bezier.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>

#include <boost/bind.hpp>

#include <algorithm>
#include <string>
#include <vector>

class SwarmStartupSyncNode
{
public:
  void init(ros::NodeHandle &nh)
  {
    nh_ = nh;

    nh_.param("startup_sync_enabled", startup_sync_enabled_, false);
    nh_.param("leader_first_traj_topic", leader_first_traj_topic_,
              std::string("/swarm/leader/corrected_bezier"));
    nh_.param("release_topic", release_topic_, std::string("/swarm/startup_sync/release"));
    nh_.param("ready_timeout_sec", ready_timeout_sec_, 2.0);
    nh_.param("ready_timeout_safety_margin_sec", ready_timeout_safety_margin_sec_, 0.0);
    nh_.param("timeout_action", timeout_action_, std::string("release"));
    nh_.param("watchdog_rate_hz", watchdog_rate_hz_, 20.0);

    ready_timeout_sec_ = std::max(0.0, ready_timeout_sec_);
    ready_timeout_safety_margin_sec_ = std::max(0.0, ready_timeout_safety_margin_sec_);
    watchdog_rate_hz_ = std::max(1.0, watchdog_rate_hz_);

    if (timeout_action_ != "release" && timeout_action_ != "abort")
    {
      ROS_WARN("[StartupSync] invalid timeout_action=%s, fallback to release.", timeout_action_.c_str());
      timeout_action_ = "release";
    }

    if (!nh_.getParam("follower_ready_topics", follower_ready_topics_))
    {
      follower_ready_topics_.clear();
    }
    follower_ready_.assign(follower_ready_topics_.size(), false);

    release_pub_ = nh_.advertise<std_msgs::Bool>(release_topic_, 1, true);

    if (!startup_sync_enabled_)
    {
      released_ = true;
      publishRelease(true, "disabled-bypass");
      ROS_INFO("[StartupSync] disabled, publish release=true directly.");
      return;
    }

    publishRelease(false, "init-blocked");

    leader_traj_sub_ = nh_.subscribe(leader_first_traj_topic_, 10,
                                     &SwarmStartupSyncNode::leaderFirstTrajCb, this);

    follower_ready_subs_.reserve(follower_ready_topics_.size());
    for (size_t i = 0; i < follower_ready_topics_.size(); ++i)
    {
      follower_ready_subs_.push_back(
          nh_.subscribe<std_msgs::Bool>(follower_ready_topics_[i], 10,
                                        boost::bind(&SwarmStartupSyncNode::followerReadyCb, this, _1, i)));
    }

    watchdog_timer_ = nh_.createTimer(ros::Duration(1.0 / watchdog_rate_hz_),
                                      &SwarmStartupSyncNode::watchdogCb, this);

    ROS_INFO("[StartupSync] initialized: enabled=%d leader_first_traj_topic=%s release_topic=%s followers=%zu timeout=%.2f+%.2f action=%s",
             static_cast<int>(startup_sync_enabled_),
             leader_first_traj_topic_.c_str(),
             release_topic_.c_str(),
             follower_ready_topics_.size(),
             ready_timeout_sec_,
             ready_timeout_safety_margin_sec_,
             timeout_action_.c_str());
  }

private:
  bool allFollowersReady() const
  {
    for (size_t i = 0; i < follower_ready_.size(); ++i)
    {
      if (!follower_ready_[i])
      {
        return false;
      }
    }
    return true;
  }

  int countReadyFollowers() const
  {
    int cnt = 0;
    for (size_t i = 0; i < follower_ready_.size(); ++i)
    {
      if (follower_ready_[i])
      {
        ++cnt;
      }
    }
    return cnt;
  }

  void publishRelease(const bool released, const std::string &reason) const
  {
    std_msgs::Bool msg;
    msg.data = released;
    release_pub_.publish(msg);

    ROS_INFO("[StartupSync] release=%d reason=%s", static_cast<int>(released), reason.c_str());
  }

  void doRelease(const std::string &reason)
  {
    if (released_ || aborted_)
    {
      return;
    }

    released_ = true;
    publishRelease(true, reason);
  }

  void tryReleaseByReady()
  {
    if (!startup_sync_enabled_ || released_ || aborted_)
    {
      return;
    }

    if (!leader_first_traj_seen_)
    {
      return;
    }

    if (allFollowersReady())
    {
      doRelease("all-followers-ready");
    }
  }

  void leaderFirstTrajCb(const ego_planner::BezierConstPtr &msg)
  {
    if (!startup_sync_enabled_ || released_ || aborted_)
    {
      return;
    }

    if (!leader_first_traj_seen_)
    {
      leader_first_traj_seen_ = true;
      leader_first_traj_stamp_ = ros::Time::now();
      const double start_lag = (leader_first_traj_stamp_ - msg->start_time).toSec();
      ROS_INFO("[StartupSync] previewing: leader first trajectory arrived (traj_id=%ld lag=%.3f s), waiting follower ready %d/%zu.",
               static_cast<long>(msg->traj_id),
               start_lag,
               countReadyFollowers(),
               follower_ready_topics_.size());
      if (follower_ready_topics_.empty())
      {
        ROS_WARN("[StartupSync] no follower_ready_topics configured, release on leader-first-trajectory.");
      }
    }

    tryReleaseByReady();
  }

  void followerReadyCb(const std_msgs::BoolConstPtr &msg, const size_t idx)
  {
    if (!startup_sync_enabled_ || released_ || aborted_)
    {
      return;
    }

    if (idx >= follower_ready_.size())
    {
      return;
    }

    const bool prev = follower_ready_[idx];
    follower_ready_[idx] = msg->data;

    if (prev != msg->data)
    {
      ROS_INFO("[StartupSync] follower ready update: topic=%s ready=%d (%d/%zu)",
               follower_ready_topics_[idx].c_str(),
               static_cast<int>(msg->data),
               countReadyFollowers(),
               follower_ready_topics_.size());
    }

    tryReleaseByReady();
  }

  void watchdogCb(const ros::TimerEvent &)
  {
    if (!startup_sync_enabled_ || released_ || aborted_ || !leader_first_traj_seen_)
    {
      return;
    }

    if (allFollowersReady())
    {
      doRelease("all-followers-ready");
      return;
    }

    const double timeout = ready_timeout_sec_ + ready_timeout_safety_margin_sec_;
    if (timeout <= 1e-9)
    {
      return;
    }

    const double elapsed = (ros::Time::now() - leader_first_traj_stamp_).toSec();
    if (elapsed < timeout)
    {
      return;
    }

    if (timeout_action_ == "release")
    {
      ROS_WARN("[StartupSync] timeout reached %.2f s (%d/%zu ready), force release.",
               elapsed,
               countReadyFollowers(),
               follower_ready_topics_.size());
      doRelease("timeout-release");
      return;
    }

    aborted_ = true;
    ROS_ERROR("[StartupSync] timeout reached %.2f s (%d/%zu ready), abort release.",
              elapsed,
              countReadyFollowers(),
              follower_ready_topics_.size());
    publishRelease(false, "timeout-abort");
  }

private:
  ros::NodeHandle nh_;
  ros::Subscriber leader_traj_sub_;
  std::vector<ros::Subscriber> follower_ready_subs_;
  ros::Publisher release_pub_;
  ros::Timer watchdog_timer_;

  bool startup_sync_enabled_{false};
  bool leader_first_traj_seen_{false};
  bool released_{false};
  bool aborted_{false};

  std::string leader_first_traj_topic_{"/swarm/leader/corrected_bezier"};
  std::string release_topic_{"/swarm/startup_sync/release"};
  std::vector<std::string> follower_ready_topics_;

  ros::Time leader_first_traj_stamp_;
  std::vector<bool> follower_ready_;

  double ready_timeout_sec_{2.0};
  double ready_timeout_safety_margin_sec_{0.0};
  std::string timeout_action_{"release"};
  double watchdog_rate_hz_{20.0};
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "swarm_startup_sync_node");
  ros::NodeHandle nh("~");

  SwarmStartupSyncNode node;
  node.init(nh);

  ros::spin();
  return 0;
}
