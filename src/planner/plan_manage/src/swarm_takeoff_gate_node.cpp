#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_srvs/Trigger.h>

class SwarmTakeoffGate
{
public:
  void init(ros::NodeHandle &nh)
  {
    nh_ = nh;

    nh_.param("require_preflight_ok", require_preflight_ok_, true);
    nh_.param("required_consecutive_ok", required_consecutive_ok_, 3);
    nh_.param("preflight_ok_topic", preflight_ok_topic_, std::string("/swarm/preflight/ok"));
    nh_.param("trigger_in_topic", trigger_in_topic_, std::string("/traj_start_trigger"));
    nh_.param("trigger_out_topic", trigger_out_topic_, std::string("/swarm/traj_start_trigger_gated"));

    preflight_sub_ = nh_.subscribe(preflight_ok_topic_, 10, &SwarmTakeoffGate::preflightOkCb, this);
    trigger_sub_ = nh_.subscribe(trigger_in_topic_, 10, &SwarmTakeoffGate::triggerCb, this);
    trigger_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(trigger_out_topic_, 5);

    can_start_srv_ = nh_.advertiseService("/swarm/takeoff_gate/can_start", &SwarmTakeoffGate::canStartCb, this);

    ROS_INFO("[TakeoffGate] initialized. require_preflight_ok=%d required_consecutive_ok=%d in=%s out=%s",
             static_cast<int>(require_preflight_ok_), required_consecutive_ok_, trigger_in_topic_.c_str(),
             trigger_out_topic_.c_str());
  }

private:
  void preflightOkCb(const std_msgs::BoolConstPtr &msg)
  {
    have_preflight_ok_ = true;
    preflight_ok_ = msg->data;

    if (preflight_ok_)
      ok_streak_++;
    else
      ok_streak_ = 0;
  }

  bool gateOpen() const
  {
    if (!require_preflight_ok_)
      return true;
    return have_preflight_ok_ && preflight_ok_ && ok_streak_ >= required_consecutive_ok_;
  }

  bool canStartCb(std_srvs::Trigger::Request &, std_srvs::Trigger::Response &res)
  {
    const bool open = gateOpen();
    res.success = open;

    if (open)
    {
      res.message = "TAKEOFF_GATE_OPEN";
    }
    else if (!require_preflight_ok_)
    {
      res.message = "TAKEOFF_GATE_BYPASS";
    }
    else if (!have_preflight_ok_)
    {
      res.message = "TAKEOFF_GATE_BLOCKED: no preflight status";
    }
    else if (!preflight_ok_)
    {
      res.message = "TAKEOFF_GATE_BLOCKED: preflight not ok";
    }
    else
    {
      res.message = "TAKEOFF_GATE_BLOCKED: waiting consecutive preflight ok";
    }
    return true;
  }

  void triggerCb(const geometry_msgs::PoseStampedConstPtr &msg)
  {
    if (gateOpen())
    {
      trigger_pub_.publish(*msg);
      return;
    }

    ROS_ERROR_THROTTLE(1.0,
                       "[TakeoffGate] blocked traj_start_trigger. require_preflight_ok=%d have_ok=%d preflight_ok=%d "
                       "ok_streak=%d/%d",
                       static_cast<int>(require_preflight_ok_), static_cast<int>(have_preflight_ok_),
                       static_cast<int>(preflight_ok_), ok_streak_, required_consecutive_ok_);
  }

private:
  ros::NodeHandle nh_;

  bool require_preflight_ok_{true};
  int required_consecutive_ok_{3};

  bool have_preflight_ok_{false};
  bool preflight_ok_{false};
  int ok_streak_{0};

  std::string preflight_ok_topic_;
  std::string trigger_in_topic_;
  std::string trigger_out_topic_;

  ros::Subscriber preflight_sub_;
  ros::Subscriber trigger_sub_;
  ros::Publisher trigger_pub_;
  ros::ServiceServer can_start_srv_;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "swarm_takeoff_gate_node");
  ros::NodeHandle nh("~");

  SwarmTakeoffGate gate;
  gate.init(nh);

  ros::spin();
  return 0;
}
