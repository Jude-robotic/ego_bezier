/**
 * @file traj_server.cpp
 * @brief Trajectory server for executing Bezier curve trajectories
 *
 * 关键修改：实现轨迹切换时的平滑速度过渡，避免速度突变
 * 第六步增强：接入 preflight gate，未通过时阻塞轨迹执行（禁止起飞/运动）
 */

#include "bezier_opt/piecewise_bezier.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Empty.h"
#include "visualization_msgs/Marker.h"
#include <ego_planner/Bezier.h>
#include <nav_msgs/Odometry.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <ros/ros.h>
#include <traj_utils/polynomial_traj.h>

ros::Publisher pos_cmd_pub;

quadrotor_msgs::PositionCommand cmd;
double pos_gain[3] = {0, 0, 0};
double vel_gain[3] = {0, 0, 0};

using ego_planner::PiecewiseBezier;

bool receive_traj_ = false;
vector<PiecewiseBezier> traj_;
double traj_duration_;
ros::Time start_time_;
int traj_id_;

bool require_preflight_ok_ = false;
bool preflight_ok_ = false;
bool have_preflight_status_ = false;
std::string preflight_ok_topic_ = "/swarm/preflight/ok";

Eigen::Vector3d odom_pos_(0.0, 0.0, 0.0);
bool have_odom_ = false;

// 关键新增：用于平滑过渡的变量
Eigen::Vector3d last_cmd_vel_(0, 0, 0);
Eigen::Vector3d last_cmd_acc_(0, 0, 0);
bool traj_completed_ = false;
ros::Time traj_end_time_;
const double VELOCITY_SMOOTHING_TIME = 0.5; // 速度平滑过渡时间(秒)
// Bug-2 修复：新轨迹起点与当前执行点距离超过此值时触发 C0/C1 钳位修正 (m)
const double TRAJ_JUMP_WARN_DIST = 0.30;

void preflightOkCallback(const std_msgs::BoolConstPtr &msg)
{
  preflight_ok_ = msg->data;
  have_preflight_status_ = true;
}

void odomCallback(const nav_msgs::OdometryConstPtr &msg)
{
  odom_pos_(0) = msg->pose.pose.position.x;
  odom_pos_(1) = msg->pose.pose.position.y;
  odom_pos_(2) = msg->pose.pose.position.z;
  have_odom_ = true;
}

void publishHoldCommand(const ros::Time &time_now)
{
  cmd.header.stamp = time_now;
  cmd.header.frame_id = "world";
  cmd.trajectory_flag = quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_COMPLETED;

  if (have_odom_)
  {
    cmd.position.x = odom_pos_(0);
    cmd.position.y = odom_pos_(1);
    cmd.position.z = odom_pos_(2);
  }

  cmd.velocity.x = 0.0;
  cmd.velocity.y = 0.0;
  cmd.velocity.z = 0.0;
  cmd.acceleration.x = 0.0;
  cmd.acceleration.y = 0.0;
  cmd.acceleration.z = 0.0;
  cmd.yaw_dot = 0.0;

  pos_cmd_pub.publish(cmd);
}

/**
 * @brief Bezier trajectory callback - receives and stores the trajectory
 *
 * 关键修改：接收新轨迹时保存当前速度用于平滑过渡
 */
void bezierCallback(ego_planner::BezierConstPtr msg)
{
  if (require_preflight_ok_ && (!have_preflight_status_ || !preflight_ok_))
  {
    ROS_ERROR_THROTTLE(1.0, "[Traj Server] preflight not OK, reject trajectory id=%ld", (long)msg->traj_id);
    return;
  }

  // Reconstruct control points matrix
  Eigen::MatrixXd pos_pts(3, msg->pos_pts.size());
  for (size_t i = 0; i < msg->pos_pts.size(); ++i)
  {
    pos_pts(0, i) = msg->pos_pts[i].x;
    pos_pts(1, i) = msg->pos_pts[i].y;
    pos_pts(2, i) = msg->pos_pts[i].z;
  }

  // === Bug-2 修复（兜底防御）：检测并修正新轨迹起点跳变 ===
  // swarm_master 已在 guidance 生成端做了 C0/C1 保证（Bug-1 修复），
  // 此处作为最后防线：若 optimizer 修改了 P0 或其他场景产生残余跳变，
  // 将新轨迹起点钳位到当前执行位置，阻止脉冲控制指令的产生。
  // 注：主机轨迹由 reboundReplan 自身保证 C0，该逻辑对主机无副作用。
  if (receive_traj_ && pos_pts.cols() >= 2)
  {
    double t_clamp = (ros::Time::now() - start_time_).toSec();
    t_clamp = std::max(0.0, std::min(t_clamp, traj_duration_));
    const Eigen::Vector3d curr_pos = traj_[0].evaluate(t_clamp);
    const Eigen::Vector3d curr_vel = traj_[1].evaluate(t_clamp);
    const double jump = (pos_pts.col(0) - curr_pos).norm();
    if (jump > TRAJ_JUMP_WARN_DIST)
    {
      ROS_WARN("[Traj Server] C0 jump %.3f m (id %d->%ld), pinning start to current pos.",
               jump, traj_id_, static_cast<long>(msg->traj_id));
      // C0：钳位起点到当前执行位置
      pos_pts.col(0) = curr_pos;
      // C1：P1 = P0 + (ts/order)*vel，使初始速度连续
      const double ts_l  = (!msg->segment_durations.empty()) ? msg->segment_durations[0] : 0.1;
      const int    ord_l = (msg->order > 0) ? msg->order : 3;
      pos_pts.col(1) = curr_pos + (ts_l / static_cast<double>(ord_l)) * curr_vel;
    }
  }

  // Calculate segment duration from segment_durations
  double segment_duration = 0.1; // default
  if (!msg->segment_durations.empty())
  {
    segment_duration = msg->segment_durations[0];
  }

  PiecewiseBezier pos_traj(pos_pts, msg->order, segment_duration);

  // Set segment durations if available
  if (!msg->segment_durations.empty())
  {
    Eigen::VectorXd durations(msg->segment_durations.size());
    for (size_t i = 0; i < msg->segment_durations.size(); ++i)
    {
      durations(i) = msg->segment_durations[i];
    }
    pos_traj.setKnot(durations);
  }

  traj_.clear();
  traj_.push_back(pos_traj);
  traj_.push_back(traj_[0].getDerivative());
  traj_.push_back(traj_[1].getDerivative());

  traj_id_ = msg->traj_id;
  traj_duration_ = traj_[0].getTimeSum();

  // 重置轨迹完成标志
  traj_completed_ = false;

  // 首次轨迹（从机起飞）：将 start_time 修正为当前时刻
  // 消除主机时间戳延迟导致的 t_cur 初始非零，确保从 P0（已设为从机当前位置）出发
  if (!receive_traj_)
  {
    const double lag = (ros::Time::now() - msg->start_time).toSec();
    ROS_INFO("[Traj Server] First trajectory: overriding start_time to now "
             "(lag=%.3f s, traj_id=%d)", lag, traj_id_);
    start_time_ = ros::Time::now();
  }
  else
  {
    start_time_ = msg->start_time;
  }

  receive_traj_ = true;

  ROS_DEBUG("[Traj Server] New trajectory received, id=%d, duration=%.2f", traj_id_, traj_duration_);
}

/**
 * @brief Calculate yaw angle based on velocity direction
 */
std::pair<double, double> calculate_yaw(double t_cur, Eigen::Vector3d &pos,
                                        ros::Time &time_now, ros::Time &time_last)
{
  constexpr double YAW_DOT_MAX_PER_SEC = 2 * M_PI;
  std::pair<double, double> yaw_yawdot(0, 0);

  Eigen::Vector3d dir = t_cur + 0.3 < traj_duration_ ? traj_[0].evaluate(t_cur + 0.3) - pos
                                                      : traj_[0].evaluate(traj_duration_) - pos;
  double yaw_temp = dir.norm() > 0.1 ? atan2(dir(1), dir(0)) : cmd.yaw;

  double d_yaw = yaw_temp - cmd.yaw;
  if (d_yaw >= M_PI)
  {
    d_yaw -= 2 * M_PI;
  }
  if (d_yaw <= -M_PI)
  {
    d_yaw += 2 * M_PI;
  }

  const double dt_yaw = (time_now - time_last).toSec();
  double yawdot = 0;
  if (dt_yaw > 0.0)
  {
    yawdot = d_yaw / dt_yaw;
  }

  double yawdot_max = YAW_DOT_MAX_PER_SEC;
  if (yawdot > yawdot_max)
    yawdot = yawdot_max;
  if (yawdot < -yawdot_max)
    yawdot = -yawdot_max;

  yaw_yawdot.first = cmd.yaw + yawdot * dt_yaw;
  yaw_yawdot.second = yawdot;

  return yaw_yawdot;
}

/**
 * @brief Publish command based on Bezier curve evaluation
 *
 * 关键修改：轨迹结束时使用平滑过渡而非突变到零速度
 */
void cmdCallback(const ros::TimerEvent &e)
{
  ros::Time time_now = ros::Time::now();

  if (require_preflight_ok_ && (!have_preflight_status_ || !preflight_ok_))
  {
    receive_traj_ = false;
    publishHoldCommand(time_now);
    return;
  }

  if (!receive_traj_)
  {
    // Keep the vehicle stationary before the first trajectory arrives.
    publishHoldCommand(time_now);
    return;
  }

  double t_cur = (time_now - start_time_).toSec();
  static ros::Time time_last = ros::Time::now();

  if (t_cur < traj_duration_ && t_cur >= 0.0)
  {
    Eigen::Vector3d pos = traj_[0].evaluate(t_cur);
    Eigen::Vector3d vel = traj_[1].evaluate(t_cur);
    Eigen::Vector3d acc = traj_[2].evaluate(t_cur);

    cmd.header.stamp = time_now;
    cmd.header.frame_id = "world";
    cmd.trajectory_flag = quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_READY;
    cmd.trajectory_id = traj_id_;

    cmd.position.x = pos(0);
    cmd.position.y = pos(1);
    cmd.position.z = pos(2);

    cmd.velocity.x = vel(0);
    cmd.velocity.y = vel(1);
    cmd.velocity.z = vel(2);

    cmd.acceleration.x = acc(0);
    cmd.acceleration.y = acc(1);
    cmd.acceleration.z = acc(2);

    // 保存当前速度和加速度用于平滑过渡
    last_cmd_vel_ = vel;
    last_cmd_acc_ = acc;

    auto yaw_yawdot = calculate_yaw(t_cur, pos, time_now, time_last);
    cmd.yaw = yaw_yawdot.first;
    cmd.yaw_dot = yaw_yawdot.second;

    time_last = time_now;
    traj_completed_ = false;

    pos_cmd_pub.publish(cmd);
  }
  else if (t_cur >= traj_duration_)
  {
    // 关键修改：使用轨迹末端的速度进行平滑过渡，而非立即归零
    if (!traj_completed_)
    {
      traj_completed_ = true;
      traj_end_time_ = time_now;
      // 获取轨迹末端的速度（而非强制为零）
      last_cmd_vel_ = traj_[1].evaluate(traj_duration_);
      last_cmd_acc_ = traj_[2].evaluate(traj_duration_);
    }

    // 计算从轨迹结束后经过的时间
    double time_since_end = (time_now - traj_end_time_).toSec();

    cmd.header.stamp = time_now;
    cmd.header.frame_id = "world";
    cmd.trajectory_flag = quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_COMPLETED;
    cmd.trajectory_id = traj_id_;

    Eigen::Vector3d end_pos = traj_[0].evaluate(traj_duration_);

    // 平滑速度过渡：使用指数衰减让速度逐渐趋近于零
    // v(t) = v_end * exp(-t / tau)
    double decay_factor = exp(-time_since_end / VELOCITY_SMOOTHING_TIME);
    Eigen::Vector3d smooth_vel = last_cmd_vel_ * decay_factor;
    Eigen::Vector3d smooth_acc = -last_cmd_vel_ / VELOCITY_SMOOTHING_TIME * decay_factor;

    // 位置也需要相应更新（积分速度）
    // p(t) = p_end + v_end * tau * (1 - exp(-t/tau))
    Eigen::Vector3d pos_offset = last_cmd_vel_ * VELOCITY_SMOOTHING_TIME * (1.0 - decay_factor);
    Eigen::Vector3d smooth_pos = end_pos + pos_offset;

    // 当速度足够小时，停止平滑过渡
    if (smooth_vel.norm() < 0.01)
    {
      smooth_vel.setZero();
      smooth_acc.setZero();
      smooth_pos = end_pos + last_cmd_vel_ * VELOCITY_SMOOTHING_TIME; // 最终位置
    }

    cmd.position.x = smooth_pos(0);
    cmd.position.y = smooth_pos(1);
    cmd.position.z = smooth_pos(2);

    cmd.velocity.x = smooth_vel(0);
    cmd.velocity.y = smooth_vel(1);
    cmd.velocity.z = smooth_vel(2);

    cmd.acceleration.x = smooth_acc(0);
    cmd.acceleration.y = smooth_acc(1);
    cmd.acceleration.z = smooth_acc(2);

    pos_cmd_pub.publish(cmd);
  }
  else
  {
    // t_cur < 0: 轨迹尚未开始（start_time 在未来），发送保持指令防止失控
    publishHoldCommand(time_now);
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "traj_server");
  ros::NodeHandle node;
  ros::NodeHandle nh("~");

  nh.param("require_preflight_ok", require_preflight_ok_, false);
  nh.param("preflight_ok_topic", preflight_ok_topic_, std::string("/swarm/preflight/ok"));

  ros::Subscriber bezier_sub = node.subscribe("planning/bezier", 10, bezierCallback);
  ros::Subscriber odom_sub = node.subscribe("/odom_world", 20, odomCallback);
  ros::Subscriber preflight_sub = node.subscribe(preflight_ok_topic_, 10, preflightOkCallback);

  pos_cmd_pub = node.advertise<quadrotor_msgs::PositionCommand>("/position_cmd", 50);
  ros::Timer cmd_timer = node.createTimer(ros::Duration(0.01), cmdCallback);

  nh.param("gains/p_x", pos_gain[0], 0.0);
  nh.param("gains/p_y", pos_gain[1], 0.0);
  nh.param("gains/p_z", pos_gain[2], 0.0);

  nh.param("gains/v_x", vel_gain[0], 0.0);
  nh.param("gains/v_y", vel_gain[1], 0.0);
  nh.param("gains/v_z", vel_gain[2], 0.0);

  cmd.kx[0] = pos_gain[0];
  cmd.kx[1] = pos_gain[1];
  cmd.kx[2] = pos_gain[2];

  cmd.kv[0] = vel_gain[0];
  cmd.kv[1] = vel_gain[1];
  cmd.kv[2] = vel_gain[2];

  ROS_INFO("[Traj server] Bezier trajectory server started. require_preflight_ok=%d topic=%s",
           static_cast<int>(require_preflight_ok_), preflight_ok_topic_.c_str());

  ros::Duration(1.0).sleep();

  ROS_INFO("[Traj server] Waiting for Bezier trajectory...");

  ros::spin();

  return 0;
}
