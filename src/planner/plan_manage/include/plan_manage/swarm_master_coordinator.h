#ifndef _SWARM_MASTER_COORDINATOR_H_
#define _SWARM_MASTER_COORDINATOR_H_

#include <ros/ros.h>

#include <Eigen/Eigen>
#include <ego_planner/Bezier.h>
#include <ego_planner/SwarmAgentState.h>
#include <string>
#include <unordered_map>
#include <vector>

namespace ego_planner
{

class SwarmMasterCoordinator
{
public:
  SwarmMasterCoordinator() = default;
  ~SwarmMasterCoordinator() = default;

  void init(ros::NodeHandle &nh);
  void runOnce();

private:
  struct AgentState
  {
    Eigen::Vector3d pos = Eigen::Vector3d::Zero();
    Eigen::Vector3d vel = Eigen::Vector3d::Zero();
    ros::Time stamp;
    bool valid = false;
  };

  void leaderBezierCallback(const ego_planner::BezierConstPtr &msg);
  void agentStateCallback(const ego_planner::SwarmAgentStateConstPtr &msg, int agent_id);
  void planTimerCallback(const ros::TimerEvent &e);

  bool checkReady() const;
  Eigen::MatrixXd leaderBezierToCtrlPts(const ego_planner::Bezier &msg) const;

  void generateFormationGuidance(const Eigen::MatrixXd &leader_ctrl_pts,
                                 std::unordered_map<int, Eigen::MatrixXd> &agent_ctrl_pts_map) const;
  void applySwarmCollisionPenalty(std::unordered_map<int, Eigen::MatrixXd> &agent_ctrl_pts_map) const;

  ego_planner::Bezier buildAgentBezierMsg(int agent_id, const Eigen::MatrixXd &ctrl_pts,
                                          const ros::Time &start_time, int64_t traj_id) const;

/** @brief 对整条主机 Bezier 轨迹计算每个控制点处的平滑航向向量。
   *  1) 用曲线真实切线 B'(t) 替代控制多边形差分
   *  2) 对切线序列做滑动平均消除段间接缝抖动
   *  返回 (3 x N) 矩阵，每列为归一化的 XY 平面航向 */
  Eigen::MatrixXd computeSmoothedHeadings(const Eigen::MatrixXd &ctrl_pts) const;
  std::string formatTopic(const std::string &topic_template, int agent_id) const;

private:
  ros::NodeHandle nh_;

  ros::Subscriber leader_bezier_sub_;
  std::vector<ros::Subscriber> agent_state_subs_;
  std::unordered_map<int, ros::Publisher> guidance_pubs_;
  ros::Timer plan_timer_;

  std::vector<int> agent_ids_;
  std::unordered_map<int, AgentState> agent_states_;

  ego_planner::Bezier latest_leader_bezier_;
  bool have_leader_traj_{false};
  int64_t out_traj_id_{0};
  int64_t last_published_leader_traj_id_{-1};
  ros::Time last_published_leader_start_time_;

  std::string leader_bezier_topic_;
  std::string state_topic_template_;
  std::string guidance_topic_template_;

  std::string formation_type_;
  double formation_spacing_{1.6};
  double formation_angle_deg_{35.0};
  double formation_z_offset_{0.0};

  double plan_rate_{20.0};
  double safe_radius_{1.2};
  double collision_weight_{80.0};
  double collision_gain_cap_{200.0};   // Layer-C: 二次惩罚增益上界
  double penalty_step_{0.005};
  int penalty_iters_{3};

  double start_time_offset_{0.0};
  double state_timeout_{0.2};
  // Bug-1 修复：guidance C0 跳变距离混合阈值 (m)
  // 当 ideal_P0 与从机实际位置偏差超过此值时，P1 完全按当前速度方向修正
  double guidance_c0_blend_dist_{0.3};
};

} // namespace ego_planner

#endif
