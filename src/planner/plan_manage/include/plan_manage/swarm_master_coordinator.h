#ifndef _SWARM_MASTER_COORDINATOR_H_
#define _SWARM_MASTER_COORDINATOR_H_

#include <ros/ros.h>

#include <Eigen/Eigen>
#include <ego_planner/Bezier.h>
#include <ego_planner/SwarmAgentState.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
#include <string>
#include <unordered_map>
#include <vector>

#include "plan_env/grid_map.h"

namespace ego_planner
{

class SwarmMasterCoordinator
{
public:
  SwarmMasterCoordinator() = default;
  ~SwarmMasterCoordinator() = default;

  void init(ros::NodeHandle &nh, GridMap::Ptr grid_map = nullptr);
  void runOnce();

private:
  struct AgentState
  {
    Eigen::Vector3d pos = Eigen::Vector3d::Zero();
    Eigen::Vector3d vel = Eigen::Vector3d::Zero();
    ros::Time stamp;
    bool valid = false;
  };

  // ── 障碍物类型枚举 ─────────────────────────────────────────────────────
  enum class ObstacleType {
    NONE = 0,        // 无障碍物，默认水平V形编队
    DOOR_FRAME,      // 门框型：XY窄+Z宽 → 垂直V形编队（已实现）
    RING,            // 圆环型：前方有环洞 → 紧凑人字形编队（中心穿越）
    Z_SLIT           // Z轴窄缝：Z窄+XY宽 → 保持水平V形编队
  };

  // ── 每个控制点的编队指令 ─────────────────────────────────────────────────
  struct FormationCommand {
    ObstacleType type{ObstacleType::NONE};
    double blend{0.0};  // 过渡混合因子 [0,1]：0=完整默认V，1=完整目标编队
    int obstacle_id{-1};
  };

  void leaderBezierCallback(const ego_planner::BezierConstPtr &msg);
  void leaderStateCallback(const nav_msgs::OdometryConstPtr &msg);
  void agentStateCallback(const ego_planner::SwarmAgentStateConstPtr &msg, int agent_id);
  void planTimerCallback(const ros::TimerEvent &e);

  bool checkReady() const;

  /** @brief 检测主机轨迹经过的通道是否为窄通道。
   *
   *  对每个主机控制点，沿航向垂直方向（XY平面左右）以 ray_step 步长射线检测，
   *  找出两侧最近障碍物间距；若间距 < narrow_passage_threshold_，
   *  则标记该控制点 is_narrow[c] = true。
   *
   *  前置条件：grid_map_ 不为空且地图已有有效数据；否则全部置 false 返回 false。
   *
   *  @param leader_ctrl_pts  (3×N) 主机控制点矩阵
   *  @param is_narrow_xy     输出：XY 方向窄通道标记（需要 Z 轴编队）
 *  @param is_narrow_z      输出：Z 方向窄缝标记（强制保持水平编队）
   *  @param ray_step         射线采样步长（m），默认 0.1m
   *  @return true 表示存在至少一个窄通道控制点 */
  bool detectNarrowPassage(const Eigen::MatrixXd& leader_ctrl_pts,
                           std::vector<bool>& is_narrow_xy,
                           std::vector<bool>& is_narrow_z,
                           double ray_step = 0.1);

  /** @brief 对主机轨迹每个控制点进行障碍物类型分类。
   *
   * 检测逻辑（按优先级）：
   * 1. Z_SLIT: Z轴上下受限（< z_narrow_threshold）且 XY 通畅
   * 2. RING:   前方存在环洞特征（周围被障碍物包围，但中心通畅）
   * 3. DOOR_FRAME: XY 两侧受限（< narrow_passage_threshold）且 Z 通畅
   * 4. NONE:   以上均不满足
   *
   * @param leader_ctrl_pts  (3×N) 主机控制点矩阵
   * @param commands          输出：每个控制点的编队指令（类型+混合因子）
   * @param ray_step          射线采样步长（m）
   * @return true 若存在至少一个非 NONE 类型 */
  bool classifyObstacleType(const Eigen::MatrixXd& leader_ctrl_pts,
                            std::vector<FormationCommand>& commands,
                            double ray_step = 0.1);

  /** @brief 检查从机编队轨迹控制点与主机地图是否碰撞。
   *
   *  利用贝塞尔凸包性质（必要条件）+ De Casteljau 三点采样（充分补充）:
   *  Phase-1: 若控制点全部无碰，曲线段必在凸包内，可快速通过；
   *  Phase-2: 对每段在 t=0.25/0.50/0.75 处采样，捕捉窄通道漏检。
   *
   *  @param follower_ctrl_pts  (3×N) 列矩阵，从机编队控制点（ENU 全局坐标）
   *  @param collision_indices  输出：所有碰撞控制点的列索引
   *  @return true 表示存在碰撞 */
  bool checkFollowerCollision(const Eigen::MatrixXd& follower_ctrl_pts,
                              std::vector<int>& collision_indices);

  /** @brief 对有障碍物碰撞的从机引导轨迹进行梯度下降重优化。
   *
   *  代价函数 = w_obs * 障碍物排斥 + w_form * 编队保持 + w_smooth * 平滑度
   *  前两个和后两个控制点固定以保持 C0/C1 边界约束。
   *
   *  @param initial_ctrl_pts   初始猜测（来自 generateFormationGuidance 输出）
   *  @param leader_ctrl_pts    主机控制点（编队参考基准，当前用于接口兼容性保留）
   *  @param follower_index     从机在编队中的序号（0-based）
   *  @param segment_duration   每段贝塞尔时长（s）
   *  @param optimized_ctrl_pts 输出：优化后控制点
   *  @return true 表示优化后碰撞已完全消除 */
  bool optimizeFollowerTrajectory(
      const Eigen::MatrixXd& initial_ctrl_pts,
      const Eigen::MatrixXd& leader_ctrl_pts,
      int                    follower_index,
      double                 segment_duration,
      Eigen::MatrixXd&       optimized_ctrl_pts);

  Eigen::MatrixXd leaderBezierToCtrlPts(const ego_planner::Bezier &msg) const;

  void generateFormationGuidance(const Eigen::MatrixXd &leader_ctrl_pts,
                                 std::unordered_map<int, Eigen::MatrixXd> &agent_ctrl_pts_map,
                                 const std::vector<FormationCommand>& formation_cmds = {});
  void applySwarmCollisionPenalty(std::unordered_map<int, Eigen::MatrixXd> &agent_ctrl_pts_map) const;

  ego_planner::Bezier buildAgentBezierMsg(int agent_id, const Eigen::MatrixXd &ctrl_pts,
                                          const ros::Time &start_time, int64_t traj_id) const;

/** @brief 对整条主机 Bezier 轨迹计算每个控制点处的平滑航向向量。
   *  1) 用曲线真实切线 B'(t) 替代控制多边形差分
   *  2) 对切线序列做滑动平均消除段间接缝抖动
   *  返回 (3 x N) 矩阵，每列为归一化的 XY 平面航向 */
  Eigen::MatrixXd computeSmoothedHeadings(const Eigen::MatrixXd &ctrl_pts) const;
  std::string formatTopic(const std::string &topic_template, int agent_id) const;
  void runFixedObstacleSchedule(const Eigen::MatrixXd &leader_ctrl_pts,
                                const ros::Time &publish_start_time);
  bool optimizeFixedObstacleWindow(Eigen::MatrixXd &leader_corrected_ctrl,
                                   const Eigen::MatrixXd &leader_nominal_ctrl,
                                   std::unordered_map<int, Eigen::MatrixXd> &agent_ctrl_pts_map,
                                   const FormationCommand &active_cmd,
                                   int seg_begin, int seg_end);
  bool optimizeOnlinePayloadWindow(Eigen::MatrixXd &leader_corrected_ctrl,
                                   const Eigen::MatrixXd &leader_nominal_ctrl,
                                   std::unordered_map<int, Eigen::MatrixXd> &agent_ctrl_pts_map);
  void buildFixedFormationCommands(const Eigen::MatrixXd &leader_ctrl_pts,
                                   const FormationCommand &active_cmd,
                                   double release_blend,
                                   bool blanket_hold,
                                   std::vector<FormationCommand> &commands) const;
  void buildDefaultFixedObstacleSchedule();
  void loadFixedObstacleSchedule();
  Eigen::Vector3d obstacleLocal(const int obstacle_id, const Eigen::Vector3d &world_pt) const;
  double obstacleAlong(const int obstacle_id, const Eigen::Vector3d &world_pt) const;
  bool computePayloadPositionFromStates(Eigen::Vector3d &payload_center) const;
  void applyFollowerBoundaryCorrections(std::unordered_map<int, Eigen::MatrixXd> &agent_ctrl_pts_map) const;
  void applyLeaderBoundaryCorrection(Eigen::MatrixXd &leader_ctrl_pts) const;
  double getSegmentDuration() const;
  Eigen::Vector3d evalBezierPosition(const Eigen::MatrixXd &ctrl_pts, double t) const;
  Eigen::Vector3d evalCurrentLeaderNominalPosition(const Eigen::MatrixXd &leader_ctrl_pts) const;
  std::string obstacleTypeName(const ObstacleType type) const;

private:
  enum class FixedObstacleState {
    IDLE = 0,
    APPROACH,
    ACTIVE,
    HOLD,
    RELEASE,
    DONE
  };

  struct FixedObstacleTemplate
  {
    double back_offset{1.1};
    double primary_span{1.0};
    double aux_span{0.2};
    Eigen::Vector3d leader_bias_local{Eigen::Vector3d::Zero()};
  };

  struct FixedObstacleSpec
  {
    int id{-1};
    std::string name;
    ObstacleType type{ObstacleType::NONE};
    Eigen::Vector3d center{Eigen::Vector3d::Zero()};
    Eigen::Vector3d forward_axis{Eigen::Vector3d::UnitX()};
    Eigen::Vector3d primary_axis{Eigen::Vector3d::UnitZ()};
    Eigen::Vector3d auxiliary_axis{Eigen::Vector3d::UnitY()};
    Eigen::Matrix3d world_from_local{Eigen::Matrix3d::Identity()};
    double physical_half_extent{0.2};
    double activation_window_enter{-0.2};
    double activation_window_exit{0.2};
    double blend_in{1.0};
    double blend_out{1.0};
    double release_margin{0.8};
    int opt_window_margin{1};
    FixedObstacleTemplate mode_template;

    double gap_center_y{0.0};
    double gap_width{1.3};
    double thickness{0.4};
    double z_gap_low{0.6};
    double z_gap_high{2.0};
    double major_r{1.8};
    double minor_r{0.4};
  };

  struct FixedObstacleRuntime
  {
    FixedObstacleState state{FixedObstacleState::IDLE};
    bool hold_latched{false};
    bool completed{false};
    int window_seg_begin{-1};
    int window_seg_end{-1};
    Eigen::Vector3d last_valid_payload_center{Eigen::Vector3d::Zero()};
    bool have_last_valid_payload{false};
    double last_release_blend{0.0};
  };

  void enforcePayloadFeasibleTemplate(FixedObstacleSpec &spec) const;
  double computeTemplateCircumradius(const FixedObstacleSpec &spec) const;

  ros::NodeHandle nh_;
  GridMap::Ptr grid_map_;  ///< 共享主机障碍物地图，用于从机编队轨迹碰撞检测

  ros::Subscriber leader_bezier_sub_;
  ros::Subscriber leader_state_sub_;
  std::vector<ros::Subscriber> agent_state_subs_;
  std::unordered_map<int, ros::Publisher> guidance_pubs_;
  ros::Publisher leader_corrected_pub_;
  ros::Publisher obs_state_marker_pub_;   ///< RViz 障碍物状态文字 Marker
  ros::Timer plan_timer_;

  std::vector<int> agent_ids_;
  std::unordered_map<int, AgentState> agent_states_;
  AgentState leader_state_;

  ego_planner::Bezier latest_leader_bezier_;
  bool have_leader_traj_{false};
  int64_t out_traj_id_{0};
  int64_t last_published_leader_traj_id_{-1};
  ros::Time last_published_leader_start_time_;

  std::string leader_bezier_topic_;
  std::string leader_state_topic_;
  std::string leader_corrected_topic_;
  std::string state_topic_template_;
  std::string guidance_topic_template_;
  double leader_state_timeout_{0.2};

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

  // C2 混合权重：对 P2 进行从速度方向到理想编队方向的加权混合
  // 取值范围 [0,1]，0 表示完全保留理想编队 P2，1 表示完全按速度方向延伸
  // 仅在 blend > 0 时（即检测到 P0 跳变时）生效，无跳变时为 0 效果
  double guidance_c2_blend_weight_{0.6};

  // ── 从机轨迹避障优化参数（Step 4.5 / Prompt 4-5 新增）──────────────
  bool   follower_collision_check_{true};     ///< 是否启用从机碰撞检测
  bool   follower_optimization_enabled_{true};///< 是否启用从机 L-BFGS 优化
  double formation_weight_{20.0};             ///< λ5 编队保持权重（软约束）
  double follower_clearance_{0.4};            ///< 从机碰撞检测安全距离(m)
  int    follower_bezier_sample_density_{4};  ///< 每段 De Casteljau 额外采样点数
  double opt_timeout_ms_{15.0};              ///< 单次优化耗时警告阈值(ms)
  double follower_max_vel_{2.0};             ///< 从机最大速度(m/s)，用于 C1 速度钳制

  // ── 窄通道编队自适应参数 ─────────────────────────────────────────────
  double narrow_passage_threshold_{3.0};  ///< 通道宽度阈值(m)，低于此值触发竖直V形
  int    transition_ctrl_pts_{6};         ///< 水平/垂直编队之间的过渡区控制点数
  double z_formation_max_{2.5};           ///< 竖直V形Z轴偏移上限(m)，防撞天花板
  double z_formation_min_{0.5};           ///< 竖直V形Z轴偏移下限(m)，防撞地面

  // ── Trailing Protection：记忆窄通道区域，直到从机安全通过 ──────────
  struct NarrowPassageRecord {
    Eigen::Vector3d center;       ///< 窄通道中心位置（世界坐标）
    Eigen::Vector3d forward;      ///< 通过方向（主机航向，已归一化）
    double extent{0.5};           ///< 窄通道沿飞行方向的半宽度 (m)
    ros::Time detected_time;      ///< 首次检测到的时间
    ObstacleType type{ObstacleType::DOOR_FRAME};  // 新增：记录障碍物类型
  };
  std::vector<NarrowPassageRecord> active_narrow_passages_;  ///< 当前活跃窄通道记录

  /// 从机必须超过窄通道出口 + trailing_safety_margin_ 才解除Z轴编队保护
  double trailing_safety_margin_{3.0};  ///< 单位 m，含编队后退距离+安全裕量
  /// 窄通道记录最大保持时间，防止永久残留
  double narrow_passage_max_age_{10.0}; ///< 单位 s

  // ── 圆环检测参数 ──────────────────────────────────────────────────────
  double ring_detect_radius_{2.0};     ///< 圆环检测半径(m)，在此范围内做多角度射线
  int    ring_detect_angles_{12};      ///< 周向射线数量（每30°一条）
  double ring_center_clear_dist_{1.0}; ///< 中心通畅判定距离(m)
  double ring_surround_ratio_{0.5};    ///< 周围被占比阈值，超过此值视为被障碍物包围

  // ── Z_SLIT 钝角三角形编队参数 ─────────────────────────────────────────
  double slit_spread_factor_{2.5};   ///< Z_SLIT 时 XY 平面横纵向扩展系数

  // ── RING 紧凑纵列编队参数 ─────────────────────────────────────────────
  double ring_longitudinal_spacing_{1.0}; ///< RING 时纵列间距(m)
  double ring_compact_lateral_scale_{0.35}; ///< RING 时横向保留比例(0~1), 防止从机重合

  // ── 圆环多距离前向扫描 ──────────────────────────────────────────────────
  int ring_scan_steps_{6};               ///< 前向扫描步数，在[0.5x, 2.0x]ring_detect_radius_范围均匀采样

  // ── 障碍物识别去抖动（时间窗口确认缓冲区）──────────────────────────────
  // 原理：只有连续 obs_confirm_frames_ 帧都识别到同一类型才"确认"；
  //       确认后保持至少 obs_release_duration_ 秒再允许释放（抑制闪变）。
  int          obs_confirm_frames_{4};           ///< 确认所需连续帧数
  double       obs_release_duration_{0.5};       ///< 确认后最少保持时长(s)
  ObstacleType obs_confirmed_type_{ObstacleType::NONE}; ///< 当前已确认的稳定类型
  ros::Time    obs_confirm_time_;                ///< 最近一次确认的时间戳
  int          obs_consecutive_count_{0};        ///< 当前连续识别同类型的帧数
  ObstacleType obs_last_raw_type_{ObstacleType::NONE};  ///< 上一帧原始识别类型

  bool use_fixed_obstacle_schedule_{false};
  bool disable_online_classification_{false};
  bool refresh_guidance_on_same_leader_traj_{false};
  bool fixed_obstacle_release_require_payload_clear_{true};
  std::vector<FixedObstacleSpec> fixed_obstacle_schedule_;
  std::vector<FixedObstacleRuntime> fixed_obstacle_runtimes_;
  int current_fixed_obstacle_idx_{0};

  double payload_rope_length_{1.0};
  double payload_radius_{0.2};
  double payload_extra_margin_{0.05};
  double payload_template_rope_margin_{0.05};
  double triangle_area_min_{0.05};
  double inter_uav_sep_min_{0.8};
  int payload_samples_per_seg_{3};

  double cooperative_w_smooth_{10.0};
  double cooperative_w_feas_{1.0};
  double cooperative_w_uav_obs_{100.0};
  double cooperative_w_leader_ref_{50.0};
  double cooperative_w_mode_shape_{30.0};
  double cooperative_w_payload_feas_{120.0};
  double cooperative_w_payload_obs_{160.0};
  double cooperative_w_sep_{40.0};
  double cooperative_payload_invalid_penalty_{2000.0};
  int cooperative_max_iterations_{60};
  double cooperative_fd_eps_{1e-3};
  double cooperative_obstacle_search_radius_{1.5};

  bool online_payload_opt_enabled_{true};
  int online_payload_window_segs_{3};
};

} // namespace ego_planner

#endif
