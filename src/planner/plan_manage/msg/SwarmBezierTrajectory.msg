# SwarmBezierTrajectory.msg
# 集群Bezier轨迹消息 - 主机发布给从机的引导轨迹
# 
# 设计原则：
# 1. 包含完整的Bezier曲线控制点（作为从机优化的初值）
# 2. 使用相对于主机的坐标系
# 3. 包含动力学约束信息，确保从机可执行
# 4. 支持丢包预测继续飞行

Header header

# ============== 轨迹标识 ==============
int64 traj_id                           # 轨迹唯一ID（用于丢包检测）
uint32 seq_num                          # 序列号（递增，用于检测丢包）
uint8 follower_id                       # 目标从机ID (1~N)
uint8 total_followers                   # 从机总数

# ============== 主机状态（用于相对坐标系转换） ==============
geometry_msgs/Point leader_position     # 主机当前位置（世界坐标系）
geometry_msgs/Vector3 leader_velocity   # 主机当前速度
geometry_msgs/Quaternion leader_orientation  # 主机当前姿态
time leader_state_timestamp             # 主机状态时间戳

# ============== Bezier轨迹参数 ==============
int32 order                             # Bezier曲线阶数 (3 for cubic)
float64[] segment_durations             # 每段Bezier的时间长度
geometry_msgs/Point[] control_points    # 控制点（相对于主机的坐标系）
time traj_start_time                    # 轨迹起始时间

# ============== 编队位置信息 ==============
geometry_msgs/Point formation_offset    # 相对于主机的编队偏移（相对坐标系）
float64 formation_radius                # 编队半径
float64 formation_angle                 # 编队角度（0~2*PI）

# ============== 动力学约束（与主机一致） ==============
float64 max_vel                         # 最大速度限制 (m/s)
float64 max_acc                         # 最大加速度限制 (m/s^2)
float64 max_jerk                        # 最大加加速度限制 (m/s^3)

# ============== 预测信息（用于丢包处理） ==============
float64 expected_update_interval        # 期望更新间隔 (秒)
geometry_msgs/Point[] predicted_positions  # 未来N秒的预测位置（用于丢包时继续飞行）
float64[] prediction_timestamps         # 预测位置的时间戳（相对于traj_start_time）

# ============== 其他从机位置（用于机间避障） ==============
uint8[] other_follower_ids              # 其他从机ID列表
geometry_msgs/Point[] other_follower_positions  # 其他从机位置（相对坐标系）
geometry_msgs/Vector3[] other_follower_velocities  # 其他从机速度

# ============== 状态标志 ==============
uint8 STATUS_NORMAL = 0
uint8 STATUS_EMERGENCY_STOP = 1
uint8 STATUS_FORMATION_CHANGE = 2
uint8 STATUS_RETURN_HOME = 3
uint8 status                            # 当前状态
