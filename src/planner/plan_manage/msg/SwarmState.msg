# SwarmState.msg
# 从机状态消息 - 从机发布自身状态供主机和其他从机使用
# 用于机间避障和编队协调

Header header

# ============== 无人机标识 ==============
uint8 uav_id                            # 无人机ID (0=主机, 1~N=从机)
uint8 UAV_TYPE_LEADER = 0
uint8 UAV_TYPE_FOLLOWER = 1
uint8 uav_type                          # 无人机类型

# ============== 位置和速度（世界坐标系） ==============
geometry_msgs/Point position
geometry_msgs/Vector3 velocity
geometry_msgs/Vector3 acceleration
geometry_msgs/Quaternion orientation

# ============== 轨迹执行状态 ==============
int64 current_traj_id                   # 当前执行的轨迹ID
float64 traj_progress                   # 轨迹执行进度 (0~1)
bool is_executing_traj                  # 是否正在执行轨迹

# ============== 传感器状态 ==============
bool has_valid_odom                     # 里程计是否有效
bool has_obstacle_ahead                 # 前方是否有障碍物
float64 min_obstacle_distance           # 最近障碍物距离

# ============== 通信质量 ==============
uint32 last_leader_seq                  # 最后收到的主机消息序列号
float64 leader_msg_delay                # 主机消息延迟（秒）
bool leader_connection_ok               # 与主机通信是否正常

# ============== 运行模式 ==============
uint8 MODE_IDLE = 0
uint8 MODE_FOLLOWING = 1
uint8 MODE_LOCAL_AVOIDANCE = 2
uint8 MODE_EMERGENCY_STOP = 3
uint8 MODE_LOST_CONNECTION = 4
uint8 current_mode

# ============== 安全边界 ==============
float64 safety_radius                   # 安全半径（其他无人机需保持距离）
