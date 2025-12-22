# Metrics系统改进指南

## 问题分析

根据对 `/tmp/ego_planner_metrics/metrics_20251221_052306.csv` 的分析，发现以下问题：

### 1. 数据记录时间范围问题
- **问题**: 当前从系统启动就开始记录，而不是从无人机起飞到到达终点
- **表现**: 220-270秒的时间跨度，包含大量无人机静止状态的数据
- **影响**: 导致平均值失真，图表显示为"定值"

### 2. 关键指标缺失
以下指标全部为0或初始值：
- `avg_opt_time_ms` = 0 (优化时间)
- `avg_iterations` = 0 (迭代次数)
- `avg_smoothness` = 0 (平滑度)
- `path_length` = 0 (路径长度)
- `final_cost` = 0 (最终代价)
- `min_clearance` = 1e+10 (最小间隙 - 初始值)

**原因**: `metrics_recorder_node` 只订阅了 `/planning/pos_cmd` 话题，没有订阅优化器内部信息。

### 3. 速度和加速度数据异常
- 速度范围: 1.70-2.68 m/s (变化极小)
- 加速度范围: 4.31-5.18 m/s² (几乎不变)
- 重规划频率: 0.57842 Hz (完全不变)

**分析**: 
1. 数据几乎是常量，说明可能是从 PositionCommand 消息中读取的"目标"速度/加速度，而不是实际飞行状态
2. 无人机可能存在跟踪误差，实际飞行轨迹与规划轨迹差异较大

## 改进方案

### 方案1: 修改 metrics_recorder_node (推荐) ⭐

#### 改进点：
1. **添加飞行状态机**: 检测起飞→飞行→到达的状态转换
2. **只在飞行期间记录数据**: 过滤掉待机和到达后的数据
3. **记录真实飞行状态**: 从 `/odom_world` 获取实际速度/加速度

#### 实施步骤：

**步骤1**: 备份原文件
```bash
cd /home/jude/ego-planner-bezier/src/planner/plan_manage/src
cp metrics_recorder_node.cpp metrics_recorder_node.cpp.backup
```

**步骤2**: 替换为改进版本
```bash
cp /tmp/improved_metrics_recorder_node.cpp /home/jude/ego-planner-bezier/src/planner/plan_manage/src/metrics_recorder_node.cpp
```

**步骤3**: 在 planner_metrics.h 中添加 resetMetrics() 声明
编辑 `/home/jude/ego-planner-bezier/src/planner/plan_manage/include/plan_manage/planner_metrics.h`，
在 `PlannerMetricsRecorder` 类的 public 部分添加：
```cpp
void resetMetrics();  // 重置所有指标
```

**步骤4**: 在 planner_metrics.cpp 中添加实现
编辑 `/home/jude/ego-planner-bezier/src/planner/plan_manage/src/planner_metrics.cpp`，添加：
```cpp
void PlannerMetricsRecorder::resetMetrics()
{
    metrics_.reset();
    opt_time_window_.clear();
    iter_window_.clear();
    smoothness_window_.clear();
    clearance_window_.clear();
    path_length_window_.clear();
    replan_intervals_.clear();
    
    last_replan_time_ = ros::Time::now();
    
    ROS_INFO("\033[1;33m[Metrics] Metrics reset for new flight\033[0m");
}
```

**步骤5**: 重新编译
```bash
cd /home/jude/ego-planner-bezier
catkin_make
```

#### 改进后的特点：
- ✅ 只记录起飞后到到达前的数据
- ✅ 自动检测起飞（高度变化 > 0.3m 或速度 > 0.1 m/s）
- ✅ 自动检测到达（距离目标 < 0.5m 且速度 < 0.1 m/s）
- ✅ 每次新任务自动重置指标
- ✅ 记录实际飞行时间和路径长度

### 方案2: 手动分析数据

使用提供的分析脚本查看和导出数据：

```bash
# 快速分析
/tmp/simple_analyze.sh /tmp/ego_planner_metrics/metrics_20251221_052306.csv

# 查看原始CSV (前30行)
head -30 /tmp/ego_planner_metrics/metrics_20251221_052306.csv

# 提取特定时间段的数据 (例如: 10-50秒)
awk -F',' 'NR==1 || ($1-start>=10 && $1-start<=50) {print} NR==2{start=$1}' \
  /tmp/ego_planner_metrics/metrics_20251221_052306.csv > /tmp/filtered_metrics.csv
```

## 无人机抖动和不连贯问题分析

根据数据分析，发现以下可能原因：

### 1. 速度/加速度几乎恒定
**现象**: 
- 速度在 1.7-2.68 m/s 之间变化极小
- 加速度在 4.3-5.2 m/s² 之间变化极小

**可能原因**:
1. **控制器增益过高**: 导致过度响应，产生震荡
2. **规划频率与控制频率不匹配**: 0.58 Hz 的重规划频率可能太低
3. **轨迹不够平滑**: `avg_smoothness=0` 说明没有记录到平滑度信息

**建议改进**:
```yaml
# 在 ego_planner 参数文件中调整
control:
  kp_pos: [降低位置增益]
  kd_vel: [增加速度阻尼]
  
planning:
  replan_thresh: [调整重规划阈值]
  max_vel: 2.5  # 限制最大速度
  max_acc: 3.0  # 限制最大加速度
```

### 2. 缺少优化指标
**问题**: 无法评估优化器性能
- 优化时间 = 0
- 迭代次数 = 0
- 代价函数值 = 0

**原因**: metrics_recorder_node 没有订阅优化器的输出话题

**解决方案**: 
需要修改 `ego_planner_node` 或 `bezier_optimizer`，发布优化结果到专门的话题：
```cpp
// 在优化器中添加
ros::Publisher opt_result_pub_ = nh.advertise<...>("/optimization_result", 10);

// 优化完成后发布
OptimizationResult msg;
msg.time_ms = ...;
msg.iterations = ...;
msg.final_cost = ...;
opt_result_pub_.publish(msg);
```

### 3. 重规划频率异常
**现象**: 重规划频率完全不变 (0.57842 Hz)

**可能原因**:
1. 触发机制单一（只基于时间，没有基于事件）
2. 或者是计算方法有误

**建议**: 检查 ego_planner 的触发条件，添加基于位置误差的触发：
```cpp
if (tracking_error > threshold || time_since_last_plan > timeout) {
    replan();
}
```

## 使用方法

### 运行改进后的系统
```bash
# 终端1: 启动仿真
roslaunch ego_planner simple_run.launch

# 终端2: 启动改进的metrics recorder
rosrun ego_planner metrics_recorder_node

# 终端3 (可选): 实时查看状态
rostopic echo /planner_metrics_text
```

### 分析数据
```bash
# 使用简单分析脚本
/tmp/simple_analyze.sh

# 查看最新的CSV
ls -lht /tmp/ego_planner_metrics/

# 手动提取数据
awk -F',' '{print $1-start, $7, $8}' /tmp/ego_planner_metrics/metrics_*.csv > data.txt
```

## 进一步改进建议

1. **添加更多话题订阅**:
   - `/optimization_result` - 优化器详细信息
   - `/planning/path` - 完整规划路径
   - `/grid_map/occupancy` - 障碍物地图

2. **记录轨迹跟踪误差**:
   ```cpp
   double tracking_error = (actual_pos - planned_pos).norm();
   metrics_recorder_->recordTrackingError(tracking_error);
   ```

3. **添加实时可视化**:
   - 在RViz中显示性能指标
   - 添加rviz plugin 显示图表

4. **导出更多格式**:
   - JSON格式（便于程序读取）
   - ROS bag（便于回放）

## 文件位置

- 改进的代码: `/tmp/improved_metrics_recorder_node.cpp`
- 分析脚本: `/tmp/simple_analyze.sh`
- 补丁说明: `/tmp/planner_metrics_resetMetrics_patch.txt`
- 数据分析脚本: `/tmp/analyze_metrics.py` (需要pandas)

## 参考

- 原始设计文档: `PERFORMANCE_METRICS_SYSTEM.md`
- 可视化指南: `VISUALIZATION_GUIDE.md`
- 快速开始: `QUICK_START_METRICS.md`
