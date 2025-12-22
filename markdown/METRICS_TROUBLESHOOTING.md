# Metrics记录和分析问题解决方案

## 问题总结

### 1. CSV文件只有表头无数据
**现象**: 运行 `/tmp/improved_simple_analyze.sh` 后显示"CSV文件为空,没有记录任何数据"

**原因**: `metrics_recorder_node` 只在无人机**飞行时**记录数据(状态为 TAKING_OFF 或 FLYING)，而不是从启动就开始记录。

### 2. 飞行状态判断逻辑
`metrics_recorder_node` 通过以下状态机判断飞行阶段：

```
IDLE (空闲) 
  ↓ (高度变化 > 0.3m 或 速度 > 0.1m/s)
TAKING_OFF (起飞中)
  ↓ (高度变化 > 0.5m)
FLYING (飞行中) ← 只在这个阶段记录数据
  ↓ (距离目标 < 0.5m)
ARRIVED (已到达)
```

## 解决方案

### 步骤1: 使用新的分析脚本

我已经创建了改进的分析工具，能够处理空数据情况并提供诊断建议：

```bash
# 分析最新的CSV文件(支持空数据诊断)
./analyze_metrics.sh

# 分析指定文件
./analyze_metrics.sh /tmp/ego_planner_metrics/metrics_20251222_044849.csv
```

### 步骤2: 检查飞行状态

使用新创建的状态检查脚本：

```bash
./check_flight_status.sh
```

这个脚本会检查：
- ✅ metrics_recorder_node 是否在运行
- ✅ 订阅的话题是否正常
- ✅ 飞行参数配置
- ✅ 最近的飞行状态日志

### 步骤3: 手动监控飞行状态转换

实时查看状态变化：

```bash
rostopic echo /rosout | grep -i flight
```

**应该看到的正常输出**:
```
[Flight] State: IDLE -> TAKING_OFF (height: 0.35 m, vel: 0.12 m/s)
[Flight] State: TAKING_OFF -> FLYING
[Flight] State: FLYING -> ARRIVED (time: 12.50 s, distance: 5.20 m)
```

## 诊断清单

### 1. 确认节点运行
```bash
rosnode list | grep metrics_recorder
# 应该看到: /metrics_recorder_node
```

### 2. 确认话题有数据
```bash
# 检查里程计数据
rostopic hz /odom_world
# 应该显示: average rate: ~100 Hz

# 检查轨迹命令
rostopic hz /planning/pos_cmd
# 在飞行时应该有数据
```

### 3. 查看当前位置
```bash
rostopic echo /odom_world/pose/pose/position -n 1
```

### 4. 手动发送目标点测试
```bash
rostopic pub /move_base_simple/goal geometry_msgs/PoseStamped \
  '{header: {frame_id: "world"}, 
    pose: {position: {x: 1.0, y: 0.0, z: 1.0}}}' --once
```

### 5. 检查飞行参数
```bash
# 起飞高度阈值 (默认0.3m)
rosparam get /metrics_recorder_node/takeoff_height_threshold

# 到达目标阈值 (默认0.5m)
rosparam get /metrics_recorder_node/goal_reach_threshold

# 速度阈值 (默认0.1m/s)
rosparam get /metrics_recorder_node/velocity_threshold
```

### 6. 查看数据文件
```bash
# 列出所有记录文件
ls -lh /tmp/ego_planner_metrics/

# 查看最新文件的内容
tail -20 $(ls -t /tmp/ego_planner_metrics/metrics_*.csv | head -1)
```

## 常见问题

### Q1: 为什么CSV文件一直是空的？

**A**: 检查以下几点：
1. 无人机是否真的起飞了？(高度变化 > 0.3m)
2. 是否设置了目标点？
3. `metrics_recorder_node` 是否在运行？
4. 查看日志确认状态转换: `rostopic echo /rosout | grep Flight`

### Q2: 如何判断无人机是否进入FLYING状态？

**A**: 使用以下命令实时监控：
```bash
rostopic echo /rosout | grep -i flight
```
看到 `TAKING_OFF -> FLYING` 表示进入飞行状态，开始记录数据。

### Q3: 数据记录频率是多少？

**A**: 默认2Hz (每0.5秒记录一次)，在 `metrics_recorder_node.cpp` 的 `metrics_timer_` 中定义。

### Q4: 如何修改起飞检测阈值？

**A**: 启动节点时设置参数：
```bash
rosrun ego_planner metrics_recorder_node \
  _takeoff_height_threshold:=0.2 \
  _goal_reach_threshold:=0.3 \
  _velocity_threshold:=0.05
```

## 完整测试流程

```bash
# 1. 启动仿真
roslaunch ego_planner simple_run.launch

# 2. 启动metrics记录节点
rosrun ego_planner metrics_recorder_node

# 3. 在RViz中设置目标点，让无人机飞行

# 4. 实时监控飞行状态
rostopic echo /rosout | grep -i flight

# 5. 飞行结束后分析数据
./analyze_metrics.sh

# 6. 可视化结果
python3 scripts/visualize_metrics.py --save
```

## 文件位置

- **分析脚本**: `/home/jude/ego-planner-bezier/analyze_metrics.sh`
- **状态检查脚本**: `/home/jude/ego-planner-bezier/check_flight_status.sh`
- **源代码**: `/home/jude/ego-planner-bezier/src/planner/plan_manage/src/metrics_recorder_node.cpp`
- **数据文件**: `/tmp/ego_planner_metrics/metrics_*.csv`

## 参考文档

- `Jude_help_doc.md` - 快速参考指南
- `PERFORMANCE_METRICS_SYSTEM.md` - 性能指标系统详细说明
- `METRICS_IMPROVEMENT_GUIDE.md` - Metrics系统改进指南
