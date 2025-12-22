# Metrics分析工具使用总结

## ✅ 已解决的问题

### 1. CSV文件只有表头无数据
- **原因**: `metrics_recorder_node` 只在无人机飞行时记录数据 (TAKING_OFF 或 FLYING 状态)
- **解决**: 创建了改进的分析脚本,提供空数据诊断和解决建议

### 2. 无法判断飞行状态
- **解决**: 创建了 `check_flight_status.sh` 脚本,可以检查节点状态和飞行日志

### 3. 原有脚本不支持空数据
- **解决**: 新的 `analyze_metrics.sh` 智能处理空数据和有数据两种情况

## 📝 新创建的工具

### 1. analyze_metrics.sh
改进的分析脚本,支持:
- ✅ 自动检测最新CSV文件
- ✅ 空数据时提供诊断建议
- ✅ 有数据时显示详细统计
- ✅ 数据质量检查和警告

**使用方法**:
```bash
./analyze_metrics.sh                    # 分析最新文件
./analyze_metrics.sh path/to/file.csv   # 分析指定文件
```

### 2. check_flight_status.sh
飞行状态检查脚本,可以:
- ✅ 检查 metrics_recorder_node 是否运行
- ✅ 检查订阅的话题连接
- ✅ 显示飞行参数配置
- ✅ 查看最近的飞行状态日志

**使用方法**:
```bash
./check_flight_status.sh
```

## 🔍 如何检查飞行状态转换

### 方法1: 使用check_flight_status.sh (推荐)
```bash
./check_flight_status.sh
```

### 方法2: 实时监控rosout
```bash
rostopic echo /rosout | grep -i flight
```

**正常的状态转换**:
```
[Flight] IDLE → TAKING_OFF (height: 0.35 m, vel: 0.12 m/s)
[Flight] TAKING_OFF → FLYING
[Flight] FLYING → ARRIVED (time: 12.50 s, distance: 5.20 m)
```

### 方法3: 查看历史日志
```bash
rostopic echo /rosout -n 50 | grep -i flight
```

## 🎯 完整工作流程

```bash
# 1. 启动仿真和metrics记录
roslaunch ego_planner simple_run.launch   # 终端1
rosrun ego_planner metrics_recorder_node  # 终端2

# 2. 实时监控飞行状态 (可选)
rostopic echo /rosout | grep -i flight    # 终端3

# 3. 在RViz中设置目标点,让无人机飞行

# 4. 飞行结束后分析数据
./analyze_metrics.sh                      # 终端4

# 5. 可视化(可选)
python3 scripts/visualize_metrics.py --save
eog /tmp/ego_planner_metrics/metrics_plot.png
```

## 📊 状态判断逻辑

metrics_recorder_node 通过以下条件判断飞行状态:

| 状态 | 进入条件 | 记录数据 |
|------|---------|---------|
| IDLE | 初始状态 | ❌ 否 |
| TAKING_OFF | 高度变化 > 0.3m 或 速度 > 0.1m/s | ✅ 是 |
| FLYING | 高度变化 > 0.5m | ✅ 是 |
| ARRIVED | 距离目标 < 0.5m | ❌ 否 |

**关键**: 只有在 `TAKING_OFF` 和 `FLYING` 状态时才记录数据到CSV！

## 🛠️ 常用调试命令

```bash
# 检查节点是否运行
rosnode list | grep metrics

# 检查话题频率
rostopic hz /odom_world
rostopic hz /planning/pos_cmd

# 查看当前位置
rostopic echo /odom_world/pose/pose/position -n 1

# 手动发送目标点
rostopic pub /move_base_simple/goal geometry_msgs/PoseStamped \
  '{header: {frame_id: "world"}, pose: {position: {x: 1.0, y: 0.0, z: 1.0}}}' --once

# 查看飞行参数
rosparam get /metrics_recorder_node/takeoff_height_threshold
rosparam get /metrics_recorder_node/goal_reach_threshold
rosparam get /metrics_recorder_node/velocity_threshold

# 查看数据文件
ls -lh /tmp/ego_planner_metrics/
tail -20 $(ls -t /tmp/ego_planner_metrics/metrics_*.csv | head -1)
```

## 📚 相关文档

- `Jude_help_doc.md` - 快速参考指南
- `METRICS_TROUBLESHOOTING.md` - 详细问题诊断和解决方案
- `PERFORMANCE_METRICS_SYSTEM.md` - 性能指标系统说明
- `METRICS_IMPROVEMENT_GUIDE.md` - Metrics系统改进指南

## 🎉 总结

现在你有了完整的工具链来:
1. ✅ **记录** - metrics_recorder_node 在飞行时自动记录
2. ✅ **检查** - check_flight_status.sh 检查节点和状态
3. ✅ **分析** - analyze_metrics.sh 分析数据并提供诊断
4. ✅ **可视化** - visualize_metrics.py 生成图表

所有工具都支持空数据情况,并提供清晰的诊断建议！
