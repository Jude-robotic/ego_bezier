# EGO-Planner性能指标评估系统

## 概述

我已经为ego-planner-bezier创建了一个完整的性能指标评估系统,包括以下组件:

### 已创建的文件

1. **性能指标核心代码**
   - `src/planner/plan_manage/include/plan_manage/planner_metrics.h` - 指标定义
   - `src/planner/plan_manage/src/planner_metrics.cpp` - 指标实现  
   - `src/planner/plan_manage/src/metrics_recorder_node.cpp` - 独立记录节点

2. **可视化脚本**
   - `scripts/visualize_metrics.py` - Python实时可视化

3. **文档**
   - `METRICS_INTEGRATION_GUIDE.md` - 集成指南

## 评估指标说明

### 1. 任务完成指标
- **成功率 (Success Rate)**: 规划成功次数 / 总尝试次数
- **目标到达次数**: 机器人成功到达目标点的次数

### 2. 轨迹质量指标  
- **平滑度 (Smoothness)**: 基于jerk/snap的轨迹平滑性评估
- **路径长度 (Path Length)**: 实际飞行路径总长度
- **最大jerk值**: 控制点三阶差分的最大值

### 3. 安全性指标
- **最小障碍物距离 (Min Clearance)**: 轨迹离障碍物的最小距离
- **平均安全余量 (Avg Clearance)**: 平均clearance值
- **碰撞次数**: 违反安全距离阈值的次数
- **安全违规率**: 碰撞次数/总尝试次数

### 4. 计算效率指标
- **平均优化时间 (Avg Opt Time)**: L-BFGS优化平均耗时 (ms)
- **最大优化时间 (Max Opt Time)**: 单次优化最大耗时
- **平均迭代次数 (Avg Iterations)**: L-BFGS平均迭代次数
- **重规划频率 (Replan Freq)**: 每秒重规划次数 (Hz)

### 5. 动力学可行性指标
- **最大速度 (Max Velocity)**: 轨迹上的最大速度值
- **最大加速度 (Max Acceleration)**: 轨迹上的最大加速度值
- **速度违规率**: 超过速度限制的比率
- **加速度违规率**: 超过加速度限制的比率

### 6. 优化收敛指标
- **最终代价 (Final Cost)**: 优化后的目标函数值
- **代价下降率 (Cost Reduction)**: (初始代价-最终代价)/初始代价

## 使用方法

### 方法1: 使用独立的metrics记录节点 (推荐)

1. **编译系统** (需要修复原始文件后):
```bash
cd /home/jude/ego-planner-bezier
catkin_make
source devel/setup.bash
```

2. **启动ego-planner**:
```bash
roslaunch ego_planner simple_run.launch
```

3. **在新终端启动metrics记录节点**:
```bash
source /home/jude/ego-planner-bezier/devel/setup.bash
rosrun ego_planner metrics_recorder_node
```

4. **在另一个终端启动可视化**:
```bash
python3 /home/jude/ego-planner-bezier/scripts/visualize_metrics.py
```

### 方法2: 使用rosbag离线分析

1. **录制rosbag**:
```bash
rosbag record -O metrics_data.bag /odom_world /planning/pos_cmd /move_base_simple/goal
```

2. **回放并分析**:
```bash
# Terminal 1
roscore

# Terminal 2  
rosrun ego_planner metrics_recorder_node

# Terminal 3
python3 scripts/visualize_metrics.py

# Terminal 4
rosbag play metrics_data.bag
```

## 输出文件

指标数据会自动保存到:
```
/tmp/ego_planner_metrics/metrics_YYYYMMDD_HHMMSS.csv
```

CSV文件包含所有指标的时间序列数据,可用于:
- Excel/MATLAB分析
- Python后处理 (pandas)
- 论文图表生成

## 可视化说明

实时可视化界面包含12个子图:

**第一行** - 效率指标:
1. 成功率趋势图
2. 优化时间趋势图
3. 迭代次数趋势图
4. 重规划频率图

**第二行** - 质量指标:
5. 轨迹平滑度图
6. 路径长度图
7. 优化代价图
8. 安全距离图 (最小/平均)

**第三行** - 动力学指标:
9. 最大速度图
10. 最大加速度图
11. 综合评分雷达图
12. 统计信息文本

## 参数调优建议

基于指标反馈进行参数调优:

### 如果平滑度差 (smoothness高):
```yaml
# 在 bezier_opt.yaml 中
lambda_smoothness: 增大 (例如 1.0 → 2.0)
```

### 如果碰撞多 (min_clearance小):
```yaml
lambda_distance: 增大 (例如 10.0 → 20.0)
dist0: 增大安全阈值 (例如 0.4 → 0.6)
```

### 如果速度/加速度违规:
```yaml
lambda_feasibility: 增大 (例如 1.0 → 2.0)
max_vel: 降低速度限制
max_acc: 降低加速度限制
```

### 如果优化时间过长:
```yaml
max_iterations: 减少 (例如 200 → 100)
g_epsilon: 放宽收敛条件 (例如 0.01 → 0.05)
```

### 如果重规划频率过高:
```yaml
planning_horizon: 增大规划时域 (例如 5.0 → 7.0)
```

## 性能基准参考

**优秀性能指标参考**:
- 成功率: > 95%
- 平均优化时间: < 50ms
- 最小安全距离: > 0.3m
- 碰撞次数: 0
- 速度违规率: < 5%
- 重规划频率: 1-3 Hz

**可接受性能指标**:
- 成功率: > 85%
- 平均优化时间: < 100ms
- 最小安全距离: > 0.2m
- 碰撞次数: < 3
- 速度违规率: < 15%
- 重规划频率: < 5 Hz

## 故障排除

### 问题1: 编译错误
- 确保所有依赖都已安装
- 检查ROS版本 (noetic)
- 清理build目录: `rm -rf build devel && catkin_make`

### 问题2: 没有数据显示
- 检查metrics_recorder_node是否运行: `rosnode list | grep metrics`
- 检查话题连接: `rostopic echo /planner_metrics`
- 确认ego_planner正在运行

### 问题3: 可视化窗口无响应
- 安装matplotlib: `pip3 install matplotlib`
- 检查ROS_MASTER_URI设置

## 扩展功能

系统支持以下扩展:

1. **添加自定义指标**: 修改 `planner_metrics.h` 中的 `PlanningMetrics` 结构
2. **修改采样窗口**: 调整 `window_size_` 参数
3. **更改发布频率**: 修改 `metricsTimerCallback` 的定时器周期
4. **导出其他格式**: 在 `saveMetricsToFile()` 中添加JSON/XML导出

