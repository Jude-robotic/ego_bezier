# 性能指标系统快速入门

## 三步启动（Docker 环境推荐）

### 步骤 1: 启动 EGO-Planner
```bash
cd ~/ego-planner-bezier
source devel/setup.bash
roslaunch ego_planner simple_run.launch
```

### 步骤 2: 启动指标记录节点
打开新终端：
```bash
cd ~/ego-planner-bezier
source devel/setup.bash
rosrun ego_planner metrics_recorder_node
```

### 步骤 3: 启动可视化（文件模式）
打开新终端：
```bash
cd ~/ego-planner-bezier
python3 scripts/visualize_metrics.py --save
```

这会将图表保存到 `/tmp/ego_planner_metrics/metrics_plot.png`，每收到 10 个数据点自动更新一次。

## 查看结果

### 在 Docker 容器内
```bash
# 安装图像查看器（如果尚未安装）
apt-get install -y eog

# 查看图表
eog /tmp/ego_planner_metrics/metrics_plot.png
```

### 复制到主机系统
```bash
# 在主机上运行（替换 container_id）
docker cp <container_id>:/tmp/ego_planner_metrics/metrics_plot.png ./
```

### 使用卷挂载（推荐）
启动 Docker 容器时添加卷挂载：
```bash
docker run -v /host/path:/tmp/ego_planner_metrics ...
```

## 实时监控

### 监控文件更新
```bash
watch -n 2 "ls -lh /tmp/ego_planner_metrics/metrics_plot.png"
```

### 查看话题数据
```bash
# 查看原始指标数据
rostopic echo /planner_metrics

# 查看话题频率
rostopic hz /planner_metrics
```

### 查看 CSV 数据
```bash
# 查看最新记录
tail -20 /tmp/ego_planner_metrics/planner_metrics_YYYYMMDD_HHMMSS.csv

# 统计总记录数
wc -l /tmp/ego_planner_metrics/*.csv
```

## 交互式模式（需要图形界面）

如果你的系统有图形界面（非 Docker 环境）：
```bash
# 使用默认 TkAgg 后端的实时显示
python3 scripts/visualize_metrics.py
```

## 自定义输出路径

```bash
# 保存到指定位置
python3 scripts/visualize_metrics.py --save --output ~/my_metrics.png

# 保存到共享目录
python3 scripts/visualize_metrics.py --save --output /path/to/shared/folder/plot.png
```

## 12 个性能指标说明

1. **Success Rate** - 规划成功率（%）
2. **Optimization Time** - 单次优化耗时（毫秒）
3. **Iterations** - L-BFGS 迭代次数
4. **Replan Frequency** - 重规划频率（Hz）
5. **Smoothness** - 轨迹平滑度（jerk 的积分）
6. **Path Length** - 路径总长度（米）
7. **Final Cost** - 优化后的最终代价
8. **Clearance** - 与障碍物的间隙（米）
9. **Max Velocity** - 轨迹最大速度（m/s）
10. **Max Acceleration** - 轨迹最大加速度（m/s²）
11. **Statistics** - 实时统计摘要
12. **Radar Chart** - 多维性能雷达图

## 故障排查

### 问题 1: "No module named 'tkinter'"
✅ **已修复**: 脚本现在使用 Agg 后端，不需要 tkinter

### 问题 2: "Unable to register with master node"
**原因**: ROS master 未运行  
**解决**: 先执行步骤 1，启动 `roslaunch ego_planner simple_run.launch`

### 问题 3: 没有数据显示
**检查步骤**:
```bash
# 1. 确认 metrics_recorder_node 在运行
rosnode list | grep metrics

# 2. 检查话题是否有数据
rostopic echo /planner_metrics -n 1

# 3. 确认你已经给了目标点
# 在 RViz 中使用 "2D Nav Goal" 工具点击地图设置目标
```

### 问题 4: 图表文件不存在
**检查**:
```bash
# 检查目录是否存在
ls -la /tmp/ego_planner_metrics/

# 如果不存在，创建目录
mkdir -p /tmp/ego_planner_metrics/

# 检查权限
chmod 777 /tmp/ego_planner_metrics/
```

## 性能调优建议

根据指标数据调整参数（`advanced_param.xml`）：

| 指标问题 | 可能原因 | 调整建议 |
|---------|---------|---------|
| 成功率低 | 优化失败过多 | 增加 `max_iteration_num` |
| 优化时间长 | 迭代次数过多 | 减少 `max_iteration_num` 或降低收敛阈值 |
| 轨迹不平滑 | smoothness 权重小 | 增加 `lambda_smoothness` |
| 频繁碰撞风险 | clearance 过小 | 增加 `lambda_distance` 和 `safe_distance` |
| 速度超限 | 动力学约束松 | 减小 `max_vel` 或增加 `lambda_fitness` |
| 重规划频繁 | 环境变化快 | 增加 `replan_thresh` |

## 下一步

- 📊 查看 `PERFORMANCE_METRICS_SYSTEM.md` 了解完整的指标定义
- 🔧 查看 `METRICS_INTEGRATION_GUIDE.md` 了解技术实现细节
- �� 查看 `COMPILATION_FIX.md` 了解编译问题的解决方案
- 🎨 查看 `VISUALIZATION_GUIDE.md` 了解可视化的详细用法

## 快速命令参考

```bash
# 启动完整系统（三个终端）
# Terminal 1:
roslaunch ego_planner simple_run.launch

# Terminal 2:
rosrun ego_planner metrics_recorder_node

# Terminal 3:
python3 scripts/visualize_metrics.py --save

# 实用命令
rostopic list                          # 查看所有话题
rosnode list                           # 查看所有节点
rostopic echo /planner_metrics         # 查看指标数据
ls -lh /tmp/ego_planner_metrics/       # 查看输出文件
```

