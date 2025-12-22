# 性能指标可视化使用指南

## 问题修复说明

原始脚本在 Docker 容器中运行时会遇到 `tkinter` 模块缺失的问题。已修复：

1. **安装了 python3-tk**: `sudo apt-get install python3-tk`
2. **修改了脚本**: 使用 Agg 后端，兼容无显示环境
3. **添加了文件保存模式**: 适用于 Docker 等无图形界面环境

## 使用方式

### 方式 1: 保存到文件（推荐用于 Docker）

```bash
# 终端 1: 启动 ego-planner
roslaunch ego_planner simple_run.launch

# 终端 2: 启动指标记录节点
rosrun ego_planner metrics_recorder_node

# 终端 3: 启动可视化（保存到文件）
python3 scripts/visualize_metrics.py --save

# 查看生成的图表
# 图表保存在: /tmp/ego_planner_metrics/metrics_plot.png
# 每收到10个数据点自动更新一次
```

自定义输出路径：
```bash
python3 scripts/visualize_metrics.py --save --output /path/to/output.png
```

### 方式 2: 实时交互显示（需要图形界面）

```bash
# 如果你的系统有图形界面（非 Docker）
python3 scripts/visualize_metrics.py
```

## 查看结果

### Docker 环境
```bash
# 方法 1: 复制到主机查看
docker cp <container_id>:/tmp/ego_planner_metrics/metrics_plot.png ./

# 方法 2: 使用 eog 或其他查看器（如果已安装）
eog /tmp/ego_planner_metrics/metrics_plot.png

# 方法 3: 在主机上挂载卷
# 启动容器时添加: -v /host/path:/tmp/ego_planner_metrics
```

### 监控更新
```bash
# 实时监控文件更新
watch -n 1 "ls -lh /tmp/ego_planner_metrics/metrics_plot.png"
```

## 数据说明

可视化脚本监听 `/planner_metrics` 话题，显示以下 12 个指标：

1. **Success Rate** - 规划成功率
2. **Optimization Time** - 优化耗时（毫秒）
3. **Iterations** - 迭代次数
4. **Replan Frequency** - 重规划频率（Hz）
5. **Smoothness** - 平滑度（jerk 积分）
6. **Path Length** - 路径长度（米）
7. **Final Cost** - 最终代价函数值
8. **Clearance** - 障碍物间隙（最小/平均，米）
9. **Max Velocity** - 最大速度（m/s）
10. **Max Acceleration** - 最大加速度（m/s²）
11. **Statistics** - 实时统计信息

## 故障排查

### 问题: "No module named 'tkinter'"
**解决**: 已通过使用 Agg 后端修复，无需 tkinter

### 问题: "Unable to register with master node"
**解决**: 确保先启动 `roslaunch ego_planner simple_run.launch`

### 问题: 图表不更新
**检查**:
```bash
# 1. 检查话题是否有数据
rostopic echo /planner_metrics

# 2. 检查节点是否运行
rosnode list | grep metrics

# 3. 检查输出目录权限
ls -la /tmp/ego_planner_metrics/
```

### 问题: Docker 中看不到图形窗口
**解决**: 使用 `--save` 模式保存到文件，不使用交互式显示

## 命令行参数

```
usage: visualize_metrics.py [-h] [--save] [--output OUTPUT]

optional arguments:
  -h, --help       显示帮助信息
  --save           保存到文件而不是交互式显示
  --output OUTPUT  指定输出文件路径（默认: /tmp/ego_planner_metrics/metrics_plot.png）
```

## 技术细节

- **后端**: matplotlib Agg（非交互式，兼容无显示环境）
- **更新频率**: 每收到10个数据点更新一次图表
- **数据缓冲**: 最多保留最近 100 个数据点
- **图表分辨率**: 150 DPI
- **图表尺寸**: 16x10 英寸

