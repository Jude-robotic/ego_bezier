# Docker 环境中查看性能指标图表

## 当前状态
✅ 图表已生成: `/tmp/ego_planner_metrics/metrics_plot.png` (153KB)
⚠️ Docker 环境无法直接显示图形界面

## 解决方案

### 方案 1: 复制到主机系统（最简单）

在**主机**上执行：
```bash
# 获取容器 ID
docker ps

# 复制文件到主机当前目录
docker cp <container_id>:/tmp/ego_planner_metrics/metrics_plot.png ./

# 使用主机的图像查看器打开
xdg-open metrics_plot.png
# 或者
eog metrics_plot.png
```

### 方案 2: 使用卷挂载（推荐，持续使用）

**下次启动 Docker 时**添加卷挂载：
```bash
docker run -it \
  -v ~/ego_planner_metrics:/tmp/ego_planner_metrics \
  <其他参数> \
  <镜像名>
```

然后在主机上实时查看：
```bash
# 在主机上查看
ls ~/ego_planner_metrics/
eog ~/ego_planner_metrics/metrics_plot.png

# 实时监控更新
watch -n 2 "ls -lh ~/ego_planner_metrics/metrics_plot.png"
```

### 方案 3: 在 Docker 内安装图像查看器

```bash
# 安装 eog (Eye of GNOME)
apt-get update && apt-get install -y eog

# 如果有 X11 转发，可以查看
eog /tmp/ego_planner_metrics/metrics_plot.png
```

**注意**: 需要 X11 转发支持，启动 Docker 时添加：
```bash
docker run -it \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  <其他参数> \
  <镜像名>
```

### 方案 4: 转换为 Base64 在终端显示（文本模式）

```bash
# 生成 Base64 编码（可以复制到主机解码）
base64 /tmp/ego_planner_metrics/metrics_plot.png > /tmp/plot_base64.txt

# 在主机上解码并查看
base64 -d /path/to/plot_base64.txt > metrics_plot.png
xdg-open metrics_plot.png
```

### 方案 5: 使用 Python 在浏览器中查看

我为你创建了一个 HTTP 服务器脚本：
```bash
# 启动简单的 HTTP 服务器
python3 /home/jude/ego-planner-bezier/scripts/view_metrics_http.py

# 然后在主机浏览器访问:
# http://localhost:8000
```

## 快速操作指南

### 步骤 1: 在 Docker 内运行完整流程

```bash
# Terminal 1: 启动 ego-planner
roslaunch ego_planner simple_run.launch

# Terminal 2: 启动指标记录
rosrun ego_planner metrics_recorder_node

# Terminal 3: 生成图表（保存模式）
python3 scripts/visualize_metrics.py --save

# 确认文件生成
ls -lh /tmp/ego_planner_metrics/metrics_plot.png
```

### 步骤 2: 在主机上查看

```bash
# 获取容器 ID
docker ps

# 复制文件（替换 <container_id>）
docker cp <container_id>:/tmp/ego_planner_metrics/metrics_plot.png ~/Desktop/

# 打开查看
xdg-open ~/Desktop/metrics_plot.png
```

## 自动化脚本

我已经为你创建了便捷脚本：
```bash
# 在 Docker 内执行，自动复制到可访问位置
./copy_metrics_to_host.sh

# 在主机上执行，自动从 Docker 复制
./fetch_metrics_from_docker.sh <container_id>
```

## 当前文件信息

```
文件路径: /tmp/ego_planner_metrics/metrics_plot.png
文件大小: 153KB
图像尺寸: 2348 x 1467 像素
格式: PNG (RGBA)
生成时间: 2025-12-21 04:44
```

## 故障排查

### 问题: "Unable to register with master node"
**原因**: ROS master 未运行  
**解决**: 必须先运行 `roslaunch ego_planner simple_run.launch`

### 问题: 文件存在但看不到
**原因**: Docker 容器与主机文件系统隔离  
**解决**: 使用 `docker cp` 复制到主机，或使用卷挂载

### 问题: 图表不更新
**检查**:
```bash
# 检查 metrics_recorder_node 是否在运行
rosnode list | grep metrics

# 检查话题是否有数据
rostopic echo /planner_metrics -n 1

# 手动触发更新（在 RViz 中设置新目标点）
```

## 推荐工作流程

1. **使用卷挂载**启动 Docker（一次性设置）
2. 在 Docker 内正常运行三步启动流程
3. 在主机上实时查看 `~/ego_planner_metrics/metrics_plot.png`
4. 图表会自动更新（每 10 个数据点）

