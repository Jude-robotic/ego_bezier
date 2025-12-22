# Ego-Planner 飞行评价系统使用说明

## 系统功能

本系统实现了你的核心诉求：
1. ✅ **数据保存**: 运行`simple_run.launch`时自动记录飞行数据到CSV
2. ✅ **指标评价**: 分析飞行数据并给出评分（0-100分）
3. ✅ **数据可视化**: 生成PNG图表和HTML交互式报告（Docker友好）

## 快速使用

### 1. 运行飞行仿真
```bash
roslaunch ego_planner simple_run.launch
```
数据会自动保存到 `/tmp/ego_planner_metrics/metrics_YYYYMMDD_HHMMSS.csv`

### 2. 分析飞行性能（一键完成）
```bash
./analyze_flight.sh
```

这会自动：
- 评价飞行性能（成功率、安全性、平滑性、效率）
- 生成性能图表
- 创建HTML可视化报告

### 3. 查看结果

**方法1: 在Docker内启动HTTP服务**
```bash
cd /tmp/ego_planner_metrics && python3 -m http.server 8000
```
然后在宿主机浏览器访问: `http://localhost:8000/flight_report.html`

**方法2: 复制到宿主机查看**
```bash
# 在宿主机终端运行
docker ps  # 查看容器ID
docker cp <容器ID>:/tmp/ego_planner_metrics/ ~/ego_metrics/
# 然后用浏览器打开 ~/ego_metrics/flight_report.html
```

## 生成的文件

运行完成后会生成3个文件：

1. **evaluation_result.txt** - 文本评价报告
   - 总分（0-100）和等级（A+/A/B/C/D/F）
   - 各维度详细得分

2. **flight_analysis.png** - 性能图表（6个子图）
   - 速度曲线
   - 加速度曲线
   - 成功率趋势
   - 轨迹平滑度
   - 安全间隙
   - 重规划频率

3. **flight_report.html** - 交互式HTML报告
   - 关键指标卡片展示
   - 嵌入性能图表
   - 安全警告提示
   - 响应式设计，支持各种屏幕

## 评价指标说明

### 评分维度（总分100分）

1. **成功率** (20分)
   - 规划任务成功率
   - 100%成功率 = 满分

2. **优化效率** (20分)
   - 平均优化时间（越短越好）
   - 平均迭代次数（越少越好）

3. **安全性** (25分)
   - 最小安全间隙（>2m理想）
   - 碰撞次数（0次满分）

4. **平滑性** (15分)
   - 轨迹平滑度（数值越小越好）
   - 速度/加速度合理性

5. **效率** (20分)
   - 重规划频率（1-2Hz理想）
   - 最终代价函数值

### 评级标准

- **A+** (90-100): 优秀，飞行性能卓越
- **A**  (80-89):  很好，飞行性能良好
- **B**  (70-79):  良好，但仍有改进空间
- **C**  (60-69):  及格，需要优化
- **D**  (50-59):  较差，需要重点改进
- **F**  (<50):    不及格，存在严重问题

## 单独使用各程序

如果需要单独运行某个程序：

```bash
# 只做评价
/home/jude/anaconda3/bin/python3 evaluate_flight.py [csv文件路径]

# 只做可视化
/home/jude/anaconda3/bin/python3 visualize_flight.py [csv文件路径]
```

省略文件路径时自动使用最新的数据。

## 技术说明

- **数据记录**: 通过ROS节点实时订阅话题并保存
- **Python环境**: 使用 `/home/jude/anaconda3/bin/python3`（已安装pandas和matplotlib）
- **Docker兼容**: 所有可视化不依赖X11，生成HTML/PNG文件
- **自动化**: `analyze_flight.sh` 脚本整合完整流程

## 常见问题

**Q: 为什么没有数据？**
A: 确保运行了 `roslaunch ego_planner simple_run.launch` 并让无人机飞行一段时间

**Q: 如何查看历史飞行数据？**
A: CSV文件按时间戳命名，可指定文件路径运行评价程序

**Q: Docker内如何查看HTML？**
A: 启动HTTP服务器或复制文件到宿主机

**Q: 可以修改评分标准吗？**
A: 可以编辑 `evaluate_flight.py` 中的评分逻辑
