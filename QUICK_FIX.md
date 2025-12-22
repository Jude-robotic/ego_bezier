# 快速修复指南

## 问题1: 指标记录时间不对（全程记录而非飞行期间）

### 解决方案
```bash
cd /home/jude/ego-planner-bezier

# 1. 备份
cp src/planner/plan_manage/src/metrics_recorder_node.cpp \
   src/planner/plan_manage/src/metrics_recorder_node.cpp.backup

# 2. 替换为改进版本
cp /tmp/improved_metrics_recorder_node.cpp \
   src/planner/plan_manage/src/metrics_recorder_node.cpp

# 3. 添加 resetMetrics() 函数
# 编辑 src/planner/plan_manage/include/plan_manage/planner_metrics.h
# 在 public 部分添加: void resetMetrics();

# 编辑 src/planner/plan_manage/src/planner_metrics.cpp
# 添加实现（详见 METRICS_IMPROVEMENT_GUIDE.md）

# 4. 编译
catkin_make

# 5. 测试
roslaunch ego_planner simple_run.launch
# 另一个终端:
rosrun ego_planner metrics_recorder_node
```

## 问题2: 查看和分析数据

### 快速分析
```bash
/tmp/simple_analyze.sh
```

### 查看原始数据
```bash
# 最新的CSV
ls -lht /tmp/ego_planner_metrics/

# 查看前20行
head -20 /tmp/ego_planner_metrics/metrics_*.csv

# 查看后20行
tail -20 /tmp/ego_planner_metrics/metrics_*.csv
```

### 提取特定时间段
```bash
# 提取10-50秒的数据
awk -F',' 'NR==1 || ($1-start>=10 && $1-start<=50) {print} NR==2{start=$1}' \
  /tmp/ego_planner_metrics/metrics_20251221_052306.csv > filtered.csv
```

### 统计分析
```bash
# 速度统计
awk -F',' 'NR>1 && $7>0 {sum+=$7; count++; if($7>max)max=$7; if(min==""||$7<min)min=$7} \
     END {print "速度: max="max", min="min", avg="sum/count}' \
  /tmp/ego_planner_metrics/metrics_*.csv

# 加速度统计
awk -F',' 'NR>1 && $8>0 {sum+=$8; count++; if($8>max)max=$8; if(min==""||$8<min)min=$8} \
     END {print "加速度: max="max", min="min", avg="sum/count}' \
  /tmp/ego_planner_metrics/metrics_*.csv
```

## 问题3: 无人机抖动/不连贯

### 临时解决（调参）
编辑 `src/planner/plan_env/launch/rviz.launch` 或参数文件:

```yaml
# 降低速度限制
max_vel: 2.0  # 从 2.5 降到 2.0

# 降低加速度限制  
max_acc: 2.5  # 从 3.0 降到 2.5

# 提高重规划频率
replan_thresh: 0.5  # 调整阈值
```

### 检查当前参数
```bash
# 查看当前运行的参数
rosparam list | grep ego_planner
rosparam get /ego_planner_node/planning/max_vel
rosparam get /ego_planner_node/planning/max_acc
```

## 常用命令速查

| 操作 | 命令 |
|------|------|
| 分析数据 | `/tmp/simple_analyze.sh` |
| 查看CSV | `cat /tmp/ego_planner_metrics/metrics_*.csv \| less` |
| 提取列 | `awk -F',' '{print $7, $8}' file.csv` |
| 查看图片 | `eog /tmp/ego_planner_metrics/metrics_plot.png` |
| 查看话题 | `rostopic list \| grep planning` |
| 回放数据 | `rosbag play your_bag.bag` |

## 文档索引

- 🔧 **详细改进**: `METRICS_IMPROVEMENT_GUIDE.md`
- 📊 **分析总结**: `ANALYSIS_SUMMARY.md`  
- 📝 **使用文档**: `Jude_help_doc.md`
- ⚡ **本文档**: `QUICK_FIX.md`

## 问题排查

### CSV文件为空或很小
```bash
# 检查 metrics_recorder_node 是否运行
rosnode list | grep metrics

# 检查是否有数据发布
rostopic hz /planning/pos_cmd
rostopic hz /odom_world
```

### 改进后仍然记录全程
- 检查是否成功编译: `catkin_make --pkg ego_planner`
- 查看日志: `rosnode info /metrics_recorder_node`
- 确认使用了新版本节点

### 无法查看图片
```bash
# Docker内
eog /tmp/ego_planner_metrics/metrics_plot.png

# 或复制到主机
docker cp <container_id>:/tmp/ego_planner_metrics/metrics_plot.png ~/
```

---
生成时间: 2025-12-21
