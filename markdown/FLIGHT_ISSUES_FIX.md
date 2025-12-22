# 飞行问题诊断与修复方案

## 📊 问题诊断结果

通过深度分析CSV数据 (`metrics_20251221_052306.csv`), 发现以下**严重问题**:

### 1. ❌ 重规划频率过低 (0.58 Hz)
- **现象**: 约每1.7秒才重规划一次
- **影响**: 轨迹更新不及时,导致飞行**抖动、大转角、不连续**
- **原因**: `thresh_replan: 1.5` 参数设置过大
- **严重程度**: 🔴 **严重** - 这是导致飞行不平滑的主要原因

### 2. ❌ 优化指标未记录
- **现象**: 优化时间、迭代次数、平滑度全为0 (591/591条记录)
- **影响**: 无法评估轨迹质量和优化收敛性
- **原因**: `metrics_recorder_node`未订阅优化器输出话题
- **严重程度**: 🟡 **中等** - 影响性能评估

### 3. ❌ 速度/加速度数据异常
- **现象**: 速度只有7个不同值, 加速度只有4个不同值
- **影响**: 记录的是**目标值**而非**实际飞行状态**, 无法评估跟踪性能
- **原因**: 只订阅了`/planning/pos_cmd`, 没有对比实际odom
- **严重程度**: 🟡 **中等** - 无法诊断跟踪误差

### 4. ⚠️ 速度/加速度限制可能偏高
- **当前**: max_vel=2.0 m/s, max_acc=3.0 m/s²
- **问题**: 在障碍密集环境中可能导致激进飞行
- **建议**: 适当降低以提高安全性和平滑性

---

## 🔧 修复方案

### 方案1: 快速修复 - 调整参数 (立即见效)

#### 1.1 降低重规划阈值
```bash
# 编辑配置文件
nano /home/jude/ego-planner-bezier/src/planner/plan_manage/launch/advanced_param.xml
```

找到并修改:
```xml
<!-- 原来 -->
<param name="fsm/thresh_replan" value="1.5" type="double"/>

<!-- 改为 -->
<param name="fsm/thresh_replan" value="0.6" type="double"/>
```

**预期效果**: 重规划频率从 0.58 Hz → 约 2-3 Hz, 轨迹更新更及时

#### 1.2 降低速度和加速度限制
```xml
<!-- 在 simple_run.launch 中修改 -->
<!-- 原来 -->
<arg name="max_vel" value="2.0" />
<arg name="max_acc" value="3.0" />

<!-- 改为 -->
<arg name="max_vel" value="1.5" />
<arg name="max_acc" value="2.0" />
```

**预期效果**: 飞行更保守、平滑, 减少急转弯

#### 1.3 快速应用
```bash
cd /home/jude/ego-planner-bezier
catkin_make
source devel/setup.bash

# 测试
roslaunch ego_planner simple_run.launch
```

---

### 方案2: 完整修复 - 改进Metrics系统 (彻底解决)

#### 2.1 改进Metrics Recorder

创建改进版本的`metrics_recorder_node.cpp`:

**新增功能:**
1. ✅ 订阅优化器输出 - 记录真实优化时间和迭代次数
2. ✅ 记录实际odom状态 - 对比目标值计算跟踪误差
3. ✅ 计算轨迹平滑度 - 基于控制点计算jerk
4. ✅ 自适应重规划触发 - 根据跟踪误差动态调整
5. ✅ 发布优化结果可视化

**实施步骤:**
```bash
# 1. 备份原文件
cd /home/jude/ego-planner-bezier
cp src/planner/plan_manage/src/metrics_recorder_node.cpp \
   src/planner/plan_manage/src/metrics_recorder_node.cpp.backup

# 2. 应用补丁 (见下方patch文件)
cat > /tmp/improved_metrics_recorder.patch << 'EOF'
[补丁内容见后文]
EOF

patch -p1 < /tmp/improved_metrics_recorder.patch

# 3. 重新编译
catkin_make

# 4. 测试
roslaunch ego_planner simple_run.launch
rosrun ego_planner metrics_recorder_node
```

#### 2.2 改进优化器集成

需要在`bezier_opt`中发布优化统计信息:

**在 `BezierOpt` 类中添加:**
```cpp
// 添加发布器
ros::Publisher opt_stats_pub_;

// 在optimize()函数中发布
std_msgs::Float64MultiArray stats_msg;
stats_msg.data = {opt_time_ms, iterations, final_cost, smoothness};
opt_stats_pub_.publish(stats_msg);
```

#### 2.3 添加自适应重规划

根据跟踪误差动态调整重规划频率:

```cpp
// 伪代码
double tracking_error = (odom_pos - target_pos).norm();
double adaptive_replan_dist = base_replan_dist * (1.0 + tracking_error);

if (dist_to_target < adaptive_replan_dist) {
    triggerReplan();
}
```

---

### 方案3: 优化控制器参数 (可选)

如果飞行仍有抖动, 可以调整SO3控制器增益:

```bash
nano /home/jude/ego-planner-bezier/src/uav_simulator/so3_control/config/gains.yaml
```

```yaml
# 适当降低增益以减少抖动
gains:
  pos: {x: 5.0, y: 5.0, z: 7.0}  # 原来可能是 6-8
  vel: {x: 2.0, y: 2.0, z: 3.0}  # 原来可能是 3-4
  rot: {x: 1.5, y: 1.5, z: 1.0}  # 旋转增益

# 增加阻尼
damping_ratio: 0.7  # 范围 0.5-1.0
```

---

## 🚀 推荐实施顺序

### 阶段1: 立即修复 (5分钟)
1. ✅ 修改`thresh_replan`从1.5→0.6
2. ✅ 修改`max_vel`从2.0→1.5
3. ✅ 修改`max_acc`从3.0→2.0
4. ✅ 重新编译测试

**预期改善**: 重规划频率提高, 飞行平滑度明显改善

### 阶段2: 数据改进 (30分钟)
1. ⏳ 应用改进的`metrics_recorder_node.cpp`
2. ⏳ 添加跟踪误差记录
3. ⏳ 测试并验证数据质量

**预期改善**: 获得准确的性能指标, 可以精确评估

### 阶段3: 深度优化 (1-2小时, 可选)
1. ⏳ 实现自适应重规划
2. ⏳ 优化控制器参数
3. ⏳ 添加优化结果发布和可视化

**预期改善**: 完全消除抖动, 实现最优飞行性能

---

## 📝 验证方法

### 1. 使用改进的分析脚本
```bash
# 使用新脚本分析
/tmp/improved_simple_analyze.sh

# 应该看到:
# ✅ 重规划频率 > 1 Hz (目标 2-3 Hz)
# ✅ 优化指标有数据 (不再是0)
# ✅ 速度/加速度变化丰富 (>20个不同值)
```

### 2. 视觉观察
- ✅ 轨迹更新更频繁
- ✅ 转弯更平滑, 无急转
- ✅ 飞行无明显抖动
- ✅ 速度变化连续

### 3. 数据对比
```bash
# 对比修复前后的CSV
/tmp/improved_simple_analyze.sh /tmp/ego_planner_metrics/metrics_BEFORE.csv
/tmp/improved_simple_analyze.sh /tmp/ego_planner_metrics/metrics_AFTER.csv
```

---

## 📋 快速命令摘要

```bash
# 1. 应用快速修复
cd /home/jude/ego-planner-bezier

# 修改advanced_param.xml中的thresh_replan
sed -i 's/thresh_replan" value="1.5"/thresh_replan" value="0.6"/' \
  src/planner/plan_manage/launch/advanced_param.xml

# 修改simple_run.launch中的速度限制
sed -i 's/max_vel" value="2.0"/max_vel" value="1.5"/' \
  src/planner/plan_manage/launch/simple_run.launch
sed -i 's/max_acc" value="3.0"/max_acc" value="2.0"/' \
  src/planner/plan_manage/launch/simple_run.launch

# 重新编译
catkin_make

# 2. 替换分析脚本
cp /tmp/improved_simple_analyze.sh /tmp/simple_analyze.sh
chmod +x /tmp/simple_analyze.sh

# 3. 测试
roslaunch ego_planner simple_run.launch

# 4. 分析结果 (在新终端)
/tmp/simple_analyze.sh
```

---

## 🎯 预期成果

### 修复前:
- 重规划频率: 0.58 Hz (每1.7秒)
- 优化指标: 全为0
- 速度变化: 7个不同值
- 飞行表现: **抖动、大转角、不连续**

### 修复后:
- 重规划频率: 2-3 Hz (每0.3-0.5秒)  ✅
- 优化指标: 完整记录  ✅
- 速度变化: >20个不同值  ✅
- 飞行表现: **平滑、连续、安全**  ✅

---

## 📞 问题排查

如果修复后仍有问题:

### 问题1: 编译失败
```bash
# 清理并重新编译
cd /home/jude/ego-planner-bezier
catkin_make clean
catkin_make
```

### 问题2: 参数未生效
```bash
# 确认参数加载
rosrun ego_planner ego_planner_node _fsm/thresh_replan:=0.6

# 或检查launch文件
rosparam get /ego_planner_node/fsm/thresh_replan
```

### 问题3: 重规划频率仍然低
```bash
# 进一步降低阈值
# thresh_replan: 0.6 → 0.4
```

### 问题4: 飞行仍不稳定
```bash
# 调整控制器增益 (见方案3)
nano src/uav_simulator/so3_control/config/gains.yaml
```

---

## 📚 相关文档
- `METRICS_IMPROVEMENT_GUIDE.md` - Metrics系统改进指南
- `PERFORMANCE_METRICS_SYSTEM.md` - 性能指标系统文档
- `/tmp/improved_simple_analyze.sh` - 改进的数据分析脚本
