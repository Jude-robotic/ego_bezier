# EGO Planner Metrics 分析总结

生成时间: 2025-12-21

## 问题总结

### 1. 指标记录时间范围不正确 ❌
**问题**: 从系统启动就开始记录，而不是从无人机起飞到到达终点
- 当前记录了 220-270秒的数据
- 包含大量静止状态的数据
- 导致图表显示为"定值"

**解决方案**: ✅ 已提供改进的 `metrics_recorder_node.cpp`
- 添加飞行状态机 (IDLE → TAKING_OFF → FLYING → ARRIVED)
- 只在飞行期间记录数据
- 自动检测起飞和到达

### 2. 关键优化指标缺失 ❌
以下指标全部为 0 或初始值：
- ✗ `avg_opt_time_ms` = 0
- ✗ `avg_iterations` = 0  
- ✗ `avg_smoothness` = 0
- ✗ `path_length` = 0
- ✗ `final_cost` = 0
- ✗ `min_clearance` = 1e+10

**原因**: `metrics_recorder_node` 只订阅了 `/planning/pos_cmd`，没有订阅优化器内部信息

**解决方案**: 需要修改优化器代码，发布详细的优化结果

### 3. 速度/加速度数据异常 ⚠️
**现象**:
- 速度: 1.70-2.68 m/s (变化极小，几乎是常量)
- 加速度: 4.31-5.18 m/s² (变化极小)
- 重规划频率: 0.57842 Hz (完全不变)

**分析结果**:
```
速度统计:
  最大: 2.68 m/s
  最小: 1.70 m/s
  平均: 2.64 m/s
  变化范围: 0.98 m/s (仅37%变化)

加速度统计:
  最大: 5.18 m/s²
  最小: 4.31 m/s²
  平均: 5.15 m/s²
  变化范围: 0.87 m/s² (仅17%变化)
```

**可能原因**:
1. 记录的是规划目标值，而非实际飞行状态
2. 控制器参数设置不当，导致跟踪性能差
3. 重规划频率太低 (0.58 Hz ≈ 每1.7秒重规划一次)

## 无人机抖动和不连贯问题分析

### 根本原因推测

#### 1. 规划频率过低
- 当前: 0.58 Hz (每 1.7 秒规划一次)
- 建议: 2-5 Hz

**影响**: 
- 轨迹更新不及时
- 对环境变化响应慢
- 可能导致轨迹不连续

#### 2. 轨迹平滑度问题
- 当前无法评估 (smoothness = 0)
- Bezier 曲线的控制点可能设置不当
- 相邻轨迹片段可能不连续

#### 3. 控制器参数需要调整
可能的问题:
- 位置增益 (kp) 过高 → 过度响应
- 速度阻尼 (kd) 过低 → 震荡
- 控制频率与规划频率不匹配

### 改进建议

#### 短期改进 (立即可做)
1. **使用改进的 metrics_recorder_node**
   ```bash
   cp /tmp/improved_metrics_recorder_node.cpp \
      src/planner/plan_manage/src/metrics_recorder_node.cpp
   ```

2. **降低速度/加速度限制**
   ```yaml
   # 在参数文件中
   max_vel: 2.0  # 从 2.5 降低到 2.0
   max_acc: 2.5  # 从 3.0 降低到 2.5
   ```

3. **提高重规划频率**
   ```yaml
   replan_frequency: 2.0  # 从 0.58 提高到 2.0 Hz
   ```

#### 中期改进 (需要修改代码)
1. **添加优化结果发布**
   - 在优化器中发布详细信息
   - 包括: 优化时间、迭代次数、代价值、控制点

2. **记录轨迹跟踪误差**
   - 对比规划位置 vs 实际位置
   - 监控跟踪性能

3. **调整 Bezier 优化器参数**
   - 增加平滑度权重
   - 调整碰撞惩罚
   - 优化迭代次数上限

#### 长期改进 (系统级优化)
1. **实现自适应重规划**
   ```cpp
   if (tracking_error > threshold) {
       replan();  // 基于跟踪误差触发
   }
   ```

2. **添加轨迹连续性约束**
   - 确保相邻轨迹的速度/加速度连续
   - 使用热启动 (warm start)

3. **优化控制器参数**
   - 使用自动调参工具
   - 添加前馈控制

## 数据分析工具

### 已创建的工具

1. **简单分析脚本**: `/tmp/simple_analyze.sh`
   ```bash
   /tmp/simple_analyze.sh  # 分析最新CSV
   ```

2. **改进的 metrics_recorder**: `/tmp/improved_metrics_recorder_node.cpp`
   - 只记录飞行期间数据
   - 自动检测起飞/到达

3. **数据提取命令**: 在 `Jude_help_doc.md` 中

### 使用示例

```bash
# 1. 分析当前数据
/tmp/simple_analyze.sh

# 2. 提取特定时间段
awk -F',' 'NR==1 || ($1-start>=10 && $1-start<=50) {print} NR==2{start=$1}' \
  /tmp/ego_planner_metrics/metrics_*.csv > filtered.csv

# 3. 查看速度统计
awk -F',' 'NR>1 && $7>0 {print $7}' /tmp/ego_planner_metrics/metrics_*.csv | \
  awk '{sum+=$1; count++; if($1>max)max=$1; if(min==""||$1<min)min=$1} \
       END {print "max="max", min="min", avg="sum/count}'
```

## 下一步行动

### 立即执行
1. ✅ 查看改进指南: `METRICS_IMPROVEMENT_GUIDE.md`
2. ⬜ 应用改进的 metrics_recorder_node
3. ⬜ 重新运行测试并记录数据
4. ⬜ 使用分析工具查看改进效果

### 后续工作
1. ⬜ 调整控制器参数
2. ⬜ 提高重规划频率
3. ⬜ 修改优化器发布详细信息
4. ⬜ 添加轨迹跟踪误差监控

## 文件清单

创建的文件:
- ✅ `METRICS_IMPROVEMENT_GUIDE.md` - 详细改进指南
- ✅ `ANALYSIS_SUMMARY.md` - 本文件
- ✅ `/tmp/improved_metrics_recorder_node.cpp` - 改进的代码
- ✅ `/tmp/simple_analyze.sh` - 数据分析脚本
- ✅ `/tmp/planner_metrics_resetMetrics_patch.txt` - 补丁说明
- ✅ `/tmp/analyze_metrics.py` - Python分析脚本 (需要pandas)

更新的文件:
- ✅ `Jude_help_doc.md` - 添加了数据分析命令

## 参考文档

- 改进指南: `METRICS_IMPROVEMENT_GUIDE.md`
- 使用文档: `Jude_help_doc.md`
- 原始指标系统: `PERFORMANCE_METRICS_SYSTEM.md`
- 可视化指南: `VISUALIZATION_GUIDE.md`
