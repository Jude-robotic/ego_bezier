# 性能指标集成指南

## 已完成的工作

### 1. 创建的文件
- `src/planner/plan_manage/include/plan_manage/planner_metrics.h` - 指标头文件
- `src/planner/plan_manage/src/planner_metrics.cpp` - 指标实现
- `scripts/visualize_metrics.py` - Python实时可视化脚本

### 2. 已修改的文件
- `src/planner/plan_manage/CMakeLists.txt` - 添加了planner_metrics.cpp
- `src/planner/plan_manage/include/plan_manage/planner_manager.h` - 添加了metrics成员
- `src/planner/plan_manage/src/planner_manager.cpp` - 添加了头文件和初始化

## 手动集成步骤 (需要在bezier_optimizer和planner_manager中)

### 在 bezier_optimizer.cpp 中记录优化指标:

```cpp
// 在 rebound_optimize() 函数开始处:
auto opt_start = std::chrono::high_resolution_clock::now();

// 在优化完成后:
auto opt_end = std::chrono::high_resolution_clock::now();
double opt_time_ms = std::chrono::duration<double, std::milli>(opt_end - opt_start).count();

// 记录: 优化时间、迭代次数、最终代价
// (通过planner_manager传递到metrics_recorder)
```

### 在 planner_manager.cpp 的 reboundReplan() 函数中:

```cpp
bool EGOPlannerManager::reboundReplan(...) {
  // 1. 开始计时
  auto plan_start = std::chrono::high_resolution_clock::now();
  
  // 2. 执行规划...
  bool success = /* 规划逻辑 */;
  
  // 3. 计算耗时
  auto plan_end = std::chrono::high_resolution_clock::now();
  double plan_time = std::chrono::duration<double, std::milli>(plan_end - plan_start).count();
  
  // 4. 记录指标
  if (metrics_recorder_) {
    metrics_recorder_->recordPlanningAttempt(success);
    metrics_recorder_->recordOptimizationTime(plan_time);
    
    // 如果成功,记录轨迹质量
    if (success) {
      // 计算轨迹平滑度
      metrics_recorder_->recordTrajectorySmooth(control_points, time_step);
      
      // 计算安全距离
      double min_dist = /* 从grid_map计算 */;
      double avg_clear = /* 平均clearance */;
      metrics_recorder_->recordSafety(min_dist, avg_clear);
      
      // 记录动力学
      double max_vel = /* 最大速度 */;
      double max_acc = /* 最大加速度 */;
      bool vel_violated = max_vel > pp_.max_vel_;
      bool acc_violated = max_acc > pp_.max_acc_;
      metrics_recorder_->recordDynamics(max_vel, max_acc, vel_violated, acc_violated);
      
      // 计算路径长度
      double path_len = /* 沿轨迹积分 */;
      metrics_recorder_->recordPathLength(path_len);
    }
    
    // 保存到文件并发布可视化
    metrics_recorder_->saveMetricsToFile("");
    metrics_recorder_->publishMetricsVisualization();
  }
  
  return success;
}
```

## 快速集成代码片段

将以下代码添加到 `planner_manager.cpp` 的 `reboundReplan` 函数中:

