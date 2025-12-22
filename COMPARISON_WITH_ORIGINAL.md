# Ego-Planner (B-Spline) vs Ego-Planner-Bezier 对比测试指南

本指南旨在帮助你量化评估 Bezier 曲线版本与原始 B-Spline 版本在**轨迹质量**和**计算效率**上的差异。

## 1. 测试环境统一 (控制变量)

为了保证公平，必须确保两个工程运行在完全相同的场景下。

1.  **地图与障碍物**: 确保两个工程的 `simple_run.launch` 中：
    *   `map_size` 相同 (例如 40x40x3)
    *   `obstacle_number` (或 `c_num`/`p_num`) 相同 (例如 200)
    *   `seed` (随机种子) 最好固定，或者进行多次测试取平均。
2.  **起点与终点**: 使用 `flight_type = 2` (Waypoints模式) 并设置完全相同的 `point_num` 和坐标。
3.  **动力学限制**: `max_vel` 和 `max_acc` 必须一致。

## 2. 数据采集 (Rosbag)

我们需要录制两个工程的规划指令和里程计数据。

### 步骤 A: 录制原始 Ego-Planner
在原始工程目录下运行：
```bash
# 终端 1
roslaunch ego_planner simple_run.launch

# 终端 2 (开始录制)
rosbag record /planning/pos_cmd /visual_slam/odom -O original_ego.bag
```
执行飞行任务，完成后停止录制。

### 步骤 B: 录制 Ego-Planner-Bezier
在你当前的工程目录下运行：
```bash
# 终端 1
roslaunch ego_planner simple_run.launch

# 终端 2 (开始录制)
rosbag record /planning/pos_cmd /visual_slam/odom -O bezier_ego.bag
```
执行相同的飞行任务，完成后停止录制。

## 3. 数据分析

我已经为你准备了一个自动分析脚本 `compare_planners.py` (位于项目根目录)。

使用方法：
```bash
# 确保已安装依赖
pip3 install matplotlib scipy

# 运行对比脚本
./compare_planners.py original_ego.bag bezier_ego.bag
```

脚本运行后会：
1.  在终端输出详细的指标对比（时间、长度、平滑度）。
2.  生成 `comparison_result.png` 图表，直观展示 Jerk 曲线和各项指标差异。

## 4. 核心对比指标

我们将从以下维度进行对比：

| 维度 | 指标 | 说明 | 预期差异 |
| :--- | :--- | :--- | :--- |
| **轨迹平滑度** | **平均加加速度 (Mean Jerk)** | $\int |j|^2 dt$，反映控制输入的剧烈程度 | B-Spline 通常更平滑，Bezier 取决于连续性约束的实现 |
| **轨迹平滑度** | **最大加加速度 (Max Jerk)** | 峰值冲击 | 越小越好 |
| **路径效率** | **轨迹长度 (Path Length)** | 总飞行距离 | 两者应相近，过长说明走了弯路 |
| **计算效率** | **优化耗时 (Opt Time)** | 每次重规划的计算时间 | **重点关注**。Bezier 的控制点数量通常多于 B-Spline，可能更慢 |
| **控制刚度** | **位置误差 (Tracking Error)** | `pos_cmd` 与 `odom` 的偏差 | 反映轨迹是否超出控制器能力 |

## 4. 改进方向 (基于代码分析)

### 发现的问题：软约束 vs 硬约束
在你的 `bezier_optimizer.cpp` 中，我发现了以下代码：
```cpp
double cont_weight_vel = 10000.0;  // Velocity continuity
double cont_weight_acc = 10000.0;  // Acceleration continuity
```
这说明你使用了**罚函数法 (Penalty Method)** 来强行拉近分段 Bezier 曲线连接处的速度和加速度。

*   **原始 Ego-Planner (B-Spline)**: B-Spline 的数学特性决定了它**天然满足** $C^2$ 或 $C^3$ 连续性（只要节点向量均匀）。它不需要在优化函数里加连续性惩罚项。
*   **你的 Bezier 版本**: 你必须消耗优化器的“算力”去满足连续性。如果权重不够大，轨迹会断裂；如果权重太大，优化会变慢且容易陷入局部极小值。

### 建议改进方案：
1.  **改为硬约束 (Hard Constraint)**: 不要在 Cost Function 里加惩罚，而是通过数学推导，将第 $i+1$ 段的前两个控制点直接用第 $i$ 段的后两个控制点表示。这样优化变量会减少，且天然保证连续。
2.  **或者使用 "Clamped B-Spline"**: 实际上，多段 Bezier 曲线等价于非均匀节点的 B-Spline。你可以直接复用 B-Spline 的代码结构，只是改变节点向量 (Knot Vector) 的定义，这样既保留了 Bezier 的直观性，又利用了 B-Spline 的连续性优势。
