# 编译错误修复说明

## 问题描述
在添加性能指标系统后，`catkin_make` 编译失败，出现多个编译错误。

## 错误原因

1. **命名空间语法错误**: 在 `planner_manager.h` 中添加前向声明时，破坏了 `namespace ego_planner` 的大括号结构
2. **未删除的成员变量**: 在 `planner_manager.h` 中保留了 `metrics_recorder_` 成员变量声明，但没有对应的头文件
3. **不必要的头文件包含**: 在 `planner_manager.cpp` 中包含了 `planner_metrics.h` 和 `<chrono>`
4. **未删除的初始化代码**: 在 `planner_manager.cpp` 中残留了 `metrics_recorder_` 的初始化代码
5. **链接错误**: `metrics_recorder_node` 的 CMakeLists.txt 中缺少 `planner_metrics.cpp`
6. **类型定义错误**: `metrics_recorder_node.cpp` 中使用了不存在的 `Ptr` 类型别名

## 修复内容

### 1. 修复 planner_manager.h
```bash
# 删除错误的前向声明
# 修复命名空间大括号
# 删除 metrics_recorder_ 成员变量
```

**修改前**:
```cpp
namespace ego_planner
  class PlannerMetricsRecorder;  // Forward declaration
{
    GridMap::Ptr grid_map_;
    std::shared_ptr<PlannerMetricsRecorder> metrics_recorder_;
```

**修改后**:
```cpp
namespace ego_planner
{
    GridMap::Ptr grid_map_;
```

### 2. 修复 planner_manager.cpp
```bash
# 删除 #include <plan_manage/planner_metrics.h>
# 删除 #include <chrono>
# 删除 metrics_recorder_ 初始化代码
# 删除 Metrics 相关注释和变量
```

**删除的代码**:
- `#include <plan_manage/planner_metrics.h>`
- `#include <chrono>`
- `metrics_recorder_.reset(new PlannerMetricsRecorder(nh));`
- `ROS_INFO("\033[32m[Metrics] Performance monitoring enabled\033[0m");`
- `auto plan_start = std::chrono::high_resolution_clock::now();`
- `bool plan_success = false;`

### 3. 修复 metrics_recorder_node.cpp
**修改前**:
```cpp
PlannerMetricsRecorder::Ptr metrics_recorder_;
```

**修改后**:
```cpp
std::shared_ptr<PlannerMetricsRecorder> metrics_recorder_;
```

### 4. 修复 CMakeLists.txt
**修改前**:
```cmake
add_executable(metrics_recorder_node
  src/metrics_recorder_node.cpp
)
```

**修改后**:
```cmake
add_executable(metrics_recorder_node
  src/metrics_recorder_node.cpp
  src/planner_metrics.cpp
)
```

## 编译结果

✅ **编译成功！**

生成的可执行文件:
- `devel/lib/ego_planner/ego_planner_node` (13MB)
- `devel/lib/ego_planner/metrics_recorder_node` (4.5MB)

仅有2个编译警告（类型比较），不影响运行：
```
warning: comparison of integer expressions of different signedness: 
'std::deque<double>::size_type' {aka 'long unsigned int'} and 
'const int' [-Wsign-compare]
```

## 使用说明

现在可以正常使用性能指标系统了：

```bash
# 1. 启动 ego-planner
roslaunch ego_planner simple_run.launch

# 2. 启动指标记录节点（新终端）
source devel/setup.bash
rosrun ego_planner metrics_recorder_node

# 3. 启动可视化（新终端）
python3 scripts/visualize_metrics.py
```

详细使用说明请查看:
- `README_METRICS.txt` - 快速总览
- `QUICK_START_METRICS.md` - 快速入门
- `PERFORMANCE_METRICS_SYSTEM.md` - 完整文档

## 注意事项

`ego_planner_node` 本身**不包含**指标记录功能，指标系统作为**独立节点**运行：
- ✅ 优点: 不修改原代码，解耦设计
- ✅ 优点: 可以独立开关，不影响主节点
- ℹ️ 说明: 通过订阅ROS话题收集数据，而非直接集成到代码中


---

## 可视化脚本修复 (2025-12-21)

### 问题描述
运行 `python3 scripts/visualize_metrics.py` 时报错：
```
ModuleNotFoundError: No module named 'tkinter'
```

### 根本原因
matplotlib 默认使用 TkAgg 后端，需要 tkinter 模块。在 Docker 容器中通常未安装 tkinter。

### 解决方案

#### 1. 安装 python3-tk（可选）
```bash
sudo apt-get update
sudo apt-get install -y python3-tk
```

#### 2. 修改脚本使用 Agg 后端（主要修复）
在 `scripts/visualize_metrics.py` 中添加：
```python
import matplotlib
matplotlib.use('Agg')  # 必须在 import pyplot 之前
```

#### 3. 添加文件保存模式
新增功能：
- `--save` 参数：保存图表到文件而不是显示
- `--output` 参数：自定义输出路径
- 默认输出：`/tmp/ego_planner_metrics/metrics_plot.png`
- 自动更新：每收到 10 个数据点更新一次

### 使用方法

**Docker 环境（推荐）**：
```bash
python3 scripts/visualize_metrics.py --save
```

**有图形界面的系统**：
```bash
python3 scripts/visualize_metrics.py
```

**自定义输出路径**：
```bash
python3 scripts/visualize_metrics.py --save --output ~/my_plot.png
```

### 验证修复
```bash
# 检查脚本语法
python3 -m py_compile scripts/visualize_metrics.py

# 查看帮助信息
python3 scripts/visualize_metrics.py --help

# 测试运行（需要 ROS master）
python3 scripts/visualize_metrics.py --save
```

### 相关文档
- `VISUALIZATION_GUIDE.md` - 可视化详细使用指南
- `QUICK_START_METRICS.md` - 更新了 Docker 环境的启动步骤

