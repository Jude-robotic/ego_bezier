# 多机协同系统 - 项目总览

## 🎯 项目目标

实现主机(uav_0)执行核心SLAM建图和路径规划，从机(uav_1~4)通过集群算法环绕跟随的多无人机协同系统。

## ✅ 已完成功能

### 1. 系统架构 ✅
- [x] 主从机分离设计
- [x] 独立的topic命名空间（避免冲突）
- [x] 清晰的模块化结构

### 2. 主机功能 ✅
- [x] 完整的EGO-Planner-Bezier规划器
- [x] 轨迹发布到 `/uav_0/planning/trajectory`
- [x] Bezier轨迹发布到 `/uav_0/planning/bezier_trajectory`

### 3. 集群轨迹生成器 ✅
- [x] 独立节点 `swarm_traj_generator_node`
- [x] 订阅主机轨迹
- [x] 为每个从机生成独立轨迹
- [x] 发布到独立topic：`/uav_1~/uav_4/planning/trajectory`

### 4. 集群算法接口 ✅
- [x] 清晰的算法基类 `SwarmAlgorithmBase`
- [x] 默认实现：`SimpleCircleFormationAlgorithm`（圆形编队）
- [x] 详细的自定义算法模板
- [x] 易于替换和扩展

### 5. 系统配置 ✅
- [x] Launch文件：`swarm_run.launch`
- [x] 可配置参数（从机数量、编队半径、高度偏移）
- [x] 完整的编译配置（CMakeLists.txt）

### 6. 文档 ✅
- [x] 系统架构文档
- [x] 快速启动指南
- [x] 算法接口说明
- [x] 故障排查指南

## 🔲 待补充部分

### 1. 集群算法（优先级：高）⭐

**当前状态**: 使用默认简单圆形编队算法  
**需要做**: 实现您的自定义集群算法

**位置**: `src/planner/plan_manage/include/plan_manage/custom_swarm_algorithm.h`

**文档**: 详见 [SWARM_SYSTEM_README.md](./markdown/SWARM_SYSTEM_README.md#集群算法接口)

**建议算法方向**:
- 圆形编队（已有默认实现）
- V字编队
- 队列编队
- 自适应编队（根据障碍物动态调整）

### 2. 真实从机执行器（优先级：中）

**需要开发**:
- 从机轨迹订阅节点
- 轨迹转换为控制命令
- 真实无人机通信接口

### 3. 安全机制（优先级：中）

**建议添加**:
- 从机间碰撞检测
- 与主机最小距离保持
- 通信丢失处理
- 紧急停止机制

## 📂 项目结构

```
ego-planner-bezier/
├── src/planner/plan_manage/
│   ├── include/plan_manage/
│   │   ├── swarm_trajectory_generator.h      # 集群轨迹生成器
│   │   ├── custom_swarm_algorithm.h          # 自定义算法模板 ⭐
│   │   └── ego_replan_fsm.h                  # 主机FSM（已修改）
│   ├── src/
│   │   ├── swarm_trajectory_generator.cpp    # 实现
│   │   ├── swarm_traj_generator_node.cpp     # ROS节点
│   │   └── ego_replan_fsm.cpp                # 主机FSM（已修改）
│   ├── launch/
│   │   └── swarm_run.launch                   # 多机协同启动文件
│   └── CMakeLists.txt                         # 已更新
├── markdown/
│   ├── SWARM_SYSTEM_README.md                 # 完整系统文档
│   └── SWARM_QUICK_START.md                   # 快速启动指南
└── SWARM_PROJECT_OVERVIEW.md                  # 本文件
```

## 🚀 快速开始

### 编译
```bash
cd /home/jude/ego-planner-bezier
catkin_make
source devel/setup.bash
```

### 运行
```bash
roslaunch ego_planner swarm_run.launch
```

### 验证
```bash
# 查看topic
rostopic list | grep uav

# 查看主机轨迹
rostopic echo /uav_0/planning/trajectory

# 查看从机轨迹
rostopic echo /uav_1/planning/trajectory
```

**详细步骤**: 见 [SWARM_QUICK_START.md](./markdown/SWARM_QUICK_START.md)

## 📊 系统接口

### Topic接口

| Topic | 类型 | 说明 |
|-------|------|------|
| `/uav_0/planning/trajectory` | `nav_msgs/Path` | 主机轨迹（发布） |
| `/uav_0/planning/bezier_trajectory` | `ego_planner/Bezier` | 主机Bezier轨迹（发布） |
| `/uav_1/planning/trajectory` | `nav_msgs/Path` | 从机1轨迹（发布） |
| `/uav_2/planning/trajectory` | `nav_msgs/Path` | 从机2轨迹（发布） |
| `/uav_3/planning/trajectory` | `nav_msgs/Path` | 从机3轨迹（发布） |
| `/uav_4/planning/trajectory` | `nav_msgs/Path` | 从机4轨迹（发布） |

### 参数配置

| 参数 | 默认值 | 说明 |
|-----|-------|------|
| `num_followers` | 4 | 从机数量 |
| `formation_radius` | 3.0 | 编队半径（米） |
| `altitude_offset` | 0.5 | 高度偏移（米） |

## 🎓 补充集群算法指南

### 步骤1: 复制模板
```bash
cd src/planner/plan_manage/include/plan_manage
cp custom_swarm_algorithm.h my_swarm_algorithm.h
```

### 步骤2: 实现算法

编辑 `my_swarm_algorithm.h`，修改类名并实现：

```cpp
class MySwarmAlgorithm : public SwarmAlgorithmBase
{
  std::vector<std::vector<Eigen::Vector3d>> computeFollowerTrajectories(
      const std::vector<Eigen::Vector3d> &leader_trajectory,
      const std::vector<Eigen::Vector3d> &leader_velocities,
      const std::vector<Eigen::Vector3d> &leader_accelerations,
      int num_followers,
      const std::vector<double> &time_stamps) override
  {
    // 在这里实现您的算法
    // ...
  }
};
```

### 步骤3: 修改节点

编辑 `src/swarm_traj_generator_node.cpp`:

```cpp
#include <plan_manage/my_swarm_algorithm.h>

int main(int argc, char **argv)
{
  // ...
  ego_planner::MySwarmAlgorithm* my_algo = 
      new ego_planner::MySwarmAlgorithm();
  swarm_generator.setSwarmAlgorithm(my_algo);
  // ...
}
```

### 步骤4: 重新编译测试
```bash
catkin_make
roslaunch ego_planner swarm_run.launch
```

## 📖 详细文档

- **系统架构**: [SWARM_SYSTEM_README.md](./markdown/SWARM_SYSTEM_README.md)
- **快速启动**: [SWARM_QUICK_START.md](./markdown/SWARM_QUICK_START.md)
- **算法接口**: [SWARM_SYSTEM_README.md#集群算法接口](./markdown/SWARM_SYSTEM_README.md#集群算法接口)

## ✨ 系统特点

1. **清晰的架构**: 主从分离，职责明确
2. **独立的命名空间**: 避免topic冲突，支持真实多机
3. **灵活的算法接口**: 易于替换和测试不同算法
4. **完整的闭环**: 可先在仿真中验证，再部署到真机
5. **丰富的文档**: 从架构到使用都有详细说明

## 🎯 当前状态

✅ **系统已实现完整闭环**  
✅ **可以运行和测试**  
🔲 **等待补充高级集群算法**

## 🤝 下一步行动

1. **立即可做**: 启动系统，验证基本功能
2. **优先补充**: 实现您的自定义集群算法
3. **后续扩展**: 添加真实从机执行器和安全机制

---

**项目状态**: ✅ 核心框架完成，可开始算法研发  
**最后更新**: 2025年12月29日

**开始使用**: 阅读 [快速启动指南](./markdown/SWARM_QUICK_START.md)
