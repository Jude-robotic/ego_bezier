# 多机协同系统 (Multi-UAV Swarm System)

## 📋 系统概述

这是一个基于 **ego-planner-bezier** 的多无人机协同飞行系统。系统采用**主从架构**，由一个主机和多个从机组成：

- **主机 (uav_0)**: 执行完整的SLAM建图、路径规划和导航
- **从机 (uav_1~4)**: 接收集群算法生成的轨迹，实现环绕跟随

## 🏗️ 系统架构

```
┌─────────────────────────────────────────────────────────────┐
│                    多机协同系统架构                            │
└─────────────────────────────────────────────────────────────┘
    ┌──────────────┐
    │   主机 UAV_0  │  执行SLAM、建图、规划
    │              │
    │ EGO Planner  │──► /uav_0/planning/trajectory
    │   + Bezier   │──► /uav_0/planning/bezier_trajectory
    └──────┬───────┘
           │ 发布轨迹
           ▼
    ┌──────────────────────┐
    │  集群轨迹生成器节点     │
    │ SwarmTrajGenerator   │
    │                      │
    │  ┌────────────────┐  │
    │  │  集群算法接口    │  │ ◄─── 胡舜的集群算法
    │  │ (可替换/补充)    │  │
    │  └────────────────┘  │
    └──────┬───────────────┘
           │ 生成从机轨迹
           │
           ├──► /uav_1/planning/trajectory
           ├──► /uav_2/planning/trajectory
           ├──► /uav_3/planning/trajectory
           └──► /uav_4/planning/trajectory
                    │
                    ▼
            ┌────────────────┐
            │  从机执行器      │ (后续补充)
            │  (真实无人机)    │
            └────────────────┘
```

## 📁 核心文件

### 1. 集群轨迹生成器
```
src/planner/plan_manage/
├── include/plan_manage/
│   ├── swarm_trajectory_generator.h      # 集群轨迹生成器类
│   └── custom_swarm_algorithm.h          # 自定义算法模板
├── src/
│   ├── swarm_trajectory_generator.cpp    # 实现文件
│   └── swarm_traj_generator_node.cpp     # ROS节点
└── launch/
    └── swarm_run.launch                   # 多机协同启动文件
```

### 2. 主机节点修改
```
src/planner/plan_manage/
├── include/plan_manage/ego_replan_fsm.h  # 添加了轨迹发布器
└── src/ego_replan_fsm.cpp                 # 实现轨迹发布
```

## 🚀 快速开始

### 1. 编译系统

```bash
cd /home/jude/ego-planner-bezier
catkin_make
source devel/setup.bash
```

### 2. 启动多机协同系统

```bash
roslaunch ego_planner swarm_run.launch
```

### 3. 查看运行状态

```bash
# 查看所有topic
rostopic list | grep uav

# 查看主机轨迹
rostopic echo /uav_0/planning/trajectory

# 查看从机1的轨迹
rostopic echo /uav_1/planning/trajectory
```

## 📊 Topic接口

### 主机发布的Topic
| Topic | 类型 | 说明 |
|-------|------|------|
| `/uav_0/planning/trajectory` | `nav_msgs/Path` | 主机轨迹（Path格式） |
| `/uav_0/planning/bezier_trajectory` | `ego_planner/Bezier` | 主机轨迹（Bezier格式） |

### 从机订阅的Topic
| Topic | 类型 | 说明 |
|-------|------|------|
| `/uav_1/planning/trajectory` | `nav_msgs/Path` | 从机1轨迹 |
| `/uav_2/planning/trajectory` | `nav_msgs/Path` | 从机2轨迹 |
| `/uav_3/planning/trajectory` | `nav_msgs/Path` | 从机3轨迹 |
| `/uav_4/planning/trajectory` | `nav_msgs/Path` | 从机4轨迹 |

**重要特性**：
- ✅ 每个从机有独立的topic，避免命名空间冲突
- ✅ TF坐标系统正确配置（world frame）
- ✅ 真实无人机可直接订阅对应topic获取轨迹

## 🔧 集群算法接口

### 默认算法
系统默认使用 `SimpleCircleFormationAlgorithm` - 简单圆形编队：
- 从机在主机周围等角度分布
- 保持固定半径
- 可调整高度偏移

### 自定义算法步骤

#### 第1步：创建算法类

参考 `custom_swarm_algorithm.h` 模板，继承 `SwarmAlgorithmBase`：

```cpp
class MySwarmAlgorithm : public SwarmAlgorithmBase
{
public:
  std::vector<std::vector<Eigen::Vector3d>> computeFollowerTrajectories(
      const std::vector<Eigen::Vector3d> &leader_trajectory,
      const std::vector<Eigen::Vector3d> &leader_velocities,
      const std::vector<Eigen::Vector3d> &leader_accelerations,
      int num_followers,
      const std::vector<double> &time_stamps) override
  {
    // 在这里实现您的集群算法
    // ...
  }
};
```

#### 第2步：在节点中使用

修改 `swarm_traj_generator_node.cpp`：

```cpp
#include <plan_manage/custom_swarm_algorithm.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "swarm_trajectory_generator");
  ros::NodeHandle nh("~");

  ego_planner::SwarmTrajectoryGenerator swarm_generator(nh);
  
  // 创建您的算法实例
  ego_planner::MySwarmAlgorithm* my_algo = new ego_planner::MySwarmAlgorithm();
  
  // 设置算法
  swarm_generator.setSwarmAlgorithm(my_algo);
  
  swarm_generator.init();
  ros::spin();
  return 0;
}
```

#### 第3步：重新编译

```bash
cd /home/jude/ego-planner-bezier
catkin_make
```

### 算法接口说明

**输入参数**：
- `leader_trajectory`: 主机位置序列 `[Eigen::Vector3d]`
- `leader_velocities`: 主机速度序列 `[Eigen::Vector3d]`
- `leader_accelerations`: 主机加速度序列 `[Eigen::Vector3d]`
- `num_followers`: 从机数量 `int`
- `time_stamps`: 时间戳序列 `[double]`

**返回值**：
- `vector<vector<Eigen::Vector3d>>`: 
  - 第一维：从机索引 (0 ~ num_followers-1)
  - 第二维：轨迹点序列（与主机轨迹长度一致）

**算法设计建议**：
1. **编队形状**: 圆形、V字形、队列等
2. **动态调整**: 根据主机速度方向调整编队朝向
3. **避障考虑**: 让从机规避障碍物
4. **平滑轨迹**: 避免突然的方向变化

## ⚙️ 参数配置

在 `swarm_run.launch` 中配置：

```xml
<!-- 从机数量 -->
<arg name="num_followers" default="4"/>

<!-- 编队参数 -->
<arg name="formation_radius" default="3.0"/>  <!-- 编队半径(米) -->
<arg name="altitude_offset" default="0.5"/>   <!-- 高度偏移(米) -->
```

或在命令行指定：

```bash
roslaunch ego_planner swarm_run.launch num_followers:=6 formation_radius:=5.0
```

## 🔍 调试工具

### 1. 查看节点状态
```bash
rosnode list
rosnode info /swarm_traj_generator
```

### 2. 可视化轨迹
在RViz中添加：
- **主机轨迹**: `/uav_0/planning/trajectory` (Path)
- **从机轨迹**: `/uav_1/planning/trajectory` (Path)
- 设置不同颜色区分

### 3. 监控消息频率
```bash
rostopic hz /uav_0/planning/trajectory
rostopic hz /uav_1/planning/trajectory
```

### 4. 记录数据
```bash
rosbag record /uav_0/planning/trajectory /uav_1/planning/trajectory \
              /uav_2/planning/trajectory /uav_3/planning/trajectory \
              /uav_4/planning/trajectory
```

## 📝 待补充部分

### 🚧 集群算法（优先级：高）

**位置**: `src/planner/plan_manage/include/plan_manage/custom_swarm_algorithm.h`

**需要实现**:
- [ ] 复制模板文件创建自定义算法
- [ ] 实现 `computeFollowerTrajectories()` 方法
- [ ] 考虑编队类型（圆形/V字/队列/自适应）
- [ ] 考虑障碍物规避
- [ ] 考虑主机速度方向

**提示**: 模板文件已包含详细注释和示例代码

### 🚧 真实从机执行器（优先级：中）

**需要开发**:
- [ ] 创建从机轨迹订阅节点
- [ ] 实现轨迹转换为控制命令
- [ ] 配置真实无人机通信
- [ ] 测试TF坐标变换

### 🚧 安全机制（优先级：中）

**建议添加**:
- [ ] 从机间碰撞检测
- [ ] 从机与主机最小距离保持
- [ ] 通信丢失处理
- [ ] 紧急停止机制

## 🎯 系统优势

✅ **清晰的架构设计**: 主从分离，职责明确  
✅ **独立的topic命名空间**: 无冲突，易于扩展  
✅ **灵活的算法接口**: 易于替换和测试不同算法  
✅ **完整的闭环验证**: 可先在仿真中测试  
✅ **真实硬件友好**: 从机可直接订阅topic  

## 📚 相关文档

- [EGO Planner原理](./COMPARISON_WITH_ORIGINAL.md)
- [Bezier优化器说明](./COMPILATION_SUCCESS.md)
- [项目快速开始](./START_HERE.md)

## 🤝 贡献指南

当您实现了自定义集群算法后，欢迎：
1. 在 `custom_swarm_algorithm.h` 中添加您的算法
2. 更新此README的算法说明部分
3. 分享您的测试结果和参数配置

## 📧 问题反馈

如有问题或建议，请检查：
1. ROS节点是否正常运行
2. Topic是否正确发布/订阅
3. 参数配置是否合理
4. 日志输出是否有错误信息

---

**系统状态**: ✅ 已实现系统闭环，等待集群算法补充  
**最后更新**: 2025年12月29日
