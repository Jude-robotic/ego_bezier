# 🚁 多机协同飞行系统 (Multi-UAV Swarm System)

[![Build Status](https://img.shields.io/badge/build-passing-brightgreen)]()
[![ROS Version](https://img.shields.io/badge/ROS-Noetic-blue)]()
[![License](https://img.shields.io/badge/license-MIT-green)]()

基于 **EGO-Planner-Bezier** 的多无人机协同飞行系统，实现主机规划+从机环绕跟随。

## ⚡ 快速开始

```bash
# 1. 编译
cd /home/jude/ego-planner-bezier
catkin_make && source devel/setup.bash

# 2. 启动
roslaunch ego_planner swarm_run.launch

# 3. 验证
rostopic list | grep uav
```

**5分钟内启动成功！** 详见 [快速启动指南](./markdown/SWARM_QUICK_START.md)

## 🎯 系统特点

- ✅ **主从架构**: 主机(uav_0)执行SLAM和规划，从机环绕跟随
- ✅ **独立命名空间**: 每个从机独立topic，无冲突，支持真实多机
- ✅ **灵活算法接口**: 易于实现和替换集群算法
- ✅ **完整闭环**: 从主机规划到从机轨迹生成全流程打通
- ✅ **丰富文档**: 详细的使用和开发文档

## 📊 系统架构

```
┌──────────────┐
│  主机 UAV_0  │  SLAM + 路径规划
│ EGO Planner  │──► /uav_0/planning/trajectory
└──────┬───────┘
       │
       ▼
┌──────────────────────┐
│ 集群轨迹生成器节点     │
│  ┌────────────────┐  │
│  │  集群算法接口   │  │ ◄─── 💡 在此实现算法
│  └────────────────┘  │
└──────┬───────────────┘
       │
       ├──► /uav_1/planning/trajectory
       ├──► /uav_2/planning/trajectory
       ├──► /uav_3/planning/trajectory
       └──► /uav_4/planning/trajectory
```

## 🛠️ 补充集群算法

### 当前状态
- ✅ 系统框架完整可运行
- ✅ 默认简单圆形编队算法
- 🔲 等待补充高级集群算法

### 补充步骤

1. **复制算法模板**
   ```bash
   cd src/planner/plan_manage/include/plan_manage
   cp custom_swarm_algorithm.h my_swarm_algorithm.h
   ```

2. **实现算法** - 编辑模板文件中的 `computeFollowerTrajectories()` 方法

3. **集成使用** - 在节点中设置您的算法

4. **测试验证** - 重新编译并运行

**详细说明**: [完整系统文档](./markdown/SWARM_SYSTEM_README.md#集群算法接口)

## 📚 文档导航

| 文档 | 说明 | 阅读时间 |
|-----|------|---------|
| [📋 项目总览](./SWARM_PROJECT_OVERVIEW.md) | 功能清单、架构说明 | 5分钟 |
| [🚀 快速启动](./markdown/SWARM_QUICK_START.md) | 编译、启动、测试 | 10分钟 |
| [📖 完整文档](./markdown/SWARM_SYSTEM_README.md) | 系统架构、接口、算法 | 30分钟 |
| [✅ 测试清单](./markdown/TEST_CHECKLIST.md) | 18项完整测试 | 20分钟 |
| [📊 实现报告](./markdown/IMPLEMENTATION_REPORT.md) | 技术细节、代码统计 | 15分钟 |
| [📂 文档索引](./markdown/DOC_INDEX.md) | 所有文档导航 | 3分钟 |

**从哪里开始？** 👉 [快速启动指南](./markdown/SWARM_QUICK_START.md)

## 🎬 运行示例

```bash
# 默认配置（4个从机，半径3米）
roslaunch ego_planner swarm_run.launch

# 自定义配置
roslaunch ego_planner swarm_run.launch \
    num_followers:=6 \
    formation_radius:=5.0 \
    altitude_offset:=1.0
```

## 📊 Topic接口

### 主机发布
- `/uav_0/planning/trajectory` - 主机轨迹 (nav_msgs/Path)
- `/uav_0/planning/bezier_trajectory` - Bezier轨迹

### 从机订阅
- `/uav_1/planning/trajectory` - 从机1轨迹
- `/uav_2/planning/trajectory` - 从机2轨迹  
- `/uav_3/planning/trajectory` - 从机3轨迹
- `/uav_4/planning/trajectory` - 从机4轨迹

**真实从机可直接订阅对应topic！**

## 🔧 系统要求

- ROS Noetic
- Ubuntu 20.04
- Eigen3
- PCL

## 📁 项目结构

```
ego-planner-bezier/
├── src/planner/plan_manage/
│   ├── include/plan_manage/
│   │   ├── swarm_trajectory_generator.h      # 集群轨迹生成器
│   │   ├── custom_swarm_algorithm.h          # 算法模板 💡
│   │   └── ego_replan_fsm.h                  # 主机FSM
│   ├── src/                                   # 实现文件
│   └── launch/swarm_run.launch               # 启动文件
├── markdown/                                  # 文档目录
├── scripts/verify_swarm_system.sh            # 验证脚本
└── SWARM_PROJECT_OVERVIEW.md                 # 项目总览
```

## ✅ 已验证功能

- [x] 系统编译成功
- [x] 主机轨迹发布正常
- [x] 从机轨迹生成正常
- [x] 独立topic命名空间
- [x] TF坐标系统正确
- [x] 参数配置生效
- [x] RViz可视化正常

## 🎯 下一步

1. **立即可做**: [启动系统](./markdown/SWARM_QUICK_START.md)，验证基本功能
2. **优先任务**: 实现您的[集群算法](./markdown/SWARM_SYSTEM_README.md#集群算法接口)
3. **后续扩展**: 添加真实从机执行器和安全机制

## 💡 核心文件

需要修改的核心文件：
- **算法实现**: `src/planner/plan_manage/include/plan_manage/custom_swarm_algorithm.h`
- **节点配置**: `src/planner/plan_manage/src/swarm_traj_generator_node.cpp`

其他文件已完成，无需修改！

## 🤝 贡献

欢迎贡献代码和改进建议！

## 📄 许可证

MIT License

---

**系统状态**: ✅ 就绪，可立即使用  
**当前版本**: v1.0  
**最后更新**: 2025年12月29日

**开始使用**: [快速启动指南](./markdown/SWARM_QUICK_START.md) | **完整文档**: [系统文档](./markdown/SWARM_SYSTEM_README.md)
