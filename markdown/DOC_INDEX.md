# 多机协同系统 - 文档导航

## 🚀 快速开始

**您是第一次使用？从这里开始 👇**

1. **[项目总览](./SWARM_PROJECT_OVERVIEW.md)** ⭐
   - 了解系统架构和功能
   - 查看完成情况和待补充部分
   - 预计5分钟阅读

2. **[快速启动指南](./SWARM_QUICK_START.md)** ⭐⭐⭐
   - 编译、启动、验证系统
   - 参数配置和调试
   - 预计10分钟完成启动

3. **[测试检查清单](./TEST_CHECKLIST.md)** ⭐⭐
   - 18项完整测试步骤
   - 验证系统功能
   - 预计20分钟完成所有测试

4. **[阶段6联调验收模板](./SWARM_STAGE6_INTEGRATION_ACCEPTANCE.md)** ⭐⭐⭐
  - 主从编队 + 通信可视化联调模板
  - 现场勾选式验收清单
  - 含后续启用主动避障的最小改动说明

5. **[阶段6验收记录样例（已填）](./SWARM_STAGE6_ACCEPTANCE_SAMPLE.md)** ⭐⭐
  - 可直接复制复用的联调记录样例
  - 适合每次回归测试后归档

## 📖 详细文档

### 核心文档

- **[完整系统文档](./SWARM_SYSTEM_README.md)**
  - 系统架构详解
  - Topic接口说明
  - 集群算法接口
  - 调试工具和技巧
  - 约30分钟阅读

- **[实现报告](./IMPLEMENTATION_REPORT.md)**
  - 已完成功能清单
  - 代码统计
  - 技术细节
  - 约15分钟阅读

### 开发指南

- **[集群算法模板](../src/planner/plan_manage/include/plan_manage/custom_swarm_algorithm.h)**
  - 自定义算法接口
  - 详细注释和示例
  - 使用说明
  - 约10分钟阅读

## 🎯 按需查阅

### 我想...

#### 🏃 立即运行系统
👉 [快速启动指南](./SWARM_QUICK_START.md) → "快速开始"章节

#### 🔍 了解系统架构
👉 [项目总览](../SWARM_PROJECT_OVERVIEW.md) → "系统架构"章节  
👉 [完整系统文档](./SWARM_SYSTEM_README.md) → "系统架构"章节

#### 🛠️ 补充集群算法
👉 [完整系统文档](./SWARM_SYSTEM_README.md) → "集群算法接口"章节  
👉 [算法模板文件](../src/planner/plan_manage/include/plan_manage/custom_swarm_algorithm.h)

#### 📊 查看Topic接口
👉 [完整系统文档](./SWARM_SYSTEM_README.md) → "Topic接口"章节  
👉 [项目总览](../SWARM_PROJECT_OVERVIEW.md) → "系统接口"章节

#### ⚙️ 配置参数
👉 [快速启动指南](./SWARM_QUICK_START.md) → "参数调整"章节  
👉 [完整系统文档](./SWARM_SYSTEM_README.md) → "参数配置"章节

#### 🐛 排查问题
👉 [快速启动指南](./SWARM_QUICK_START.md) → "故障排查"章节  
👉 [测试检查清单](./TEST_CHECKLIST.md) → "测试失败处理"章节

#### 📝 查看实现细节
👉 [实现报告](./IMPLEMENTATION_REPORT.md)

#### ✅ 测试系统
👉 [测试检查清单](./TEST_CHECKLIST.md)

## 📂 文件结构

```
ego-planner-bezier/
├── SWARM_PROJECT_OVERVIEW.md        # 项目总览 ⭐
├── markdown/
│   ├── SWARM_SYSTEM_README.md       # 完整系统文档 📖
│   ├── SWARM_QUICK_START.md         # 快速启动指南 🚀
│   ├── TEST_CHECKLIST.md            # 测试检查清单 ✅
│   ├── IMPLEMENTATION_REPORT.md     # 实现报告 📊
│   └── DOC_INDEX.md                 # 本文件（文档导航）
├── src/planner/plan_manage/
│   ├── include/plan_manage/
│   │   ├── swarm_trajectory_generator.h          # 系统头文件
│   │   ├── custom_swarm_algorithm.h              # 算法模板 🛠️
│   │   └── ego_replan_fsm.h                      # 主机FSM
│   ├── src/
│   │   ├── swarm_trajectory_generator.cpp        # 系统实现
│   │   ├── swarm_traj_generator_node.cpp         # ROS节点
│   │   └── ego_replan_fsm.cpp                    # 主机FSM
│   └── launch/
│       └── swarm_run.launch                       # 启动文件
└── scripts/
    └── verify_swarm_system.sh                     # 验证脚本
```

## 📚 推荐阅读顺序

### 新手入门（1小时）
1. [项目总览](../SWARM_PROJECT_OVERVIEW.md) (5分钟)
2. [快速启动指南](./SWARM_QUICK_START.md) (15分钟)
3. [测试检查清单](./TEST_CHECKLIST.md) - 运行测试 (30分钟)
4. [完整系统文档](./SWARM_SYSTEM_README.md) - 浏览 (10分钟)

### 开发算法（2小时）
1. [完整系统文档](./SWARM_SYSTEM_README.md) - "集群算法接口"章节 (20分钟)
2. [算法模板](../src/planner/plan_manage/include/plan_manage/custom_swarm_algorithm.h) (20分钟)
3. 实现自己的算法 (60分钟+)
4. [快速启动指南](./SWARM_QUICK_START.md) - 测试算法 (20分钟)

### 深入理解（3小时）
1. [完整系统文档](./SWARM_SYSTEM_README.md) - 完整阅读 (40分钟)
2. [实现报告](./IMPLEMENTATION_REPORT.md) (20分钟)
3. 阅读源代码 (2小时+)

## 🔗 相关链接

### 内部文档
- [EGO Planner原理](./COMPARISON_WITH_ORIGINAL.md)
- [Bezier优化器](./COMPILATION_SUCCESS.md)
- [原始项目说明](./START_HERE.md)

### 外部资源
- ROS Navigation Stack
- Swarm Robotics相关论文
- Formation Control算法

## 💡 使用技巧

### 快速查找
- 使用Ctrl+F在文档中搜索关键词
- 所有文档都有详细的目录
- Markdown支持链接跳转

### 文档更新
- 当您补充了集群算法后，建议更新相关文档
- 可以在算法模板中添加您的实现说明
- 测试结果可以记录在测试检查清单中

### 获取帮助
1. 先查看对应章节的文档
2. 运行验证脚本：`./scripts/verify_swarm_system.sh`
3. 查看系统日志：`roslaunch ... --screen`

## 📧 文档反馈

如果您发现文档有任何问题或需要补充：
- 不清楚的地方
- 错误或遗漏
- 需要更多示例
- 建议改进

请记录下来，方便后续完善。

---

**最后更新**: 2025年12月29日  
**文档版本**: v1.0  
**系统状态**: ✅ 就绪，可开始使用
