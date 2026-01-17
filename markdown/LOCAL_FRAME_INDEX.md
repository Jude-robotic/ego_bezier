# 相对位置规划器 - 文档索引

## 📖 文档列表

### 快速入门
1. **[快速开始指南](QUICK_START_LOCAL_FRAME.md)** ⭐ 推荐首先阅读
   - 快速测试步骤
   - 配置示例
   - API简介
   - 常见问题

### 详细文档
2. **[详细实施文档](RELATIVE_POSITION_GUIDE.md)**
   - 完整实施方案
   - 详细API说明
   - 使用场景分析
   - 调试诊断

3. **[修改总结](MODIFICATION_SUMMARY.md)**
   - 修改文件清单
   - 代码统计
   - 技术实现细节
   - 编译验证

---

## 🎯 根据需求选择文档

### 我想快速开始使用
→ [快速开始指南](QUICK_START_LOCAL_FRAME.md)
- 5分钟快速测试
- 最小配置即可运行

### 我想了解详细原理
→ [详细实施文档](RELATIVE_POSITION_GUIDE.md)
- 系统架构设计
- 工作原理说明
- 完整API文档

### 我想知道修改了什么
→ [修改总结](MODIFICATION_SUMMARY.md)
- 完整修改清单
- 代码行数统计
- 编译验证结果

### 我遇到了问题
→ [快速开始指南 - 故障排除](QUICK_START_LOCAL_FRAME.md#-故障排除)
→ [详细实施文档 - 调试诊断](RELATIVE_POSITION_GUIDE.md#调试和诊断)

---

## 📂 相关源代码

### 核心实现
```
src/planner/plan_manage/
├── include/plan_manage/
│   ├── local_coordinate_system.h     ← 局部坐标系统类
│   └── ego_replan_fsm.h              ← FSM（已修改）
└── src/
    ├── local_coordinate_system.cpp   ← 坐标系统实现
    └── ego_replan_fsm.cpp            ← FSM实现（已修改）
```

### 配置文件
```
src/planner/plan_manage/launch/
└── advanced_param.xml                ← 参数配置（已添加）
```

### 编译配置
```
src/planner/plan_manage/
└── CMakeLists.txt                    ← 编译配置（已修改）
```

---

## 🔑 关键概念

### 局部坐标系统 (Local Coordinate System)
以无人机为中心的动态移动坐标系，核心特性：
- **滑动原点**: 随无人机位置动态更新
- **相对规划**: 在小范围坐标内进行规划
- **透明转换**: 自动处理全局↔局部坐标转换

### 坐标转换
```cpp
// 全局 → 局部
local_pos = global_pos - local_origin

// 局部 → 全局  
global_pos = local_pos + local_origin
```

### 原点更新
```cpp
if (distance_to_origin > threshold) {
    update_origin(current_position);
}
```

---

## ⚙️ 配置参数快查

| 参数 | 默认值 | 说明 | 推荐范围 |
|------|--------|------|----------|
| `enable` | true | 启用局部坐标系 | true/false |
| `auto_update` | true | 自动更新原点 | true/false |
| `update_thresh` | 5.0 | 更新距离阈值(m) | 3.0~10.0 |
| `map_size_x` | 40.0 | X轴地图尺寸(m) | 20.0~100.0 |
| `map_size_y` | 40.0 | Y轴地图尺寸(m) | 20.0~100.0 |
| `map_size_z` | 10.0 | Z轴地图尺寸(m) | 5.0~20.0 |

---

## 🚀 快速命令

### 编译
```bash
cd /home/jude/ego-planner-bezier
catkin_make
source devel/setup.bash
```

### 运行测试
```bash
./scripts/test_local_frame.sh
```

### 查看配置
```bash
grep "local_frame" src/planner/plan_manage/launch/advanced_param.xml
```

### 查看日志
启动系统后观察终端输出

---

## 📋 检查清单

### 首次使用前
- [ ] 已阅读快速开始指南
- [ ] 已编译通过
- [ ] 已检查配置参数
- [ ] 已准备测试环境

### 遇到问题时
- [ ] 检查编译是否成功
- [ ] 检查配置文件是否正确
- [ ] 查看故障排除章节
- [ ] 查看日志输出

### 实际使用前
- [ ] 在仿真环境测试
- [ ] 根据场景调整参数
- [ ] 进行对比测试
- [ ] 验证功能正常

---

## 📞 支持

### 文档问题
查看对应文档的"常见问题"章节

### 技术问题
参考"故障排除"和"调试诊断"章节

### 功能增强
参考"进一步优化建议"章节

---

## 📅 版本信息

- **版本**: v1.0
- **发布日期**: 2026-01-16
- **状态**: ✅ 稳定版本
- **测试**: 编译通过

---

## 🌟 推荐阅读路径

### 路径1: 快速上手（推荐新手）
1. [快速开始指南](QUICK_START_LOCAL_FRAME.md) - 10分钟
2. 运行测试
3. 遇到问题查看故障排除

### 路径2: 深入理解（推荐开发者）
1. [修改总结](MODIFICATION_SUMMARY.md) - 了解修改内容
2. [详细实施文档](RELATIVE_POSITION_GUIDE.md) - 理解实现原理
3. 查看源代码
4. 进行二次开发

### 路径3: 问题解决
1. 快速开始指南 - 故障排除章节
2. 详细实施文档 - 调试诊断章节
3. 查看日志输出
4. 检查配置文件

---

**最后更新**: 2026-01-16  
**文档版本**: v1.0
