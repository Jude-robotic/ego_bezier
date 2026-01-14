# Fixed Cross-Wall Test Environment

## 概述
`first_testenv.launch` 提供了一个固定的、可重复的十字形墙壁测试环境，用于精确测试无人机路径规划算法。

## 环境特点

### 地图布局
- **地图大小**: 20m × 20m × 3m
- **墙壁结构**: 十字形墙壁（沿X轴和Y轴的中心线）
- **门洞**: 4个固定门洞（每条墙的中心位置，宽度2m）
- **边界**: 完全封闭的外墙
- **墙壁厚度**: 0.3m
- **墙壁高度**: 2.5m

### 飞行任务
无人机从右下角起飞，依次穿过门洞到达目标点：

1. **起点**: (7.5, -7.5, 1.0) - 右下象限
2. **航点1**: (-7.5, -7.5, 1.0) - 左下象限（需穿过下方门洞）
3. **航点2**: (-7.5, 7.5, 1.0) - 左上象限（需穿过左侧门洞）
4. **终点**: 悬停在航点2位置

## 使用方法

### 启动测试环境
```bash
cd ~/ego-planner-bezier
source devel/setup.bash
roslaunch ego_planner first_testenv.launch
```

### 参数配置
在 `first_testenv.launch` 中可以修改以下参数：

- `max_vel`: 最大速度（默认1.5 m/s）
- `max_acc`: 最大加速度（默认2.0 m/s²）
- `planning_horizon`: 规划视野（默认7.5 m）

### 到达判定阈值
在 `advanced_param.xml` 中已优化：
- `fsm/thresh_no_replan`: 0.5m（从原来的2.0m减小）
- `fsm/thresh_replan`: 0.6m

## 与随机环境的对比

| 特性 | simple_run.launch | first_testenv.launch |
|------|-------------------|----------------------|
| 环境类型 | 随机生成 | 固定十字墙 |
| 可重复性 | 每次不同 | 完全相同 |
| 地图大小 | 40×40m | 20×20m |
| 障碍物 | 200个圆柱+200个柱体 | 仅墙壁 |
| 复杂度 | 高 | 低 |
| 用途 | 压力测试 | 精确调试 |

## 环境可视化

在RViz中可以看到：
- **白色墙壁**: 固定的十字形结构
- **绿色路径**: 规划的飞行轨迹
- **红色球体**: 无人机当前位置
- **蓝色标记**: 目标航点

## 技术实现

### 代码修改
1. `random_forest_sensing.cpp`: 添加 `GenerateFixedCrossWallMap()` 函数
2. 支持 `map/type` 参数选择地图类型：
   - `"random"`: 随机圆柱体地图（默认）
   - `"fixed_cross"`: 固定十字墙地图

### 地图生成逻辑
```cpp
void GenerateFixedCrossWallMap() {
  // 生成水平墙（沿X轴，除门洞外）
  // 生成垂直墙（沿Y轴，除门洞外）
  // 生成4个边界墙（封闭空间）
  // 每个墙中心留有2m宽的门洞
}
```

## 故障排除

### 无人机撞墙
- 检查 `planning_horizon` 是否足够大（建议≥7.5m）
- 检查 `max_vel` 和 `max_acc` 是否过高

### 未到达目标点
- 检查 `fsm/thresh_no_replan` 参数（应为0.5m）
- 查看终端输出的规划状态

### RViz无显示
- 确认已执行 `source devel/setup.bash`
- 检查 `/map_generator/global_cloud` topic是否发布

## 下一步扩展

可以基于此环境创建更多测试场景：
- 调整门洞宽度测试狭窄通道
- 增加墙壁高度测试垂直避障
- 添加动态障碍物
- 测试不同速度和加速度参数

