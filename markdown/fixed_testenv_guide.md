# 固定十字墙测试环境使用指南

## 快速启动

```bash
cd ~/ego-planner-bezier
source devel/setup.bash
roslaunch ego_planner first_testenv.launch
```

或使用快捷脚本：
```bash
cd ~/ego-planner-bezier
./shell/run_testenv.sh
```

## 环境说明

### 地图布局（俯视图）
```
    ┌─────────────────────────┐
    │                         │
    │         左上角           │
    │      (-7.5, 7.5)        │
    │           ★ 目标2       │
    │           │             │
    ├───────┐   │   ┌─────────┤
    │       └───┼───┘         │
    │           │             │
    │   ★       ┼       ○     │ 
    │ 目标1     │     起点    │
    │(-7.5,-7.5)│  (7.5,-7.5)│
    │           │             │
    │       ┌───┼───┐         │
    ├───────┘   │   └─────────┤
    │           │             │
    │                         │
    └─────────────────────────┘

  图例：
  ○ 起点
  ★ 目标点
  ─│ 墙壁（有4个门洞）
```

### 飞行路径
1. **起点**: 右下角 (7.5, -7.5, 1.0)
2. **第一段**: 穿过下方门洞到达左下角 (-7.5, -7.5, 1.0)
3. **第二段**: 穿过左侧门洞到达左上角 (-7.5, 7.5, 1.0)
4. **悬停**: 在左上角目标点悬停

## 关键参数

### 已优化的到达判定
- `fsm/thresh_no_replan`: **0.5米**（原2.0米）
- 无人机会更精确地到达目标点

### 速度限制
- 最大速度: 1.5 m/s
- 最大加速度: 2.0 m/s²

### 地图尺寸
- 20m × 20m × 3m
- 比simple_run环境小一半，更适合调试

## 与simple_run对比

| 特性 | simple_run | first_testenv |
|------|------------|---------------|
| 环境 | 随机障碍物 | 固定十字墙 |
| 地图 | 40×40m | 20×20m |
| 障碍物数量 | 400个 | 仅墙壁 |
| 可重复性 | 每次不同 | 完全相同 ✓ |
| 适用场景 | 压力测试 | 精确调试 ✓ |

## 故障排除

### 看不到墙壁
- 检查RViz中 `/map_generator/global_cloud` topic
- 确认已执行 `source devel/setup.bash`

### 无人机撞墙
- 减小 `max_vel` 和 `max_acc`
- 增加 `planning_horizon`（建议≥7.5m）

### 未到达目标
- 确认 `advanced_param.xml` 中 `thresh_no_replan=0.5`
- 查看终端输出的状态信息

## 文件位置

- Launch文件: `src/planner/plan_manage/launch/first_testenv.launch`
- 详细文档: `src/planner/plan_manage/launch/TESTENV_README.md`
- 启动脚本: `shell/run_testenv.sh`
- 地图生成: `src/uav_simulator/map_generator/src/random_forest_sensing.cpp`

