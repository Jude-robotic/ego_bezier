# 相对位置规划器实施文档

## 概述
本文档说明如何将EGO Planner从**绝对位置规划**改为**相对位置规划**，以适应倾斜巷道等复杂环境。

## 核心改进

### 1. 局部参考坐标系统 (Local Coordinate System)

创建了一个动态移动的局部坐标系，核心特性：
- **滑动原点**：坐标系原点随无人机位置动态更新
- **相对规划**：所有规划在局部坐标系中进行
- **坐标变换**：提供全局↔局部坐标的双向转换
- **边界检查**：基于相对位置检查工作空间边界

### 2. 实施的文件

#### 新增文件
1. [`src/planner/plan_manage/include/plan_manage/local_coordinate_system.h`](src/planner/plan_manage/include/plan_manage/local_coordinate_system.h)
   - LocalCoordinateSystem 类定义
   - 提供坐标变换接口

2. [`src/planner/plan_manage/src/local_coordinate_system.cpp`](src/planner/plan_manage/src/local_coordinate_system.cpp)
   - 实现坐标变换逻辑
   - 动态原点更新机制

#### 修改的文件
1. [`src/planner/plan_manage/include/plan_manage/ego_replan_fsm.h`](src/planner/plan_manage/include/plan_manage/ego_replan_fsm.h)
   - 添加 `#include <plan_manage/local_coordinate_system.h>`
   - 添加成员变量 `LocalCoordinateSystem::Ptr local_frame_`

2. [`src/planner/plan_manage/src/ego_replan_fsm.cpp`](src/planner/plan_manage/src/ego_replan_fsm.cpp)
   - 在`init()`中初始化局部坐标系
   - 在`odometryCallback()`中更新局部原点

3. [`src/planner/plan_manage/CMakeLists.txt`](src/planner/plan_manage/CMakeLists.txt)
   - 添加 `src/local_coordinate_system.cpp` 到编译列表

4. [`src/planner/plan_manage/launch/advanced_param.xml`](src/planner/plan_manage/launch/advanced_param.xml)
   - 添加局部坐标系统配置参数

## 配置参数

在 `advanced_param.xml` 中添加的参数：

```xml
<!-- 局部参考坐标系统 (Local Coordinate System) -->
<param name="local_frame/enable" value="true" type="bool"/>
<param name="local_frame/auto_update" value="true" type="bool"/>
<param name="local_frame/update_thresh" value="5.0" type="double"/>
<param name="local_frame/map_size_x" value="40.0" type="double"/>
<param name="local_frame/map_size_y" value="40.0" type="double"/>
<param name="local_frame/map_size_z" value="10.0" type="double"/>
<param name="local_frame/max_history" value="100" type="int"/>
```

### 参数说明

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `enable` | bool | true | 是否启用局部坐标系统 |
| `auto_update` | bool | true | 是否自动更新原点 |
| `update_thresh` | double | 5.0 | 原点更新距离阈值(米) |
| `map_size_x` | double | 40.0 | 局部地图X方向尺寸(米) |
| `map_size_y` | double | 40.0 | 局部地图Y方向尺寸(米) |
| `map_size_z` | double | 10.0 | 局部地图Z方向尺寸(米) |
| `max_history` | int | 100 | 保存的原点历史记录数 |

## 工作原理

### 1. 初始化
```cpp
// 在 EGOReplanFSM::init() 中
local_frame_.reset(new LocalCoordinateSystem);
local_frame_->init(nh);
```

### 2. 动态更新
```cpp
// 在 EGOReplanFSM::odometryCallback() 中
if (local_frame_->isEnabled() && have_odom_)
{
  if (local_frame_->updateOrigin(odom_pos_))
  {
    ROS_INFO("[FSM] Local frame origin updated to: [%.2f, %.2f, %.2f]",
             odom_pos_.x(), odom_pos_.y(), odom_pos_.z());
  }
}
```

### 3. 坐标变换

#### 全局 → 局部
```cpp
Eigen::Vector3d global_waypoint(10.0, 20.0, 1.5);
Eigen::Vector3d local_waypoint = local_frame_->globalToLocal(global_waypoint);
```

#### 局部 → 全局
```cpp
Eigen::Vector3d local_pos(2.0, 3.0, 1.5);
Eigen::Vector3d global_pos = local_frame_->localToGlobal(local_pos);
```

### 4. 边界检查
```cpp
// 检查全局位置是否在有效范围内
if (local_frame_->isGlobalPosValid(global_pos))
{
  // 位置有效，可以规划
}

// 或检查局部位置
if (local_frame_->isInLocalBounds(local_pos))
{
  // 在局部边界内
}
```

## 使用场景

### 场景1：倾斜巷道
无人机在向下倾斜的巷道中飞行：
- **问题**：绝对Z轴持续下降，超出预设地图边界
- **解决**：局部坐标系随无人机移动，相对地面保持定高

### 场景2：长距离飞行
无人机需要飞行很长距离：
- **问题**：绝对坐标数值很大，地图无法覆盖
- **解决**：局部坐标系滑动跟随，始终保持在小范围内

### 场景3：复杂地形
地形高度变化大：
- **问题**：固定地图高度不适应地形
- **解决**：局部坐标系原点高度动态调整

## 编译和运行

### 1. 编译
```bash
cd /home/jude/ego-planner-bezier
catkin_make
source devel/setup.bash
```

### 2. 运行
```bash
roslaunch ego_planner simple_run.launch
```

### 3. 验证
检查日志中是否有：
```
[LocalCoordinateSystem] Initialized
  Enable: true
  Auto Update: true
  Update Threshold: 5.00 m
  Local Map Size: [40.0, 40.0, 10.0] m
```

当无人机移动超过5米时，应该看到：
```
[FSM] Local frame origin updated to: [x, y, z]
```

## API 使用示例

### 示例1：在航点回调中使用相对坐标
```cpp
void EGOReplanFSM::waypointCallback(const nav_msgs::PathConstPtr &msg)
{
  // 1. 获取全局航点
  Eigen::Vector3d global_waypoint;
  global_waypoint << msg->poses[0].pose.position.x,
                     msg->poses[0].pose.position.y,
                     msg->poses[0].pose.position.z;
  
  // 2. 转换为局部坐标
  Eigen::Vector3d local_waypoint = local_frame_->globalToLocal(global_waypoint);
  
  // 3. 检查边界
  if (!local_frame_->isInLocalBounds(local_waypoint))
  {
    ROS_WARN("Waypoint outside local bounds!");
    return;
  }
  
  // 4. 在局部坐标系中规划
  // ... 规划逻辑 ...
}
```

### 示例2：发布全局坐标轨迹
```cpp
void publishTrajectory()
{
  nav_msgs::Path path;
  
  for (double t = 0; t < duration; t += dt)
  {
    // 1. 在局部坐标系中计算轨迹点
    Eigen::Vector3d local_pos = bezier_curve_->evaluate(t);
    
    // 2. 转换为全局坐标
    Eigen::Vector3d global_pos = local_frame_->localToGlobal(local_pos);
    
    // 3. 添加到路径
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = global_pos.x();
    pose.pose.position.y = global_pos.y();
    pose.pose.position.z = global_pos.z();
    path.poses.push_back(pose);
  }
  
  traj_pub_.publish(path);
}
```

## 调试和诊断

### 1. 打印调试信息
```cpp
local_frame_->printDebugInfo();
```

输出示例：
```
========== Local Coordinate System ==========
Enabled: Yes
Initialized: Yes
Origin (Global): [15.32, 20.45, 1.05]
Map Size: [40.0, 40.0, 10.0]
Auto Update: Yes
Update Threshold: 5.0 m
Origin History: 42 entries
============================================
```

### 2. 检查原点更新频率
如果原点更新过于频繁，增大 `update_thresh`：
```xml
<param name="local_frame/update_thresh" value="10.0" type="double"/>
```

### 3. 禁用局部坐标系（用于对比测试）
```xml
<param name="local_frame/enable" value="false" type="bool"/>
```

## 进一步优化建议

### 1. 边界柔性处理
当接近局部地图边界时，提前触发原点更新：
```cpp
double dist_to_boundary = getDistanceToBoundary(local_pos);
if (dist_to_boundary < margin_thresh)
{
  local_frame_->updateOrigin(odom_pos_);
}
```

### 2. 平滑原点转换
原点更新时可能导致轨迹跳变，可以使用平滑过渡：
```cpp
void smoothOriginTransition()
{
  // 在一段时间内平滑过渡到新原点
  // 实现细节...
}
```

### 3. 多层级坐标系
对于非常大规模的环境，可以实现多层级坐标系：
- 全局坐标系（GPS）
- 区域坐标系（公里级）
- 局部坐标系（米级）

## 常见问题

### Q1: 为什么启用局部坐标系后，rviz显示位置不对？
A: rviz需要在正确的frame中显示。确保`grid_map/frame_id`设置为`world`。

### Q2: 原点更新会导致轨迹跳变吗？
A: 不会。所有轨迹在更新时会自动转换坐标。但建议在安全时刻更新（如航点到达后）。

### Q3: 如何调整局部地图大小？
A: 修改 `local_frame/map_size_*` 参数。建议根据传感器范围设置。

### Q4: 可以禁用自动更新吗？
A: 可以。设置 `auto_update=false`，然后手动调用 `setOrigin()`。

## 总结

本实现提供了一个完整的相对位置规划框架：
- ✅ 局部参考坐标系统
- ✅ 自动原点更新
- ✅ 透明的坐标转换
- ✅ 边界检查
- ✅ 易于配置和调试

适用于倾斜巷道、长距离飞行、复杂地形等场景。

## 相关文件

- 实施指南: `RELATIVE_POSITION_GUIDE.md`
- 代码位置: `src/planner/plan_manage/`
- Launch文件: `src/planner/plan_manage/launch/advanced_param.xml`
- 测试脚本: `scripts/test_local_frame.sh`

## 作者

Jude - 2026-01-16

## 版本历史

- v1.0 (2026-01-16): 初始实现
  - 基础局部坐标系统
  - FSM集成
  - 配置参数
