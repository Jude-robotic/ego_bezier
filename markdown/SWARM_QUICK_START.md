# 多机协同系统快速启动指南

## 🚀 快速启动

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

### 3. 查看运行效果

打开新终端，查看topic列表：

```bash
# 查看所有UAV相关topic
rostopic list | grep uav

# 预期输出：
# /uav_0/planning/trajectory          # 主机轨迹
# /uav_1/planning/trajectory          # 从机1轨迹
# /uav_2/planning/trajectory          # 从机2轨迹
# /uav_3/planning/trajectory          # 从机3轨迹
# /uav_4/planning/trajectory          # 从机4轨迹
```

### 4. 监控轨迹数据

```bash
# 查看主机轨迹
rostopic echo /uav_0/planning/trajectory

# 查看从机1轨迹
rostopic echo /uav_1/planning/trajectory

# 查看消息频率
rostopic hz /uav_0/planning/trajectory
rostopic hz /uav_1/planning/trajectory
```

## 📊 系统验证

### 检查节点状态

```bash
# 查看所有节点
rosnode list

# 预期关键节点：
# /uav_0/ego_planner_node          # 主机规划器
# /swarm_traj_generator            # 集群轨迹生成器
# /uav_0/traj_server               # 轨迹服务器
```

### 检查节点信息

```bash
# 查看集群轨迹生成器详情
rosnode info /swarm_traj_generator

# 预期看到：
# Subscriptions:
#   * /uav_0/planning/trajectory
#   * /uav_0/planning/bezier_trajectory
# Publications:
#   * /uav_1/planning/trajectory
#   * /uav_2/planning/trajectory
#   * /uav_3/planning/trajectory
#   * /uav_4/planning/trajectory
```

## 🎯 在RViz中可视化

1. RViz会自动启动
2. 添加Path显示：
   - 点击"Add" -> "Path"
   - Topic: `/uav_0/planning/trajectory`
   - Color: 红色
3. 重复添加从机轨迹：
   - `/uav_1/planning/trajectory` (蓝色)
   - `/uav_2/planning/trajectory` (绿色)
   - `/uav_3/planning/trajectory` (黄色)
   - `/uav_4/planning/trajectory` (紫色)

## ⚙️ 参数调整

### 修改从机数量

```bash
roslaunch ego_planner swarm_run.launch num_followers:=6
```

### 修改编队参数

```bash
roslaunch ego_planner swarm_run.launch formation_radius:=5.0 altitude_offset:=1.0
```

### 修改所有参数

```bash
roslaunch ego_planner swarm_run.launch \
    num_followers:=6 \
    formation_radius:=5.0 \
    altitude_offset:=1.0
```

## 🔧 补充集群算法

### 当前状态
- ✅ 系统框架完整
- ✅ 默认简单圆形编队算法
- 🔲 需要补充高级集群算法

### 补充步骤

1. **复制模板文件**
   ```bash
   cd /home/jude/ego-planner-bezier/src/planner/plan_manage/include/plan_manage
   cp custom_swarm_algorithm.h my_swarm_algorithm.h
   ```

2. **编辑算法文件**
   ```bash
   vim my_swarm_algorithm.h
   # 修改类名为 MySwarmAlgorithm
   # 实现 computeFollowerTrajectories() 方法
   ```

3. **修改节点以使用新算法**
   编辑 `src/planner/plan_manage/src/swarm_traj_generator_node.cpp`:
   
   ```cpp
   #include <plan_manage/my_swarm_algorithm.h>
   
   int main(int argc, char **argv)
   {
     ros::init(argc, argv, "swarm_trajectory_generator");
     ros::NodeHandle nh("~");
   
     ego_planner::SwarmTrajectoryGenerator swarm_generator(nh);
     
     // 使用您的自定义算法
     ego_planner::MySwarmAlgorithm* my_algo = 
         new ego_planner::MySwarmAlgorithm(参数...);
     
     swarm_generator.setSwarmAlgorithm(my_algo);
     swarm_generator.init();
   
     ros::spin();
     return 0;
   }
   ```

4. **重新编译**
   ```bash
   cd /home/jude/ego-planner-bezier
   catkin_make
   ```

## 📝 记录数据

### 录制bag

```bash
rosbag record -O swarm_test.bag \
    /uav_0/planning/trajectory \
    /uav_1/planning/trajectory \
    /uav_2/planning/trajectory \
    /uav_3/planning/trajectory \
    /uav_4/planning/trajectory \
    /tf /tf_static
```

### 回放bag

```bash
rosbag play swarm_test.bag
```

## 🐛 故障排查

### 问题1: 从机轨迹没有发布

**检查**:
```bash
rostopic hz /uav_0/planning/trajectory
```

如果主机轨迹也没有，说明主机规划器未工作。检查waypoint是否设置正确。

### 问题2: 编译错误

**解决**:
```bash
cd /home/jude/ego-planner-bezier
catkin_make clean
catkin_make
```

### 问题3: 节点启动失败

**检查日志**:
```bash
roslaunch ego_planner swarm_run.launch --screen
```

查看红色错误信息，通常是参数配置问题。

## 📚 相关文档

- [完整系统文档](./SWARM_SYSTEM_README.md)
- [集群算法接口说明](./SWARM_SYSTEM_README.md#集群算法接口)
- [原始EGO Planner说明](./START_HERE.md)

## ✅ 系统检查清单

启动后请检查：

- [ ] 主机规划器节点运行正常
- [ ] 集群轨迹生成器节点运行正常  
- [ ] `/uav_0/planning/trajectory` 有数据发布
- [ ] `/uav_1~4/planning/trajectory` 有数据发布
- [ ] RViz可视化正常显示
- [ ] 从机轨迹形成预期编队形状

## 🎉 成功标志

当您看到以下输出时，系统运行成功：

```
[SwarmTrajGen] ============================================
[SwarmTrajGen] Swarm Trajectory Generator Initialized!
[SwarmTrajGen] ============================================
[SwarmTrajGen] Generated trajectories for 4 followers, each with XXX points
```

并且在RViz中能看到主机和从机的轨迹形成编队效果。

---

**下一步**: 补充您的集群算法并测试实际效果！
