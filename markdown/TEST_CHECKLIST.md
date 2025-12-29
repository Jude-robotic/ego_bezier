# 多机协同系统 - 测试检查清单

## ✅ 基础功能测试

### 测试1: 编译验证
```bash
cd /home/jude/ego-planner-bezier
catkin_make
```
**预期结果**: 
- ✅ 编译成功，无错误
- ✅ 生成可执行文件：`devel/lib/ego_planner/swarm_traj_generator_node`

---

### 测试2: 启动系统
```bash
source devel/setup.bash
roslaunch ego_planner swarm_run.launch
```
**预期结果**:
- ✅ 所有节点正常启动
- ✅ 看到日志输出：
  ```
  [SwarmTrajGen] ============================================
  [SwarmTrajGen] Swarm Trajectory Generator Initialized!
  [SwarmTrajGen] ============================================
  ```

---

### 测试3: 检查节点
打开新终端：
```bash
rosnode list
```
**预期结果**:
```
/swarm_traj_generator          # ← 集群轨迹生成器
/uav_0/ego_planner_node        # ← 主机规划器
/uav_0/traj_server
...其他节点
```

---

### 测试4: 检查Topic
```bash
rostopic list | grep uav
```
**预期结果**:
```
/uav_0/planning/trajectory
/uav_1/planning/trajectory
/uav_2/planning/trajectory
/uav_3/planning/trajectory
/uav_4/planning/trajectory
```

---

### 测试5: 查看主机轨迹
```bash
rostopic echo /uav_0/planning/trajectory
```
**预期结果**:
- ✅ 看到Path消息输出
- ✅ 包含poses数组
- ✅ 每个pose有position信息

**示例输出**:
```yaml
header: 
  seq: 1
  stamp: 
    secs: 123456
    nsecs: 789012
  frame_id: "world"
poses: 
  - 
    header: 
      frame_id: "world"
    pose: 
      position: 
        x: -15.0
        y: 0.0
        z: 1.0
  ...
```

---

### 测试6: 查看从机轨迹
```bash
rostopic echo /uav_1/planning/trajectory
```
**预期结果**:
- ✅ 看到Path消息输出
- ✅ 位置与主机有偏移（环绕效果）

---

### 测试7: 检查发布频率
```bash
rostopic hz /uav_0/planning/trajectory
rostopic hz /uav_1/planning/trajectory
```
**预期结果**:
- ✅ 主机和从机topic都有数据发布
- ✅ 频率稳定（根据规划器更新频率）

---

### 测试8: 节点详细信息
```bash
rosnode info /swarm_traj_generator
```
**预期结果**:
```
Subscriptions: 
 * /uav_0/planning/trajectory
 * /uav_0/planning/bezier_trajectory

Publications: 
 * /uav_1/planning/trajectory
 * /uav_2/planning/trajectory
 * /uav_3/planning/trajectory
 * /uav_4/planning/trajectory
 * /uav_1/planning/bezier_trajectory
 * /uav_2/planning/bezier_trajectory
 * /uav_3/planning/bezier_trajectory
 * /uav_4/planning/bezier_trajectory
```

---

## 🎨 可视化测试

### 测试9: RViz可视化

1. RViz应该已经自动启动
2. 在RViz中添加Path显示：
   - 点击左下角 "Add"
   - 选择 "By topic"
   - 找到 `/uav_0/planning/trajectory` 并添加
   - 设置颜色为**红色**

3. 重复添加从机轨迹：
   - `/uav_1/planning/trajectory` - **蓝色**
   - `/uav_2/planning/trajectory` - **绿色**
   - `/uav_3/planning/trajectory` - **黄色**
   - `/uav_4/planning/trajectory` - **紫色**

**预期效果**:
- ✅ 看到主机轨迹（红色）
- ✅ 看到4条从机轨迹环绕在主机周围
- ✅ 从机形成圆形编队
- ✅ 编队半径约3米（默认参数）

---

## ⚙️ 参数配置测试

### 测试10: 修改从机数量
```bash
roslaunch ego_planner swarm_run.launch num_followers:=2
```
**预期结果**:
- ✅ 只生成2个从机轨迹
- ✅ Topic列表中只有`/uav_1`和`/uav_2`

---

### 测试11: 修改编队参数
```bash
roslaunch ego_planner swarm_run.launch formation_radius:=5.0
```
**预期结果**:
- ✅ 在RViz中看到从机距离主机更远（5米）

---

## 🔍 数据验证测试

### 测试12: 轨迹点数量一致性
```bash
# 主机轨迹点数
rostopic echo /uav_0/planning/trajectory -n 1 | grep -c "pose:"

# 从机轨迹点数
rostopic echo /uav_1/planning/trajectory -n 1 | grep -c "pose:"
```
**预期结果**:
- ✅ 主机和从机的轨迹点数量相同

---

### 测试13: 坐标系验证
```bash
rostopic echo /uav_0/planning/trajectory -n 1 | grep "frame_id"
```
**预期结果**:
```yaml
frame_id: "world"
```
- ✅ 所有消息使用统一的world坐标系

---

## 📊 性能测试

### 测试14: 系统资源占用
```bash
top -p $(pgrep -d',' -f swarm_traj_generator_node)
```
**预期结果**:
- ✅ CPU占用合理（< 10%）
- ✅ 内存占用稳定

---

### 测试15: 延迟测试
```bash
rostopic delay /uav_1/planning/trajectory
```
**预期结果**:
- ✅ 延迟较小（< 100ms）

---

## 🐛 异常处理测试

### 测试16: 主机轨迹中断
停止主机规划器，观察从机轨迹生成器：
```bash
rosnode kill /uav_0/ego_planner_node
```
**预期行为**:
- ✅ 从机轨迹生成器继续运行（不崩溃）
- ✅ 等待新的主机轨迹

---

### 测试17: 重新启动
```bash
Ctrl+C  # 停止所有节点
roslaunch ego_planner swarm_run.launch
```
**预期结果**:
- ✅ 系统可以正常重启
- ✅ 所有功能恢复正常

---

## 📝 日志验证

### 测试18: 查看日志输出
在启动终端中查看：
```bash
roslaunch ego_planner swarm_run.launch --screen
```
**预期看到**:
```
[SwarmTrajGen] Initializing with 4 followers
[SwarmTrajGen] Formation radius: 3.00 m, Altitude offset: 0.50 m
[SwarmTrajGen] Using default SimpleCircleFormationAlgorithm
[SwarmTrajGen] Created publishers for /uav_1
[SwarmTrajGen] Created publishers for /uav_2
...
[SwarmTrajGen] Generated trajectories for 4 followers, each with XXX points
```

---

## ✅ 完整性检查清单

将测试完成的项目打勾：

- [ ] 编译成功
- [ ] 系统启动正常
- [ ] 所有关键节点运行
- [ ] 所有Topic存在
- [ ] 主机轨迹正常发布
- [ ] 从机轨迹正常生成
- [ ] 发布频率稳定
- [ ] 节点订阅/发布关系正确
- [ ] RViz可视化正常
- [ ] 编队效果符合预期
- [ ] 参数配置生效
- [ ] 轨迹点数量一致
- [ ] 坐标系正确
- [ ] 性能表现良好
- [ ] 异常处理正常
- [ ] 日志输出正确

---

## 🎯 通过标准

**系统验收条件**:
1. ✅ 所有18项测试通过
2. ✅ 完整性检查清单全部打勾
3. ✅ RViz中能看到清晰的编队效果
4. ✅ 无崩溃或错误日志

**如果所有测试通过**: 🎉 系统就绪，可以开始补充集群算法！

---

## 🆘 测试失败处理

如果某项测试失败，参考：
- **快速启动指南**: `markdown/SWARM_QUICK_START.md` - 故障排查章节
- **系统文档**: `markdown/SWARM_SYSTEM_README.md` - 问题反馈章节
- **验证脚本**: `scripts/verify_swarm_system.sh` - 自动化检查

---

**最后更新**: 2025年12月29日
