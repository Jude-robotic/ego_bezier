# 固定十字墙环境问题诊断与修复

## 问题根源

运行 `first_testenv.launch` 时，仍然看到随机障碍物，而不是固定的十字墙环境。

### 根本原因分析

1. **`simulator.xml` 中有两个地图生成器**：
   - `random_forest` 节点（被 `<![CDATA[...]]>` 注释掉了）
   - `mockamap_node` 节点（正在运行）

2. **`mockamap` 不支持 `fixed_cross` 类型**：
   - 虽然 `first_testenv.launch` 传递了 `map_type="fixed_cross"`
   - 但 `mockamap` 只支持 Perlin噪声随机地图
   - 查看源码确认：`mockamap` 中没有 `fixed_cross` 处理逻辑

3. **`GenerateFixedCrossWallMap()` 函数没有被调用**：
   - 该函数在 `random_forest_sensing.cpp` 中实现
   - 但 `random_forest` 节点被注释了，所以从未执行

## 修复方案

### 1. 修改 `simulator.xml` 
   - ✅ **启用** `random_forest` 节点
   - ✅ **禁用** `mockamap_node` 节点
   - ✅ 添加 `<param name="map/type" value="$(arg map_type)"/>` 给 random_forest

### 2. 重新编译
   ```bash
   cd ~/ego-planner-bezier
   catkin_make -DCATKIN_WHITELIST_PACKAGES="map_generator"
   ```

### 3. 验证修复
   ```bash
   ./test_fixed_cross.sh
   ```

## 修改的文件

1. **`src/planner/plan_manage/launch/simulator.xml`**
   - 移除 random_forest 的 CDATA 注释
   - 添加 map/type 参数支持
   - 注释掉 mockamap_node

2. **已备份原文件**：`simulator.xml.backup`

## 为什么之前没发现

- `simulator.xml` 的 CDATA 语法很隐蔽
- launch 文件不会报错，只是默默使用了错误的地图生成器
- `mockamap` 忽略了不支持的 `map_type` 参数，继续生成随机地图

## 现在的工作流程

```
first_testenv.launch
    ↓
simulator.xml (map_type="fixed_cross")
    ↓
random_forest 节点
    ↓
读取 map/type 参数
    ↓
调用 GenerateFixedCrossWallMap()
    ↓
发布 /map_generator/global_cloud
    ↓
RViz 显示固定十字墙
```

## 测试方法

### 快速测试
```bash
cd ~/ego-planner-bezier
./test_fixed_cross.sh
```

### 手动测试
```bash
cd ~/ego-planner-bezier
source devel/setup.bash
roslaunch ego_planner first_testenv.launch
```

### 验证点
- [ ] RViz 中看到4面墙（十字形）
- [ ] 每面墙上有一个2米宽的门洞
- [ ] 没有随机障碍物
- [ ] 无人机从右下角 (7.5, -7.5) 起飞
- [ ] 穿过门洞到达左下角 (-7.5, -7.5)
- [ ] 再穿过门洞到达左上角 (-7.5, 7.5)

---
修复时间: 2026-01-13
