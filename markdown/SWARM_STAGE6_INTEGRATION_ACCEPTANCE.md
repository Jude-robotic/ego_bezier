# Swarm 阶段6：联调验收模板 + 检查清单

> 适用：`swarm_multi_sim.launch` 主机+多从机联调

## 1. 基本信息

- 测试日期：
- 测试人：
- 代码分支/提交：
- 场景描述：

## 2. 启动参数快照

- `follower_use_local_map`：
- `follower_use_real_data`：
- `enable_comm_viz`：
- `comm_viz_state_timeout`：
- `comm_viz_guidance_timeout`：
- `uav1_viz_color(r,g,b,a)`：
- `uav2_viz_color(r,g,b,a)`：

## 3. 核心验收清单（勾选）

- [ ] 主机持续发布 `/swarm/agent_1/guidance_bezier`
- [ ] 主机持续发布 `/swarm/agent_2/guidance_bezier`
- [ ] 从机持续发布 `/swarm/agent_1/state`
- [ ] 从机持续发布 `/swarm/agent_2/state`
- [ ] `/uav1/planning/pos_cmd` 连续有效
- [ ] `/uav2/planning/pos_cmd` 连续有效
- [ ] `/swarm/comm/markers` 持续发布
- [ ] RViz可见主从通信连线
- [ ] RViz文本含 `agent_id`、`state_age`、`guidance_age`
- [ ] RViz文本含 `timeout_state`、`timeout_guidance`
- [ ] `uav1/uav2` 颜色可区分
- [ ] 颜色设置不影响控制行为
- [ ] 默认关闭主动避障（`follower_use_local_map=false`）

## 4. 结果记录

| 检查项 | 结果 | 备注 |
|---|---|---|
| 主从通信链路 |  |  |
| 编队执行稳定性 |  |  |
| 可视化状态完整性 |  |  |
| 默认参数稳定性 |  |  |

## 5. 问题清单

| 编号 | 现象 | 原因分析 | 修复建议 |
|---|---|---|---|
| 1 |  |  |  |
| 2 |  |  |  |

## 6. 最终结论

- [ ] 通过
- [ ] 有条件通过
- [ ] 不通过

结论说明：

## 7. 后续启用主动避障（最小改动）

### 仿真

- `follower_use_local_map:=true`
- `follower_use_real_data:=false`
- 确认：`follower_sim_odom_topic`、`follower_sim_cloud_topic`

### 实机

- `follower_use_local_map:=true`
- `follower_use_real_data:=true`
- 传入：`real_odom_topic`、`real_cloud_topic`、`real_depth_topic`、`real_pose_topic`
