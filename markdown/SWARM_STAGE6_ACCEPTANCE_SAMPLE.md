# 阶段6验收记录样例（已填）

> 示例用途：作为每次联调复用模板。可复制后按日期重命名归档。

## 1. 基本信息

- 测试日期：2026-02-22
- 测试人：Jude
- 代码分支/提交：feature/swarm-stage6 / abcdef1
- 场景描述：双从机仿真编队（默认不启用主动避障）

## 2. 启动参数快照

- `follower_use_local_map`：false
- `follower_use_real_data`：false
- `enable_comm_viz`：true
- `comm_viz_state_timeout`：0.3
- `comm_viz_guidance_timeout`：0.3
- `uav1_viz_color(r,g,b,a)`：0.1, 0.6, 1.0, 1.0
- `uav2_viz_color(r,g,b,a)`：1.0, 0.45, 0.1, 1.0

## 3. 核心验收清单（勾选）

- [x] 主机持续发布 `/swarm/agent_1/guidance_bezier`
- [x] 主机持续发布 `/swarm/agent_2/guidance_bezier`
- [x] 从机持续发布 `/swarm/agent_1/state`
- [x] 从机持续发布 `/swarm/agent_2/state`
- [x] `/uav1/planning/pos_cmd` 连续有效
- [x] `/uav2/planning/pos_cmd` 连续有效
- [x] `/swarm/comm/markers` 持续发布
- [x] RViz可见主从通信连线
- [x] RViz文本含 `agent_id`、`state_age`、`guidance_age`
- [x] RViz文本含 `timeout_state`、`timeout_guidance`
- [x] `uav1/uav2` 颜色可区分
- [x] 颜色设置不影响控制行为
- [x] 默认关闭主动避障（`follower_use_local_map=false`）

## 4. 结果记录

| 检查项 | 结果 | 备注 |
|---|---|---|
| 主从通信链路 | 通过 | 两路 guidance 与两路 state 持续更新 |
| 编队执行稳定性 | 通过 | 60s 内未见异常抖动与中断 |
| 可视化状态完整性 | 通过 | 文本字段完整，超时字段在正常通信时为 N |
| 默认参数稳定性 | 通过 | 默认配置可重复启动 |

## 5. 问题清单

| 编号 | 现象 | 原因分析 | 修复建议 |
|---|---|---|---|
| 1 | 暂无 | - | - |

## 6. 最终结论

- [x] 通过
- [ ] 有条件通过
- [ ] 不通过

结论说明：当前阶段目标满足，可进入“开启局部地图+主动避障”专项联调。

## 7. 后续启用主动避障（最小改动）

### 仿真

- 将 `follower_use_local_map` 改为 `true`
- 保持 `follower_use_real_data=false`
- 核对 `follower_sim_odom_topic`、`follower_sim_cloud_topic`

### 实机

- 将 `follower_use_local_map` 改为 `true`
- 将 `follower_use_real_data` 改为 `true`
- 传入：`real_odom_topic`、`real_cloud_topic`、`real_depth_topic`、`real_pose_topic`
