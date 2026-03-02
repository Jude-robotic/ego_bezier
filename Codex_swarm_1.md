# Codex Swarm 改造说明（阶段 1）

> 文档目的：系统梳理本次会话中已落地的“集群主机 + 从机协同 + 仿真到实机过渡 + 自检闭环”代码改造，便于你后续二次开发与工程化增强。

---

## 1. 改造目标与设计约束

本次改造围绕以下目标展开：

1. **主机统一规划/编队管理**，向各从机下发引导控制点（Bezier）。
2. **从机状态回传**，主机获得所有智能体状态后再生成引导轨迹。
3. **从机本地二次优化**，确保每台机体结合本地地图/模型再做动态可行优化。
4. **仿真优先，接口不变迁移实机**，避免“仿真和实机两套协议”。
5. **主从通信（不走全互联）**，通信关系简化为 Master <-> Follower。
6. **上机前可快速自检**，避免“系统未就绪就起飞”。

关键约束：

- 保持核心规划主链路兼容原工程（EGO-Bezier）。
- 不破坏已有 `planning/bezier` 消费路径（`traj_server` 仍可直接使用）。
- 通过 launch 参数切换仿真/实机，不改业务节点内部主逻辑。

---

## 2. 本次改造的总体架构

### 2.1 数据流（高层）

1. 每个从机上报 `SwarmAgentState` 到主机侧。  
2. 主机汇总状态，按编队策略生成每个从机的引导 `Bezier`。  
3. 从机收到引导后，注入本地优化器，生成可执行 `planning/bezier`。  
4. `traj_server` 使用优化后的轨迹进行执行。  
5. 自检节点监控关键 topic 时效与有效性，给出统一状态摘要。

### 2.2 主从职责划分

- **Master 侧**
	- 编队目标分配
	- 控制点引导轨迹生成
	- 对每个 agent 的引导 topic 定向发布

- **Follower 侧**
	- 里程计/状态封装并上报
	- 读取主机引导轨迹
	- 执行本地二次优化并重发布

---

## 3. 文件改动总览（本次会话）

> 以下均为已落地（含编译验证）的主要改动。

### 3.1 消息与构建系统

- 新增消息：
	- [src/planner/plan_manage/msg/SwarmAgentState.msg](src/planner/plan_manage/msg/SwarmAgentState.msg)

- 修改构建脚本：
	- [src/planner/plan_manage/CMakeLists.txt](src/planner/plan_manage/CMakeLists.txt)

主要内容：

- 注册 `SwarmAgentState.msg`
- 增加节点可执行目标：
	- `swarm_master_node`
	- `swarm_agent_state_reporter_node`
	- `swarm_follower_guidance_optimizer_node`
	- `swarm_preflight_check_node`

### 3.2 Master 节点

- 头文件：
	- [src/planner/plan_manage/include/plan_manage/swarm_master_coordinator.h](src/planner/plan_manage/include/plan_manage/swarm_master_coordinator.h)

- 实现：
	- [src/planner/plan_manage/src/swarm_master_coordinator.cpp](src/planner/plan_manage/src/swarm_master_coordinator.cpp)
	- [src/planner/plan_manage/src/swarm_master_node.cpp](src/planner/plan_manage/src/swarm_master_node.cpp)

- 参数与启动：
	- [src/planner/plan_manage/launch/swarm_master_params.yaml](src/planner/plan_manage/launch/swarm_master_params.yaml)
	- [src/planner/plan_manage/launch/swarm_master_only.launch](src/planner/plan_manage/launch/swarm_master_only.launch)

### 3.3 Follower 状态回传节点

- [src/planner/plan_manage/src/swarm_agent_state_reporter_node.cpp](src/planner/plan_manage/src/swarm_agent_state_reporter_node.cpp)

### 3.4 Follower 引导+二次优化节点

- [src/planner/plan_manage/src/swarm_follower_guidance_optimizer_node.cpp](src/planner/plan_manage/src/swarm_follower_guidance_optimizer_node.cpp)

### 3.5 多机启动与仿真/实机配置

- [src/planner/plan_manage/launch/swarm_follower_agent.launch](src/planner/plan_manage/launch/swarm_follower_agent.launch)
- [src/planner/plan_manage/launch/swarm_multi_sim.launch](src/planner/plan_manage/launch/swarm_multi_sim.launch)
- [src/planner/plan_manage/launch/swarm_follower_local_map_sim.yaml](src/planner/plan_manage/launch/swarm_follower_local_map_sim.yaml)
- [src/planner/plan_manage/launch/swarm_follower_local_map_real.yaml](src/planner/plan_manage/launch/swarm_follower_local_map_real.yaml)
- [src/planner/plan_manage/launch/swarm_multi_real_template.launch](src/planner/plan_manage/launch/swarm_multi_real_template.launch)

### 3.6 自检（Preflight）

- [src/planner/plan_manage/src/swarm_preflight_check_node.cpp](src/planner/plan_manage/src/swarm_preflight_check_node.cpp)
- [src/planner/plan_manage/launch/swarm_preflight_check.launch](src/planner/plan_manage/launch/swarm_preflight_check.launch)

---

## 4. 关键通信接口说明

### 4.1 状态上报消息 `SwarmAgentState`

用途：从机向主机报告当前状态。常见字段语义包括：

- `header.stamp`：状态时间戳
- `agent_id`：智能体编号
- 位姿/速度相关字段
- `is_valid`：当前状态是否有效

### 4.2 主从核心 topics

- 状态回传：
	- `/swarm/agent_1/state`
	- `/swarm/agent_2/state`

- 主机引导下发：
	- `/swarm/agent_1/guidance_bezier`
	- `/swarm/agent_2/guidance_bezier`

- 本机最终执行轨迹：
	- `/planning/bezier`

> 注：topic 命名与数量可扩展，但建议保持“编号可推导、语义固定”。

---

## 5. 各节点行为详解

### 5.1 `swarm_master_node`

职责：

1. 订阅所有从机 `SwarmAgentState`。
2. 根据编队参数生成目标偏移（例如人字形）。
3. 将编队偏移映射到引导 Bezier 控制点。
4. 向每个从机的 `guidance_bezier` topic 发布。

实现特点：

- 采用统一协调器类管理状态缓存与轨迹生成。
- 预留代价项（如指数防碰撞惩罚）扩展口。

### 5.2 `swarm_agent_state_reporter_node`

职责：

1. 订阅本机里程计（仿真或实机映射）。
2. 封装为 `SwarmAgentState`。
3. 发布到本机对应 `/swarm/agent_X/state`。

意义：将主机输入标准化，减少主机侧传感器依赖差异。

### 5.3 `swarm_follower_guidance_optimizer_node`

职责：

1. 订阅主机下发 `guidance_bezier`。
2. 将引导转化为本地优化初值或软约束。
3. 基于本地地图/动力学进行二次优化。
4. 发布优化后的 `planning/bezier` 给执行链路。

意义：

- 保证集群协调意图存在；
- 同时保留单机局部安全与可行性。

### 5.4 `swarm_preflight_check_node`

职责：

1. 检查关键 topic 是否已被发布。
2. 检查状态与轨迹消息是否超时。
3. 检查状态时间戳延迟是否超阈值。
4. 检查 Bezier `start_time` 裕量是否足够。
5. 输出汇总状态到 `preflight/summary`。

当前输出语义：

- `PRECHECK_OK`
- `PRECHECK_WARN_ERR: err=... warn=...`

---

## 6. launch 组织策略

### 6.1 主机单独启动

- [src/planner/plan_manage/launch/swarm_master_only.launch](src/planner/plan_manage/launch/swarm_master_only.launch)

用途：只验证主机编队协调与引导发布。

### 6.2 从机单实例入口

- [src/planner/plan_manage/launch/swarm_follower_agent.launch](src/planner/plan_manage/launch/swarm_follower_agent.launch)

用途：一个 follower 的标准启动模板（可多次 include 并参数化）。

### 6.3 多机仿真入口

- [src/planner/plan_manage/launch/swarm_multi_sim.launch](src/planner/plan_manage/launch/swarm_multi_sim.launch)

用途：快速仿真回归，验证 Master + 多 Follower 全流程。

### 6.4 实机模板入口

- [src/planner/plan_manage/launch/swarm_multi_real_template.launch](src/planner/plan_manage/launch/swarm_multi_real_template.launch)

用途：保持同一系统结构，仅替换传感器/topic 映射。

### 6.5 自检入口

- [src/planner/plan_manage/launch/swarm_preflight_check.launch](src/planner/plan_manage/launch/swarm_preflight_check.launch)

用途：一键拉起系统并持续监控“可飞行就绪性”。

---

## 7. 仿真与实机统一策略（关键）

在从机入口引入两类参数：

- `use_local_map`
- `use_real_data`

并通过 yaml 模板拆分：

- 仿真：`swarm_follower_local_map_sim.yaml`
- 实机：`swarm_follower_local_map_real.yaml`

收益：

1. 业务节点逻辑尽量不分叉；
2. 仅通过 launch/yaml 切换数据源；
3. 便于回归测试与现场切换。

---

## 8. 已完成的阶段性成果

### 阶段 A：主机骨架

- 主机节点可接收状态并发布多路引导 Bezier。

### 阶段 B：从机状态回传

- 每个从机可标准化上报 `SwarmAgentState`。

### 阶段 C：从机二次优化链路

- 引导轨迹可注入本地优化并产出 `planning/bezier`。

### 阶段 D：双配置增强（sim/real）

- 启动层完成仿真/实机切换抽象。

### 阶段 E：Preflight 自检闭环

- 已具备 topic/时效/有效性等基础自检能力。

---

## 9. 运行与验证建议

### 9.1 编译

建议在工程根目录编译：

- `catkin_make --pkg ego_planner -j4`

### 9.2 推荐验证顺序

1. 先跑多机仿真入口，确认主从消息收发稳定；
2. 再跑 preflight 入口，观察 `preflight/summary` 长时间稳定；
3. 最后切换到 real template，仅替换数据源映射验证。

### 9.3 常见观察项

- `state` 是否持续更新且无超时
- `guidance_bezier` 是否持续下发
- follower 是否稳定输出 `planning/bezier`
- `start_time` 裕量是否长期为正并高于阈值

---

## 10. 当前已知局限与改进方向

### 10.1 局限

1. 目前自检主要“告警提示”，尚未硬性阻塞“arm/起飞动作”。
2. 编队策略仍偏基础（可继续引入任务分配、拓扑保持、避障协同约束）。
3. 时钟同步策略在实机上仍需结合硬件/PTP/外部同步源强化。

### 10.2 下一步（你已确认要做）

第六步目标：

- 将 `preflight/summary` 接入**阻塞起飞逻辑**；
- 未通过时禁止发出 arm/起飞触发；
- 仅当连续若干周期 `PRECHECK_OK` 后解除阻塞。

建议实现方式：

1. 新增 `swarm_takeoff_gate_node`（网关节点）；
2. 订阅 `preflight/summary` 与人工/任务触发信号；
3. 当且仅当健康状态满足阈值时，转发到真实 arm/takeoff topic 或 service；
4. 记录阻塞原因并输出可读日志（便于现场排障）。

---

## 11. 维护建议（工程化）

1. **参数集中化**：将阈值（超时、delay、start_margin）统一到一个 yaml。  
2. **日志标准化**：为每类故障定义编号（例如 `PF-001`）。  
3. **回归脚本化**：做一键 smoke test（启动->观察->退出）。  
4. **消息契约冻结**：版本化 `SwarmAgentState`，避免字段漂移。  
5. **CI 编译检查**：至少保证 `catkin_make --pkg ego_planner` 自动通过。

---

## 12. 快速索引

- 主机参数入口：
	- [src/planner/plan_manage/launch/swarm_master_params.yaml](src/planner/plan_manage/launch/swarm_master_params.yaml)
- 主机单机调试入口：
	- [src/planner/plan_manage/launch/swarm_master_only.launch](src/planner/plan_manage/launch/swarm_master_only.launch)
- 多机仿真入口：
	- [src/planner/plan_manage/launch/swarm_multi_sim.launch](src/planner/plan_manage/launch/swarm_multi_sim.launch)
- 实机模板入口：
	- [src/planner/plan_manage/launch/swarm_multi_real_template.launch](src/planner/plan_manage/launch/swarm_multi_real_template.launch)
- 自检入口：
	- [src/planner/plan_manage/launch/swarm_preflight_check.launch](src/planner/plan_manage/launch/swarm_preflight_check.launch)

---

## 13. 结论

本次“集群主机代码编写”已从“单点功能”推进到“可联调系统”：

- 有主机协调；
- 有从机状态回传；
- 有引导 + 本地二次优化；
- 有仿真/实机切换；
- 有上机前基础自检。

你后续只需在此基础上补上“起飞阻塞网关（第六步）”，即可形成更完整的实飞安全闭环。

---

## 14. 第六步已落地内容（阻塞起飞/触发门控）

本次已完成第六步代码接入，核心是“**预检不通过，阻塞任务触发与轨迹执行**”。

### 14.1 新增节点

- [src/planner/plan_manage/src/swarm_takeoff_gate_node.cpp](src/planner/plan_manage/src/swarm_takeoff_gate_node.cpp)

节点能力：

1. 订阅 `/swarm/preflight/ok`；
2. 订阅原始触发 topic（默认 `/traj_start_trigger`）；
3. 仅当预检连续通过达到阈值时，转发到门控后 topic（默认 `/swarm/traj_start_trigger_gated`）；
4. 提供服务 `/swarm/takeoff_gate/can_start`（`std_srvs/Trigger`）用于上层检查“是否允许起飞/启动任务”。

### 14.2 构建系统变更

- [src/planner/plan_manage/CMakeLists.txt](src/planner/plan_manage/CMakeLists.txt)

新增目标：

- `swarm_takeoff_gate_node`

### 14.3 启动链路接线变更

- [src/planner/plan_manage/launch/simple_run.launch](src/planner/plan_manage/launch/simple_run.launch)

变更点：

1. 新增参数：
	- `traj_start_trigger_raw_topic`（默认 `/traj_start_trigger`）
	- `traj_start_trigger_gated_topic`（默认 `/swarm/traj_start_trigger_gated`）
2. 新增节点 `swarm_takeoff_gate_node`；
3. 将 `waypoint_generator` 的 `~traj_start_trigger` 改为订阅门控后 topic。

### 14.4 与既有第六步增强协同

当前形成双保险：

1. **触发前门控**：`swarm_takeoff_gate_node` 不放行 `traj_start_trigger`。  
2. **执行侧门控**：`traj_server` 已启用 `require_preflight_ok` 时拒绝新轨迹并输出 hold 指令。

即便误触发，也会在执行层被拦截，减少风险。


