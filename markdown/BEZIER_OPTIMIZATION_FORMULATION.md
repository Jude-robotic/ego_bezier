# 贝塞尔优化器当前目标函数与约束条件说明

本文基于以下代码路径整理当前实现中的优化目标、约束与几何定义：

- `src/planner/bezier_opt/src/bezier_optimizer.cpp`
- `src/planner/bezier_opt/src/cooperative_payload_optimizer.cpp`
- `src/planner/bezier_opt/src/payload_geometry.cpp`
- `src/planner/plan_manage/src/swarm_master_coordinator.cpp`
- `src/planner/plan_manage/launch/swarm_nonehall.launch`
- `src/planner/plan_manage/launch/swarm_master_params.yaml`

本文只描述“当前代码真实实现”的数学形式，不对尚未实现的理想模型做额外扩展。

## 1. 轨迹参数化

系统使用分段三阶贝塞尔曲线表示位置轨迹。设共有 $N_s$ 段，则总控制点数为

$$
N_c = 3N_s + 1.
$$

第 $k$ 段（$k=0,\dots,N_s-1$）使用控制点

$$
\mathbf{P}_{3k},\mathbf{P}_{3k+1},\mathbf{P}_{3k+2},\mathbf{P}_{3k+3}\in\mathbb{R}^3,
$$

段内局部参数记为 $u\in[0,1]$，每段时长记为 $T>0$。则该段曲线为

$$
\mathbf{B}_k(u)=\sum_{r=0}^{3}\beta_r(u)\,\mathbf{P}_{3k+r},
$$

其中伯恩斯坦基函数为

$$
\beta_0(u)=(1-u)^3,\quad
\beta_1(u)=3(1-u)^2u,\quad
\beta_2(u)=3(1-u)u^2,\quad
\beta_3(u)=u^3.
$$

相应地，三阶贝塞尔曲线的速度与加速度控制点为

$$
\mathbf{V}_{k,0}=\frac{3}{T}\left(\mathbf{P}_{3k+1}-\mathbf{P}_{3k}\right),\quad
\mathbf{V}_{k,1}=\frac{3}{T}\left(\mathbf{P}_{3k+2}-\mathbf{P}_{3k+1}\right),\quad
\mathbf{V}_{k,2}=\frac{3}{T}\left(\mathbf{P}_{3k+3}-\mathbf{P}_{3k+2}\right),
$$

$$
\mathbf{A}_{k,0}=\frac{6}{T^2}\left(\mathbf{P}_{3k+2}-2\mathbf{P}_{3k+1}+\mathbf{P}_{3k}\right),\quad
\mathbf{A}_{k,1}=\frac{6}{T^2}\left(\mathbf{P}_{3k+3}-2\mathbf{P}_{3k+2}+\mathbf{P}_{3k+1}\right).
$$

## 2. 单机贝塞尔优化器 `BezierOptimizer`

`BezierOptimizer` 当前实现了三类代价组合：

1. `REBOUND`：避障重规划
2. `REFINE`：参考轨迹拟合/细化
3. `FORMATION`：从机编队保持 + 避障优化

### 2.1 总目标函数

`REBOUND` 模式：

$$
J_{\mathrm{rebound}}
=\lambda_1 J_{\mathrm{smooth}}
+\tilde{\lambda}_2 J_{\mathrm{dist}}
+\lambda_3 J_{\mathrm{feas}}.
$$

`REFINE` 模式：

$$
J_{\mathrm{refine}}
=\lambda_1 J_{\mathrm{smooth}}
+\lambda_4 J_{\mathrm{fit}}
+\lambda_3 J_{\mathrm{feas}}.
$$

`FORMATION` 模式：

$$
J_{\mathrm{form}}
=\lambda_1 J_{\mathrm{smooth}}
+\tilde{\lambda}_2 J_{\mathrm{dist}}
+\lambda_3 J_{\mathrm{feas}}
+\lambda_5 J_{\mathrm{formation}}.
$$

其中 $\tilde{\lambda}_2$ 是内部碰撞权重寄存器，当前代码中初始化为 $\lambda_2$，本文按“当前实现”等同处理。

### 2.2 平滑项 $J_{\mathrm{smooth}}$

当前代码的平滑项由“分段 jerk 惩罚”和“段间连续性惩罚”组成。

#### 2.2.1 Jerk 惩罚

对每段定义三阶差分

$$
\Delta^3\mathbf{P}_k
=\mathbf{P}_{3k+3}-3\mathbf{P}_{3k+2}+3\mathbf{P}_{3k+1}-\mathbf{P}_{3k}.
$$

代码中使用固定权重 $10.0$，故 jerk 代价写为

$$
J_{\mathrm{jerk}}
=10\sum_{k=0}^{N_s-1}\left\|\Delta^3\mathbf{P}_k\right\|_2^2.
$$

注意：代码注释里保留了按 $T^{-5}$ 缩放的旧形式，但当前真正执行的是固定权重 $10.0$。

#### 2.2.2 段间速度连续项

对相邻段连接处，代码使用

$$
\mathbf{c}^{(v)}_k
=\mathbf{P}_{3k+4}-2\mathbf{P}_{3k+3}+\mathbf{P}_{3k+2},
\qquad k=0,\dots,N_s-2.
$$

并构造代价

$$
J_{\mathrm{cont},v}
=w_v\sum_{k=0}^{N_s-2}\left\|\mathbf{c}^{(v)}_k\right\|_2^2,
\qquad w_v=10000.
$$

该式对应三阶贝塞尔在拼接点的速度连续条件。

#### 2.2.3 段间加速度连续项

对相邻段连接处，代码使用

$$
\mathbf{c}^{(a)}_k
=\mathbf{P}_{3k+1}-2\mathbf{P}_{3k+2}+2\mathbf{P}_{3k+4}-\mathbf{P}_{3k+5},
\qquad k=0,\dots,N_s-2.
$$

代价为

$$
J_{\mathrm{cont},a}
=w_a\sum_{k=0}^{N_s-2}\left\|\mathbf{c}^{(a)}_k\right\|_2^2,
\qquad w_a=10000.
$$

该式对应三阶贝塞尔在拼接点的加速度连续条件。

因此

$$
J_{\mathrm{smooth}}
=J_{\mathrm{jerk}}+J_{\mathrm{cont},v}+J_{\mathrm{cont},a}.
$$

### 2.3 避障项 $J_{\mathrm{dist}}$

`calcDistanceCostRebound()` 对每个控制点 $\mathbf{P}_i$ 收集若干障碍物支撑信息：

- 障碍体素中心 $\mathbf{b}_{i,j}\in\mathbb{R}^3$
- 从障碍体素指向控制点的单位方向 $\mathbf{n}_{i,j}\in\mathbb{R}^3$

这里 $j\in\mathcal{M}_i$，$\mathcal{M}_i$ 表示控制点 $i$ 的有效射线命中集合。定义投影距离

$$
d_{i,j}
=\left(\mathbf{P}_i-\mathbf{b}_{i,j}\right)^\top \mathbf{n}_{i,j},
$$

安全距离阈值为 $d_0$，误差定义为

$$
e_{i,j}=d_0-d_{i,j}.
$$

代码中的单项代价是一个分段函数

$$
\phi_{\mathrm{obs}}(e;d_0)=
\begin{cases}
0, & e<0,\\
e^3, & 0\le e<d_0,\\
3d_0e^2-3d_0^2e+d_0^3, & e\ge d_0.
\end{cases}
$$

因此障碍距离项为

$$
J_{\mathrm{obs}}
=\sum_{i=0}^{N_c-1}\sum_{j\in\mathcal{M}_i}\phi_{\mathrm{obs}}(e_{i,j};d_0).
$$

此外，代码还显式加入地面净空惩罚。设最低允许高度为

$$
z_{\min}=0.3,
$$

则地面项为

$$
J_{\mathrm{ground}}
=1000\sum_{i=0}^{N_c-1}\left[z_{\min}-P_{i,z}\right]_+^3.
$$

这里 $P_{i,z}$ 表示控制点 $\mathbf{P}_i$ 的 $z$ 坐标，$[x]_+=\max(x,0)$。

故总避障项为

$$
J_{\mathrm{dist}}=J_{\mathrm{obs}}+J_{\mathrm{ground}}.
$$

### 2.4 动力学可行项 $J_{\mathrm{feas}}$

代码对速度和加速度采用“逐坐标轴超限二次罚函数”，而不是对向量范数限幅。

设 $v_{\max}>0$、$a_{\max}>0$ 分别是最大速度和最大加速度阈值，则

$$
J_{\mathrm{vel}}
=\sum_{k=0}^{N_s-1}\sum_{r=0}^{2}\sum_{d\in\{x,y,z\}}
\left[\left|V_{k,r,d}\right|-v_{\max}\right]_+^2,
$$

$$
J_{\mathrm{acc}}
=\sum_{k=0}^{N_s-1}\sum_{r=0}^{1}\sum_{d\in\{x,y,z\}}
\left[\left|A_{k,r,d}\right|-a_{\max}\right]_+^2.
$$

于是

$$
J_{\mathrm{feas}}=J_{\mathrm{vel}}+J_{\mathrm{acc}}.
$$

### 2.5 参考拟合项 $J_{\mathrm{fit}}$

若给定参考控制点 $\mathbf{R}_i\in\mathbb{R}^3$，则

$$
J_{\mathrm{fit}}
=\sum_{i=0}^{N_c-1}\left\|\mathbf{P}_i-\mathbf{R}_i\right\|_2^2.
$$

### 2.6 编队保持项 $J_{\mathrm{formation}}$

若给定理想编队参考控制点 $\mathbf{F}_i\in\mathbb{R}^3$，则

$$
J_{\mathrm{formation}}
=\sum_{i=0}^{N_c-1}\left\|\mathbf{P}_i-\mathbf{F}_i\right\|_2^2.
$$

该项在 `FORMATION` 模式中作为软约束使用，代码默认思想是“安全优先、编队次之”。

### 2.7 `swarm_master` 中从机优化实际采用的参数

在 `SwarmMasterCoordinator::optimizeFollowerTrajectory()` 中，`FORMATION` 模式被直接用于从机引导轨迹优化。当前硬编码/参数映射为

$$
\lambda_1=10,\quad
\tilde{\lambda}_2=500,\quad
\lambda_3=1,\quad
\lambda_4=1,\quad
\lambda_5=\texttt{formation\_weight},
$$

$$
d_0=\texttt{follower\_clearance},\quad
v_{\max}=\texttt{follower\_max\_vel},\quad
a_{\max}=3.0.
$$

在默认 `swarm_master_params.yaml` 下，

$$
\lambda_5=20,\quad d_0=0.4,\quad v_{\max}=2.0,\quad a_{\max}=3.0.
$$

## 3. 集群参考几何与优化外约束

这一部分不全部属于 `BezierOptimizer` 本身，但它们决定了“从机优化的参考形状”和“优化后的额外修正”，所以必须一并考虑。

### 3.1 默认水平 V 形编队

设主机在控制点 $c$ 处的平滑航向单位向量为 $\mathbf{e}^{\mathrm{fwd}}_c$，对应左向单位向量为

$$
\mathbf{e}^{\mathrm{left}}_c=
\frac{(-e^{\mathrm{fwd}}_{c,y},\,e^{\mathrm{fwd}}_{c,x},\,0)^\top}
{\left\|(-e^{\mathrm{fwd}}_{c,y},\,e^{\mathrm{fwd}}_{c,x},\,0)^\top\right\|_2}.
$$

对第 $m$ 个从机，若其层级为 $\ell_m$、左右符号为 $s_m\in\{+1,-1\}$，则默认偏置为

$$
d^{\mathrm{back}}_m=\ell_m\,d_{\mathrm{form}}\cos\theta,\qquad
d^{\mathrm{lat}}_m=s_m\,\ell_m\,d_{\mathrm{form}}\sin\theta,
$$

$$
\Delta\mathbf{p}^{\mathrm{default}}_{m,c}
=-d^{\mathrm{back}}_m\mathbf{e}^{\mathrm{fwd}}_c
+d^{\mathrm{lat}}_m\mathbf{e}^{\mathrm{left}}_c
+z_{\mathrm{off}}\mathbf{e}_z.
$$

其中 $d_{\mathrm{form}}$ 是编队间距，$\theta$ 是编队夹角，$z_{\mathrm{off}}$ 是统一竖直偏置。

### 3.2 障碍自适应编队模式

当前 `SwarmMasterCoordinator::generateFormationGuidance()` 支持以下三类模式，并通过每个控制点的混合因子 $\alpha_c\in[0,1]$ 平滑切换：

#### 3.2.1 门框型 `DOOR_FRAME`

将原本横向展开的水平 V 形逐渐抬升为竖向 V 形：

$$
y_{m,c}=d^{\mathrm{lat}}_m(1-\alpha_c),\qquad
z_{m,c}=d^{\mathrm{lat}}_m\alpha_c,
$$

$$
\Delta\mathbf{p}^{\mathrm{door}}_{m,c}
=-d^{\mathrm{back}}_m\mathbf{e}^{\mathrm{fwd}}_c
+y_{m,c}\mathbf{e}^{\mathrm{left}}_c
+(z_{m,c}+z_{\mathrm{off}})\mathbf{e}_z.
$$

#### 3.2.2 Z 向窄缝 `Z_SLIT`

保持水平编队，但在前向和横向同时放大：

$$
s_c=1+\alpha_c(\kappa_{\mathrm{slit}}-1),
$$

$$
\Delta\mathbf{p}^{\mathrm{slit}}_{m,c}
=-s_c d^{\mathrm{back}}_m\mathbf{e}^{\mathrm{fwd}}_c
+s_c d^{\mathrm{lat}}_m\mathbf{e}^{\mathrm{left}}_c
+z_{\mathrm{off}}\mathbf{e}_z.
$$

其中 $\kappa_{\mathrm{slit}}=\texttt{slit\_spread\_factor}$。

#### 3.2.3 圆环 `RING`

将编队压缩为较紧凑的纵列/人字形：

$$
\eta_c=(1-\alpha_c)+\alpha_c\kappa_{\mathrm{ring}},
$$

$$
d^{\mathrm{back,ring}}_{m,c}
=(1-\alpha_c)d^{\mathrm{back}}_m+\alpha_c\,\ell_m\,d_{\mathrm{ring}},
$$

$$
d^{\mathrm{lat,ring}}_{m,c}=\eta_c d^{\mathrm{lat}}_m,
$$

$$
\Delta\mathbf{p}^{\mathrm{ring}}_{m,c}
=-d^{\mathrm{back,ring}}_{m,c}\mathbf{e}^{\mathrm{fwd}}_c
+d^{\mathrm{lat,ring}}_{m,c}\mathbf{e}^{\mathrm{left}}_c
+(1-\alpha_c)z_{\mathrm{off}}\mathbf{e}_z.
$$

其中 $d_{\mathrm{ring}}=\texttt{ring\_longitudinal\_spacing}$，$\kappa_{\mathrm{ring}}=\texttt{ring\_compact\_lateral\_scale}$。

### 3.3 集群间距修正（非 L-BFGS）

`applySwarmCollisionPenalty()` 不是 L-BFGS 目标的一部分，而是对生成后的从机控制点做额外迭代推开。若第 $i,j$ 个从机在同一控制点列 $c$ 处距离

$$
d_{ij,c}=\left\|\mathbf{p}_{i,c}-\mathbf{p}_{j,c}\right\|_2
$$

小于安全半径 $r_{\mathrm{safe}}$，则误差

$$
e_{ij,c}=r_{\mathrm{safe}}-d_{ij,c}>0.
$$

代码中构造有界二次增益

$$
g_{ij,c}=\min\left(w_{\mathrm{coll}}e_{ij,c}^2,\ g_{\max}\right),
$$

沿连线方向对两个控制点做对称更新。这里 $w_{\mathrm{coll}}=\texttt{collision\_weight}$，$g_{\max}=\texttt{collision\_gain\_cap}$。

### 3.4 边界连续性修正（非 L-BFGS）

`applyLeaderBoundaryCorrection()` 和 `applyFollowerBoundaryCorrections()` 会在发布前将首端控制点与当前状态对齐，核心关系是

$$
\mathbf{P}_1=\mathbf{P}_0+\frac{T}{p}\mathbf{v}_0,
$$

其中 $p=3$ 是贝塞尔阶数，$\mathbf{v}_0$ 是当前速度估计。这是系统层面的 $C^0/C^1$ 修正，不属于优化目标项本身。

## 4. 三机协同吊运负载优化器 `CooperativePayloadOptimizer`

该优化器用于主机 + 两个从机协同通过狭窄障碍或执行在线负载窗口修正。

### 4.1 决策变量与固定边界

设三架无人机集合为

$$
\mathcal{A}=\{L,1,2\},
$$

分别对应主机、从机 1、从机 2。对每架无人机 $a\in\mathcal{A}$，其控制点矩阵记为

$$
\mathbf{Q}^{(a)}=
\left[\mathbf{Q}^{(a)}_0,\mathbf{Q}^{(a)}_1,\dots,\mathbf{Q}^{(a)}_{N_c-1}\right]\in\mathbb{R}^{3\times N_c}.
$$

当前代码只把中间控制点作为优化变量：

$$
\mathcal{V}
=\left\{\mathbf{Q}^{(a)}_j\mid a\in\mathcal{A},\ 2\le j\le N_c-3\right\}.
$$

因此边界点满足硬约束

$$
\mathbf{Q}^{(a)}_0,\ \mathbf{Q}^{(a)}_1,\ \mathbf{Q}^{(a)}_{N_c-2},\ \mathbf{Q}^{(a)}_{N_c-1}
\text{ 固定不动}.
$$

这保证了窗口两端的位置与一阶边界速度结构不会被在线协同优化破坏。

### 4.2 采样点定义

协同优化不是只在控制点上工作，而是对每段进行均匀采样。若每段采样密度为 $n_s=\texttt{samples\_per\_seg}$，则对每段 $k$ 和采样索引 $\sigma=0,\dots,n_s$，

$$
u_\sigma=\frac{\sigma}{n_s},
$$

$$
\mathbf{x}^{(a)}_{k,\sigma}
=\sum_{r=0}^{3}\beta_r(u_\sigma)\,\mathbf{Q}^{(a)}_{3k+r}.
$$

其中相邻段共享端点样本只保留一次。

### 4.3 协同总目标函数

当前实现的总代价为

$$
J_{\mathrm{coop}}
=w_{\mathrm{s}}J_{\mathrm{smooth}}^{\mathrm{coop}}
+w_{\mathrm{f}}J_{\mathrm{feas}}^{\mathrm{coop}}
+w_{\mathrm{o}}J_{\mathrm{uav\mbox{-}obs}}
+w_{\mathrm{r}}J_{\mathrm{leader\mbox{-}ref}}
+w_{\mathrm{m}}J_{\mathrm{mode}}
+w_{\mathrm{pf}}J_{\mathrm{payload\mbox{-}feas}}
+w_{\mathrm{po}}J_{\mathrm{payload\mbox{-}obs}}
+w_{\mathrm{sep}}J_{\mathrm{sep}}.
$$

默认权重来自 `swarm_master_params.yaml`：

$$
\begin{aligned}
w_{\mathrm{s}}&=10, & w_{\mathrm{f}}&=1, & w_{\mathrm{o}}&=100, & w_{\mathrm{r}}&=50,\\
w_{\mathrm{m}}&=30, & w_{\mathrm{pf}}&=120, & w_{\mathrm{po}}&=160, & w_{\mathrm{sep}}&=40.
\end{aligned}
$$

下面给出每一项的实现公式。

### 4.4 平滑项 $J_{\mathrm{smooth}}^{\mathrm{coop}}$

协同优化对三架无人机分别施加与单机类似的平滑罚，但权重略有不同。

对任意 $a\in\mathcal{A}$，

$$
\Delta^3\mathbf{Q}^{(a)}_k
=\mathbf{Q}^{(a)}_{3k+3}-3\mathbf{Q}^{(a)}_{3k+2}+3\mathbf{Q}^{(a)}_{3k+1}-\mathbf{Q}^{(a)}_{3k},
$$

$$
J_{\mathrm{jerk}}^{(a)}
=\sum_{k=0}^{N_s-1}\frac{10}{\max(T^3,10^{-3})}
\left\|\Delta^3\mathbf{Q}^{(a)}_k\right\|_2^2.
$$

段间连续性项为

$$
J_{\mathrm{cont},v}^{(a)}
=3000\sum_{k=0}^{N_s-2}
\left\|\mathbf{Q}^{(a)}_{3k+4}-2\mathbf{Q}^{(a)}_{3k+3}+\mathbf{Q}^{(a)}_{3k+2}\right\|_2^2,
$$

$$
J_{\mathrm{cont},a}^{(a)}
=3000\sum_{k=0}^{N_s-2}
\left\|\mathbf{Q}^{(a)}_{3k+1}-2\mathbf{Q}^{(a)}_{3k+2}+2\mathbf{Q}^{(a)}_{3k+4}-\mathbf{Q}^{(a)}_{3k+5}\right\|_2^2.
$$

所以

$$
J_{\mathrm{smooth}}^{\mathrm{coop}}
=\sum_{a\in\mathcal{A}}
\left(
J_{\mathrm{jerk}}^{(a)}+J_{\mathrm{cont},v}^{(a)}+J_{\mathrm{cont},a}^{(a)}
\right).
$$

### 4.5 动力学项 $J_{\mathrm{feas}}^{\mathrm{coop}}$

对每个 agent、每一段、每一维坐标，使用与单机同形的逐轴二次超限罚函数：

$$
J_{\mathrm{feas}}^{\mathrm{coop}}
=\sum_{a\in\mathcal{A}}
\left(
\sum_{k,r,d}\left[\left|V^{(a)}_{k,r,d}\right|-v_{\max}\right]_+^2
+\sum_{k,r,d}\left[\left|A^{(a)}_{k,r,d}\right|-a_{\max}\right]_+^2
\right).
$$

当前协同优化默认参数为

$$
v_{\max}=2.0,\qquad a_{\max}=3.0.
$$

### 4.6 无人机对地图避障项 $J_{\mathrm{uav\mbox{-}obs}}$

对三架无人机的控制点逐个做地图净空约束。定义点到最近膨胀占据体素的距离为

$$
d_{\mathrm{occ}}(\mathbf{p}),
$$

安全净空为 $c_{\mathrm{uav}}$，则

$$
\phi_{\mathrm{map}}(\mathbf{p};c_{\mathrm{uav}})
=\left[c_{\mathrm{uav}}-d_{\mathrm{occ}}(\mathbf{p})\right]_+^2.
$$

于是

$$
J_{\mathrm{uav\mbox{-}obs}}
=\sum_{a\in\mathcal{A}}\sum_{j=0}^{N_c-1}
\phi_{\mathrm{map}}\!\left(\mathbf{Q}^{(a)}_j;c_{\mathrm{uav}}\right).
$$

若点超出地图包围盒 $\mathcal{B}$，代码会先投影到盒边界 $\Pi_{\mathcal{B}}(\mathbf{p})$，并使用

$$
\phi_{\mathrm{out}}(\mathbf{p};c)
=\left(c+\left\|\mathbf{p}-\Pi_{\mathcal{B}}(\mathbf{p})\right\|_2\right)^2
$$

作为出界惩罚。

### 4.7 主机参考项 $J_{\mathrm{leader\mbox{-}ref}}$

设主机名义控制点为 $\bar{\mathbf{Q}}^{(L)}$，则

$$
J_{\mathrm{leader\mbox{-}ref}}
=\left\|\mathbf{Q}^{(L)}-\bar{\mathbf{Q}}^{(L)}\right\|_F^2.
$$

这里 $\|\cdot\|_F$ 表示 Frobenius 范数。

### 4.8 模式形状项 $J_{\mathrm{mode}}$

对固定障碍模板或在线负载窗口，代码都会构造一个局部编队模式。定义模式参考偏置

$$
\boldsymbol{\delta}_1
=-b\,\mathbf{e}^{\mathrm{fwd}}
+\frac{s_p}{2}\mathbf{e}^{\mathrm{pri}}
+s_a\mathbf{e}^{\mathrm{aux}},
$$

$$
\boldsymbol{\delta}_2
=-b\,\mathbf{e}^{\mathrm{fwd}}
-\frac{s_p}{2}\mathbf{e}^{\mathrm{pri}}
-s_a\mathbf{e}^{\mathrm{aux}},
$$

其中 $b=\texttt{back\_offset}$，$s_p=\texttt{primary\_span}$，$s_a=\texttt{aux\_span}$。

若主机名义采样点为 $\bar{\mathbf{x}}^{(L)}_{k,\sigma}$，主机附加偏置为 $\mathbf{b}_L$，则

$$
J_{\mathrm{mode}}
=\sum_{k,\sigma}
\left\|\mathbf{x}^{(L)}_{k,\sigma}-\left(\bar{\mathbf{x}}^{(L)}_{k,\sigma}+\mathbf{b}_L\right)\right\|_2^2
$$

$$
\qquad
+\sum_{k,\sigma}
\left\|\mathbf{x}^{(1)}_{k,\sigma}-\left(\mathbf{x}^{(L)}_{k,\sigma}+\boldsymbol{\delta}_1\right)\right\|_2^2
+\sum_{k,\sigma}
\left\|\mathbf{x}^{(2)}_{k,\sigma}-\left(\mathbf{x}^{(L)}_{k,\sigma}+\boldsymbol{\delta}_2\right)\right\|_2^2.
$$

这一项体现了“集群形状约束”：不是简单保持原始 V 形，而是保持当前障碍模式下的理想几何关系。

### 4.9 吊运几何可行项 $J_{\mathrm{payload\mbox{-}feas}}$

对每个采样点，设三机位置分别为

$$
\mathbf{p}_0=\mathbf{x}^{(L)}_{k,\sigma},\qquad
\mathbf{p}_1=\mathbf{x}^{(1)}_{k,\sigma},\qquad
\mathbf{p}_2=\mathbf{x}^{(2)}_{k,\sigma}.
$$

#### 4.9.1 三角形面积约束

三机形成三角形的面积为

$$
A=\frac{1}{2}\left\|(\mathbf{p}_1-\mathbf{p}_0)\times(\mathbf{p}_2-\mathbf{p}_0)\right\|_2.
$$

要求

$$
A\ge A_{\min},
$$

对应软约束代价

$$
J_{\mathrm{area}}
=\sum_{k,\sigma}\left[A_{\min}-A_{k,\sigma}\right]_+^2.
$$

#### 4.9.2 绳长/外接圆半径约束

三点的外接圆半径记为 $R$。当前代码用几何公式和解析梯度计算，物理含义是：

- 若三架无人机到吊点的绳长统一为 $L_{\mathrm{rope}}$
- 则载荷中心存在的必要条件是

$$
R\le L_{\mathrm{rope}}.
$$

因此对应软约束代价为

$$
J_{\mathrm{rope}}
=\sum_{k,\sigma}\left[R_{k,\sigma}-L_{\mathrm{rope}}\right]_+^2.
$$

#### 4.9.3 无效几何重罚

若面积或外接圆半径的几何计算本身退化，则 `addPayloadFeasibilityCost()` 会加入额外重罚

$$
J_{\mathrm{invalid}}^{\mathrm{feas}}
=\sum_{k,\sigma}
\rho_{\mathrm{inv}}
\left(
1
+10\left[R_{k,\sigma}-L_{\mathrm{rope}}\right]_+
+5\left[A_{\min}-A_{k,\sigma}\right]_+
\right)^2,
$$

其中 $\rho_{\mathrm{inv}}=\texttt{payload\_invalid\_penalty}$，默认值为 $2000$。

于是

$$
J_{\mathrm{payload\mbox{-}feas}}
=J_{\mathrm{area}}+J_{\mathrm{rope}}+J_{\mathrm{invalid}}^{\mathrm{feas}}.
$$

### 4.10 吊点几何解与载荷中心

当几何有效时，当前代码按“下支路”求吊点。设三角形外接圆圆心为 $\mathbf{c}$、单位法向量为 $\mathbf{n}$、外接圆半径为 $R$，统一绳长为 $L_{\mathrm{rope}}$，则高度为

$$
h=\sqrt{L_{\mathrm{rope}}^2-R^2}.
$$

上下两个候选吊点为

$$
\mathbf{p}_{\mathrm{upper}}=\mathbf{c}+h\mathbf{n},\qquad
\mathbf{p}_{\mathrm{lower}}=\mathbf{c}-h\mathbf{n}.
$$

代码最终取

$$
\mathbf{p}_{\mathrm{payload}}
=
\arg\min_{\mathbf{p}\in\{\mathbf{p}_{\mathrm{upper}},\mathbf{p}_{\mathrm{lower}}\}}
(\mathbf{p})_z,
$$

即选择世界坐标 $z$ 更低的一支作为实际载荷中心。

### 4.11 载荷避障项 $J_{\mathrm{payload\mbox{-}obs}}$

设有效载荷半径为

$$
r_{\mathrm{eff}}=r_{\mathrm{payload}}+m_{\mathrm{payload}},
$$

其中 $r_{\mathrm{payload}}=\texttt{payload\_radius}$，$m_{\mathrm{payload}}=\texttt{payload\_extra\_margin}$。

#### 4.11.1 载荷对地图净空

对每个有效载荷中心 $\mathbf{p}_{\mathrm{payload}}$，代码加入

$$
J_{\mathrm{payload\mbox{-}map}}
=\sum_{k,\sigma}
\phi_{\mathrm{map}}\!\left(\mathbf{p}_{\mathrm{payload},k,\sigma};r_{\mathrm{eff}}\right).
$$

#### 4.11.2 障碍模板净空

若当前窗口带有门框、圆环或 Z 向窄缝模板，则代码还会计算有符号净空 $\chi$。当 $\chi<0$ 时，表示载荷侵入障碍物，需要惩罚

$$
J_{\mathrm{payload\mbox{-}template}}
=\sum_{k,\sigma}\left[\chi_{k,\sigma}\right]_-^2,
$$

其中 $[x]_-=\max(-x,0)$。

于是

$$
J_{\mathrm{payload\mbox{-}obs}}
=J_{\mathrm{payload\mbox{-}map}}+J_{\mathrm{payload\mbox{-}template}}+J_{\mathrm{invalid}}^{\mathrm{obs}}.
$$

这里 $J_{\mathrm{invalid}}^{\mathrm{obs}}$ 指 `payloadObstacleCostOnly()` 中的无效几何重罚。它的触发条件是“载荷中心求解失败”，因此包括：

1. 三角形退化；
2. 外接圆半径超过绳长，导致 $L_{\mathrm{rope}}^2-R^2<0$；
3. 其他导致 `solvePayloadCenterLowerBranch()` 返回无效的情况。

因此，退化几何会同时在 $J_{\mathrm{payload\mbox{-}feas}}$ 和 $J_{\mathrm{payload\mbox{-}obs}}$ 两路中受罚；而 $R>L_{\mathrm{rope}}$ 的情形则会在 $J_{\mathrm{payload\mbox{-}feas}}$ 中通过 $J_{\mathrm{rope}}$ 受罚，并在 $J_{\mathrm{payload\mbox{-}obs}}$ 中通过 $J_{\mathrm{invalid}}^{\mathrm{obs}}$ 再次受罚。

### 4.12 三类障碍模板的净空函数

为与代码一致，下面给出 `payload_geometry.cpp` 中的精确形式。设障碍局部坐标为

$$
(x_\ell,y_\ell,z_\ell)=\mathbf{T}_{\mathrm{local}}(\mathbf{p}_{\mathrm{payload}}),
$$

其中 $\mathbf{T}_{\mathrm{local}}$ 表示从世界坐标到障碍局部坐标系的刚体变换。

#### 4.12.1 门框 `DOOR_FRAME`

若厚度半宽为 $h_x$、门洞中心横向坐标为 $y_c$、门洞宽度为 $w_g$，则先定义板厚净空

$$
\chi_{\mathrm{slab}}^{\mathrm{door}}
=|x_\ell|-(h_x+r_{\mathrm{eff}}).
$$

若 $\chi_{\mathrm{slab}}^{\mathrm{door}}>0$，说明载荷中心已离开门框厚度区域，直接取该正净空；否则在门洞内部净空为

$$
\chi_{\mathrm{door}}
=\frac{w_g}{2}-r_{\mathrm{eff}}-|y_\ell-y_c|.
$$

#### 4.12.2 Z 向窄缝 `Z_SLIT`

若上下边界分别为 $z_{\mathrm{low}}$ 与 $z_{\mathrm{high}}$，厚度半宽为 $h_x$，则同样先定义

$$
\chi_{\mathrm{slab}}^{\mathrm{slit}}
=|x_\ell|-(h_x+r_{\mathrm{eff}}).
$$

若 $\chi_{\mathrm{slab}}^{\mathrm{slit}}>0$，直接返回该正净空；否则在缝内净空为

$$
\chi_{\mathrm{slit}}
=\min\left(z_\ell-z_{\mathrm{low}},\ z_{\mathrm{high}}-z_\ell\right)-r_{\mathrm{eff}}.
$$

#### 4.12.3 圆环 `RING`

若主半径为 $R_{\mathrm{maj}}$、管半径为 $R_{\mathrm{min}}$，则先定义环面有符号距离

$$
\chi_{\mathrm{torus}}
=\sqrt{\left(\sqrt{y_\ell^2+z_\ell^2}-R_{\mathrm{maj}}\right)^2+x_\ell^2}-R_{\mathrm{min}},
$$

再减去载荷半径，得到

$$
\chi_{\mathrm{ring}}
=\chi_{\mathrm{torus}}-r_{\mathrm{eff}}.
$$

### 4.13 无人机间最小间距项 $J_{\mathrm{sep}}$

对每个采样时刻、每一对无人机 $(a,b)$，若距离

$$
d^{(a,b)}_{k,\sigma}
=\left\|\mathbf{x}^{(a)}_{k,\sigma}-\mathbf{x}^{(b)}_{k,\sigma}\right\|_2
$$

小于最小间距 $d_{\mathrm{sep}}$，则加入代价

$$
J_{\mathrm{sep}}
=\sum_{k,\sigma}\sum_{a<b}
\left[d_{\mathrm{sep}}-d^{(a,b)}_{k,\sigma}\right]_+^2.
$$

当前默认值为

$$
d_{\mathrm{sep}}=\texttt{inter\_uav\_sep\_min}=0.8.
$$

### 4.14 梯度实现方式

当前协同优化器的梯度实现有两部分：

1. `smooth / feas / uav_obs / leader_ref / mode / payload_feas / sep` 使用解析梯度。
2. `payload_obs` 由于包含“载荷中心求解 + 障碍净空复合函数”，代码采用中心差分：

$$
\frac{\partial J_{\mathrm{payload\mbox{-}obs}}}{\partial x_i}
\approx
\frac{
J_{\mathrm{payload\mbox{-}obs}}(x_i+\varepsilon)
-J_{\mathrm{payload\mbox{-}obs}}(x_i-\varepsilon)
}{2\varepsilon},
$$

其中 $\varepsilon=\texttt{finite\_diff\_eps}$，默认值为 $10^{-3}$。

## 5. `swarm_nonehall.launch` 下的当前启用状态

`swarm_nonehall.launch` 代表“空走廊、无固定障碍模板”的场景。它加载 `swarm_master_params.yaml`，但又显式覆盖了以下关键参数：

$$
\texttt{use\_fixed\_obstacle\_schedule}=\mathrm{false},
\qquad
\texttt{online\_payload\_opt\_enabled}=\mathrm{false}.
$$

因此在该 launch 的默认配置下：

1. 集群编队几何仍然存在，默认 V 形参考和从机边界修正仍然生效。
2. 从机 `FORMATION` 模式避障优化仍可能被调用。
3. 三机协同吊运负载优化器虽然已经接入代码，但在该 launch 默认不在线启用。
4. 载荷可视化节点仍可工作，因为它不要求协同优化必须开启。

换言之，本文第 4 节描述的是“当前代码已实现的协同吊运负载优化模型”；而在 `swarm_nonehall.launch` 默认参数下，它属于“可用但关闭”的能力。

## 6. 约束与代价的归类总结

为了便于阅读，当前系统中的约束可以归纳为下表。

| 类型 | 数学形式 | 在代码中的体现 |
|---|---|---|
| 软约束 | jerk 最小化 | `J_smooth` |
| 软约束 | 段间速度/加速度连续 | `J_smooth` |
| 软约束 | 控制点障碍净空 | `J_dist`, `J_uav-obs` |
| 软约束 | 地面净空 | `J_dist` |
| 软约束 | 逐轴速度/加速度限幅 | `J_feas` |
| 软约束 | 主机参考保持 | `J_leader-ref` |
| 软约束 | 编队/模式形状保持 | `J_formation`, `J_mode` |
| 软约束 | 吊点三角形面积下界 | `J_payload-feas` |
| 软约束 | 绳长可实现性 $R\le L_{\mathrm{rope}}$ | `J_payload-feas` |
| 软约束 | 载荷对地图/门框/圆环/窄缝净空 | `J_payload-obs` |
| 软约束 | 无人机间最小采样间距 | `J_sep` |
| 硬约束 | 协同窗口首尾两组控制点固定 | `variable_layout_` 只优化列 $2\sim N_c-3$ |
| 系统级修正 | 起点位置/速度连续性 | `applyLeaderBoundaryCorrection`, `applyFollowerBoundaryCorrections` |
| 系统级修正 | 控制点级集群推开 | `applySwarmCollisionPenalty` |

## 7. 符号表

下面对本文出现的符号做统一解释。

| 符号 | 含义 |
|---|---|
| $N_s$ | 贝塞尔段数 |
| $N_c$ | 总控制点数，满足 $N_c=3N_s+1$ |
| $k$ | 段索引，$k=0,\dots,N_s-1$ |
| $u$ | 段内归一化参数，$u\in[0,1]$ |
| $T$ | 单段贝塞尔持续时间 |
| $\mathbf{P}_i$ | 单机优化器中的第 $i$ 个控制点 |
| $\mathbf{B}_k(u)$ | 第 $k$ 段贝塞尔曲线在参数 $u$ 处的位置 |
| $\beta_r(u)$ | 三阶伯恩斯坦基函数 |
| $\mathbf{V}_{k,r}$ | 第 $k$ 段速度控制点 |
| $\mathbf{A}_{k,r}$ | 第 $k$ 段加速度控制点 |
| $V_{k,r,d}$ | 速度控制点在坐标轴 $d\in\{x,y,z\}$ 上的分量 |
| $A_{k,r,d}$ | 加速度控制点在坐标轴 $d\in\{x,y,z\}$ 上的分量 |
| $\Delta^3\mathbf{P}_k$ | 第 $k$ 段的三阶差分，等价于常值 jerk 的几何核 |
| $\lambda_1,\lambda_2,\lambda_3,\lambda_4,\lambda_5$ | 单机优化器各代价项权重 |
| $\tilde{\lambda}_2$ | 单机优化器内部碰撞权重寄存器，当前代码初始化为 $\lambda_2$ |
| $J_{\mathrm{smooth}}$ | 单机平滑代价 |
| $J_{\mathrm{dist}}$ | 单机避障代价 |
| $J_{\mathrm{feas}}$ | 单机动力学可行代价 |
| $J_{\mathrm{fit}}$ | 单机参考轨迹拟合代价 |
| $J_{\mathrm{formation}}$ | 单机编队保持代价 |
| $\mathbf{b}_{i,j}$ | 控制点 $\mathbf{P}_i$ 对应的第 $j$ 个障碍支撑点（体素中心） |
| $\mathbf{n}_{i,j}$ | 从障碍支撑点指向控制点的单位方向 |
| $\mathcal{M}_i$ | 控制点 $i$ 的有效障碍支撑集合 |
| $d_{i,j}$ | 控制点到障碍支撑点在方向 $\mathbf{n}_{i,j}$ 上的投影距离 |
| $d_0$ | 单机避障安全距离阈值 |
| $e_{i,j}$ | 障碍距离误差，定义为 $d_0-d_{i,j}$ |
| $z_{\min}$ | 允许的最低飞行高度，当前实现中固定为 $0.3$ m |
| $v_{\max}$ | 允许最大速度阈值 |
| $a_{\max}$ | 允许最大加速度阈值 |
| $\mathbf{R}_i$ | 单机 `REFINE` 模式中的参考控制点 |
| $\mathbf{F}_i$ | 单机 `FORMATION` 模式中的理想编队参考控制点 |
| $\mathcal{A}$ | 三机集合 $\{L,1,2\}$ |
| $L,1,2$ | 分别表示主机、从机 1、从机 2 |
| $\mathbf{Q}^{(a)}_j$ | 协同优化中 agent $a$ 的第 $j$ 个控制点 |
| $\mathcal{V}$ | 协同优化真正参与优化的控制点变量集合 |
| $\mathbf{x}^{(a)}_{k,\sigma}$ | agent $a$ 在第 $k$ 段第 $\sigma$ 个采样点的空间位置 |
| $\mathbf{p}_{i,c}$ | 第 $i$ 个从机在控制点列 $c$ 处的位置 |
| $n_s$ | 每段采样数 `samples_per_seg` |
| $w_{\mathrm{s}},w_{\mathrm{f}},w_{\mathrm{o}},w_{\mathrm{r}},w_{\mathrm{m}},w_{\mathrm{pf}},w_{\mathrm{po}},w_{\mathrm{sep}}$ | 协同优化各代价项权重 |
| $J_{\mathrm{smooth}}^{\mathrm{coop}}$ | 三机总平滑代价 |
| $J_{\mathrm{feas}}^{\mathrm{coop}}$ | 三机总动力学可行代价 |
| $J_{\mathrm{uav\mbox{-}obs}}$ | 三机对地图的避障代价 |
| $J_{\mathrm{leader\mbox{-}ref}}$ | 主机控制点保持接近名义轨迹的代价 |
| $J_{\mathrm{mode}}$ | 集群模式形状代价 |
| $J_{\mathrm{payload\mbox{-}feas}}$ | 吊运几何可行代价 |
| $J_{\mathrm{payload\mbox{-}obs}}$ | 载荷避障代价 |
| $J_{\mathrm{sep}}$ | 无人机间最小距离代价 |
| $J_{\mathrm{area}}$ | 载荷三角形面积下界惩罚 |
| $J_{\mathrm{rope}}$ | 载荷绳长可实现性惩罚 |
| $J_{\mathrm{invalid}}^{\mathrm{feas}}$ | `payload_feas` 路径中的退化几何重罚 |
| $J_{\mathrm{payload\mbox{-}map}}$ | 载荷对地图净空惩罚 |
| $J_{\mathrm{payload\mbox{-}template}}$ | 载荷对门框/圆环/窄缝模板的净空惩罚 |
| $J_{\mathrm{invalid}}^{\mathrm{obs}}$ | `payload_obs` 路径中的载荷中心求解失败重罚 |
| $\bar{\mathbf{Q}}^{(L)}$ | 主机名义控制点轨迹 |
| $\bar{\mathbf{x}}^{(L)}_{k,\sigma}$ | 主机名义采样点位置 |
| $\mathbf{b}_L$ | 主机在协同模式中的附加偏置 |
| $\mathbf{e}^{\mathrm{fwd}}_c$ | 主机在控制点 $c$ 处的平滑前向单位向量 |
| $\mathbf{e}^{\mathrm{left}}_c$ | 与 $\mathbf{e}^{\mathrm{fwd}}_c$ 正交的水平左向单位向量 |
| $\mathbf{e}^{\mathrm{fwd}},\mathbf{e}^{\mathrm{pri}},\mathbf{e}^{\mathrm{aux}}$ | 协同障碍模板的前向、主轴、辅助轴单位向量 |
| $b$ | `back_offset`，从机相对主机的后退距离 |
| $s_p$ | `primary_span`，主轴方向展开尺度 |
| $s_a$ | `aux_span`，辅助轴方向展开尺度 |
| $\boldsymbol{\delta}_1,\boldsymbol{\delta}_2$ | 两个从机相对主机的模式偏置向量 |
| $A$ | 三机三角形面积 |
| $A_{\min}$ | 允许的最小三角形面积 |
| $R$ | 三机三角形外接圆半径 |
| $L_{\mathrm{rope}}$ | 吊绳长度 |
| $\rho_{\mathrm{inv}}$ | 载荷几何无效重罚系数 `payload_invalid_penalty` |
| $\mathbf{c}$ | 三机三角形外接圆圆心 |
| $\mathbf{n}$ | 三机三角形平面单位法向量 |
| $h$ | 吊点相对外接圆圆心的法向偏移高度 |
| $\mathbf{p}_{\mathrm{upper}},\mathbf{p}_{\mathrm{lower}}$ | 上下两个候选吊点 |
| $\mathbf{p}_{\mathrm{payload}}$ | 最终选中的载荷中心 |
| $(\mathbf{p})_z$ | 向量 $\mathbf{p}$ 的世界坐标 $z$ 分量 |
| $r_{\mathrm{payload}}$ | 物理载荷半径 |
| $m_{\mathrm{payload}}$ | 载荷附加安全裕量 |
| $r_{\mathrm{eff}}$ | 有效载荷半径，等于 $r_{\mathrm{payload}}+m_{\mathrm{payload}}$ |
| $d_{\mathrm{occ}}(\mathbf{p})$ | 点 $\mathbf{p}$ 到最近膨胀占据体素的距离 |
| $c_{\mathrm{uav}}$ | 无人机对地图的安全净空半径 |
| $\phi_{\mathrm{map}}(\mathbf{p};c)$ | 半径为 $c$ 的地图净空惩罚函数 |
| $\phi_{\mathrm{out}}(\mathbf{p};c)$ | 点超出地图边界盒时的出界惩罚函数 |
| $\mathcal{B}$ | 地图边界盒 |
| $\Pi_{\mathcal{B}}(\mathbf{p})$ | 点 $\mathbf{p}$ 到地图边界盒 $\mathcal{B}$ 的投影 |
| $\mathbf{T}_{\mathrm{local}}$ | 世界坐标到障碍局部坐标系的刚体变换 |
| $\chi$ | 载荷对障碍模板的有符号净空 |
| $\chi_{\mathrm{slab}}^{\mathrm{door}}$ | 门框厚度方向的正净空 |
| $\chi_{\mathrm{door}}$ | 门框门洞方向的有符号净空 |
| $\chi_{\mathrm{slab}}^{\mathrm{slit}}$ | Z 向窄缝厚度方向的正净空 |
| $\chi_{\mathrm{slit}}$ | Z 向窄缝在竖直开口方向的有符号净空 |
| $\chi_{\mathrm{torus}}$ | 圆环环面本体的有符号距离 |
| $\chi_{\mathrm{ring}}$ | 圆环对载荷的最终有符号净空 |
| $x_\ell,y_\ell,z_\ell$ | 障碍局部坐标系中的坐标分量 |
| $h_x$ | 门框或窄缝模板在局部 $x$ 方向的半厚度 |
| $y_c$ | 门洞中心在局部 $y$ 方向的位置 |
| $w_g$ | 门洞宽度 |
| $z_{\mathrm{low}},z_{\mathrm{high}}$ | Z 向窄缝的上下边界 |
| $R_{\mathrm{maj}},R_{\mathrm{min}}$ | 圆环（环面）主半径与管半径 |
| $d_{\mathrm{sep}}$ | 无人机间最小安全间距 |
| $d^{(a,b)}_{k,\sigma}$ | 采样点 $(k,\sigma)$ 处无人机 $a,b$ 的欧氏距离 |
| $[x]_+$ | 正部函数，$\max(x,0)$ |
| $[x]_-$ | 负部函数，$\max(-x,0)$ |
| $\|\cdot\|_2$ | Euclidean 二范数 |
| $\|\cdot\|_F$ | Frobenius 范数 |
| $\theta$ | 编队夹角 `formation_angle_deg` 对应的弧度值 |
| $d_{\mathrm{form}}$ | 默认编队间距 `formation/spacing` |
| $z_{\mathrm{off}}$ | 默认编队统一竖向偏置 |
| $\alpha_c$ | 第 $c$ 个控制点处的模式混合因子 |
| $\ell_m$ | 第 $m$ 个从机所在的编队层级 |
| $s_m$ | 第 $m$ 个从机的左右侧符号，取值为 $\pm1$ |
| $d^{\mathrm{back}}_m$ | 第 $m$ 个从机相对主机的后退距离 |
| $d^{\mathrm{lat}}_m$ | 第 $m$ 个从机相对主机的横向展开距离 |
| $\kappa_{\mathrm{slit}}$ | Z 向窄缝模式的展开倍率 |
| $s_c$ | Z 向窄缝模式的控制点放大系数 |
| $d_{\mathrm{ring}}$ | 圆环模式中的纵向间距 |
| $\kappa_{\mathrm{ring}}$ | 圆环模式中的横向压缩比例 |
| $\eta_c$ | 圆环模式下控制点横向压缩系数 |
| $d^{\mathrm{back,ring}}_{m,c}$ | 圆环模式下第 $m$ 个从机在控制点 $c$ 的等效后退距离 |
| $d^{\mathrm{lat,ring}}_{m,c}$ | 圆环模式下第 $m$ 个从机在控制点 $c$ 的等效横向距离 |
| $r_{\mathrm{safe}}$ | 集群控制点级推开所用安全半径 |
| $w_{\mathrm{coll}}$ | 集群控制点级推开权重 |
| $g_{\max}$ | 集群控制点级推开增益上界 |
| $g_{ij,c}$ | 控制点级集群推开使用的有界二次增益 |
| $d_{ij,c}$ | 第 $i,j$ 个从机在控制点列 $c$ 处的距离 |
| $e_{ij,c}$ | 对应的集群推开误差，等于 $r_{\mathrm{safe}}-d_{ij,c}$ |
| $\mathbf{e}_z$ | 世界坐标系竖直方向单位向量 $(0,0,1)^\top$ |
| $p$ | 贝塞尔阶数，当前实现为 $3$ |
| $\varepsilon$ | 协同载荷避障项中心差分步长 |
