#!/usr/bin/env python3
"""
从机跟踪延迟诊断工具
=====================
订阅：
  /visual_slam/odom            — 主机 odom
  /uav1/odom                   — 从机1 odom
  /uav2/odom                   — 从机2 odom
  /planning/bezier             — 主机 Bezier 轨迹（用于直接推算期望位置）
  /swarm/agent_1/guidance_bezier — 从机1 引导 Bezier（Master 发给 follower 的轨迹）
  /swarm/agent_2/guidance_bezier — 从机2 引导 Bezier

逻辑：
  1. 「期望位置」= 用最新 guidance_bezier 在当前时刻评估的位置
     （即从机 traj_server 理想情况下正在执行的轨迹点）
  2. 「实际位置」= 从机 odom
  3. 跟踪误差 = 实际 - 期望

按 Ctrl+C 结束采集后，自动弹出 x/y vs t 对比图。

用法：
  cd ~/ego-planner-bezier
  source devel/setup.bash
  python3 scripts/follower_tracking_diagnosis.py
  
  # 指定采集时长（秒），到时自动绘图：
  python3 scripts/follower_tracking_diagnosis.py --duration 60

  # 仅保存图片，不弹窗（适合无 GUI 环境）：
  python3 scripts/follower_tracking_diagnosis.py --save --no-show
"""

import argparse
import math
import sys
import threading
import time

import numpy as np

# ──────────────────────────────────────────────
#  Matplotlib 中文字体配置（必须在 import matplotlib.pyplot 之前执行）
# ──────────────────────────────────────────────
import matplotlib as _mpl
import matplotlib.font_manager as _fm
import os as _os, glob as _glob

def _setup_chinese_font():
    """清理旧字体缓存，优先选取支持 CJK 的字体，fallback 到 DejaVu Sans。"""
    # 清除过期缓存，确保新安装的字体被识别
    _cache_dir = _mpl.get_cachedir()
    for _f in _glob.glob(_os.path.join(_cache_dir, "fontlist*.json")):
        _os.remove(_f)
    _fm.fontManager = _fm.FontManager()

    # 按优先级列出候选中文字体
    _candidates = [
        "Noto Sans CJK SC", "Noto Sans CJK JP", "Noto Sans CJK TC",
        "WenQuanYi Micro Hei", "WenQuanYi Zen Hei",
        "Source Han Sans CN", "Source Han Sans",
        "SimHei", "Microsoft YaHei",
    ]
    _available = {f.name for f in _fm.fontManager.ttflist}
    _chosen = next((c for c in _candidates if c in _available), None)

    if _chosen:
        _mpl.rcParams["font.family"] = "sans-serif"
        _mpl.rcParams["font.sans-serif"] = [_chosen, "DejaVu Sans"]
    else:
        # 无 CJK 字体，退回英文显示，防止方框
        _mpl.rcParams["font.family"] = "sans-serif"
        _mpl.rcParams["font.sans-serif"] = ["DejaVu Sans"]

    # 修复负号显示为方框的问题
    _mpl.rcParams["axes.unicode_minus"] = False

_setup_chinese_font()

# ──────────────────────────────────────────────
#  ROS imports
# ──────────────────────────────────────────────
import rospy
from nav_msgs.msg import Odometry

# 动态加载 ego_planner 消息（需先 source devel/setup.bash）
try:
    from ego_planner.msg import Bezier as BezierMsg
except ImportError:
    sys.exit(
        "[ERROR] ego_planner.msg.Bezier not found.\n"
        "请先执行: source ~/ego-planner-bezier/devel/setup.bash"
    )


# ──────────────────────────────────────────────
#  Bezier 评估工具
# ──────────────────────────────────────────────
def eval_piecewise_bezier(msg: BezierMsg, t_query: float):
    """
    在时间 t_query (相对于 msg.start_time 的秒数) 处评估分段 Bezier 曲线。
    返回 (x, y, z)，超出范围则返回端点。
    """
    n_pts = len(msg.pos_pts)
    n_seg = len(msg.segment_durations)
    order = msg.order  # 通常为 3（三次）

    if n_seg == 0 or n_pts == 0:
        return None

    cp_per_seg = order  # 每段新增控制点数（三次 Bezier: P0,P1,P2,P3 → 4点，但段间共享）
    # 三次分段 Bezier: N段 → 3N+1 控制点
    # 取出第 seg_idx 段的控制点：索引 [3*seg_idx .. 3*seg_idx+3]
    expected_n_pts = order * n_seg + 1
    if n_pts != expected_n_pts:
        # 尝试容错：平均分配
        pass  # 继续，用下面的索引方式

    # 找到 t 所在段
    t = max(0.0, t_query)
    seg_idx = 0
    t_local = t
    for i, dur in enumerate(msg.segment_durations):
        if t_local <= dur or i == n_seg - 1:
            seg_idx = i
            break
        t_local -= dur

    dur_seg = msg.segment_durations[seg_idx]
    if dur_seg < 1e-9:
        s = 1.0
    else:
        s = min(1.0, max(0.0, t_local / dur_seg))

    # 取控制点
    base = seg_idx * order  # 三次 Bezier: 每段起始索引
    pts = []
    for k in range(order + 1):
        idx = base + k
        if idx >= n_pts:
            idx = n_pts - 1
        p = msg.pos_pts[idx]
        pts.append(np.array([p.x, p.y, p.z]))

    # de Casteljau
    while len(pts) > 1:
        pts = [(1 - s) * pts[i] + s * pts[i + 1] for i in range(len(pts) - 1)]
    return pts[0]


# ──────────────────────────────────────────────
#  数据收集器
# ──────────────────────────────────────────────
class TrackingLogger:
    def __init__(self):
        self._lock = threading.Lock()

        # 最新 guidance bezier（含 start_time）
        self._guidance: dict = {1: None, 2: None}

        # 时序数据  {agent_id: {'t': [], 'x_des': [], 'y_des': [], 'x_act': [], 'y_act': [], 'ex': [], 'ey': []}}
        self.data = {
            1: dict(t=[], x_des=[], y_des=[], x_act=[], y_act=[], ex=[], ey=[]),
            2: dict(t=[], x_des=[], y_des=[], x_act=[], y_act=[], ex=[], ey=[]),
        }
        self._t0 = None  # 第一条消息的时间基准

    # ── Guidance bezier 回调 ──────────────────
    def guidance_cb(self, msg: BezierMsg, agent_id: int):
        with self._lock:
            self._guidance[agent_id] = msg

    # ── 从机 odom 回调 ───────────────────────
    def follower_odom_cb(self, msg: Odometry, agent_id: int):
        now = msg.header.stamp
        with self._lock:
            guide = self._guidance[agent_id]

        if guide is None:
            return  # 还没收到引导轨迹

        # 初始化时间基准
        with self._lock:
            if self._t0 is None:
                self._t0 = now

        t_abs = now.to_sec()
        t_rel = t_abs - self._t0.to_sec()

        # 评估期望位置
        t_in_traj = (now - guide.start_time).to_sec()
        des = eval_piecewise_bezier(guide, t_in_traj)
        if des is None:
            return

        x_act = msg.pose.pose.position.x
        y_act = msg.pose.pose.position.y

        with self._lock:
            d = self.data[agent_id]
            d["t"].append(t_rel)
            d["x_des"].append(des[0])
            d["y_des"].append(des[1])
            d["x_act"].append(x_act)
            d["y_act"].append(y_act)
            d["ex"].append(x_act - des[0])
            d["ey"].append(y_act - des[1])


# ──────────────────────────────────────────────
#  绘图
# ──────────────────────────────────────────────
def plot_results(logger: TrackingLogger, save_path: str = None, show: bool = True):
    if not show:
        _mpl.use("Agg")
    import matplotlib.pyplot as plt
    import matplotlib.gridspec as gridspec

    fig = plt.figure(figsize=(16, 14))
    fig.suptitle("从机跟踪诊断：期望位置 vs 实际位置", fontsize=15, fontweight="bold")

    colors = {
        "des": "#2196F3",   # 蓝色 = 期望
        "act": "#F44336",   # 红色 = 实际
        "err": "#FF9800",   # 橙色 = 误差
    }

    for row_idx, agent_id in enumerate([1, 2]):
        d = logger.data[agent_id]
        if len(d["t"]) < 5:
            print(f"[WARN] 从机{agent_id} 数据点不足 ({len(d['t'])} 条)，跳过绘图")
            continue

        t = np.array(d["t"])
        x_des = np.array(d["x_des"])
        y_des = np.array(d["y_des"])
        x_act = np.array(d["x_act"])
        y_act = np.array(d["y_act"])
        ex = np.array(d["ex"])
        ey = np.array(d["ey"])
        e_total = np.sqrt(ex**2 + ey**2)

        base_row = row_idx * 3  # 每个从机占 3 行（x, y, err）

        gs = gridspec.GridSpec(6, 2, figure=fig, hspace=0.45, wspace=0.3)

        uav_label = f"从机 {agent_id} (UAV{agent_id})"

        # ── X 方向 ──────────────────────────────────────
        ax_x = fig.add_subplot(gs[base_row, :])
        ax_x.plot(t, x_des, color=colors["des"], linewidth=1.2, label="期望 x")
        ax_x.plot(t, x_act, color=colors["act"], linewidth=1.0, alpha=0.85, linestyle="--", label="实际 x")
        ax_x.fill_between(t, x_des, x_act, alpha=0.15, color=colors["err"])
        ax_x.set_ylabel("x [m]")
        ax_x.set_title(f"{uav_label} — X 方向跟踪", fontsize=11)
        ax_x.legend(loc="upper right", fontsize=8)
        ax_x.grid(True, alpha=0.4)
        _annotate_rmse(ax_x, ex)

        # ── Y 方向 ──────────────────────────────────────
        ax_y = fig.add_subplot(gs[base_row + 1, :])
        ax_y.plot(t, y_des, color=colors["des"], linewidth=1.2, label="期望 y")
        ax_y.plot(t, y_act, color=colors["act"], linewidth=1.0, alpha=0.85, linestyle="--", label="实际 y")
        ax_y.fill_between(t, y_des, y_act, alpha=0.15, color=colors["err"])
        ax_y.set_ylabel("y [m]")
        ax_y.set_title(f"{uav_label} — Y 方向跟踪", fontsize=11)
        ax_y.legend(loc="upper right", fontsize=8)
        ax_y.grid(True, alpha=0.4)
        _annotate_rmse(ax_y, ey)

        # ── 跟踪误差 ─────────────────────────────────────
        ax_e = fig.add_subplot(gs[base_row + 2, :])
        ax_e.plot(t, ex, color="#9C27B0", linewidth=1.0, label="误差 x")
        ax_e.plot(t, ey, color="#4CAF50", linewidth=1.0, label="误差 y")
        ax_e.plot(t, e_total, color=colors["err"], linewidth=1.5, label="总误差 |e|", alpha=0.9)
        ax_e.axhline(0, color="black", linewidth=0.7, linestyle="-")

        # 标注误差超过 0.5 m 的区间（可能失控时段）
        threshold = 0.5
        large_err = e_total > threshold
        if large_err.any():
            ax_e.fill_between(t, 0, e_total, where=large_err,
                               alpha=0.25, color="red", label=f"|e|>{threshold}m")

        ax_e.set_xlabel("时间 t [s]")
        ax_e.set_ylabel("误差 [m]")
        ax_e.set_title(f"{uav_label} — 跟踪误差", fontsize=11)
        ax_e.legend(loc="upper right", fontsize=8)
        ax_e.grid(True, alpha=0.4)

        rmse = float(np.sqrt(np.mean(e_total**2)))
        max_e = float(np.max(e_total))
        ax_e.text(0.01, 0.92, f"RMSE={rmse:.3f}m  Max={max_e:.3f}m",
                  transform=ax_e.transAxes, fontsize=9,
                  color="darkred", verticalalignment="top")

    if save_path:
        plt.savefig(save_path, dpi=150, bbox_inches="tight")
        print(f"[INFO] 图表已保存到: {save_path}")

    if show:
        plt.show()
    else:
        plt.close()


def _annotate_rmse(ax, err_arr: np.ndarray):
    rmse = float(np.sqrt(np.mean(err_arr**2)))
    ax.text(0.01, 0.92, f"RMSE={rmse:.3f}m",
            transform=ax.transAxes, fontsize=9,
            color="darkred", verticalalignment="top")


# ──────────────────────────────────────────────
#  主程序
# ──────────────────────────────────────────────
def main():
    parser = argparse.ArgumentParser(description="从机跟踪延迟诊断")
    parser.add_argument("--duration", type=float, default=0.0,
                        help="采集时长（秒），0=无限，Ctrl+C 停止")
    parser.add_argument("--save", action="store_true",
                        help="保存图片到 scripts/follower_tracking_result.png（默认已自动保存）")
    parser.add_argument("--no-show", dest="show", action="store_false",
                        help="不弹出交互窗口（适合无 GUI 环境）")
    parser.set_defaults(show=True)
    args = parser.parse_args()

    rospy.init_node("follower_tracking_diagnosis", anonymous=True)
    logger = TrackingLogger()

    # ── 订阅 guidance bezier ─────────────────
    rospy.Subscriber(
        "/swarm/agent_1/guidance_bezier", BezierMsg,
        lambda msg: logger.guidance_cb(msg, 1)
    )
    rospy.Subscriber(
        "/swarm/agent_2/guidance_bezier", BezierMsg,
        lambda msg: logger.guidance_cb(msg, 2)
    )

    # ── 订阅从机 odom ────────────────────────
    rospy.Subscriber(
        "/uav1/odom", Odometry,
        lambda msg: logger.follower_odom_cb(msg, 1)
    )
    rospy.Subscriber(
        "/uav2/odom", Odometry,
        lambda msg: logger.follower_odom_cb(msg, 2)
    )

    print("=" * 55)
    print("  从机跟踪诊断已启动，等待 ROS 话题数据...")
    print("  话题：")
    print("    /swarm/agent_1/guidance_bezier  (从机1 引导轨迹)")
    print("    /swarm/agent_2/guidance_bezier  (从机2 引导轨迹)")
    print("    /uav1/odom                      (从机1 实际位置)")
    print("    /uav2/odom                      (从机2 实际位置)")
    print("  按 Ctrl+C 停止采集并绘图")
    print("=" * 55)

    try:
        if args.duration > 0:
            deadline = time.time() + args.duration
            rate = rospy.Rate(50)
            while not rospy.is_shutdown() and time.time() < deadline:
                rate.sleep()
            print(f"\n[INFO] 采集完成（{args.duration}s），正在绘图...")
        else:
            rospy.spin()
    except KeyboardInterrupt:
        pass

    # ── 统计摘要 ────────────────────────────
    print("\n────────── 数据摘要 ──────────")
    for aid in [1, 2]:
        d = logger.data[aid]
        n = len(d["t"])
        if n < 2:
            print(f"  从机{aid}: 数据不足 ({n} 条)")
            continue
        ex = np.array(d["ex"])
        ey = np.array(d["ey"])
        e = np.sqrt(ex**2 + ey**2)
        t_span = d["t"][-1] - d["t"][0]
        print(f"  从机{aid}: {n} 条数据，时长 {t_span:.1f}s")
        print(f"    x误差 RMSE={np.sqrt(np.mean(ex**2)):.3f}m  "
              f"max={np.max(np.abs(ex)):.3f}m  "
              f"mean={np.mean(ex):.3f}m")
        print(f"    y误差 RMSE={np.sqrt(np.mean(ey**2)):.3f}m  "
              f"max={np.max(np.abs(ey)):.3f}m  "
              f"mean={np.mean(ey):.3f}m")
        print(f"    总误差|e| RMSE={np.sqrt(np.mean(e**2)):.3f}m  "
              f"max={np.max(e):.3f}m")
    print("──────────────────────────────\n")

    import os
    from datetime import datetime
    _script_dir = os.path.dirname(os.path.abspath(__file__))
    _timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    # 始终保存带时间戳的版本
    save_path = os.path.join(_script_dir, f"follower_tracking_{_timestamp}.png")
    # --save 额外保存一份固定名称（方便快速查看最新结果）
    fixed_save_path = os.path.join(_script_dir, "follower_tracking_result.png") if args.save else None

    plot_results(logger, save_path=save_path, show=args.show)
    if fixed_save_path:
        import shutil
        shutil.copy2(save_path, fixed_save_path)
        print(f"[INFO] 同时保存到固定路径: {fixed_save_path}")


if __name__ == "__main__":
    main()
