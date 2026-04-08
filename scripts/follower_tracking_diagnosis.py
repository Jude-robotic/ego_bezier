#!/usr/bin/env python3
"""
从机跟踪延迟诊断工具
=====================
订阅：
    /uav1/odom                     — 从机1 odom
    /uav2/odom                     — 从机2 odom
    /swarm/agent_1/guidance_bezier — 从机1 引导 Bezier
    /swarm/agent_2/guidance_bezier — 从机2 引导 Bezier

逻辑：
    1. 期望位置：由 guidance_bezier 在当前时刻评估得到
    2. 实际位置：从机 odom 的位置
    3. 位置误差：实际 - 期望（x/y/z）
    4. 期望姿态：由轨迹速度方向估计（roll=0，pitch/yaw 来自速度向量）
    5. 实际姿态：从机 odom 四元数转 roll/pitch/yaw
    6. 姿态误差：实际 - 期望（角度差规约到 [-pi, pi]）

每次运行结束后会在脚本目录下自动生成 tracking_picture 子目录，
并保存 4 张图：
    - uav_1_line_时间戳.png
    - uav_1_angle_时间戳.png
    - uav_2_line_时间戳.png
    - uav_2_angle_时间戳.png
"""

import argparse
import math
import sys
import threading
import time
import os
from datetime import datetime

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


def eval_piecewise_bezier_velocity(msg: BezierMsg, t_query: float, dt: float = 0.02):
    """用中心差分近似轨迹速度。"""
    if len(msg.segment_durations) == 0:
        return None

    total_t = float(sum(msg.segment_durations))
    if total_t <= 1e-9:
        return np.array([0.0, 0.0, 0.0])

    t0 = max(0.0, min(total_t, t_query - dt))
    t1 = max(0.0, min(total_t, t_query + dt))
    if t1 - t0 < 1e-6:
        return np.array([0.0, 0.0, 0.0])

    p0 = eval_piecewise_bezier(msg, t0)
    p1 = eval_piecewise_bezier(msg, t1)
    if p0 is None or p1 is None:
        return None
    return (p1 - p0) / (t1 - t0)


def quat_to_rpy(qx: float, qy: float, qz: float, qw: float):
    """四元数转 roll/pitch/yaw（弧度）。"""
    sinr_cosp = 2.0 * (qw * qx + qy * qz)
    cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (qw * qy - qz * qx)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


def wrap_to_pi(angle: float):
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


# ──────────────────────────────────────────────
#  数据收集器
# ──────────────────────────────────────────────
class TrackingLogger:
    def __init__(self):
        self._lock = threading.Lock()

        # 最新 guidance bezier（含 start_time）
        self._guidance: dict = {1: None, 2: None}
        self._last_des_rpy = {
            1: np.array([0.0, 0.0, 0.0]),
            2: np.array([0.0, 0.0, 0.0]),
        }

        # 时序数据
        self.data = {
            1: dict(
                t=[],
                x_des=[], y_des=[], z_des=[],
                x_act=[], y_act=[], z_act=[],
                ex=[], ey=[], ez=[],
                roll_des=[], pitch_des=[], yaw_des=[],
                roll_act=[], pitch_act=[], yaw_act=[],
                e_roll=[], e_pitch=[], e_yaw=[]
            ),
            2: dict(
                t=[],
                x_des=[], y_des=[], z_des=[],
                x_act=[], y_act=[], z_act=[],
                ex=[], ey=[], ez=[],
                roll_des=[], pitch_des=[], yaw_des=[],
                roll_act=[], pitch_act=[], yaw_act=[],
                e_roll=[], e_pitch=[], e_yaw=[]
            ),
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
        vel = eval_piecewise_bezier_velocity(guide, t_in_traj)
        if des is None or vel is None:
            return

        x_act = msg.pose.pose.position.x
        y_act = msg.pose.pose.position.y
        z_act = msg.pose.pose.position.z

        # 期望姿态：由速度方向估计
        vx, vy, vz = float(vel[0]), float(vel[1]), float(vel[2])
        speed_xy = math.hypot(vx, vy)
        speed = math.sqrt(vx * vx + vy * vy + vz * vz)

        with self._lock:
            des_rpy = self._last_des_rpy[agent_id].copy()

        if speed > 1e-3:
            if speed_xy > 1e-5:
                des_rpy[2] = math.atan2(vy, vx)  # yaw
            des_rpy[1] = math.atan2(-vz, max(speed_xy, 1e-6))  # pitch
            des_rpy[0] = 0.0  # roll
            with self._lock:
                self._last_des_rpy[agent_id] = des_rpy.copy()

        q = msg.pose.pose.orientation
        roll_act, pitch_act, yaw_act = quat_to_rpy(q.x, q.y, q.z, q.w)
        e_roll = wrap_to_pi(roll_act - des_rpy[0])
        e_pitch = wrap_to_pi(pitch_act - des_rpy[1])
        e_yaw = wrap_to_pi(yaw_act - des_rpy[2])

        with self._lock:
            d = self.data[agent_id]
            d["t"].append(t_rel)
            d["x_des"].append(des[0])
            d["y_des"].append(des[1])
            d["z_des"].append(des[2])
            d["x_act"].append(x_act)
            d["y_act"].append(y_act)
            d["z_act"].append(z_act)
            d["ex"].append(x_act - des[0])
            d["ey"].append(y_act - des[1])
            d["ez"].append(z_act - des[2])

            d["roll_des"].append(des_rpy[0])
            d["pitch_des"].append(des_rpy[1])
            d["yaw_des"].append(des_rpy[2])
            d["roll_act"].append(roll_act)
            d["pitch_act"].append(pitch_act)
            d["yaw_act"].append(yaw_act)
            d["e_roll"].append(e_roll)
            d["e_pitch"].append(e_pitch)
            d["e_yaw"].append(e_yaw)


# ──────────────────────────────────────────────
#  绘图
# ──────────────────────────────────────────────
def _save_and_optionally_show(fig, save_path: str, show: bool):
    fig.savefig(save_path, dpi=150, bbox_inches="tight")
    print(f"[INFO] 图表已保存到: {save_path}")
    if show:
        fig.show()
    else:
        import matplotlib.pyplot as plt
        plt.close(fig)


def _draw_no_data(axs, title: str):
    axs[0].set_title(title)
    for ax in axs:
        ax.axis("off")
    axs[0].text(0.5, 0.5, "数据不足，无法绘制", ha="center", va="center", fontsize=12)


def plot_position_tracking(agent_id: int, d: dict, save_path: str, show: bool):
    if not show:
        _mpl.use("Agg")
    import matplotlib.pyplot as plt

    fig, axs = plt.subplots(4, 1, figsize=(13, 10), sharex=True)
    fig.suptitle(f"UAV{agent_id} 位置跟踪: 期望值 vs 实际值", fontsize=14, fontweight="bold")

    if len(d["t"]) < 3:
        _draw_no_data(axs, f"UAV{agent_id} 位置跟踪")
        _save_and_optionally_show(fig, save_path, show)
        return

    t = np.array(d["t"])
    x_des = np.array(d["x_des"])
    y_des = np.array(d["y_des"])
    z_des = np.array(d["z_des"])
    x_act = np.array(d["x_act"])
    y_act = np.array(d["y_act"])
    z_act = np.array(d["z_act"])

    p_des_norm = np.sqrt(x_des**2 + y_des**2 + z_des**2)
    p_act_norm = np.sqrt(x_act**2 + y_act**2 + z_act**2)

    pos_items = [
        ("x", x_des, x_act, "#1E88E5", "#F4511E"),
        ("y", y_des, y_act, "#1E88E5", "#F4511E"),
        ("z", z_des, z_act, "#1E88E5", "#F4511E"),
        ("|p|", p_des_norm, p_act_norm, "#3949AB", "#FB8C00"),
    ]

    for i, (label, des, act, c_des, c_act) in enumerate(pos_items):
        axs[i].plot(t, des, color=c_des, linewidth=1.2, label=f"期望{label}")
        axs[i].plot(t, act, color=c_act, linewidth=1.0, alpha=0.95, label=f"实际{label}")
        axs[i].fill_between(
            t,
            des,
            act,
            color="#FFB74D",
            alpha=0.24,
            label="包络区间(期望-实际)" if i == 0 else None,
        )
        rmse = np.sqrt(np.mean((act - des) ** 2))
        axs[i].text(
            0.01,
            0.88,
            f"RMSE={rmse:.3f}m",
            transform=axs[i].transAxes,
            fontsize=8,
            color="#B71C1C",
        )
        axs[i].set_ylabel(f"{label} [m]")
        axs[i].grid(True, alpha=0.35)

    axs[3].set_xlabel("时间 t [s]")
    axs[0].legend(loc="upper right", fontsize=8)
    axs[3].grid(True, alpha=0.35)

    _save_and_optionally_show(fig, save_path, show)


def plot_attitude_tracking(agent_id: int, d: dict, save_path: str, show: bool):
    if not show:
        _mpl.use("Agg")
    import matplotlib.pyplot as plt

    fig, axs = plt.subplots(4, 1, figsize=(13, 10), sharex=True)
    fig.suptitle(f"UAV{agent_id} 姿态跟踪: 期望值 vs 实际值", fontsize=14, fontweight="bold")

    if len(d["t"]) < 3:
        _draw_no_data(axs, f"UAV{agent_id} 姿态跟踪")
        _save_and_optionally_show(fig, save_path, show)
        return

    t = np.array(d["t"])
    pitch_des = np.rad2deg(np.array(d["pitch_des"]))
    yaw_des = np.rad2deg(np.array(d["yaw_des"]))
    roll_des = np.rad2deg(np.array(d["roll_des"]))
    pitch_act = np.rad2deg(np.array(d["pitch_act"]))
    yaw_act = np.rad2deg(np.array(d["yaw_act"]))
    roll_act = np.rad2deg(np.array(d["roll_act"]))

    ang_des_norm = np.sqrt(roll_des**2 + pitch_des**2 + yaw_des**2)
    ang_act_norm = np.sqrt(roll_act**2 + pitch_act**2 + yaw_act**2)

    att_items = [
        ("pitch", pitch_des, pitch_act, "#1E88E5", "#F4511E"),
        ("yaw", yaw_des, yaw_act, "#1E88E5", "#F4511E"),
        ("roll", roll_des, roll_act, "#1E88E5", "#F4511E"),
        ("|rpy|", ang_des_norm, ang_act_norm, "#3949AB", "#FB8C00"),
    ]

    for i, (label, des, act, c_des, c_act) in enumerate(att_items):
        axs[i].plot(t, des, color=c_des, linewidth=1.2, label=f"期望{label}")
        axs[i].plot(t, act, color=c_act, linewidth=1.0, alpha=0.95, label=f"实际{label}")
        axs[i].fill_between(
            t,
            des,
            act,
            color="#FFB74D",
            alpha=0.24,
            label="包络区间(期望-实际)" if i == 0 else None,
        )
        rmse = np.sqrt(np.mean((act - des) ** 2))
        axs[i].text(
            0.01,
            0.88,
            f"RMSE={rmse:.3f}deg",
            transform=axs[i].transAxes,
            fontsize=8,
            color="#B71C1C",
        )
        axs[i].set_ylabel(f"{label} [deg]")
        axs[i].grid(True, alpha=0.35)

    axs[3].set_xlabel("时间 t [s]")
    axs[0].legend(loc="upper right", fontsize=8)
    axs[3].grid(True, alpha=0.35)

    _save_and_optionally_show(fig, save_path, show)


# ──────────────────────────────────────────────
#  主程序
# ──────────────────────────────────────────────
def main():
    parser = argparse.ArgumentParser(description="从机跟踪延迟诊断")
    parser.add_argument("--duration", type=float, default=0.0,
                        help="采集时长（秒），0=无限，Ctrl+C 停止")
    parser.add_argument("--save", action="store_true",
                        help="兼容旧参数：当前版本每次都会自动保存图片")
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
        ez = np.array(d["ez"])
        e_pos = np.sqrt(ex**2 + ey**2 + ez**2)
        e_pitch = np.array(d["e_pitch"])
        e_yaw = np.array(d["e_yaw"])
        e_roll = np.array(d["e_roll"])
        e_ang = np.sqrt(e_pitch**2 + e_yaw**2 + e_roll**2)
        t_span = d["t"][-1] - d["t"][0]
        print(f"  从机{aid}: {n} 条数据，时长 {t_span:.1f}s")
        print(f"    x误差 RMSE={np.sqrt(np.mean(ex**2)):.3f}m  "
              f"max={np.max(np.abs(ex)):.3f}m  "
              f"mean={np.mean(ex):.3f}m")
        print(f"    y误差 RMSE={np.sqrt(np.mean(ey**2)):.3f}m  "
              f"max={np.max(np.abs(ey)):.3f}m  "
              f"mean={np.mean(ey):.3f}m")
        print(f"    z误差 RMSE={np.sqrt(np.mean(ez**2)):.3f}m  "
              f"max={np.max(np.abs(ez)):.3f}m  "
              f"mean={np.mean(ez):.3f}m")
        print(f"    位置总误差|e| RMSE={np.sqrt(np.mean(e_pos**2)):.3f}m  "
              f"max={np.max(e_pos):.3f}m")

        print(f"    pitch误差 RMSE={np.rad2deg(np.sqrt(np.mean(e_pitch**2))):.3f}deg  "
              f"max={np.rad2deg(np.max(np.abs(e_pitch))):.3f}deg")
        print(f"    yaw误差   RMSE={np.rad2deg(np.sqrt(np.mean(e_yaw**2))):.3f}deg  "
              f"max={np.rad2deg(np.max(np.abs(e_yaw))):.3f}deg")
        print(f"    roll误差  RMSE={np.rad2deg(np.sqrt(np.mean(e_roll**2))):.3f}deg  "
              f"max={np.rad2deg(np.max(np.abs(e_roll))):.3f}deg")
        print(f"    姿态总误差|e| RMSE={np.rad2deg(np.sqrt(np.mean(e_ang**2))):.3f}deg  "
              f"max={np.rad2deg(np.max(e_ang)):.3f}deg")
    print("──────────────────────────────\n")

    _script_dir = os.path.dirname(os.path.abspath(__file__))
    save_dir = os.path.join(_script_dir, "tracking_picture")
    os.makedirs(save_dir, exist_ok=True)

    _timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    for aid in [1, 2]:
        pos_path = os.path.join(save_dir, f"uav_{aid}_line_{_timestamp}.png")
        ang_path = os.path.join(save_dir, f"uav_{aid}_angle_{_timestamp}.png")
        plot_position_tracking(aid, logger.data[aid], pos_path, args.show)
        plot_attitude_tracking(aid, logger.data[aid], ang_path, args.show)

    print(f"[INFO] 4张图像已输出到目录: {save_dir}")


if __name__ == "__main__":
    main()
