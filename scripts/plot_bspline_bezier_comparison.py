#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
二维栅格平面示意图 - B样条曲线 vs Bezier曲线对比
展示: A*搜索轨迹、ESDF优化轨迹、无人机感知范围

关键设计:
- 感知范围内: 无碰曲线 (ESDF优化效果)
- 感知范围外: 允许碰撞 (无ESDF局部规划器的理论真谛)
"""

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch, Circle
import matplotlib.patheffects as pe

plt.rcParams['font.family'] = 'DejaVu Sans'
plt.rcParams['axes.unicode_minus'] = False

# ============ 配置 ============
GRID_SIZE_X = 20
GRID_SIZE_Y = 18
START = np.array([2.0, 3.0])
END = np.array([18.0, 13.0])
SENSOR_RANGE = 4.5

# 障碍物布局
OBSTACLES = [
    # 左下区域
    (4, 5, 'circle'), (5, 7, 'rect'), (3.5, 9, 'circle'),
    # 中间走廊
    (7, 4, 'rect'), (7, 6, 'circle'), (7, 8, 'rect'), (7, 10, 'circle'),
    (10, 3, 'circle'), (10, 5, 'rect'), (10, 7, 'circle'), (10, 9, 'rect'),
    (10, 11, 'circle'), (10, 13, 'rect'), (10, 15, 'circle'),
    (13, 4, 'rect'), (13, 6, 'circle'), (13, 8, 'rect'), (13, 10, 'circle'),
    (13, 12, 'rect'),
    # 右侧
    (15, 6, 'circle'), (15, 9, 'rect'), (15, 11, 'circle'),
    (17, 5, 'rect'), (17, 8, 'circle'), (17, 12, 'rect'),
    # 额外分散
    (5, 12, 'circle'), (9, 14, 'rect'), (12, 2, 'circle'), (16, 3, 'rect'),
    (6, 14, 'circle'), (14, 15, 'rect'),
]

# A*搜索路径
ASTAR_PATH = np.array([
    [2.0, 3.0], [3.5, 4.0], [4.5, 6.0], [5.5, 8.0], [6.0, 10.5],
    [8.5, 11.0], [9.0, 9.0], [10.5, 8.0], [11.0, 10.5], [12.5, 11.5],
    [13.5, 10.0], [14.5, 11.0], [16.0, 10.5], [17.0, 12.0], [18.0, 13.0],
])

# ============ 曲线生成函数 ============
def generate_bspline(control_points, num_points=300):
    n = len(control_points)
    degree = min(3, n - 1)

    def basis_function(i, k, t, knots):
        if k == 0:
            return 1.0 if knots[i] <= t <= knots[i+1] else 0.0
        denom1 = knots[i+k] - knots[i]
        denom2 = knots[i+k+1] - knots[i+1]
        v1 = 0.0 if denom1 == 0 else (t - knots[i]) / denom1 * basis_function(i, k-1, t, knots)
        v2 = 0.0 if denom2 == 0 else (knots[i+k+1] - t) / denom2 * basis_function(i+1, k-1, t, knots)
        return v1 + v2

    num_knots = n + degree + 1
    knots = np.concatenate([np.zeros(degree), np.linspace(0, 1, num_knots - 2*degree), np.ones(degree)])

    curve = []
    for t in np.linspace(0, 1, num_points):
        point = np.zeros(2)
        for i in range(n):
            bf = basis_function(i, degree, t, knots)
            point += control_points[i] * bf
        curve.append(point)
    return np.array(curve)

def generate_bezier(control_points, num_points=150):
    def de_casteljau(points, t):
        if len(points) == 1:
            return points[0]
        new_points = [(1-t)*points[i] + t*points[i+1] for i in range(len(points)-1)]
        return de_casteljau(new_points, t)

    curve = []
    for t in np.linspace(0, 1, num_points):
        curve.append(de_casteljau(control_points, t))
    return np.array(curve)

# ============ 碰撞检测 ============
def check_collision_point(point, obstacles, margin=0.0):
    for ox, oy, shape in obstacles:
        if shape == 'circle':
            if np.sqrt((point[0] - ox)**2 + (point[1] - oy)**2) < 0.6 + margin:
                return True
        else:
            if abs(point[0] - ox) < 0.55 + margin and abs(point[1] - oy) < 0.55 + margin:
                return True
    return False

def get_curve_in_sensor_range(curve, sensor_center, sensor_range):
    in_range = []
    for i, point in enumerate(curve):
        if np.linalg.norm(point - sensor_center) <= sensor_range:
            in_range.append((i, point))
    return in_range

# ============ 图1: 感知在起点 ============
# 感知范围: 中心(2,3), 半径4.5 -> x∈[-2.5, 6.5], y∈[-1.5, 7.5]
# 范围内障碍物: (4,5), (5,7), (3.5,9), (7,4), (7,6)
# 设计: 曲线前半段绕行避开，后半段允许穿障

BSPLINE_CTRL_1 = np.array([
    [2.0, 3.0], [3.2, 3.8], [4.0, 2.5], [5.0, 1.5], [6.0, 2.5],  # 绕行
    [7.0, 4.0], [8.0, 6.0], [9.0, 8.0], [10.5, 9.5],  # 感知范围外
    [12.0, 8.0], [14.0, 10.0], [16.0, 11.0], [18.0, 13.0],
])

BEZIER_SEG1_1 = np.array([[2.0, 3.0], [3.0, 3.2], [4.2, 2.8], [5.0, 1.8]])
BEZIER_SEG2_1 = np.array([[5.0, 1.8], [5.8, 2.5], [6.5, 3.5], [7.5, 4.5]])
BEZIER_SEG3_1 = np.array([[7.5, 4.5], [9.0, 7.0], [11.0, 8.0], [13.0, 9.0]])
BEZIER_SEG4_1 = np.array([[13.0, 9.0], [15.0, 10.0], [16.5, 11.0], [18.0, 13.0]])

# ============ 图2: 感知在中间 ============
# 感知范围: 中心(10,9), 半径4.5 -> x∈[5.5, 14.5], y∈[4.5, 13.5]
# 感知范围内障碍物密集，通道极窄。为展示"感知内无碰"的理论：
# 采用更激进的策略：感知范围内使用近似直线段+快速通过

BSPLINE_CTRL_2 = np.array([
    [2.0, 3.0], [4.0, 5.0], [5.5, 7.0],
    [6.2, 10.0],
    [6.5, 11.5],
    [9.5, 11.8],
    [10.5, 11.8],
    [12.5, 13.0],  # y>12.55 to pass rect(13,12)
    [14.5, 13.5],
    [16.0, 13.5], [17.0, 13.0], [18.0, 13.0],
])

BEZIER_SEG1_2 = np.array([[2.0, 3.0], [4.5, 6.0], [6.0, 10.0], [6.8, 11.5]])
BEZIER_SEG2_2 = np.array([[6.8, 11.5], [8.5, 11.8], [9.5, 11.8], [10.5, 11.8]])
# rect(13,12): y_range=[11.45,12.55], so go above y=12.6 to avoid
BEZIER_SEG3_2 = np.array([[10.5, 11.8], [11.5, 12.0], [12.0, 12.8], [13.5, 13.2]])
BEZIER_SEG4_2 = np.array([[13.5, 13.2], [15.0, 13.5], [16.5, 13.5], [18.0, 13.0]])

# 生成曲线
bspline_curve_1 = generate_bspline(BSPLINE_CTRL_1)
bezier_curve_1 = np.vstack([generate_bezier(BEZIER_SEG1_1), generate_bezier(BEZIER_SEG2_1),
                            generate_bezier(BEZIER_SEG3_1), generate_bezier(BEZIER_SEG4_1)])

bspline_curve_2 = generate_bspline(BSPLINE_CTRL_2)
bezier_curve_2 = np.vstack([generate_bezier(BEZIER_SEG1_2), generate_bezier(BEZIER_SEG2_2),
                            generate_bezier(BEZIER_SEG3_2), generate_bezier(BEZIER_SEG4_2)])

def draw_figure(bspline_curve, bezier_curve, bspline_ctrl, bezier_segs,
                sensor_center, output_file, title_suffix=''):
    fig, ax = plt.subplots(figsize=(16, 9))

    BG_COLOR = '#fafafa'
    GRID_COLOR = '#e0e0e0'
    OBSTACLE_COLOR = '#2c3e50'
    SPLINE_COLOR = '#2980b9'
    BEZIER_COLOR = '#27ae60'
    ASTAR_COLOR = '#e74c3c'

    # 栅格背景
    ax.set_facecolor(BG_COLOR)
    for i in range(0, GRID_SIZE_Y + 1):
        ax.axhline(i, color=GRID_COLOR, linewidth=0.8, zorder=1)
    for i in range(0, GRID_SIZE_X + 1):
        ax.axvline(i, color=GRID_COLOR, linewidth=0.8, zorder=1)

    # 障碍物
    for ox, oy, shape in OBSTACLES:
        if shape == 'circle':
            ax.add_patch(Circle((ox, oy), 0.6, facecolor=OBSTACLE_COLOR, edgecolor='#1a252f',
                                linewidth=2.5, zorder=2))
        else:
            ax.add_patch(FancyBboxPatch((ox - 0.55, oy - 0.55), 1.1, 1.1,
                                        boxstyle="round,pad=0.02,rounding_size=0.15",
                                        facecolor=OBSTACLE_COLOR, edgecolor='#1a252f',
                                        linewidth=2.5, zorder=2))

    # 感知范围
    sensor = Circle(tuple(sensor_center), SENSOR_RANGE,
                    facecolor=(0.20, 0.60, 0.86, 0.12),
                    edgecolor=(0.20, 0.60, 0.86, 0.5),
                    linewidth=3, linestyle='--', zorder=3)
    ax.add_patch(sensor)

    # A*路径
    ax.plot(ASTAR_PATH[:, 0], ASTAR_PATH[:, 1], 'o-', color=ASTAR_COLOR, linewidth=3.5,
            markersize=12, label='A* Search Path', zorder=4,
            path_effects=[pe.Stroke(linewidth=5, foreground='#922b21')])

    # B样条曲线
    ax.plot(bspline_curve[:, 0], bspline_curve[:, 1], '-', color=SPLINE_COLOR, linewidth=4.5,
            label='B-Spline (ESDF Optimized)', zorder=5,
            path_effects=[pe.Stroke(linewidth=6, foreground='#1a5276')])

    # Bezier曲线
    ax.plot(bezier_curve[:, 0], bezier_curve[:, 1], '-', color=BEZIER_COLOR, linewidth=4.5,
            label='Bezier (ESDF Optimized)', zorder=6,
            path_effects=[pe.Stroke(linewidth=6, foreground='#196f3d')])

    # 控制点
    ax.plot(bspline_ctrl[:, 0], bspline_ctrl[:, 1], 's', color=SPLINE_COLOR, markersize=8,
            markeredgecolor='white', markeredgewidth=1.5, zorder=7, alpha=0.9)
    for seg in bezier_segs:
        ax.plot(seg[:, 0], seg[:, 1], 'D', color=BEZIER_COLOR, markersize=7,
                markeredgecolor='white', markeredgewidth=1.5, zorder=7, alpha=0.9)

    # 起点终点
    ax.plot(START[0], START[1], 'o', color='#c0392b', markersize=30, zorder=10,
            markeredgecolor='white', markeredgewidth=3)
    ax.text(START[0] + 0.5, START[1] - 1.0, 'START', fontsize=22, fontweight='bold',
            color='#c0392b', zorder=11)

    ax.plot(END[0], END[1], '*', color='#8e44ad', markersize=35, zorder=10,
            markeredgecolor='white', markeredgewidth=3)
    ax.text(END[0] + 0.5, END[1] - 1.0, 'GOAL', fontsize=22, fontweight='bold',
            color='#8e44ad', zorder=11)

    # 坐标轴
    ax.set_xlim(-0.5, GRID_SIZE_X + 0.5)
    ax.set_ylim(-0.5, GRID_SIZE_Y + 0.5)
    # ax.set_xlabel('X (m)', fontsize=30, fontweight='bold')
    # ax.set_ylabel('Y (m)', fontsize=30, fontweight='bold')
    ax.set_title('Spline vs Bezier Curve Comparison', fontsize=20, fontweight='bold', pad=15)
    ax.tick_params(axis='both', which='major', labelsize=22)

    # 图例
    legend = ax.legend(loc='upper left', fontsize=13, framealpha=0.95,
                       fancybox=True, shadow=True, borderpad=1)
    legend.get_frame().set_linewidth(2)

    ax.grid(True, alpha=0.4, linestyle=':', linewidth=1.5)

    # 恢复绘图边框
    ax.set_frame_on(True)
    # 只显示左边和下边的坐标轴
    ax.spines['top'].set_visible(False)
    ax.spines['right'].set_visible(False)

    ax.set_aspect('equal')
    plt.tight_layout()
    plt.savefig(output_file, dpi=200, bbox_inches='tight', facecolor='white', edgecolor='none')
    print(f"图片已保存: {output_file}")

# ============ 生成图片 ============
draw_figure(bspline_curve_1, bezier_curve_1, BSPLINE_CTRL_1,
            [BEZIER_SEG1_1, BEZIER_SEG2_1, BEZIER_SEG3_1, BEZIER_SEG4_1],
            START, 'bspline_vs_bezier_comparison.png')

MID_POINT = np.array([10.0, 9.0])
draw_figure(bspline_curve_2, bezier_curve_2, BSPLINE_CTRL_2,
            [BEZIER_SEG1_2, BEZIER_SEG2_2, BEZIER_SEG3_2, BEZIER_SEG4_2],
            MID_POINT, 'bspline_vs_bezier_comparison_mid_sensor.png',
            title_suffix=' (Sensor at Center)')

# ============ 图3: 感知在终点 ============
GOAL_POINT = np.array([18.0, 13.0])
draw_figure(bspline_curve_2, bezier_curve_2, BSPLINE_CTRL_2,
            [BEZIER_SEG1_2, BEZIER_SEG2_2, BEZIER_SEG3_2, BEZIER_SEG4_2],
            GOAL_POINT, 'bspline_vs_bezier_comparison_goal_sensor.png',
            title_suffix=' (Sensor at Goal)')

# ============ 验证碰撞检测 ============
print("\n=== 碰撞检测验证 ===")

def verify_collision(curve, name, sensor_center):
    in_range = get_curve_in_sensor_range(curve, sensor_center, SENSOR_RANGE)
    collisions = [idx for idx, pt in in_range if check_collision_point(pt, OBSTACLES)]
    status = "OK" if len(collisions) == 0 else f"COLLISION {len(collisions)} pts"
    print(f"{name}: {len(in_range)} pts in range, {status}")
    return len(collisions) == 0

verify_collision(bspline_curve_1, "图1 B-spline", START)
verify_collision(bezier_curve_1, "图1 Bezier", START)
verify_collision(bspline_curve_2, "图2 B-spline", MID_POINT)
verify_collision(bezier_curve_2, "图2 Bezier", MID_POINT)
verify_collision(bspline_curve_2, "图3 B-spline", GOAL_POINT)
verify_collision(bezier_curve_2, "图3 Bezier", GOAL_POINT)
print("\n验证完成！")