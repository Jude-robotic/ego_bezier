#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rosbag
import numpy as np
import matplotlib.pyplot as plt
import sys
import os
# from scipy.signal import savgol_filter # Optional if needed

def read_bag_data(bag_path):
    """从rosbag读取位置指令和实际里程计"""
    pos_cmd = {'t': [], 'x': [], 'y': [], 'z': [], 'vx': [], 'vy': [], 'vz': [], 'ax': [], 'ay': [], 'az': []}
    odom = {'t': [], 'x': [], 'y': [], 'z': []}
    
    print(f"正在读取: {bag_path} ...")
    try:
        bag = rosbag.Bag(bag_path)
    except Exception as e:
        print(f"错误: 无法打开 {bag_path}. {e}")
        return None, None

    # 读取指令话题
    for topic, msg, t in bag.read_messages(topics=['/planning/pos_cmd']):
        pos_cmd['t'].append(t.to_sec())
        pos_cmd['x'].append(msg.position.x)
        pos_cmd['y'].append(msg.position.y)
        pos_cmd['z'].append(msg.position.z)
        pos_cmd['vx'].append(msg.velocity.x)
        pos_cmd['vy'].append(msg.velocity.y)
        pos_cmd['vz'].append(msg.velocity.z)
        pos_cmd['ax'].append(msg.acceleration.x)
        pos_cmd['ay'].append(msg.acceleration.y)
        pos_cmd['az'].append(msg.acceleration.z)

    # 读取里程计话题
    for topic, msg, t in bag.read_messages(topics=['/visual_slam/odom']):
        odom['t'].append(t.to_sec())
        odom['x'].append(msg.pose.pose.position.x)
        odom['y'].append(msg.pose.pose.position.y)
        odom['z'].append(msg.pose.pose.position.z)

    bag.close()
    
    # 转换为numpy数组
    for k in pos_cmd: pos_cmd[k] = np.array(pos_cmd[k])
    for k in odom: odom[k] = np.array(odom[k])
    
    # 时间归零
    if len(pos_cmd['t']) > 0:
        start_t = pos_cmd['t'][0]
        pos_cmd['t'] -= start_t
    
    if len(odom['t']) > 0:
        # 对齐odom时间
        if len(pos_cmd['t']) > 0:
            odom['t'] -= start_t
        else:
            odom['t'] -= odom['t'][0]

    return pos_cmd, odom

def calculate_metrics(data, name):
    """计算核心指标"""
    if len(data['t']) < 2:
        return None

    dt = np.diff(data['t'])
    valid_idx = dt > 0.001
    dt = dt[valid_idx]
    
    # 1. 计算 Jerk (加加速度)
    # j = da/dt
    jx = np.diff(data['ax'])[valid_idx] / dt
    jy = np.diff(data['ay'])[valid_idx] / dt
    jz = np.diff(data['az'])[valid_idx] / dt
    jerk_norm = np.sqrt(jx**2 + jy**2 + jz**2)
    
    mean_jerk = np.mean(jerk_norm)
    max_jerk = np.max(jerk_norm)
    
    # 2. 计算轨迹长度
    dx = np.diff(data['x'])
    dy = np.diff(data['y'])
    dz = np.diff(data['z'])
    dist = np.sqrt(dx**2 + dy**2 + dz**2)
    path_length = np.sum(dist)
    
    # 3. 总时间
    total_time = data['t'][-1] - data['t'][0]
    
    print(f"\n--- {name} 分析结果 ---")
    print(f"总飞行时间: {total_time:.2f} s")
    print(f"轨迹总长度: {path_length:.2f} m")
    print(f"平均 Jerk:  {mean_jerk:.2f} m/s^3 (越小越平滑)")
    print(f"最大 Jerk:  {max_jerk:.2f} m/s^3")
    
    return {
        'time': total_time,
        'length': path_length,
        'mean_jerk': mean_jerk,
        'jerk_norm': jerk_norm,
        't_jerk': data['t'][1:][valid_idx]
    }

def plot_comparison(res1, res2, name1="Original", name2="Bezier"):
    """绘制对比图"""
    plt.figure(figsize=(12, 8))
    
    # 1. Jerk 对比
    plt.subplot(2, 1, 1)
    if res1: plt.plot(res1['t_jerk'], res1['jerk_norm'], label=f'{name1} Jerk', alpha=0.7)
    if res2: plt.plot(res2['t_jerk'], res2['jerk_norm'], label=f'{name2} Jerk', alpha=0.7)
    plt.title('Trajectory Smoothness (Jerk) Comparison')
    plt.ylabel('Jerk ($m/s^3$)')
    plt.legend()
    plt.grid(True)
    
    # 2. 统计柱状图
    plt.subplot(2, 1, 2)
    metrics = ['Time (s)', 'Length (m)', 'Mean Jerk']
    x = np.arange(len(metrics))
    width = 0.35
    
    vals1 = [res1['time'], res1['length'], res1['mean_jerk']] if res1 else [0,0,0]
    vals2 = [res2['time'], res2['length'], res2['mean_jerk']] if res2 else [0,0,0]
    
    plt.bar(x - width/2, vals1, width, label=name1)
    plt.bar(x + width/2, vals2, width, label=name2)
    
    plt.ylabel('Value')
    plt.title('Overall Metrics Comparison')
    plt.xticks(x, metrics)
    plt.legend()
    
    plt.tight_layout()
    plt.savefig('comparison_result.png')
    print("\n图表已保存为 comparison_result.png")

if __name__ == "__main__":
    if len(sys.argv) < 3:
        print("使用方法: python3 compare_planners.py <original.bag> <bezier.bag>")
        print("示例: python3 compare_planners.py original_ego.bag bezier_ego.bag")
        sys.exit(1)
        
    bag1 = sys.argv[1]
    bag2 = sys.argv[2]
    
    cmd1, _ = read_bag_data(bag1)
    cmd2, _ = read_bag_data(bag2)
    
    if cmd1 is None or cmd2 is None:
        print("数据读取失败")
        sys.exit(1)
        
    res1 = calculate_metrics(cmd1, "Original (B-Spline)")
    res2 = calculate_metrics(cmd2, "New (Bezier)")
    
    plot_comparison(res1, res2)
