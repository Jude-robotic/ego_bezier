#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
飞行数据可视化 - Docker友好版本
生成HTML交互式报告和PNG图表
"""

import pandas as pd
import matplotlib
matplotlib.use('Agg')  # 不需要显示后端
import matplotlib.pyplot as plt
import glob
import os
from datetime import datetime

def find_latest_csv(metrics_dir="/tmp/ego_planner_metrics"):
    """找到最新的metrics CSV文件"""
    csv_files = glob.glob(os.path.join(metrics_dir, "metrics_*.csv"))
    if not csv_files:
        return None
    
    csv_files.sort(key=os.path.getmtime, reverse=True)
    for f in csv_files:
        if os.path.getsize(f) > 200:
            return f
    return None

def generate_plots(csv_file, output_dir="/tmp/ego_planner_metrics"):
    """生成PNG图表"""
    
    # 读取数据
    df = pd.read_csv(csv_file)
    valid_data = df[df['success_rate'] > 0]
    
    if len(valid_data) == 0:
        print("警告: 没有有效飞行数据")
        return False
    
    # 计算相对时间
    if 'timestamp' in valid_data.columns:
        start_time = valid_data['timestamp'].iloc[0]
        time_axis = (valid_data['timestamp'] - start_time).values
    else:
        time_axis = range(len(valid_data))
    
    # 设置中文字体
    plt.rcParams['font.sans-serif'] = ['DejaVu Sans', 'Arial Unicode MS', 'SimHei']
    plt.rcParams['axes.unicode_minus'] = False
    
    # 创建图表
    fig, axes = plt.subplots(3, 2, figsize=(16, 12))
    fig.suptitle('Ego-Planner Flight Performance Analysis', fontsize=16, fontweight='bold')
    
    # 1. 速度曲线
    ax = axes[0, 0]
    ax.plot(time_axis, valid_data['max_velocity'], 'b-', linewidth=2, label='Max Velocity')
    ax.axhline(y=2.0, color='r', linestyle='--', alpha=0.5, label='Speed Limit')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Velocity (m/s)')
    ax.set_title('Velocity Profile')
    ax.legend()
    ax.grid(True, alpha=0.3)
    
    # 2. 加速度曲线
    ax = axes[0, 1]
    ax.plot(time_axis, valid_data['max_acceleration'], 'r-', linewidth=2)
    ax.axhline(y=3.0, color='orange', linestyle='--', alpha=0.5, label='Acc Limit')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Acceleration (m/s²)')
    ax.set_title('Acceleration Profile')
    ax.legend()
    ax.grid(True, alpha=0.3)
    
    # 3. 成功率
    ax = axes[1, 0]
    ax.plot(time_axis, valid_data['success_rate'] * 100, 'g-', linewidth=2)
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Success Rate (%)')
    ax.set_title('Planning Success Rate')
    ax.set_ylim([0, 105])
    ax.grid(True, alpha=0.3)
    
    # 4. 平滑度
    ax = axes[1, 1]
    ax.plot(time_axis, valid_data['avg_smoothness'], 'm-', linewidth=2)
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Smoothness')
    ax.set_title('Trajectory Smoothness')
    ax.grid(True, alpha=0.3)
    
    # 5. 安全间隙
    ax = axes[2, 0]
    clearance = valid_data['min_clearance'].copy()
    clearance[clearance > 10] = 10  # 限制显示范围
    ax.plot(time_axis, clearance, 'c-', linewidth=2)
    ax.axhline(y=0.5, color='r', linestyle='--', alpha=0.5, label='Safety Threshold')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Min Clearance (m)')
    ax.set_title('Safety Clearance')
    ax.legend()
    ax.grid(True, alpha=0.3)
    
    # 6. 重规划频率
    ax = axes[2, 1]
    replan_freq = valid_data['replan_freq_hz'].copy()
    replan_freq[replan_freq > 5] = 5  # 限制显示范围
    ax.plot(time_axis, replan_freq, 'orange', linewidth=2)
    ax.axhline(y=1.0, color='g', linestyle='--', alpha=0.5, label='Ideal Range')
    ax.axhline(y=2.0, color='g', linestyle='--', alpha=0.5)
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Replanning Freq (Hz)')
    ax.set_title('Replanning Frequency')
    ax.legend()
    ax.grid(True, alpha=0.3)
    
    plt.tight_layout()
    
    # 保存图表
    plot_file = os.path.join(output_dir, "flight_analysis.png")
    plt.savefig(plot_file, dpi=150, bbox_inches='tight')
    plt.close()
    
    print(f"✓ 图表已生成: {plot_file}")
    return plot_file

def generate_html_report(csv_file, plot_file, output_dir="/tmp/ego_planner_metrics"):
    """生成HTML报告"""
    
    df = pd.read_csv(csv_file)
    valid_data = df[df['success_rate'] > 0]
    
    if len(valid_data) == 0:
        return False
    
    # 计算统计数据
    stats = {
        'success_rate': valid_data['success_rate'].mean() * 100,
        'avg_velocity': valid_data['max_velocity'].mean(),
        'max_velocity': valid_data['max_velocity'].max(),
        'avg_acceleration': valid_data['max_acceleration'].mean(),
        'max_acceleration': valid_data['max_acceleration'].max(),
        'min_clearance': valid_data['min_clearance'].min(),
        'collision_count': int(valid_data['collision_count'].max()),
        'avg_smoothness': valid_data['avg_smoothness'].mean(),
        'replan_freq': valid_data['replan_freq_hz'].mean(),
        'path_length': valid_data['path_length'].max(),
        'total_points': len(valid_data)
    }
    
    # 生成HTML
    html = f"""<!DOCTYPE html>
<html lang="zh-CN">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Ego-Planner Flight Report</title>
    <style>
        * {{ margin: 0; padding: 0; box-sizing: border-box; }}
        body {{
            font-family: 'Segoe UI', Arial, sans-serif;
            background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
            padding: 20px;
        }}
        .container {{
            max-width: 1400px;
            margin: 0 auto;
            background: white;
            border-radius: 15px;
            box-shadow: 0 15px 50px rgba(0,0,0,0.3);
            overflow: hidden;
        }}
        .header {{
            background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
            color: white;
            padding: 40px;
            text-align: center;
        }}
        .header h1 {{
            font-size: 2.5em;
            margin-bottom: 10px;
        }}
        .header p {{
            font-size: 1.2em;
            opacity: 0.9;
        }}
        .content {{
            padding: 40px;
        }}
        .stats-grid {{
            display: grid;
            grid-template-columns: repeat(auto-fit, minmax(280px, 1fr));
            gap: 20px;
            margin-bottom: 40px;
        }}
        .stat-card {{
            background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
            color: white;
            padding: 25px;
            border-radius: 10px;
            box-shadow: 0 4px 10px rgba(0,0,0,0.1);
            transition: transform 0.3s;
        }}
        .stat-card:hover {{
            transform: translateY(-5px);
        }}
        .stat-card h3 {{
            font-size: 0.9em;
            text-transform: uppercase;
            letter-spacing: 1px;
            opacity: 0.9;
            margin-bottom: 12px;
        }}
        .stat-card .value {{
            font-size: 2.2em;
            font-weight: bold;
            margin-bottom: 5px;
        }}
        .stat-card .unit {{
            font-size: 0.9em;
            opacity: 0.8;
        }}
        .chart-section {{
            background: #f8f9fa;
            padding: 30px;
            border-radius: 10px;
            margin-bottom: 30px;
        }}
        .chart-section h2 {{
            color: #333;
            margin-bottom: 20px;
            font-size: 1.8em;
        }}
        .chart-section img {{
            width: 100%;
            border-radius: 8px;
            box-shadow: 0 4px 10px rgba(0,0,0,0.1);
        }}
        .alert {{
            padding: 20px;
            border-radius: 8px;
            margin-bottom: 20px;
            font-size: 1.1em;
        }}
        .alert-success {{
            background: #d4edda;
            color: #155724;
            border-left: 5px solid #28a745;
        }}
        .alert-warning {{
            background: #fff3cd;
            color: #856404;
            border-left: 5px solid #ffc107;
        }}
        .alert-danger {{
            background: #f8d7da;
            color: #721c24;
            border-left: 5px solid #dc3545;
        }}
        .footer {{
            background: #f8f9fa;
            padding: 30px;
            text-align: center;
            color: #666;
            border-top: 2px solid #e9ecef;
        }}
        .footer p {{
            margin: 5px 0;
        }}
    </style>
</head>
<body>
    <div class="container">
        <div class="header">
            <h1>🚁 Flight Performance Report</h1>
            <p>Ego-Planner Bezier Optimizer Analysis</p>
        </div>
        
        <div class="content">
            {f'<div class="alert alert-danger">⚠️ 检测到 {stats["collision_count"]} 次碰撞！需要检查安全性</div>' if stats['collision_count'] > 0 else ''}
            {f'<div class="alert alert-warning">⚠️ 最小安全间隙 {stats["min_clearance"]:.2f}m，接近安全阈值</div>' if stats['min_clearance'] < 0.8 and stats['collision_count'] == 0 else ''}
            {f'<div class="alert alert-success">✓ 飞行安全，无碰撞记录，安全间隙充足</div>' if stats['collision_count'] == 0 and stats['min_clearance'] >= 0.8 else ''}
            
            <div class="stats-grid">
                <div class="stat-card">
                    <h3>Success Rate</h3>
                    <div class="value">{stats['success_rate']:.1f}<span class="unit">%</span></div>
                </div>
                <div class="stat-card">
                    <h3>Avg Velocity</h3>
                    <div class="value">{stats['avg_velocity']:.2f}<span class="unit">m/s</span></div>
                    <div class="unit">Max: {stats['max_velocity']:.2f} m/s</div>
                </div>
                <div class="stat-card">
                    <h3>Avg Acceleration</h3>
                    <div class="value">{stats['avg_acceleration']:.2f}<span class="unit">m/s²</span></div>
                    <div class="unit">Max: {stats['max_acceleration']:.2f} m/s²</div>
                </div>
                <div class="stat-card">
                    <h3>Safety Clearance</h3>
                    <div class="value">{stats['min_clearance']:.2f}<span class="unit">m</span></div>
                    <div class="unit">Collisions: {stats['collision_count']}</div>
                </div>
                <div class="stat-card">
                    <h3>Avg Smoothness</h3>
                    <div class="value">{stats['avg_smoothness']:.1f}</div>
                    <div class="unit">Lower is better</div>
                </div>
                <div class="stat-card">
                    <h3>Replan Frequency</h3>
                    <div class="value">{stats['replan_freq']:.2f}<span class="unit">Hz</span></div>
                </div>
            </div>
            
            <div class="chart-section">
                <h2>📊 Performance Charts</h2>
                <img src="flight_analysis.png" alt="Flight Analysis Charts">
            </div>
        </div>
        
        <div class="footer">
            <p><strong>Data Source:</strong> {os.path.basename(csv_file)}</p>
            <p><strong>Data Points:</strong> {stats['total_points']}</p>
            <p><strong>Generated:</strong> {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}</p>
            <p style="margin-top: 15px; font-size: 0.9em; opacity: 0.7;">
                Ego-Planner Performance Analysis System
            </p>
        </div>
    </div>
</body>
</html>"""
    
    output_file = os.path.join(output_dir, "flight_report.html")
    with open(output_file, 'w', encoding='utf-8') as f:
        f.write(html)
    
    print(f"✓ HTML报告已生成: {output_file}")
    return output_file

def main():
    import sys
    
    print("="*60)
    print("  Ego-Planner 飞行数据可视化")
    print("="*60)
    
    # 确定CSV文件
    if len(sys.argv) > 1:
        csv_file = sys.argv[1]
    else:
        csv_file = find_latest_csv()
        if csv_file is None:
            print("\n错误: 找不到有效的metrics CSV文件")
            return 1
        print(f"\n使用最新数据: {csv_file}")
    
    if not os.path.exists(csv_file):
        print(f"错误: 文件不存在: {csv_file}")
        return 1
    
    try:
        output_dir = "/tmp/ego_planner_metrics"
        
        # 生成图表
        print("\n生成图表...")
        plot_file = generate_plots(csv_file, output_dir)
        
        # 生成HTML报告
        print("生成HTML报告...")
        html_file = generate_html_report(csv_file, plot_file, output_dir)
        
        print("\n" + "="*60)
        print("✓ 可视化完成！")
        print("="*60)
        print(f"\n查看方式:")
        print(f"  1. 在Docker内启动HTTP服务:")
        print(f"     cd {output_dir} && python3 -m http.server 8000")
        print(f"     然后访问: http://localhost:8000/flight_report.html")
        print(f"\n  2. 复制到宿主机:")
        print(f"     docker cp <容器ID>:{html_file} ~/")
        print(f"     docker cp <容器ID>:{plot_file} ~/")
        print(f"     然后用浏览器打开 ~/flight_report.html")
        print()
        
        return 0
        
    except Exception as e:
        print(f"\n错误: {e}")
        import traceback
        traceback.print_exc()
        return 1

if __name__ == "__main__":
    exit(main())
