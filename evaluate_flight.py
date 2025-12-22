#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
飞行评价程序 - 使用pandas分析飞行指标
"""

import pandas as pd
import numpy as np
import glob
import os
from datetime import datetime

class FlightEvaluator:
    """飞行评价器"""
    
    def __init__(self, csv_file):
        """初始化评价器
        
        Args:
            csv_file: CSV数据文件路径
        """
        self.csv_file = csv_file
        self.df = pd.read_csv(csv_file)
        self.scores = {}
        
    def evaluate(self):
        """评估飞行性能"""
        
        # 过滤有效数据（success_rate > 0表示飞行中）
        valid_data = self.df[self.df['success_rate'] > 0]
        
        if len(valid_data) == 0:
            return {
                'overall_score': 0,
                'grade': 'F',
                'status': '无有效飞行数据',
                'details': {}
            }
        
        print(f"\n{'='*60}")
        print(f"飞行数据分析报告")
        print(f"{'='*60}")
        print(f"数据文件: {os.path.basename(self.csv_file)}")
        print(f"有效数据点: {len(valid_data)} / {len(self.df)}")
        print(f"{'='*60}\n")
        
        # 1. 成功率评分 (0-20分)
        success_rate = valid_data['success_rate'].mean()
        success_score = success_rate * 20
        self.scores['success_rate'] = success_score
        print(f"✓ 成功率: {success_rate*100:.1f}%  →  得分: {success_score:.1f}/20")
        
        # 2. 优化效率评分 (0-20分)
        avg_opt_time = valid_data['avg_opt_time_ms'].mean()
        avg_iterations = valid_data['avg_iterations'].mean()
        
        # 优化时间越短越好（假设理想时间<10ms，超过50ms很差）
        time_score = max(0, min(10, 10 * (1 - (avg_opt_time - 10) / 40)))
        # 迭代次数越少越好（假设理想<10次，超过50次很差）
        iter_score = max(0, min(10, 10 * (1 - (avg_iterations - 10) / 40)))
        
        opt_score = time_score + iter_score
        self.scores['optimization'] = opt_score
        print(f"✓ 平均优化时间: {avg_opt_time:.2f}ms")
        print(f"✓ 平均迭代次数: {avg_iterations:.1f}")
        print(f"  优化效率得分: {opt_score:.1f}/20")
        
        # 3. 安全性评分 (0-25分)
        min_clearance = valid_data['min_clearance'].min()
        collision_count = valid_data['collision_count'].max()
        
        # 最小间隙评分（理想>2m，<0.5m很危险）
        clearance_score = max(0, min(15, 15 * (min_clearance - 0.5) / 1.5))
        # 碰撞次数扣分
        collision_penalty = min(10, collision_count * 2)
        
        safety_score = clearance_score + (10 - collision_penalty)
        self.scores['safety'] = safety_score
        print(f"✓ 最小安全间隙: {min_clearance:.2f}m")
        print(f"✓ 碰撞次数: {int(collision_count)}")
        print(f"  安全性得分: {safety_score:.1f}/25")
        
        # 4. 平滑性评分 (0-15分)
        avg_smoothness = valid_data['avg_smoothness'].mean()
        max_vel = valid_data['max_velocity'].max()
        max_acc = valid_data['max_acceleration'].max()
        
        # 平滑性评分（值越小越好，假设理想<10，>100很差）
        smooth_score = max(0, min(8, 8 * (1 - (avg_smoothness - 10) / 90)))
        # 速度和加速度合理性（不超限得分）
        vel_penalty = max(0, (max_vel - 2.0) * 2)  # 超过2m/s扣分
        acc_penalty = max(0, (max_acc - 3.0) * 1)  # 超过3m/s²扣分
        
        smoothness_score = smooth_score + max(0, 7 - vel_penalty - acc_penalty)
        self.scores['smoothness'] = smoothness_score
        print(f"✓ 平均平滑度: {avg_smoothness:.2f}")
        print(f"✓ 最大速度: {max_vel:.2f}m/s")
        print(f"✓ 最大加速度: {max_acc:.2f}m/s²")
        print(f"  平滑性得分: {smoothness_score:.1f}/15")
        
        # 5. 效率评分 (0-20分)
        path_length = valid_data['path_length'].max()
        replan_freq = valid_data['replan_freq_hz'].mean()
        final_cost = valid_data['final_cost'].mean()
        
        # 重规划频率适中最好（1-2Hz理想，太高或太低都不好）
        if 1.0 <= replan_freq <= 2.0:
            replan_score = 10
        elif 0.5 <= replan_freq <= 3.0:
            replan_score = 7
        else:
            replan_score = max(0, 10 - abs(replan_freq - 1.5) * 2)
        
        # 最终代价越小越好
        cost_score = max(0, min(10, 10 * (1 - final_cost / 1000)))
        
        efficiency_score = replan_score + cost_score
        self.scores['efficiency'] = efficiency_score
        print(f"✓ 路径长度: {path_length:.2f}m")
        print(f"✓ 重规划频率: {replan_freq:.2f}Hz")
        print(f"✓ 最终代价: {final_cost:.2f}")
        print(f"  效率得分: {efficiency_score:.1f}/20")
        
        # 计算总分
        total_score = sum(self.scores.values())
        
        # 评级
        if total_score >= 90:
            grade = 'A+'
            comment = '优秀！飞行性能卓越'
        elif total_score >= 80:
            grade = 'A'
            comment = '很好！飞行性能良好'
        elif total_score >= 70:
            grade = 'B'
            comment = '良好，但仍有改进空间'
        elif total_score >= 60:
            grade = 'C'
            comment = '及格，需要优化'
        elif total_score >= 50:
            grade = 'D'
            comment = '较差，需要重点改进'
        else:
            grade = 'F'
            comment = '不及格，存在严重问题'
        
        print(f"\n{'='*60}")
        print(f"总分: {total_score:.1f}/100  |  评级: {grade}")
        print(f"评价: {comment}")
        print(f"{'='*60}\n")
        
        return {
            'overall_score': total_score,
            'grade': grade,
            'comment': comment,
            'scores': self.scores,
            'stats': {
                'success_rate': success_rate,
                'avg_opt_time_ms': avg_opt_time,
                'avg_iterations': avg_iterations,
                'min_clearance': min_clearance,
                'collision_count': int(collision_count),
                'avg_smoothness': avg_smoothness,
                'max_velocity': max_vel,
                'max_acceleration': max_acc,
                'path_length': path_length,
                'replan_freq_hz': replan_freq,
                'final_cost': final_cost
            }
        }

def find_latest_csv(metrics_dir="/tmp/ego_planner_metrics"):
    """找到最新的metrics CSV文件"""
    csv_files = glob.glob(os.path.join(metrics_dir, "metrics_*.csv"))
    if not csv_files:
        return None
    
    # 按修改时间排序
    csv_files.sort(key=os.path.getmtime, reverse=True)
    
    # 找到第一个非空文件
    for f in csv_files:
        if os.path.getsize(f) > 200:  # 至少有一些数据
            return f
    
    return None

def main():
    """主函数"""
    import sys
    
    # 如果提供了文件路径参数
    if len(sys.argv) > 1:
        csv_file = sys.argv[1]
    else:
        # 自动找最新的CSV文件
        csv_file = find_latest_csv()
        if csv_file is None:
            print("错误: 找不到有效的metrics CSV文件")
            print("请运行 roslaunch ego_planner simple_run.launch 生成飞行数据")
            return 1
    
    if not os.path.exists(csv_file):
        print(f"错误: 文件不存在: {csv_file}")
        return 1
    
    try:
        # 创建评价器并评估
        evaluator = FlightEvaluator(csv_file)
        result = evaluator.evaluate()
        
        # 保存评价结果
        result_file = "/tmp/ego_planner_metrics/evaluation_result.txt"
        with open(result_file, 'w') as f:
            f.write(f"飞行评价报告\n")
            f.write(f"{'='*60}\n")
            f.write(f"数据文件: {os.path.basename(csv_file)}\n")
            f.write(f"评估时间: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
            f.write(f"{'='*60}\n\n")
            f.write(f"总分: {result['overall_score']:.1f}/100\n")
            f.write(f"评级: {result['grade']}\n")
            f.write(f"评价: {result['comment']}\n\n")
            f.write(f"详细得分:\n")
            for key, score in result['scores'].items():
                f.write(f"  {key}: {score:.1f}\n")
        
        print(f"评价结果已保存到: {result_file}")
        return 0
        
    except Exception as e:
        print(f"错误: {e}")
        import traceback
        traceback.print_exc()
        return 1

if __name__ == "__main__":
    exit(main())
