#!/usr/bin/env python3
"""
实时可视化规划器性能指标
支持实时显示和保存到文件两种模式
"""

import sys
import argparse

# 必须在导入 pyplot 之前设置后端
import matplotlib
matplotlib.use('Agg')  # 使用非交互式后端，兼容无显示环境

import rospy
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from std_msgs.msg import Float64MultiArray
from collections import deque

class MetricsVisualizer:
    def __init__(self, save_to_file=False, output_path='/tmp/ego_planner_metrics/metrics_plot.png'):
        rospy.init_node('metrics_visualizer', anonymous=True)
        
        self.save_to_file = save_to_file
        self.output_path = output_path
        self.update_counter = 0
        
        # 数据缓冲区
        self.max_points = 100
        self.time_data = deque(maxlen=self.max_points)
        self.success_rate = deque(maxlen=self.max_points)
        self.opt_time = deque(maxlen=self.max_points)
        self.iterations = deque(maxlen=self.max_points)
        self.smoothness = deque(maxlen=self.max_points)
        self.min_clearance = deque(maxlen=self.max_points)
        self.avg_clearance = deque(maxlen=self.max_points)
        self.max_velocity = deque(maxlen=self.max_points)
        self.max_acceleration = deque(maxlen=self.max_points)
        self.path_length = deque(maxlen=self.max_points)
        self.final_cost = deque(maxlen=self.max_points)
        self.replan_freq = deque(maxlen=self.max_points)
        
        self.start_time = rospy.Time.now()
        
        # 订阅话题
        rospy.Subscriber('/planner_metrics', Float64MultiArray, self.metrics_callback)
        
        # 创建图表
        self.setup_plots()
        
    def setup_plots(self):
        """设置matplotlib子图"""
        plt.style.use('default')
        self.fig = plt.figure(figsize=(16, 10))
        self.fig.suptitle('EGO-Planner Performance Metrics', fontsize=16, fontweight='bold')
        
        # 创建子图
        self.ax1 = plt.subplot(3, 4, 1)
        self.ax1.set_title('Success Rate')
        self.ax2 = plt.subplot(3, 4, 2)
        self.ax2.set_title('Optimization Time')
        self.ax3 = plt.subplot(3, 4, 3)
        self.ax3.set_title('Iterations')
        self.ax4 = plt.subplot(3, 4, 4)
        self.ax4.set_title('Replan Frequency')
        self.ax5 = plt.subplot(3, 4, 5)
        self.ax5.set_title('Smoothness')
        self.ax6 = plt.subplot(3, 4, 6)
        self.ax6.set_title('Path Length')
        self.ax7 = plt.subplot(3, 4, 7)
        self.ax7.set_title('Final Cost')
        self.ax8 = plt.subplot(3, 4, 8)
        self.ax8.set_title('Clearance')
        self.ax9 = plt.subplot(3, 4, 9)
        self.ax9.set_title('Max Velocity')
        self.ax10 = plt.subplot(3, 4, 10)
        self.ax10.set_title('Max Acceleration')
        self.ax11 = plt.subplot(3, 4, 11)
        self.ax11.set_title('Statistics')
        self.ax11.axis('off')
        
        plt.tight_layout()
        
    def metrics_callback(self, msg):
        if len(msg.data) < 11:
            return
        
        current_time = (rospy.Time.now() - self.start_time).to_sec()
        self.time_data.append(current_time)
        self.success_rate.append(msg.data[0] * 100)
        self.opt_time.append(msg.data[1])
        self.iterations.append(msg.data[2])
        self.smoothness.append(msg.data[3])
        self.min_clearance.append(msg.data[4])
        self.avg_clearance.append(msg.data[5])
        self.max_velocity.append(msg.data[6])
        self.max_acceleration.append(msg.data[7])
        self.path_length.append(msg.data[8])
        self.final_cost.append(msg.data[9])
        self.replan_freq.append(msg.data[10])
        
        # 如果是保存到文件模式，每收到10个数据点就更新一次图表
        if self.save_to_file:
            self.update_counter += 1
            if self.update_counter % 10 == 0:
                self.update_plots(None)
                rospy.loginfo(f"Updated metrics plot: {len(self.time_data)} data points")
        
    def update_plots(self, frame):
        if len(self.time_data) == 0:
            return
        
        t = list(self.time_data)
        
        for ax in [self.ax1, self.ax2, self.ax3, self.ax4, self.ax5,
                   self.ax6, self.ax7, self.ax8, self.ax9, self.ax10]:
            ax.clear()
        
        self.ax1.plot(t, list(self.success_rate), 'g-', linewidth=2)
        self.ax1.set_ylabel('Rate (%)')
        self.ax1.set_title('Success Rate')
        self.ax1.grid(True)
        
        self.ax2.plot(t, list(self.opt_time), 'b-', linewidth=2)
        self.ax2.set_ylabel('Time (ms)')
        self.ax2.set_title('Optimization Time')
        self.ax2.grid(True)
        
        self.ax3.plot(t, list(self.iterations), 'purple', linewidth=2)
        self.ax3.set_ylabel('Count')
        self.ax3.set_title('Iterations')
        self.ax3.grid(True)
        
        self.ax4.plot(t, list(self.replan_freq), 'orange', linewidth=2)
        self.ax4.set_ylabel('Hz')
        self.ax4.set_title('Replan Frequency')
        self.ax4.grid(True)
        
        self.ax5.plot(t, list(self.smoothness), 'cyan', linewidth=2)
        self.ax5.set_title('Smoothness')
        self.ax5.grid(True)
        
        self.ax6.plot(t, list(self.path_length), 'brown', linewidth=2)
        self.ax6.set_ylabel('m')
        self.ax6.set_title('Path Length')
        self.ax6.grid(True)
        
        self.ax7.plot(t, list(self.final_cost), 'red', linewidth=2)
        self.ax7.set_title('Final Cost')
        self.ax7.grid(True)
        
        self.ax8.plot(t, list(self.min_clearance), 'r-', linewidth=2, label='Min')
        self.ax8.plot(t, list(self.avg_clearance), 'g-', linewidth=2, label='Avg')
        self.ax8.set_ylabel('m')
        self.ax8.set_title('Clearance')
        self.ax8.legend()
        self.ax8.grid(True)
        
        self.ax9.plot(t, list(self.max_velocity), 'blue', linewidth=2)
        self.ax9.set_ylabel('m/s')
        self.ax9.set_title('Max Velocity')
        self.ax9.grid(True)
        
        self.ax10.plot(t, list(self.max_acceleration), 'green', linewidth=2)
        self.ax10.set_ylabel('m/s²')
        self.ax10.set_title('Max Acceleration')
        self.ax10.grid(True)
        
        self.ax11.clear()
        self.ax11.axis('off')
        if len(self.time_data) > 0:
            stats = f"""Success: {self.success_rate[-1]:.1f}%
OptTime: {np.mean(list(self.opt_time)):.1f}ms
Iters: {np.mean(list(self.iterations)):.1f}
Clearance: {self.min_clearance[-1]:.3f}m
Velocity: {self.max_velocity[-1]:.2f}m/s
Frequency: {self.replan_freq[-1]:.2f}Hz"""
            self.ax11.text(0.1, 0.5, stats, fontsize=10, family='monospace')
            self.ax11.set_title('Statistics')
        
        plt.tight_layout()
        
        # 如果是保存到文件模式，立即保存
        if self.save_to_file:
            self.fig.savefig(self.output_path, dpi=150, bbox_inches='tight')
        
    def run(self):
        if self.save_to_file:
            rospy.loginfo(f"Metrics visualizer running in file mode, output: {self.output_path}")
            rospy.loginfo("Listening to /planner_metrics topic...")
            rospy.spin()
        else:
            rospy.loginfo("Metrics visualizer running in animation mode")
            ani = FuncAnimation(self.fig, self.update_plots, interval=500, cache_frame_data=False)
            plt.show()

def main():
    parser = argparse.ArgumentParser(description='EGO-Planner Metrics Visualizer')
    parser.add_argument('--save', action='store_true', 
                        help='Save plots to file instead of showing interactively')
    parser.add_argument('--output', type=str, 
                        default='/tmp/ego_planner_metrics/metrics_plot.png',
                        help='Output file path when using --save mode')
    
    args = parser.parse_args()
    
    try:
        visualizer = MetricsVisualizer(save_to_file=args.save, output_path=args.output)
        visualizer.run()
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        print("\nShutting down metrics visualizer...")

if __name__ == '__main__':
    main()
