╔═══════════════════════════════════════════════════════════════╗
║           EGO-Planner 性能指标系统 - 使用指南                    ║
╚═══════════════════════════════════════════════════════════════╝

本系统为 ego-planner-bezier 提供 12 个关键性能指标的实时监控和可视化。

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
✅ 最新修复 (2025-12-21)
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

已修复可视化脚本的 tkinter 依赖问题：
• 使用 matplotlib Agg 后端，兼容 Docker 环境
• 新增 --save 参数，支持保存到文件
• 无需图形界面即可生成性能指标图表

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
🚀 快速开始 (三步启动)
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

# Terminal 1: 启动 EGO-Planner
roslaunch ego_planner simple_run.launch

# Terminal 2: 启动指标记录节点
rosrun ego_planner metrics_recorder_node

# Terminal 3: 启动可视化（Docker 推荐）
python3 scripts/visualize_metrics.py --save

# 查看结果
ls -lh /tmp/ego_planner_metrics/metrics_plot.png

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
📊 12 个性能指标
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

1. Success Rate        - 规划成功率（%）
2. Optimization Time   - 单次优化耗时（ms）
3. Iterations          - L-BFGS 迭代次数
4. Replan Frequency    - 重规划频率（Hz）
5. Smoothness          - 轨迹平滑度（jerk 积分）
6. Path Length         - 路径总长度（m）
7. Final Cost          - 最终代价函数值
8. Clearance           - 障碍物间隙（m）
9. Max Velocity        - 最大速度（m/s）
10. Max Acceleration   - 最大加速度（m/s²）
11. Statistics         - 实时统计摘要
12. Radar Chart        - 多维性能雷达图

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
📁 数据输出
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

可视化图表:  /tmp/ego_planner_metrics/metrics_plot.png
CSV 数据:     /tmp/ego_planner_metrics/planner_metrics_*.csv
ROS 话题:     /planner_metrics (Float64MultiArray)

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
📚 文档索引
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

QUICK_START_METRICS.md          快速入门指南
PERFORMANCE_METRICS_SYSTEM.md   完整系统文档
METRICS_INTEGRATION_GUIDE.md    技术实现细节
VISUALIZATION_GUIDE.md           可视化详细用法
VISUALIZATION_FIX_SUMMARY.md    最新修复摘要
COMPILATION_FIX.md               所有编译修复记录
test_visualization.sh            自动化测试脚本

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
�� 可视化选项
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

保存到文件 (Docker 推荐):
  python3 scripts/visualize_metrics.py --save

实时交互显示 (需要图形界面):
  python3 scripts/visualize_metrics.py

自定义输出路径:
  python3 scripts/visualize_metrics.py --save --output ~/my_plot.png

查看帮助:
  python3 scripts/visualize_metrics.py --help

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
🛠️ 故障排查
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

问题: "No module named 'tkinter'"
✓ 已修复，脚本现在使用 Agg 后端

问题: "Unable to register with master node"
→ 先启动 roslaunch ego_planner simple_run.launch

问题: 没有数据显示
→ 检查: rostopic echo /planner_metrics
→ 确认在 RViz 中设置了目标点 (2D Nav Goal)

运行自动测试:
  ./test_visualization.sh

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
�� 性能调优建议
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

根据指标反馈调整参数 (advanced_param.xml):

成功率低       → 增加 max_iteration_num
优化时间长     → 减少 max_iteration_num
轨迹不平滑     → 增加 lambda_smoothness
频繁碰撞风险   → 增加 lambda_distance 和 safe_distance
速度超限       → 减小 max_vel 或增加 lambda_fitness
重规划频繁     → 增加 replan_thresh

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
✨ 系统特性
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

✓ 独立节点设计 - 不修改原有 ego_planner_node
✓ 实时数据采集 - 订阅 ROS 话题自动收集数据
✓ 滑动窗口统计 - 最近 100 个数据点
✓ CSV 导出功能 - 方便后续分析
✓ 实时可视化 - 12 个子图 + 统计信息
✓ Docker 兼容 - 支持无图形界面环境
✓ 命令行友好 - 完整的参数支持

╔═══════════════════════════════════════════════════════════════╗
║  系统已完全准备就绪，开始您的性能优化之旅！                      ║
╚═══════════════════════════════════════════════════════════════╝

