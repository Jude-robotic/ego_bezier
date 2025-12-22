//开始飞
roslaunch ego_planner simple_run.launch
//启动数据可视化节点
rosrun ego_planner metrics_recorder_node
//图像（无法及时更新）
python3 scripts/visualize_metrics.py --save
//查看docker内的图片
eog /tmp/ego_planner_metrics/metrics_plot.png
//查看分析脚本
/tmp/simple_analyze.sh
/tmp/improved_simple_analyze.sh
//分析脚本
./analyze_flight.sh
//配合分析脚本的网页可视化
cd /tmp/ego_planner_metrics && python3 -m http.server 8000

//对比代码
rosbag record /planning/pos_cmd /visual_slam/odom -O bezier_ego.bag
rosbag record /planning/pos_cmd /visual_slam/odom -O original_ego.bag
./compare_planners.py original_ego.bag bezier_ego.bag

## 参考文档
- 改进指南: `METRICS_IMPROVEMENT_GUIDE.md`
- 性能指标系统: `PERFORMANCE_METRICS_SYSTEM.md`
- 可视化指南: `VISUALIZATION_GUIDE.md`
- 更新了 Docker 环境的启动步骤：`QUICK_START_METRICS.md` 
- 快速修复: `QUICK_FIX.md`
- 分析总结: `ANALYSIS_SUMMARY.md`
- 使用文档: `Jude_help_doc.md`
- 文档: `COMPILATION_SUCCESS.md`