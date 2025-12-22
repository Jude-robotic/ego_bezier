#!/bin/bash
# 飞行评价与可视化脚本

PYTHON="/home/jude/anaconda3/bin/python3"

echo "=================================================="
echo "  Ego-Planner 飞行性能评价系统"
echo "=================================================="
echo ""

# 检查是否有metrics数据
if [ ! -d "/tmp/ego_planner_metrics" ] || [ -z "$(ls -A /tmp/ego_planner_metrics/metrics_*.csv 2>/dev/null)" ]; then
    echo "错误: 未找到飞行数据"
    echo "请先运行: roslaunch ego_planner simple_run.launch"
    exit 1
fi

echo "步骤 1/2: 评价飞行性能..."
echo "--------------------------------------------------"
$PYTHON evaluate_flight.py
if [ $? -ne 0 ]; then
    echo "评价失败"
    exit 1
fi

echo ""
echo "步骤 2/2: 生成可视化报告..."
echo "--------------------------------------------------"
$PYTHON visualize_flight.py
if [ $? -ne 0 ]; then
    echo "可视化失败"
    exit 1
fi

echo ""
echo "=================================================="
echo "  ✓ 完成！"
echo "=================================================="
echo ""
echo "生成的文件:"
echo "  - /tmp/ego_planner_metrics/evaluation_result.txt (文本评价)"
echo "  - /tmp/ego_planner_metrics/flight_analysis.png   (性能图表)"
echo "  - /tmp/ego_planner_metrics/flight_report.html    (HTML报告)"
echo ""
echo "查看方式:"
echo "  1. 在Docker内启动HTTP服务器:"
echo "     cd /tmp/ego_planner_metrics && python3 -m http.server 8000"
echo "     然后在宿主机浏览器访问: http://localhost:8000/flight_report.html"
echo ""
echo "  2. 复制文件到宿主机 (在宿主机终端运行):"
echo "     docker ps  # 查看容器ID"
echo "     docker cp <容器ID>:/tmp/ego_planner_metrics/ ~/ego_metrics/"
echo "     然后用浏览器打开 ~/ego_metrics/flight_report.html"
echo ""
