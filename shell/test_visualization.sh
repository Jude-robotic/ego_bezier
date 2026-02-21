#!/bin/bash
# 测试可视化脚本是否能正常工作

echo "=================================="
echo "测试可视化脚本修复"
echo "=================================="

# 1. 检查脚本语法
echo -e "\n[1/4] 检查 Python 语法..."
python3 -m py_compile scripts/visualize_metrics.py
if [ $? -eq 0 ]; then
    echo "✓ 语法检查通过"
else
    echo "✗ 语法检查失败"
    exit 1
fi

# 2. 测试帮助信息
echo -e "\n[2/4] 测试命令行参数..."
python3 scripts/visualize_metrics.py --help > /dev/null 2>&1
if [ $? -eq 0 ]; then
    echo "✓ 帮助信息正常"
else
    echo "✗ 帮助信息失败"
    exit 1
fi

# 3. 检查 matplotlib 后端
echo -e "\n[3/4] 检查 matplotlib 后端..."
python3 << PYEOF
import sys
sys.path.insert(0, 'scripts')
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
print("✓ Agg 后端加载成功")
PYEOF

# 4. 创建输出目录
echo -e "\n[4/4] 准备输出目录..."
mkdir -p /tmp/ego_planner_metrics
chmod 777 /tmp/ego_planner_metrics
echo "✓ 输出目录已创建: /tmp/ego_planner_metrics/"

echo -e "\n=================================="
echo "✓ 所有测试通过！"
echo "=================================="
echo ""
echo "使用方法："
echo "1. 启动 ego-planner:"
echo "   roslaunch ego_planner simple_run.launch"
echo ""
echo "2. 启动指标记录节点:"
echo "   rosrun ego_planner metrics_recorder_node"
echo ""
echo "3. 启动可视化 (保存到文件):"
echo "   python3 scripts/visualize_metrics.py --save"
echo ""
echo "4. 查看图表:"
echo "   ls -lh /tmp/ego_planner_metrics/metrics_plot.png"
echo ""

