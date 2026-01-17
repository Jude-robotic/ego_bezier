#!/bin/bash
# 测试局部坐标系统的脚本

echo "========================================="
echo "  局部坐标系统测试脚本"
echo "========================================="
echo ""

# 检查是否已编译
if [ ! -f "/home/jude/ego-planner-bezier/devel/lib/ego_planner/ego_planner_node" ]; then
    echo "错误: ego_planner_node 未找到"
    echo "请先编译项目: catkin_make"
    exit 1
fi

echo "✓ 编译检查通过"
echo ""

# 检查配置文件
CONFIG_FILE="/home/jude/ego-planner-bezier/src/planner/plan_manage/launch/advanced_param.xml"
if grep -q "local_frame/enable" "$CONFIG_FILE"; then
    echo "✓ 配置文件包含局部坐标系统参数"
else
    echo "✗ 配置文件缺少局部坐标系统参数"
    echo "  请检查 advanced_param.xml"
    exit 1
fi
echo ""

echo "当前配置:"
echo "-------------------"
grep "local_frame" "$CONFIG_FILE" | sed 's/^/  /'
echo ""

echo "按 Ctrl+C 停止测试"
sleep 2

cd /home/jude/ego-planner-bezier
source devel/setup.bash
roslaunch ego_planner simple_run.launch
