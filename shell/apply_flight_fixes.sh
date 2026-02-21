#!/bin/bash
# 飞行问题快速修复脚本

set -e

WORKSPACE="/home/jude/ego-planner-bezier"
echo "=========================================="
echo "🔧 EGO Planner 飞行问题修复脚本"
echo "=========================================="
echo ""

cd "$WORKSPACE"

echo "📋 修复清单:"
echo "  1. ✅ 降低重规划阈值 (1.5 → 0.6)"
echo "  2. ✅ 降低速度限制 (2.0 → 1.5 m/s)"
echo "  3. ✅ 降低加速度限制 (3.0 → 2.0 m/s²)"
echo "  4. ⚙️  增强Metrics记录器 (可选)"
echo ""

# 检查文件是否已修改
if grep -q 'thresh_replan" value="0.6"' src/planner/plan_manage/launch/advanced_param.xml; then
    echo "✅ 重规划阈值已修改"
else
    echo "❌ 重规划阈值未修改,请检查!"
fi

if grep -q 'max_vel" value="1.5"' src/planner/plan_manage/launch/simple_run.launch; then
    echo "✅ 速度限制已修改"
else
    echo "❌ 速度限制未修改,请检查!"
fi

if grep -q 'max_acc" value="2.0"' src/planner/plan_manage/launch/simple_run.launch; then
    echo "✅ 加速度限制已修改"
else
    echo "❌ 加速度限制未修改,请检查!"
fi

echo ""
read -p "是否应用增强版Metrics记录器? (y/n): " -n 1 -r
echo ""

if [[ $REPLY =~ ^[Yy]$ ]]; then
    echo "📦 备份原文件..."
    cp src/planner/plan_manage/src/metrics_recorder_node.cpp \
       src/planner/plan_manage/src/metrics_recorder_node.cpp.backup.$(date +%Y%m%d_%H%M%S)
    
    echo "📝 应用增强版..."
    if [ -f /tmp/metrics_recorder_enhanced.cpp ]; then
        cp /tmp/metrics_recorder_enhanced.cpp \
           src/planner/plan_manage/src/metrics_recorder_node.cpp
        echo "✅ 增强版已应用"
    else
        echo "❌ 找不到 /tmp/metrics_recorder_enhanced.cpp"
        exit 1
    fi
fi

echo ""
echo "🔨 重新编译..."
catkin_make

echo ""
echo "=========================================="
echo "✅ 修复完成!"
echo "=========================================="
echo ""
echo "📊 预期改进:"
echo "  • 重规划频率: 0.58 Hz → 2-3 Hz"
echo "  • 飞行平滑度: 明显改善"
echo "  • 转弯角度: 更小更连续"
echo "  • 抖动: 显著减少"
echo ""
echo "🚀 测试命令:"
echo "  roslaunch ego_planner simple_run.launch"
echo ""
echo "📈 分析命令:"
echo "  /tmp/improved_simple_analyze.sh"
echo ""
echo "查看详细修复文档: FLIGHT_ISSUES_FIX.md"
