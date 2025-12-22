#!/bin/bash
# 验证修复是否已正确应用

echo "🔍 验证飞行问题修复..."
echo ""

cd /home/jude/ego-planner-bezier

PASS=0
FAIL=0

# 检查1: thresh_replan
echo -n "检查 thresh_replan 参数... "
if grep -q 'thresh_replan" value="0.6"' src/planner/plan_manage/launch/advanced_param.xml; then
    echo "✅ 通过 (0.6)"
    ((PASS++))
else
    echo "❌ 失败 (应该是0.6)"
    ((FAIL++))
fi

# 检查2: max_vel
echo -n "检查 max_vel 参数... "
if grep -q 'max_vel" value="1.5"' src/planner/plan_manage/launch/simple_run.launch; then
    echo "✅ 通过 (1.5)"
    ((PASS++))
else
    echo "❌ 失败 (应该是1.5)"
    ((FAIL++))
fi

# 检查3: max_acc
echo -n "检查 max_acc 参数... "
if grep -q 'max_acc" value="2.0"' src/planner/plan_manage/launch/simple_run.launch; then
    echo "✅ 通过 (2.0)"
    ((PASS++))
else
    echo "❌ 失败 (应该是2.0)"
    ((FAIL++))
fi

# 检查4: 分析脚本
echo -n "检查改进的分析脚本... "
if [ -f /tmp/improved_simple_analyze.sh ] && [ -x /tmp/improved_simple_analyze.sh ]; then
    echo "✅ 通过"
    ((PASS++))
else
    echo "❌ 失败"
    ((FAIL++))
fi

# 检查5: 增强版记录器
echo -n "检查增强版metrics记录器... "
if [ -f /tmp/metrics_recorder_enhanced.cpp ]; then
    echo "✅ 通过"
    ((PASS++))
else
    echo "❌ 失败"
    ((FAIL++))
fi

# 检查6: 文档
echo -n "检查修复文档... "
if [ -f START_HERE.md ] && [ -f FLIGHT_ISSUES_SUMMARY.md ]; then
    echo "✅ 通过"
    ((PASS++))
else
    echo "❌ 失败"
    ((FAIL++))
fi

echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "验证结果: $PASS 通过, $FAIL 失败"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━"

if [ $FAIL -eq 0 ]; then
    echo "✅ 所有检查通过! 可以进行测试了"
    echo ""
    echo "下一步:"
    echo "  1. catkin_make"
    echo "  2. roslaunch ego_planner simple_run.launch"
    echo "  3. /tmp/improved_simple_analyze.sh"
    exit 0
else
    echo "❌ 有 $FAIL 项检查失败,请查看上面的详情"
    exit 1
fi
