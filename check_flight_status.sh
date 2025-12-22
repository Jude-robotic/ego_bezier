#!/bin/bash
# 检查metrics_recorder_node飞行状态的脚本

echo "=========================================="
echo "🔍 检查 metrics_recorder_node 状态"
echo "=========================================="
echo ""

# 1. 检查节点是否运行
echo "1️⃣  检查节点运行状态:"
if rosnode list 2>/dev/null | grep -q metrics_recorder; then
    NODE_NAME=$(rosnode list | grep metrics_recorder)
    echo "   ✅ 节点运行中: $NODE_NAME"
else
    echo "   ❌ 节点未运行!"
    echo ""
    echo "   启动方法:"
    echo "   rosrun ego_planner metrics_recorder_node"
    exit 1
fi

echo ""

# 2. 检查话题连接
echo "2️⃣  检查话题订阅:"
SUBS=$(rosnode info $NODE_NAME 2>/dev/null | grep -A 20 "Subscriptions:" | grep -E "odom_world|pos_cmd|goal")
if [ -n "$SUBS" ]; then
    echo "$SUBS" | while read line; do
        echo "   ✅ $line"
    done
else
    echo "   ⚠️  未找到订阅话题"
fi

echo ""

# 3. 检查参数
echo "3️⃣  检查飞行参数:"
TAKEOFF_H=$(rosparam get ${NODE_NAME}/takeoff_height_threshold 2>/dev/null || echo "0.3")
GOAL_DIST=$(rosparam get ${NODE_NAME}/goal_reach_threshold 2>/dev/null || echo "0.5")
VEL_THRES=$(rosparam get ${NODE_NAME}/velocity_threshold 2>/dev/null || echo "0.1")

echo "   起飞高度阈值: ${TAKEOFF_H} m"
echo "   到达目标阈值: ${GOAL_DIST} m"
echo "   速度阈值: ${VEL_THRES} m/s"

echo ""

# 4. 查看最近的飞行状态日志
echo "4️⃣  最近的飞行状态日志:"
echo ""

# 尝试从rosout获取最近的日志
rostopic echo /rosout -n 50 2>/dev/null | grep -E "Flight.*State|IDLE|TAKING_OFF|FLYING|ARRIVED" | tail -10 | sed 's/^/   /' || echo "   ⚠️  未找到飞行状态日志"

echo ""

echo "=========================================="
echo "💡 使用提示"
echo "=========================================="
echo ""
echo "实时监控飞行状态变化:"
echo "  rostopic echo /rosout | grep -i flight"
echo ""
echo "应该看到的状态转换:"
echo "  [Flight] IDLE → TAKING_OFF (起飞检测)"
echo "  [Flight] TAKING_OFF → FLYING (进入飞行)"
echo "  [Flight] FLYING → ARRIVED (到达目标)"
echo ""
echo "查看当前odom位置:"
echo "  rostopic echo /odom_world/pose/pose/position -n 1"
echo ""
echo "发送测试目标点:"
echo "  rostopic pub /move_base_simple/goal geometry_msgs/PoseStamped '{header: {frame_id: \"world\"}, pose: {position: {x: 1.0, y: 0.0, z: 1.0}}}' --once"
echo ""
echo "查看数据记录情况:"
echo "  ls -lh /tmp/ego_planner_metrics/"
echo "  tail -5 /tmp/ego_planner_metrics/metrics_*.csv"
echo ""
