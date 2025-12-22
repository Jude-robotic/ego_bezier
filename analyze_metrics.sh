#!/bin/bash
# 改进的metrics CSV数据分析脚本 - 支持空数据和有数据的情况

CSV_FILE="${1:-$(ls -t /tmp/ego_planner_metrics/metrics_*.csv 2>/dev/null | head -1)}"

if [ ! -f "$CSV_FILE" ]; then
    echo "❌ 错误: 找不到CSV文件: $CSV_FILE"
    echo ""
    echo "建议检查:"
    echo "1. metrics_recorder_node 是否在运行?"
    echo "   rosnode list | grep metrics"
    echo ""
    echo "2. 查看日志输出:"
    echo "   rostopic echo /rosout | grep -i metric"
    exit 1
fi

echo "=========================================="
echo "📊 分析文件: $CSV_FILE"
echo "=========================================="

# 统计总行数
TOTAL_LINES=$(wc -l < "$CSV_FILE")
TOTAL_RECORDS=$((TOTAL_LINES - 1))
echo "总记录数: $TOTAL_RECORDS"

if [ "$TOTAL_RECORDS" -eq 0 ]; then
    echo ""
    echo "⚠️  警告: CSV文件只有表头,没有记录任何数据!"
    echo ""
    echo "=========================================="
    echo "🔍 问题诊断"
    echo "=========================================="
    echo ""
    echo "可能原因:"
    echo "1. 无人机未起飞 (metrics_recorder只在飞行时记录)"
    echo "2. 未设置目标点 (需要先发送goal)"
    echo "3. 起飞高度不足 (默认需要>0.3m)"
    echo ""
    echo "=========================================="
    echo "✅ 解决方案"
    echo "=========================================="
    echo ""
    echo "检查飞行状态:"
    echo "  rostopic echo /rosout | grep -i flight"
    echo ""
    echo "应该看到的状态转换:"
    echo "  [Flight] State: IDLE -> TAKING_OFF"
    echo "  [Flight] State: TAKING_OFF -> FLYING"
    echo "  [Flight] State: FLYING -> ARRIVED"
    echo ""
    echo "如果没有看到状态转换,请检查:"
    echo ""
    echo "1. 确认ROS节点运行:"
    echo "   rosnode list"
    echo "   # 应该看到: /metrics_recorder_node"
    echo ""
    echo "2. 确认话题有数据:"
    echo "   rostopic hz /odom_world"
    echo "   rostopic hz /planning/pos_cmd"
    echo ""
    echo "3. 手动发送目标点:"
    echo "   rostopic pub /move_base_simple/goal geometry_msgs/PoseStamped '{header: {frame_id: \"world\"}, pose: {position: {x: 1.0, y: 0.0, z: 1.0}}}' --once"
    echo ""
    echo "4. 查看metrics_recorder日志:"
    echo "   rosnode info /metrics_recorder_node"
    echo "   rostopic echo /rosout | grep -E 'Flight|Metrics'"
    echo ""
    echo "5. 检查参数:"
    echo "   rosparam get /metrics_recorder_node/takeoff_height_threshold"
    echo "   rosparam get /metrics_recorder_node/goal_reach_threshold"
    echo ""
    exit 0
fi

echo ""
echo "=========================================="
echo "📈 飞行性能统计"
echo "=========================================="

# 计算时间跨度
TIME_STATS=$(awk -F',' 'NR==2 {start=$1} NR>1 {end=$1} END {printf "%.2f", end-start}' "$CSV_FILE")
SAMPLE_RATE=$(awk -v t=$TIME_STATS -v n=$TOTAL_RECORDS 'BEGIN {if(t>0) printf "%.2f", n/t; else print "N/A"}')
echo "记录时间跨度: ${TIME_STATS}秒 (总记录: $TOTAL_RECORDS, 采样率: ${SAMPLE_RATE} Hz)"

# 速度统计
echo ""
echo "🚀 速度分析:"
awk -F',' 'NR>1 && $7>0 {
    sum+=$7; count++; 
    if($7>max || max=="")max=$7; 
    if(min=="" || $7<min)min=$7
} 
END {
    if(count>0) {
        printf "  最大: %.2f m/s\n  最小: %.2f m/s\n  平均: %.2f m/s\n  样本数: %d\n", max, min, sum/count, count
    } else {
        print "  ❌ 无有效速度数据"
    }
}' "$CSV_FILE"

VEL_UNIQUE=$(awk -F',' 'NR>1 && $7>0 {print $7}' "$CSV_FILE" | sort -u | wc -l)
echo "  速度唯一值数量: $VEL_UNIQUE"
if [ "$VEL_UNIQUE" -lt 10 ] && [ "$VEL_UNIQUE" -gt 0 ]; then
    echo "  ⚠️  警告: 速度变化太少(只有${VEL_UNIQUE}个不同值)"
fi

# 加速度统计
echo ""
echo "⚡ 加速度分析:"
awk -F',' 'NR>1 && $8>0 {
    sum+=$8; count++; 
    if($8>max || max=="")max=$8; 
    if(min=="" || $8<min)min=$8
} 
END {
    if(count>0) {
        printf "  最大: %.2f m/s²\n  最小: %.2f m/s²\n  平均: %.2f m/s²\n  样本数: %d\n", max, min, sum/count, count
    } else {
        print "  ❌ 无有效加速度数据"
    }
}' "$CSV_FILE"

# 重规划频率分析
echo ""
echo "=========================================="
echo "🔄 重规划频率分析"
echo "=========================================="

awk -F',' 'NR>1 && $12>0 {
    sum+=$12; count++; 
    if($12>max || max=="")max=$12; 
    if(min=="" || $12<min)min=$12
} 
END {
    if(count>0) {
        avg=sum/count;
        printf "  最大: %.2f Hz\n  最小: %.2f Hz\n  平均: %.2f Hz\n  样本数: %d\n", max, min, avg, count
    } else {
        print "  ❌ 无重规划数据"
    }
}' "$CSV_FILE"

# 优化质量分析
echo ""
echo "=========================================="
echo "🎯 优化质量分析"
echo "=========================================="

awk -F',' 'NR>1 {
    opt+=$3; iter+=$4; smooth+=$5; count++;
} 
END {
    if(count>0) {
        printf "  平均优化时间: %.2f ms\n", opt/count;
        printf "  平均迭代次数: %.2f\n", iter/count;
        printf "  平均平滑度: %.4f\n", smooth/count;
    }
}' "$CSV_FILE"

echo ""
echo "=========================================="
echo "✅ 分析完成!"
echo "=========================================="
