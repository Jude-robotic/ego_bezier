#!/bin/bash

# 多机协同系统验证脚本
# 用于快速检查系统是否正常工作

echo "=========================================="
echo "  多机协同系统验证脚本"
echo "=========================================="
echo ""

# 颜色定义
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# 检查函数
check_node() {
    local node_name=$1
    if rosnode list 2>/dev/null | grep -q "$node_name"; then
        echo -e "${GREEN}✓${NC} 节点 $node_name 运行正常"
        return 0
    else
        echo -e "${RED}✗${NC} 节点 $node_name 未运行"
        return 1
    fi
}

check_topic() {
    local topic_name=$1
    if rostopic list 2>/dev/null | grep -q "$topic_name"; then
        echo -e "${GREEN}✓${NC} Topic $topic_name 存在"
        return 0
    else
        echo -e "${RED}✗${NC} Topic $topic_name 不存在"
        return 1
    fi
}

# 检查roscore
echo "1. 检查ROS核心..."
if pgrep -x "roscore" > /dev/null || pgrep -x "rosmaster" > /dev/null; then
    echo -e "${GREEN}✓${NC} ROS Master运行正常"
else
    echo -e "${RED}✗${NC} ROS Master未运行，请先启动系统"
    echo "   运行: roslaunch ego_planner swarm_run.launch"
    exit 1
fi
echo ""

sleep 2

# 检查关键节点
echo "2. 检查关键节点..."
all_nodes_ok=true

check_node "/swarm_traj_generator" || all_nodes_ok=false

if [ "$all_nodes_ok" = true ]; then
    echo -e "${GREEN}集群轨迹生成器运行正常${NC}"
fi
echo ""

# 检查Topic
echo "3. 检查轨迹Topic..."
check_topic "/uav_0/planning/trajectory"
check_topic "/uav_1/planning/trajectory"
echo ""

echo "=========================================="
echo "  验证完成"
echo "=========================================="
