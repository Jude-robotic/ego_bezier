#!/bin/bash

# 固定十字墙测试环境启动脚本
# Fixed Cross-Wall Test Environment Launch Script

echo "=========================================="
echo "  Ego-Planner 固定测试环境"
echo "  Fixed Cross-Wall Test Environment"
echo "=========================================="
echo ""
echo "环境特点 / Environment Features:"
echo "  - 地图大小 / Map Size: 20m × 20m × 3m"
echo "  - 十字墙壁 / Cross Walls: 4个门洞 / 4 door openings"
echo "  - 起点 / Start: (7.5, -7.5, 1.0) 右下 / Bottom-right"
echo "  - 航点1 / Waypoint1: (-7.5, -7.5, 1.0) 左下 / Bottom-left"
echo "  - 航点2 / Waypoint2: (-7.5, 7.5, 1.0) 左上 / Top-left"
echo ""
echo "=========================================="
echo ""

# 进入工作空间
cd ~/ego-planner-bezier

# Source环境
source devel/setup.bash

# 启动测试环境
echo "正在启动测试环境... / Launching test environment..."
roslaunch ego_planner first_testenv.launch

