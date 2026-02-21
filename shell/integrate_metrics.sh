#!/bin/bash
# 集成性能指标到planner_manager

echo "正在集成性能指标系统..."

MANAGER_CPP="/home/jude/ego-planner-bezier/src/planner/plan_manage/src/planner_manager.cpp"

# 1. 添加头文件包含
if ! grep -q "#include <plan_manage/planner_metrics.h>" "$MANAGER_CPP"; then
    sed -i '/#include <plan_manage\/planner_manager.h>/a #include <plan_manage/planner_metrics.h>\n#include <chrono>' "$MANAGER_CPP"
    echo "✓ 添加了头文件"
fi

# 2. 在initPlanModules中初始化metrics_recorder_
if ! grep -q "metrics_recorder_" "$MANAGER_CPP"; then
    sed -i '/visualization_ = vis;/i\    \n    \/\/ Initialize metrics recorder\n    metrics_recorder_.reset(new PlannerMetricsRecorder(nh));\n    ROS_INFO("\\033[32m[Metrics] Performance monitoring enabled\\033[0m");' "$MANAGER_CPP"
    echo "✓ 添加了metrics初始化"
fi

# 3. 在reboundReplan函数开始处添加计时和记录
REBOUND_LINE=$(grep -n "bool EGOPlannerManager::reboundReplan" "$MANAGER_CPP" | head -1 | cut -d: -f1)
if [ -n "$REBOUND_LINE" ]; then
    # 找到函数体开始的{位置
    BRACE_LINE=$((REBOUND_LINE + 2))
    
    # 检查是否已经添加
    if ! sed -n "${BRACE_LINE},$((BRACE_LINE+5))p" "$MANAGER_CPP" | grep -q "auto plan_start"; then
        sed -i "${BRACE_LINE}a\  \/\/ Metrics: Record planning attempt\n  auto plan_start = std::chrono::high_resolution_clock::now();\n  bool plan_success = false;" "$MANAGER_CPP"
        echo "✓ 添加了规划计时开始"
    fi
fi

echo "完成! 请运行 catkin_make 重新编译"
