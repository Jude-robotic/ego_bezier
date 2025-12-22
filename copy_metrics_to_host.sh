#!/bin/bash
# 在 Docker 内执行，提供复制指令

echo "=================================="
echo "从 Docker 复制指标图表到主机"
echo "=================================="
echo ""

# 检查文件是否存在
if [ ! -f "/tmp/ego_planner_metrics/metrics_plot.png" ]; then
    echo "❌ 错误: 图表文件不存在"
    echo "请先运行: python3 scripts/visualize_metrics.py --save"
    exit 1
fi

# 显示文件信息
echo "✓ 图表文件已找到:"
ls -lh /tmp/ego_planner_metrics/metrics_plot.png
echo ""

# 获取容器 ID
CONTAINER_ID=$(cat /proc/self/cgroup | grep "docker" | sed 's/^.*\///' | tail -n1 | cut -c1-12)

if [ -z "$CONTAINER_ID" ]; then
    echo "⚠️  无法自动检测容器 ID"
    echo ""
    echo "请在主机上手动执行："
    echo "  1. 获取容器 ID: docker ps"
    echo "  2. 复制文件: docker cp <container_id>:/tmp/ego_planner_metrics/metrics_plot.png ./"
else
    echo "✓ 检测到容器 ID: $CONTAINER_ID"
    echo ""
    echo "请在主机上执行以下命令："
    echo ""
    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    echo "docker cp $CONTAINER_ID:/tmp/ego_planner_metrics/metrics_plot.png ./"
    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    echo ""
fi

echo ""
echo "所有可用文件:"
ls -lh /tmp/ego_planner_metrics/
echo ""
echo "复制所有文件:"
if [ -n "$CONTAINER_ID" ]; then
    echo "docker cp $CONTAINER_ID:/tmp/ego_planner_metrics/ ./ego_planner_metrics/"
else
    echo "docker cp <container_id>:/tmp/ego_planner_metrics/ ./ego_planner_metrics/"
fi

