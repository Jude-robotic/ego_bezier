#!/bin/bash
# Test script for fixed cross-wall environment

echo "=========================================="
echo "Testing Fixed Cross-Wall Environment"
echo "=========================================="
echo ""
echo "Map configuration:"
echo "  - Type: fixed_cross (4 walls + 4 doors)"
echo "  - Size: 20x20x3 meters"
echo "  - Map generator: random_forest"
echo "  - Mockamap: DISABLED"
echo ""
echo "Flight path:"
echo "  Start:  ( 7.5, -7.5, 1.0) - bottom-right"
echo "  Goal 1: (-7.5, -7.5, 1.0) - bottom-left"
echo "  Goal 2: (-7.5,  7.5, 1.0) - top-left"
echo ""
echo "=========================================="
echo ""

cd /home/jude/ego-planner-bezier
source devel/setup.bash
roslaunch ego_planner first_testenv.launch
