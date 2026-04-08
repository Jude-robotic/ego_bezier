#!/usr/bin/env python3
import sys

PATH = '/home/jude/ego-planner-bezier/src/planner/plan_manage/src/swarm_master_coordinator.cpp'

with open(PATH, 'r') as f:
    lines = f.readlines()

# 找到 "last_print_time = now;" 行（障碍物状态 if 块的最后一句）
target_idx = None
for i, line in enumerate(lines):
    if 'last_print_time = now;' in line:
        target_idx = i
        break

if target_idx is None:
    print("ERROR: target line not found")
    sys.exit(1)

# 确认后两行是 "    }" 和 "  }"
print(f"target_idx={target_idx}")
print(f"  [{target_idx}]: {lines[target_idx].rstrip()}")
print(f"  [{target_idx+1}]: {lines[target_idx+1].rstrip()}")
print(f"  [{target_idx+2}]: {lines[target_idx+2].rstrip()}")

# 在 "    }" 和 "  }" 之间（即 target_idx+1 之后）插入 Marker 代码
insert_after = target_idx + 1  # 即 if 块的 "    }" 行

marker_code = [
    '\n',
    '    // ── RViz 障碍物状态 TEXT Marker（跟随主机位置实时更新）──────────\n',
    '    {\n',
    '      visualization_msgs::Marker mk;\n',
    '      mk.header.frame_id = "world";\n',
    '      mk.header.stamp    = ros::Time::now();\n',
    '      mk.ns              = "obs_state";\n',
    '      mk.id              = 9999;\n',
    '      mk.type            = visualization_msgs::Marker::TEXT_VIEW_FACING;\n',
    '      mk.action          = visualization_msgs::Marker::ADD;\n',
    '      mk.scale.z         = 0.50;\n',
    '      mk.pose.orientation.w = 1.0;\n',
    '\n',
    '      // 位置：主机轨迹第一个控制点正上方\n',
    '      const Eigen::Vector3d leader_pt = leader_ctrl.col(0);\n',
    '      mk.pose.position.x = leader_pt.x();\n',
    '      mk.pose.position.y = leader_pt.y();\n',
    '      mk.pose.position.z = leader_pt.z() + 1.3;\n',
    '\n',
    '      // 颜色随状态变化：绿=None，红=Narrow，紫=Circle，黄=door\n',
    '      if (state == "Narrow") {\n',
    '        mk.color.r = 1.0f; mk.color.g = 0.2f; mk.color.b = 0.2f;\n',
    '      } else if (state == "Circle") {\n',
    '        mk.color.r = 0.8f; mk.color.g = 0.2f; mk.color.b = 1.0f;\n',
    '      } else if (state == "door") {\n',
    '        mk.color.r = 1.0f; mk.color.g = 0.85f; mk.color.b = 0.1f;\n',
    '      } else {\n',
    '        mk.color.r = 0.2f; mk.color.g = 1.0f; mk.color.b = 0.2f;\n',
    '      }\n',
    '      mk.color.a  = 1.0f;\n',
    '      mk.text     = state;\n',
    '      mk.lifetime = ros::Duration(0.6);  // 超时自动消失\n',
    '\n',
    '      obs_state_marker_pub_.publish(mk);\n',
    '    }\n',
]

new_lines = lines[:insert_after+1] + marker_code + lines[insert_after+1:]

with open(PATH, 'w') as f:
    f.writelines(new_lines)

print(f"Inserted {len(marker_code)} lines after line {insert_after+1}")
