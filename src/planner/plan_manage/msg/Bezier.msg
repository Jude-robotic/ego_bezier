# Piecewise Bezier Curve Trajectory Message
# For N cubic Bezier segments, there are 3N+1 control points

int32 order                           # Bezier curve order (3 for cubic)
int64 traj_id                         # Trajectory ID for tracking
time start_time                       # Trajectory start time

float64[] segment_durations           # Duration of each Bezier segment
geometry_msgs/Point[] pos_pts         # Position control points

float64[] yaw_pts                     # Yaw trajectory points (optional)
float64 yaw_dt                        # Yaw segment duration (optional)
