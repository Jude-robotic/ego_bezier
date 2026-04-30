# EGO-Planner-Bezier

EGO-Planner-Bezier is derived from EGO-Planner by ZJU FAST-LAB. It replaces the original B-spline trajectory representation with piecewise cubic Bezier curves, and adds a leader-follower swarm architecture for multi-UAV flight.

This repository keeps the original EGO-Planner structure and extends it with:
- Bezier trajectory optimization
- Swarm coordination and guidance routing
- Optional local-frame planning support

## Quick Start within 3 Minutes

### Requirements
- Ubuntu with ROS (catkin)
- Eigen3, PCL
- libarmadillo-dev (required by the simulator)

### Build
```bash
sudo apt-get install libarmadillo-dev
# Replace with your GitHub URL
git clone https://github.com/<your-org>/ego-planner-bezier.git
cd ego-planner-bezier
catkin_make
source devel/setup.bash
```

### Run (single UAV, simulation)
```bash
roslaunch ego_planner simple_run.launch
```

### Run (swarm, leader + followers)
```bash
roslaunch ego_planner swarm_multi_sim.launch
```

### Run (swarm, empty corridor scene)
```bash
roslaunch ego_planner swarm_nonehall.launch
```

## Project Architecture

### System pipeline (swarm)
```
Leader planner (ego_planner_node)
  publishes /planning/bezier
        |
        v
Swarm master coordinator (swarm_master_node)
  publishes /swarm/master/agent_{id}/guidance_bezier
        |
        v
Guidance mux (swarm_guidance_mux_node)
  routes to /swarm/agent_{id}/guidance_bezier
        |
        v
Follower namespace (uav1, uav2, ...)
  swarm_follower_guidance_optimizer_node -> uavX/planning/bezier
  traj_server -> uavX/planning/pos_cmd
  so3_control -> motors
```

### Core packages
- Planner core: [src/planner/plan_manage](src/planner/plan_manage), [src/planner/plan_env](src/planner/plan_env), [src/planner/path_searching](src/planner/path_searching), [src/planner/traj_utils](src/planner/traj_utils)
- Bezier optimizer: [src/planner/bezier_opt](src/planner/bezier_opt)
- Swarm modules: [src/planner/plan_manage/src](src/planner/plan_manage/src)
- Simulation and sensing: [src/uav_simulator](src/uav_simulator)
- Utilities and messages: [src/utils](src/utils)

### Key launch files
- Single UAV: [src/planner/plan_manage/launch/simple_run.launch](src/planner/plan_manage/launch/simple_run.launch)
- Swarm simulation: [src/planner/plan_manage/launch/swarm_multi_sim.launch](src/planner/plan_manage/launch/swarm_multi_sim.launch)
- Swarm empty corridor: [src/planner/plan_manage/launch/swarm_nonehall.launch](src/planner/plan_manage/launch/swarm_nonehall.launch)

## Documentation

- Start here: [markdown/START_HERE.md](markdown/START_HERE.md)
- Swarm overview: [markdown/SWARM_PROJECT_OVERVIEW.md](markdown/SWARM_PROJECT_OVERVIEW.md)
- Swarm quick start: [markdown/SWARM_QUICK_START.md](markdown/SWARM_QUICK_START.md)
- Swarm system details: [markdown/SWARM_SYSTEM_README.md](markdown/SWARM_SYSTEM_README.md)
- Bezier formulation: [markdown/BEZIER_OPTIMIZATION_FORMULATION.md](markdown/BEZIER_OPTIMIZATION_FORMULATION.md)
- Local-frame planning: [markdown/README_LOCAL_FRAME.md](markdown/README_LOCAL_FRAME.md)

## Notes on lineage

- This repository is an improvement over EGO-Planner (ZJU FAST-LAB): Bezier splines replace B-splines, and swarm coordination is integrated on top of the original planner stack.
- For the upstream project and original references, see https://github.com/ZJU-FAST-Lab/ego-planner

## Acknowledgements

- EGO-Planner by ZJU FAST-LAB, which provides the base planner architecture and simulation stack.
