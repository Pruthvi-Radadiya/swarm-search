# Swarm Search — Multi-Robot Search & Rescue (ROS 2)

Indoor multi-robot search simulation in Gazebo. Three TurtleBot3 Burgers run SLAM and Nav2 under separate namespaces and fuse their occupancy maps into one shared team map.

Earlier PyBullet / single-drone experiments live under `archive/` and are not part of the active stack.

## Stack

| Piece | Choice |
|-------|--------|
| OS | Ubuntu 22.04 (WSL2 tested) |
| Middleware | ROS 2 Humble |
| Sim | Gazebo Classic |
| Robots | TurtleBot3 Burger ×3 |
| Mapping | slam_toolbox (online async) |
| Navigation | Nav2 (RotationShim + DWB) |
| Shared map | Custom `map_merge_swarm` package |

## Current status

- [x] 3-robot spawn with namespaced SLAM, Nav2, and TF trees
- [x] Autonomous Nav2 goal navigation (single robot, then fleet)
- [x] Live occupancy-map fusion → `/shared_map` (Gazebo ground-truth alignment)
- [x] Frozen team map (`maps/house_shared`)
- [ ] Cross-robot navigation on the shared map
- [ ] Frontier-based coverage (planned)

## Repository layout

```
swarm-search/
├── launch/              # Gazebo spawn, SLAM, Nav2 wrappers
├── nav2_params/         # Per-robot Nav2 configs
├── slam_params/         # Per-robot slam_toolbox configs
├── worlds/              # Gazebo house world
├── maps/                # Saved team map (house_shared)
├── src/map_merge_swarm/ # Map merge + GT TF nodes
└── archive/             # Old PyBullet phase (not used)
```

## Prerequisites

- ROS 2 Humble desktop
- Gazebo Classic + `turtlebot3` / `turtlebot3_gazebo` / `turtlebot3_navigation2` / `turtlebot3_teleop`
- `slam_toolbox`, `nav2_bringup`, `ros-humble-gazebo-ros-pkgs`

```bash
export TURTLEBOT3_MODEL=burger
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/humble/share/turtlebot3_gazebo/models
```

## Build

```bash
cd ~/swarm-search
source /opt/ros/humble/setup.bash
export TURTLEBOT3_MODEL=burger
colcon build --packages-select map_merge_swarm
source install/setup.bash
```

Rebuild only after changing `src/map_merge_swarm/`.

## Run

Use a **separate terminal** for each block below. In every terminal:

```bash
source /opt/ros/humble/setup.bash
source ~/swarm-search/install/setup.bash
export TURTLEBOT3_MODEL=burger
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/humble/share/turtlebot3_gazebo/models
cd ~/swarm-search
```

### 1. Gazebo + three robots

```bash
mkdir -p ~/swarm-search/launch/tmp_sdf
ros2 launch ~/swarm-search/launch/multi_robot_spawn.launch.py use_sim_time:=true
```

Expect a short pause (~8 s) before robots appear. ALSA / sound errors on WSL are harmless.

### 2. SLAM (one terminal per robot)

```bash
ros2 launch ~/swarm-search/launch/slam_tb3.launch.py \
  robot_ns:=TB3_1 use_sim_time:=true \
  slam_params_file:=$HOME/swarm-search/slam_params/mapper_params_tb3_1.yaml
```

```bash
ros2 launch ~/swarm-search/launch/slam_tb3.launch.py \
  robot_ns:=TB3_2 use_sim_time:=true \
  slam_params_file:=$HOME/swarm-search/slam_params/mapper_params_tb3_2.yaml
```

```bash
ros2 launch ~/swarm-search/launch/slam_tb3.launch.py \
  robot_ns:=TB3_3 use_sim_time:=true \
  slam_params_file:=$HOME/swarm-search/slam_params/mapper_params_tb3_3.yaml
```

Quick check: `ros2 param get /TB3_1/slam_toolbox map_frame` → `TB3_1/map`.

### 3. Nav2 (after each robot’s map TF exists)

```bash
ros2 launch ~/swarm-search/launch/nav2_tb3.launch.py \
  robot_ns:=TB3_1 use_sim_time:=true \
  params_file:=$HOME/swarm-search/nav2_params/burger_tb3_1.yaml
```

```bash
ros2 launch ~/swarm-search/launch/nav2_tb3.launch.py \
  robot_ns:=TB3_2 use_sim_time:=true \
  params_file:=$HOME/swarm-search/nav2_params/burger_tb3_2.yaml
```

```bash
ros2 launch ~/swarm-search/launch/nav2_tb3.launch.py \
  robot_ns:=TB3_3 use_sim_time:=true \
  params_file:=$HOME/swarm-search/nav2_params/burger_tb3_3.yaml
```

### 4. Map merge

```bash
ros2 launch map_merge_swarm map_merge.launch.py
```

Publishes `/shared_map` in frame `shared_map` (GT TF alignment + merge node).

### 5. RViz

```bash
ros2 run rviz2 rviz2 --ros-args -p use_sim_time:=true
```

- Global Options → Fixed Frame: `shared_map`
- Add → Map → Topic: `/shared_map`

### Optional — teleop one robot

```bash
ros2 run turtlebot3_teleop teleop_keyboard --ros-args -r __ns:=/TB3_1
```

### Optional — reload the frozen team map

```bash
ros2 run nav2_map_server map_server --ros-args \
  -p yaml_filename:=$HOME/swarm-search/maps/house_shared.yaml \
  -p use_sim_time:=true \
  -p frame_id:=shared_map

ros2 lifecycle set /map_server configure
ros2 lifecycle set /map_server activate
```

## Notes

- Start Nav2 only after that robot’s `TB3_N/map` TF exists, or the global costmap will time out.
- Do not run an old standalone `static_map_tfs` launch together with `map_merge.launch.py` (duplicate TFs).
- Kill leftover Gazebo before relaunching: `pkill -9 gzserver; pkill -9 gzclient`

## Roadmap (short)

1. Prove robot 2 navigates on the shared / saved team map (AMCL + Nav2)
2. Frontier detection and greedy assignment for decentralized coverage
3. Limited-range communication for map sharing (comms on/off ablation)

## License

MIT
