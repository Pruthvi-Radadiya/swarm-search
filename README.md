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
- [x] Frozen team map (`maps/house_shared`) + R2 Nav on saved map (AMCL)
- [x] Frontier detection + greedy multi-robot explore (`explore_swarm`)
- [x] Coverage vs time metric (see below)
- [ ] Limited-range comms (skipped for now)
- [ ] Aerial / query detect (roadmap later)

## Repository layout

```
swarm-search/
├── launch/                 # Gazebo spawn, SLAM, Nav2 wrappers
├── nav2_params/            # Per-robot Nav2 configs (RPP on burger_tb3_*)
├── slam_params/            # Per-robot slam_toolbox configs
├── worlds/                 # Gazebo turtle / house world
├── maps/                   # Saved team maps (house_shared*)
├── analysis/               # Coverage logger + CSV / plots
├── src/map_merge_swarm/    # Map merge + GT TF nodes
├── src/explore_swarm/      # Frontiers + goal assigner + explore.launch.py
├── docs/                   # Startup, plans, engineering notes
└── archive/                # Old PyBullet phase (not used)
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

## Coverage vs time

While the swarm explores, `analysis/coverage_logger.py` samples `/shared_map` every 5 s into `analysis/coverage.csv`.

The shared grid is ~20×20 m; the TurtleBot3 turtle world is much smaller, so raw `known_frac` of the canvas stays low (~few %). For demos we normalize so **100% = known cells at the end of the run** (turtle filled):

![Coverage vs time](analysis/coverage_pct.png)

```bash
# After T0–T7 are up, start logger, then explore:
cd ~/swarm-search/analysis
python3 coverage_logger.py
# other terminal:
ros2 launch explore_swarm explore.launch.py
# Ctrl+C logger when done → plot (100% = final known cells):
python3 - <<'PY'
import csv
from pathlib import Path
import matplotlib.pyplot as plt
rows = list(csv.DictReader(Path("coverage.csv").open()))
known = lambda r: int(r["free"]) + int(r["occupied"])
k_end = known(rows[-1])
t0 = float(rows[0]["wall_time"])
t = [float(r["wall_time"]) - t0 for r in rows]
y = [100.0 * known(r) / k_end for r in rows]
plt.plot(t, y, marker="o")
plt.xlabel("time (s)"); plt.ylabel("coverage (%)")
plt.title("Turtle world coverage vs time")
plt.ylim(0, 105); plt.grid(True, alpha=0.3)
plt.tight_layout(); plt.savefig("coverage_pct.png", dpi=150)
print("saved coverage_pct.png")
PY
```

Full bring-up order: `docs/startup-after-break.md`.

## Notes

- Start Nav2 only after that robot’s `TB3_N/map` TF exists, or the global costmap will time out.
- Do not run an old standalone `static_map_tfs` launch together with `map_merge.launch.py` (duplicate TFs).
- Kill leftover Gazebo before relaunching: `pkill -9 gzserver; pkill -9 gzclient`

## Roadmap (short)

1. ~~Prove robot 2 navigates on the shared / saved team map (AMCL + Nav2)~~ **done** (Step 3)
2. ~~Frontier detection and greedy assignment for decentralized coverage~~ **done** (Step 5)
3. ~~Coverage vs time~~ **done** (`analysis/`)
4. Limited-range communication (comms on/off) — **skipped for now** (future)
5. Next big: aerial / query detect (roadmap Phase 3+)

**Study notes:** `docs/engineering-notes-map-explore.md` · **Bring-up:** `docs/startup-after-break.md` (§3 live, §6 Step 3)

## License

MIT
