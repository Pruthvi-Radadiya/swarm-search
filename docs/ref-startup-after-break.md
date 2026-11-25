> **📍 Utility** · [← Roadmap](swarm_search_rescue_roadmap_v4.md) · Day-to-day startup cheat sheet

# Resume after a short break — startup

Use this when you come back tomorrow (or after `wsl --shutdown`).  
Full detail / bugs / Steps 1–5 archive notes: `P2-runbook.md`. Plan: `swarm_search_rescue_roadmap_v4.md`.  
**Current work:** Step 5 + coverage-vs-time **done** (`analysis/`). Comms **skipped for now**. Next big: roadmap Phase 3 (aerial) when ready. **Ideas:** `P2-ref-engineering-notes.md`. **Live:** §3 (T0–T9). **Step 3:** §6.

---

## 0. Quick Start: Phase 3 Unified Air-Ground Simulation (Modern Gazebo)

When resuming to test or demonstrate the full **heterogeneous 4-robot swarm** (3 Anti-Tip Mini-Rovers + 1 PX4 Aerial Drone in the House):

### Terminal 1: Launch Modern Gazebo + 3 Rovers + Drone + Bridges
```bash
cd ~/swarm-search
source /opt/ros/humble/setup.bash
source install/setup.bash
export LIBGL_ALWAYS_SOFTWARE=1
ros2 launch launch/unified_swarm_house.launch.py
```

### Terminal 2: Start PX4 DDS Bridge
```bash
MicroXRCEAgent udp4 -p 8888
```

### Terminal 3: Autonomous Aerial Scout (Overhead Recon)
```bash
cd ~/swarm-search
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run explore_swarm aerial_scout_node
```

### Terminal 4: Goal Assigner (Candidate Pin Priority + Frontiers)
```bash
cd ~/swarm-search
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run explore_swarm goal_assigner
```

---

## 0.1 Quick Start: Standalone PX4 Flight & GTA-5 Teleop (Stage 2B)

If you only want to test the standalone quadrotor with keyboard flight:
```bash
# Terminal 1:
MicroXRCEAgent udp4 -p 8888

# Terminal 2:
cd ~/PX4-Autopilot
export LIBGL_ALWAYS_SOFTWARE=1
PX4_GZ_MODEL=x500_downward_cam make px4_sitl gz_x500

# Terminal 3:
cd ~/swarm-search
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run explore_swarm px4_teleop
```
* **Flight Controls**: Click on HUD window, press **`T`** to takeoff, hold **`8` + `4`** for diagonal flight, **`W` + `8`** to climb and pitch forward, release keys to auto-brake and hover, and press **`L`** to land!

---

## 0.1 Environment Notice (NumPy Version Requirement)
To prevent C-API mismatches with Ubuntu's system SciPy (`ImportError: numpy.core.multiarray failed to import`), always ensure NumPy is locked to `< 2.0`:
```bash
pip3 install "numpy<2"
```

---

## 0.1 After WSL restart (if GUI is broken)

Gazebo/RViz icon on taskbar but no window → PowerShell:

```powershell
wsl --shutdown
wsl --update
```

Reopen Ubuntu, then continue below.

---



## 1. Every new terminal — run this first

```bash
source /opt/ros/humble/setup.bash
source ~/swarm-search/install/setup.bash
export TURTLEBOT3_MODEL=burger
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/humble/share/turtlebot3_gazebo/models
cd ~/swarm-search
```

(`install/setup.bash` is needed for `map_merge_swarm`. Rebuild only if you changed that package.)  
`GAZEBO_MODEL_PATH` is required so the custom world can find `model://turtlebot3_world`.

---



## 2. Kill old Gazebo (if leftover from last session)

`pkill` accepts **one** pattern only:

```bash
pkill -9 gzserver || true
pkill -9 gzclient || true
sleep 2
```

Or: `killall -9 gzserver gzclient 2>/dev/null || true`

---



## 3. Terminals — start in this order

Keep each launch in its **own** terminal.

### T0 — Gazebo + 3 robots

```bash
mkdir -p ~/swarm-search/launch/tmp_sdf
export TURTLEBOT3_MODEL=burger
ros2 launch ~/swarm-search/launch/multi_robot_spawn.launch.py use_sim_time:=true
```

**Expect a pause:** launch waits **~8 s** after Gazebo starts, then spawns robots. Empty arena for a few seconds is normal.

**Ignore ALSA / sound errors** on WSL (`cannot find card '0'`, `Unknown PCM default`) — no audio device; harmless.

World file: `~/swarm-search/worlds/turtlebot3_world.world` (stock world + `gazebo_ros_state` plugin for `/gazebo/model_states`).

When healthy you should see logs like:

- `[gazebo.gazebo_ros_state]: Publishing states ... [/gazebo/model_states]`
- later: successful spawn for `burger_1` / `burger_2` / `burger_3` (not `GazeboRosFactory?`)

Quick checks (another terminal, after sourcing §1):

```bash
# service must exist before spawn succeeds
ros2 service list | grep spawn_entity

# GT poses for Option C (BEST_EFFORT required)
ros2 topic echo /gazebo/model_states --qos-reliability best_effort --once
# name: list should include burger_1, burger_2, burger_3

ros2 run tf2_ros tf2_echo TB3_1/odom TB3_1/base_footprint
```

**If spawn dies with** `Service /spawn_entity unavailable. Was Gazebo started with GazeboRosFactory?`**:** Gazebo was not ready yet (race). The launch file already delays 8 s — kill Gazebo (§2), wait longer if needed, relaunch. Do **not** remove the factory; it is loaded by `gzserver.launch.py`.

### T1 — SLAM TB3_1

```bash
ros2 launch ~/swarm-search/launch/slam_tb3.launch.py \
  robot_ns:=TB3_1 use_sim_time:=true \
  slam_params_file:=$HOME/swarm-search/slam_params/mapper_params_tb3_1.yaml
```



### T2 — SLAM TB3_2

```bash
ros2 launch ~/swarm-search/launch/slam_tb3.launch.py \
  robot_ns:=TB3_2 use_sim_time:=true \
  slam_params_file:=$HOME/swarm-search/slam_params/mapper_params_tb3_2.yaml
```



### T3 — SLAM TB3_3

```bash
ros2 launch ~/swarm-search/launch/slam_tb3.launch.py \
  robot_ns:=TB3_3 use_sim_time:=true \
  slam_params_file:=$HOME/swarm-search/slam_params/mapper_params_tb3_3.yaml
```

Optional check: `ros2 param get /TB3_1/slam_toolbox map_frame` → should be `TB3_1/map`.

### T4–T6 — Nav2 (needed for auto-explore; optional if only mapping / merge)

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

Same for `TB3_2` / `burger_tb3_2.yaml` and `TB3_3` / `burger_tb3_3.yaml`.

### T7 — Map merge (`/shared_map`)

```bash
source /opt/ros/humble/setup.bash
source ~/swarm-search/install/setup.bash
ros2 launch map_merge_swarm map_merge.launch.py
```

Do **not** also run old `static_map_tfs` or a separate `merge_node`.  
(When Option C Parts 5–6 are done, this launch will use GT TFs instead of static spawn TFs.)

### T8 — RViz

```bash
ros2 run rviz2 rviz2 --ros-args -p use_sim_time:=true
```

- **Global Options** → Fixed Frame: `shared_map`
- Add → **Map** → Topic: `/shared_map` (one display only)
- Optional: PoseArray `/frontiers` or MarkerArray `/frontiers_markers`



### T9 — Explore (`explore.launch.py` — Part 7)

Needs T7 (`/shared_map`) and T4–T6 (Nav2) for useful assign. Detector alone is OK for markers.

Rebuild only after code / `setup.py` / launch / `package.xml` changes:

```bash
cd ~/swarm-search
colcon build --packages-select explore_swarm
source install/setup.bash
```

**Preferred (both nodes):**

```bash
ros2 launch explore_swarm explore.launch.py
```

Manual equivalent (two terminals):

```bash
ros2 run explore_swarm frontier_detector
ros2 run explore_swarm goal_assigner
```

Health:

```bash
ros2 action list | grep navigate_to_pose   # need /TB3_1, /TB3_2, /TB3_3
ros2 topic echo /frontiers --once
ros2 node list | grep -E 'frontier_detector|goal_assigner'
```

| Log / symptom | Meaning |
|---------------|---------|
| `clusters_kept = 0` | Detector running; no free↔unknown edge yet (empty/all-known map, or stack not exploring) |
| `TB3_N: Nav2 not ready` | That Nav2 (T4–T6) not up |
| Both nodes on screen | Launch OK |



### Optional — teleop one robot

```bash
ros2 run turtlebot3_teleop teleop_keyboard --ros-args -r __ns:=/TB3_1
```



### Optional — GT TF node (while implementing Option C)

After every edit to `gt_map_tf_node.py`:

```bash
cd ~/swarm-search
colcon build --packages-select map_merge_swarm
source install/setup.bash
ros2 run map_merge_swarm gt_map_tf_node
```

Needs T0 running with `model_states` publishing. Details: `P2-S1-plan-gt-shared-map.md`.

---



## 4. “Am I up?” (30 seconds)

```bash
ros2 topic list | grep -E 'TB3_.*/map|/shared_map|/gazebo/model_states'
ros2 run tf2_ros tf2_echo shared_map TB3_1/map
# v1 (spawn offsets): translation ≈ (-2.0, -0.5, 0.0)
# after Option C: should track live Gazebo pose instead of fixed spawn
```

---



## 5. Stopping for the day

- Ctrl+C each launch terminal (or close them)
- Optional: PowerShell `wsl --shutdown` to free RAM / reset WSLg

Nothing special to “save” — sim state is not persisted; next time you spawn fresh.  
Code/docs in `~/swarm-search` persist.

---



## 6. Step 3 retest — frozen map + AMCL + Nav2 (TB3_2)

**Different stack from §3.** Do **not** run SLAM (T1–T3), live merge (T7), or explore (T9) together with this. Full detail: `P2-S3-plan-r2-nav-on-saved-map.md`.

Env (§1) in every terminal. Kill old Gazebo (§2) if needed.

**Terminal A — Gazebo only**

```bash
mkdir -p ~/swarm-search/launch/tmp_sdf
export TURTLEBOT3_MODEL=burger
ros2 launch ~/swarm-search/launch/multi_robot_spawn.launch.py use_sim_time:=true
```

**Terminal B — map_server** (`house_shared`)

```bash
ros2 run nav2_map_server map_server --ros-args \
  -p yaml_filename:=$HOME/swarm-search/maps/house_shared.yaml \
  -p use_sim_time:=true \
  -p frame_id:=shared_map

# then (same or new terminal):
ros2 lifecycle set /map_server configure
ros2 lifecycle set /map_server activate
```

**Terminal C — AMCL + Nav2 TB3_2** (localization params / launch)

```bash
ros2 launch ~/swarm-search/launch/nav2_tb3_localization.launch.py
```

**Terminal D — RViz**

```bash
ros2 run rviz2 rviz2 --ros-args -p use_sim_time:=true
```

| RViz | Value |
|------|--------|
| Fixed Frame | `shared_map` |
| Map topic | `/map` (Transient Local) |
| LaserScan | `/TB3_2/scan` |

Then: set **2D Pose Estimate** on TB3_2 → send **Nav2 Goal** on free space.  
Params file: `nav2_params/burger_tb3_2_loc.yaml` (`global_frame: shared_map`).

---



## Where you left off

- Roadmap **v4 Phase 2**: 3 robots + Nav2 + shared map done
- **Step 1 done — Option C:** one house on `/shared_map` (`P2-S1-plan-gt-shared-map.md`)
- **Step 2 done — freeze team map:** `maps/house_shared.yaml` + `.pgm` + `maps/README.md`
- **Step 3 done:** R2 AMCL + Nav2 on `house_shared` — retest cheat sheet → **§6** above
- **Step 4 done:** live `/shared_map` + mid-session save `maps/house_shared_live.*`
- **Step 5 done:** frontiers + greedy multi-robot + **`ros2 launch explore_swarm explore.launch.py`** (T9)
- **Comms radius:** **skipped for now** — may implement later (not blocking)
- **Coverage vs time done:** `analysis/coverage_logger.py` → CSV → `coverage_pct.png` (100% = turtle filled at end of run; see root README)
- **Next (when you want):** roadmap **Phase 3 aerial**
- **Engineering notes:** `docs/P2-ref-engineering-notes.md`
- **Pitfall:** Gazebo models must be `burger_*` (not `waffle_*`) or GT never publishes `shared_map` TF
- **Pitfall:** DWB circles/tips → RPP in Nav2 params; tip on pillar → ghost walls in merge
- **Pitfall:** start Nav2 for **every** robot the assigner uses, or you get `Nav2 not ready`
- **Pitfall:** `clusters_kept = 0` with empty map is normal until free↔unknown edges exist
- **Pitfall:** no robot–robot avoidance in v1 — paths can cross
- **Pitfall:** never mix Step 3 (§6) with live SLAM+merge (§3 T1–T3/T7) on the same robot
