> **📍 Phase 2** · [← Roadmap](swarm_search_rescue_roadmap_v4.md) · Status: ✅ Core done

# Phase 2 Runbook — Multi-Robot Search & Rescue Sim

Three TurtleBot3 Burgers in Gazebo, each with its own SLAM map and Nav2 stack under namespaces `TB3_1` / `TB3_2` / `TB3_3`. Builds on the single-robot Nav2 + SLAM stack from `P1-runbook.md` (same OS, ROS2 Humble, Gazebo Classic). This file is the recreate guide: stack, files that matter, every bug hit so far with symptom → cause → fix, and the exact launch order that works.

**Day-to-day bring-up (preferred):** `docs/ref-startup-after-break.md` (§3 live explore, §6 Step 3 frozen Nav).  
**Algorithms / design:** `docs/P2-ref-engineering-notes.md`.  
**Roadmap:** `docs/swarm_search_rescue_roadmap_v4.md` · checklist: `docs/P2-checklist.md`.

| | |
|---|---|
| **Last verified** | 2026-10-03 (WSL2 Ubuntu 22.04, ROS2 Humble) |
| **Phase 2 core** | **Done** — shared map (GT Option C) + freeze + R2 on saved map + live share + multi-robot frontier explore + `explore.launch.py` |
| **Still open in Phase 2** | Coverage vs time (optional); **comms radius skipped for now** (future) |
| **Map merge note** | v1 was spawn-offset TFs (Sep 2026). **Now:** GT `shared_map→TB3_N/map` via `gt_map_tf_node` + fuse (`P2-S1-plan-gt-shared-map.md`) |

---

## Stack (inherited from Phase 1, unchanged)

OS, ROS2 Humble, Gazebo Classic 11, the WSL2 GPU rendering fix, and the WSL2 memory allocation config all carry over as-is. See `P1-runbook.md` for install steps — not repeated here.

Phase 2 differences from Phase 1:

| Item | Phase 1 | Phase 2 |
|---|---|---|
| Robot model | Waffle (typical) | **Burger** ×3 |
| Env var | `TURTLEBOT3_MODEL=waffle` | `TURTLEBOT3_MODEL=burger` |
| Namespaces | none / single | `TB3_1`, `TB3_2`, `TB3_3` |
| Map frame | `map` | **`TB3_N/map` per robot** |
| Odom / base | `odom`, `base_footprint` | **`TB3_N/odom`**, **`TB3_N/base_footprint`** |
| Nav2 launch | stock `nav2_bringup` | **local copy** of `navigation_launch.py` (TF remaps) |

---

## Robot count and model

- TurtleBot3 Burger ×3, not Waffle. `export TURTLEBOT3_MODEL=burger`
- Burger dimensions: 138 × 178 × 192 mm (L × W × H), about 1 kg.

Put in `~/.bashrc` (in addition to Phase 1 vars):

```bash
export TURTLEBOT3_MODEL=burger
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/humble/share/turtlebot3_gazebo/models
```

---

## Files that must exist in this repo

These are the Phase 2 artifacts. If you clone onto a new machine years later, you need **all** of them — stock apt packages alone are not enough.

```
~/swarm-search/
├── launch/
│   ├── multi_robot_spawn.launch.py      # Gazebo + 3 TB3s (SDF rewrite + writable tmp path)
│   ├── slam_tb3.launch.py               # namespaced slam_toolbox wrapper
│   ├── nav2_tb3.launch.py               # namespaced Nav2 → local navigation_launch.py
│   ├── nav2_tb3_localization.launch.py  # Step 3: AMCL + Nav2 on frozen map (TB3_2)
│   ├── navigation_launch.py             # COPY of nav2_bringup; /tf → /tf (absolute)
│   └── tmp_sdf/                         # must exist; runtime SDF writes go here
├── slam_params/                         # mapper_params_tb3_{1,2,3}.yaml
├── nav2_params/
│   ├── burger_tb3_{1,2,3}.yaml          # live SLAM+Nav2: RPP, inflation 0.3, TB3_N/map
│   └── burger_tb3_2_loc.yaml            # Step 3: global_frame shared_map + AMCL
├── maps/
│   ├── house_shared.yaml/.pgm           # Step 2 freeze
│   └── house_shared_live.*              # Step 4 mid-session save
├── src/map_merge_swarm/
│   ├── map_merge_swarm/
│   │   ├── merge_node.py                # fuse → /shared_map (Transient Local)
│   │   ├── gt_map_tf_node.py            # Option C: shared_map → TB3_N/map from Gazebo
│   │   └── spawn_offsets.py             # legacy spawn offsets (v1)
│   └── launch/map_merge.launch.py       # gt_map_tf_node + merge_node
├── src/explore_swarm/
│   ├── explore_swarm/
│   │   ├── frontier_detector.py         # /shared_map → /frontiers (+ markers)
│   │   └── goal_assigner.py             # greedy multi-robot → NavigateToPose
│   └── launch/explore.launch.py         # both explore nodes
└── docs/
    ├── ref-startup-after-break.md           # **use this to restart**
    ├── P2-ref-engineering-notes.md
    ├── P2-checklist.md           # Steps 1–5 checklist
    ├── P2-S5-plan-frontiers.md
    ├── P2-S4-plan-live-sharing.md
    ├── P2-S3-plan-r2-nav-on-saved-map.md      # Step 3
    ├── P2-S1-plan-gt-shared-map.md / P2-S2-plan-freeze-team-map.md
    ├── P1-runbook.md / P2-runbook.md
    └── ref-ros2-pkg-basics.md
```

### Local `navigation_launch.py` (critical)

Copied from:

```
/opt/ros/humble/share/nav2_bringup/launch/navigation_launch.py
```

into:

```
~/swarm-search/launch/navigation_launch.py
```

**Only intentional change** — remappings must be absolute so namespaced Nav2 still uses the shared global TF tree:

```python
# WRONG (stock) — under PushRosNamespace(TB3_1) this becomes /TB3_1/tf (empty)
remappings = [('/tf', 'tf'),
              ('/tf_static', 'tf_static')]

# RIGHT (this repo)
remappings = [('/tf', '/tf'),
              ('/tf_static', '/tf_static')]
```

`nav2_tb3.launch.py` must point at the **repo** copy, not the apt one:

```python
nav2_launch = os.path.join(
    os.path.expanduser("~/swarm-search/launch"), "navigation_launch.py"
)
```

And must pass `namespace` into Nav2 so `RewrittenYaml(root_key=namespace)` matches the namespaced nodes:

```python
launch_arguments={
    "namespace": robot_ns,       # required
    "use_sim_time": use_sim_time,
    "params_file": params_file,
    "autostart": "true",
}.items(),
```

### SLAM YAML top-level key (critical)

Each `mapper_params_tb3_N.yaml` must use a wildcard so params apply under `PushRosNamespace`:

```yaml
/**:
  ros__parameters:
    odom_frame: TB3_1/odom          # TB3_2 / TB3_3 in the other files
    map_frame: TB3_1/map
    base_frame: TB3_1/base_footprint
    scan_topic: /TB3_1/scan
    # ... rest unchanged
```

**Not** `slam_toolbox:` — that key does **not** load onto `/TB3_N/slam_toolbox`.

### Per-robot map frames (critical)

Do **not** share a single TF frame named `map` across three slam_toolbox nodes. Use:

| Robot | SLAM `map_frame` | Nav2 `global_frame` / `global_frame_id` |
|---|---|---|
| TB3_1 | `TB3_1/map` | `TB3_1/map` |
| TB3_2 | `TB3_2/map` | `TB3_2/map` |
| TB3_3 | `TB3_3/map` | `TB3_3/map` |

---

## Multi-robot spawn launch file

Source: copied and modified from ROBOTIS's `multi_robot.launch.py` in apt package `ros-humble-turtlebot3-gazebo`:

```
/opt/ros/humble/share/turtlebot3_gazebo/launch/multi_robot.launch.py
```

Repo copy:

```
~/swarm-search/launch/multi_robot_spawn.launch.py
```

What it does right:

- Spawns 3 robots under `TB3_1` … `TB3_3`
- Rewrites each SDF so `odometry_frame`, `robot_base_frame`, and laser `frame_name` are namespaced (`TB3_N/odom`, etc.) before spawn — TF prefixes are correct from Gazebo
- `robot_state_publisher` gets `frame_prefix` (launch file adds the trailing `/`)

Edits from the ROBOTIS original:

- `number_of_robots`: `4` → `3`
- `pose` trimmed to 3 entries for `turtlebot3_world.world`
- `use_sim_time` default `'false'` → `'true'`
- `save_path` redirected out of `/opt/ros/...` into the repo (see bug below)
- **World path:** `~/swarm-search/worlds/turtlebot3_world.world` (adds `gazebo_ros_state` for `/gazebo/model_states`)
- **Spawn delay:** `TimerAction(period=8.0)` so robots spawn after `/spawn_entity` exists (WSL race)

Create the tmp dir once:

```bash
mkdir -p ~/swarm-search/launch/tmp_sdf
```

Also export models path when using the custom world:

```bash
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/humble/share/turtlebot3_gazebo/models
```

Daily bring-up checklist: `docs/ref-startup-after-break.md`.

---

## Bugs hit, symptom → root cause → fix

### 1. PermissionError writing SDF tmp files on multi-robot spawn

**Symptom:**
```
PermissionError: [Errno 13] Permission denied: '/opt/ros/humble/share/turtlebot3_gazebo/models/turtlebot3_burger/tmp1.sdf'
```

**Root cause:** stock `save_path` writes into the apt share directory (root-owned).

**Fix:** repo-local path + `mkdir -p ~/swarm-search/launch/tmp_sdf`.

**Lesson:** launch files copied from apt that write files must never write under `/opt/ros`.

### 1b. Spawn fails: `Service /spawn_entity unavailable. Was Gazebo started with GazeboRosFactory?` — no robots

**Symptom:** all three `spawn_entity.py` die immediately; arena empty; later `gazebo_ros_state` INFO still appears. ALSA sound errors may spam but are unrelated.

**Root cause:** race — spawn starts before gzserver finishes loading. Factory **is** loaded by default (`factory:=true`); the service simply was not up yet. WSL + custom world makes startup slower.

**Fix:** `TimerAction(period=8.0)` wrapping robot GroupActions in `multi_robot_spawn.launch.py`. Kill leftover Gazebo with one pattern at a time (`pkill -9 gzserver` then `pkill -9 gzclient`). Wait for robots after the delay.

**Lesson:** “GazeboRosFactory?” often means “too early,” not “plugin missing.”

---

### 2. SLAM map topics stayed flat (`/map`) despite namespacing

**Symptom:** `ros2 topic list | grep map` showed one `/map`, not `/TB3_N/map`, even though node names looked namespaced.

**Root cause:**
1. `online_async_launch.py` has no `namespace` argument — CLI `namespace:=` is silently ignored.
2. slam_toolbox publishes `/map` and `/map_metadata` with a leading slash (absolute) → immune to `PushRosNamespace`.

**Fix:** wrapper `slam_tb3.launch.py`:

```python
PushRosNamespace(robot_ns),
SetRemap(src='/map', dst='map'),
SetRemap(src='/map_metadata', dst='map_metadata'),
```

Do **not** remap `/tf` or `/tf_static` for SLAM — keep one shared TF tree; frame IDs are already unique.

---

### 3. Nav2: `No critics defined for FollowPath`

**Symptom:**
```
Couldn't load critics! Caught exception: No critics defined for FollowPath
Failed to change state for node: controller_server
```

YAML **did** contain `critics: [...]`, but the node never saw them. Log showed defaults (e.g. controller frequency `20.0` while YAML said `10.0`).

**Root cause:** `PushRosNamespace(TB3_1)` put nodes at `/TB3_1/controller_server`, but stock `navigation_launch.py` uses `RewrittenYaml(root_key=namespace)` with default empty `namespace`. Params stayed under `/controller_server` and did not bind to namespaced nodes.

**Fix:** in `nav2_tb3.launch.py`, pass the same namespace into the include:

```python
"namespace": robot_ns,
```

**Verify after fix:** configure succeeds; you should **not** see the critics error. Frequency should match YAML (e.g. `10.0`).

---

### 4. Nav2: `Invalid frame ID "TB3_1/odom"` while `tf2_echo` works

**Symptom:**
```
Timed out waiting for transform from TB3_1/base_footprint to TB3_1/odom
Invalid frame ID "TB3_1/odom" ... frame does not exist
```

But in another terminal:

```bash
ros2 run tf2_ros tf2_echo TB3_1/odom TB3_1/base_footprint
# eventually prints transforms
```

**Root cause:** stock Nav2 remaps `/tf` → relative `tf`. Under namespace that becomes `/TB3_1/tf`. Gazebo + RSP + SLAM publish on global `/tf`. Nav2 listened to an **empty** topic.

Confirmed with:

```bash
ros2 topic info /tf -v          # many publishers (diff_drive, RSP, slam)
ros2 topic info /TB3_1/tf -v    # 0 publishers, several Nav2 subscribers
```

**Fix:** local `navigation_launch.py` with:

```python
remappings = [('/tf', '/tf'), ('/tf_static', '/tf_static')]
```

and `nav2_tb3.launch.py` including **that** file.

**Verify:** `/TB3_1/tf` should not exist (or have no Nav2 subs); Nav2 appears as subscriber on `/tf`. Odom timeouts stop.

---

### 5. Nav2 / tf2: `Invalid frame ID "map"` / no shared map TF

**Symptom:**
```
Timed out waiting for transform from TB3_1/base_footprint to map
Invalid frame ID "map" ... frame does not exist
```

```bash
ros2 run tf2_ros tf2_echo map TB3_1/odom
# never gets transforms
```

Map **topics** existed (`/TB3_1/map`, …) but TF frame `map` did not.

**Root cause:** three slam_toolbox nodes all configured with `map_frame: map` fighting over one TF frame name. OccupancyGrid topics can still appear under namespaces while the TF broadcast never becomes usable.

**Fix:** unique map frames per robot (`TB3_N/map`) in both SLAM YAMLs and Nav2 YAMLs (`global_frame`, `global_frame_id`, etc.).

---

### 6. SLAM params never applied under namespace (`map` / `odom` / `/scan` defaults)

**Symptom after editing YAML and relaunching:**

```bash
ros2 param get /TB3_1/slam_toolbox map_frame
# String value is: map          ← stock default, NOT TB3_1/map
ros2 param get /TB3_1/slam_toolbox scan_topic
# String value is: /scan        ← wrong; scans are on /TB3_1/scan
```

Scans on `/TB3_1/scan` were fine (`ros2 topic hz` ~5 Hz), but SLAM subscribed to `/scan` → no usable mapping TF.

**Root cause:** YAML top-level key `slam_toolbox:` does not match FQN `/TB3_1/slam_toolbox` when the node is under `PushRosNamespace`. Params silently ignored; node keeps declare_parameter defaults.

**Fix:** change top of each mapper YAML to:

```yaml
/**:
  ros__parameters:
```

**Also:** put the full launch on **one** command (use `\`); if you press Enter after `use_sim_time:=true`, the next line `slam_params_file:=...` is **not** part of the launch and the stock default YAML is used.

**Verify:**

```bash
ros2 param get /TB3_1/slam_toolbox map_frame
# String value is: TB3_1/map
ros2 param get /TB3_1/slam_toolbox scan_topic
# String value is: /TB3_1/scan
ros2 run tf2_ros tf2_echo TB3_1/map TB3_1/odom
# after a short wait, prints transforms
```

---

### 7. Narrow doorway inflation trap & recovery spin collision (`turtlebot3_house`)

**Symptom:**
Robot navigates toward a doorway (~0.75 m wide) or tight corner near furniture, stops abruptly, triggers Nav2 recovery behaviors (`Spin` / `Backup`), clips the wall, and physically wedges. Wheels slip against the Gazebo collision mesh, odometry reports rotation without actual movement, and `slam_toolbox` warps or rotates the entire map.

**Root cause:**
`inflation_radius: 0.45` in `nav2_params/burger_tb3_*.yaml`. The left and right doorposts each inflate by 0.45 m ($0.45 \times 2 = 0.90\text{ m} > 0.75\text{ m}$). The inflation zones overlap in the center of the doorway, marking the entire opening with high / impassable cost. The DWB controller refuses to enter, exhausts patience, and triggers recovery `Spin`, which has a wide footprint sweep that clips the door frame.

**Fix:**
In `burger_tb3_*.yaml`, under both `local_costmap` and `global_costmap`:
```yaml
inflation_layer:
  inflation_radius: 0.25      # was 0.45; Burger radius is only 0.10m
  cost_scaling_factor: 8.0     # was 5.0; steeper cost decay away from obstacles
```
Also raise clearance in `goal_assigner.py` (`_find_goal`) from `(0.3, 0.2)` to `(0.35, 0.28)` so the assigner never places candidate goals in tight dead-end corners behind furniture.

---

### 8. 1 ms TF extrapolation abort in `controller_server` (`transformPose`)

**Symptom:**
```text
[TB3_1.controller_server]: Exception in transformPose: Lookup would require extrapolation into the future.
Requested time 265.322000 but the latest data is at time 265.321000, when looking up transform from frame [TB3_1/odom] to frame [TB3_1/map]
[TB3_1.controller_server]: Unable to transform robot pose into global plan's frame
[TB3_1.controller_server]: Controller patience exceeded -> Aborting handle.
```
Global planner successfully computes a path and RViz draws the path, but the robot stands still and repeatedly aborts.

**Root cause:**
The global plan is stamped at sim time $T$. In the controller loop, `controller_server` transforms plan poses into the robot's local frame. When simulation time jitters by even 1 millisecond ($T$ vs $T - 0.001\text{ s}$), tf2 fails if no tolerance timeout is provided. `transform_tolerance: 1.0` was set under the `FollowPath` plugin, but **missing** directly under `controller_server.ros__parameters` and `local_costmap.local_costmap`, where it defaulted to 0.1 s.

**Fix:**
In `burger_tb3_*.yaml`:
1. Under `controller_server.ros__parameters`: add `transform_tolerance: 1.0`.
2. Under `local_costmap.local_costmap.ros__parameters`: add `transform_tolerance: 0.5`.
3. Note: Nav2 reads YAML files **only on startup**. Always fully restart Nav2 after editing YAML files.

---

### 9. NavFn Dijkstra failure in narrow corridors (`use_astar: false`)

**Symptom:**
Goals in or beyond doorways abort with status 6, logging:
```text
[planner_server]: Failed to create a plan from potential when a legal potential was found.
```

**Root cause:**
NavFn's default Dijkstra potential gradient can get trapped in local potential valleys when traversing narrow apertures surrounded by obstacles.

**Fix:**
In `burger_tb3_*.yaml` under `planner_server -> GridBased`:
```yaml
GridBased:
  plugin: "nav2_navfn_planner/NavfnPlanner"
  tolerance: 0.5
  use_astar: true               # was false
  allow_unknown: true
```
A* uses heuristic distance to find clear centerlines through narrow doorways.

---

### 10. Multi-robot frontier starvation & escape goal loop

**Symptom:**
Robot 1 (TB3_1) explores a small area in its starting room, then stops moving toward frontiers. Logs spam `no pick: {'close': 0, 'claimed': 13, 'rejected': 0, 'blacklisted': 5} idle=['TB3_1']`. Every 10 seconds it triggers `TB3_1: escape goal (...)` which only drives back and forth within a 2-meter circle in its own room.

**Root cause:**
1. `CLAIM_RADIUS = 2.5` m in `goal_assigner.py` was too large (~20 m² per robot). When TB3_2 and TB3_3 each claimed a frontier, their two 2.5 m claim bubbles locked out 13+ frontiers across the entire house.
2. Greedy nearest-pair assignment prioritizes smaller straight-line distance $d^2$. Since TB3_2 and TB3_3 were already in the hallway, they won every newly detected frontier, starving TB3_1.
3. `_escape_goal` picks the point with maximum clearance within `radius_m = 2.0`, which is just the center of TB3_1's current room.

**Fix:**
In `goal_assigner.py`:
- Lower `CLAIM_RADIUS` from 2.5 m to 1.0 m. Teammates won't pick the exact same cluster, but are free to explore different areas of the same large room.
- Lower `BLACKLIST_RADIUS` from 1.2 m to 0.6 m so aborted goals don't block entire corridors.
- In `_on_result`, add a 2-attempt visit limit with permanent 600 s blacklist (see Bug 12).

---

### 11. Cross-path room swapping & 0.75 m doorway deadlocks (Patterns 5 & 6)

**Symptom:**
Robots in opposite rooms swap targets (e.g. R1 in South assigned North, R3 in North assigned South). They navigate into the same narrow 0.75 m doorways, meeting head-on, triggering recovery spins, and colliding.

**Root cause:**
1. A negative dispersion reward (`- DISPERSE_W * min(d_other, 6.0)`) granted up to 3.0 m artificial discount to far-away frontiers, overpowering local Euclidean distance and pulling robots across the house.
2. Straight-line Euclidean distance ignored walls, treating frontiers in adjacent rooms as 1.5 m away when the actual navigable path was 6 m.
3. Once `_busy`, a robot was completely ignored by `_on_frontier`. When its lidar uncovered fresh frontiers in its own room, it ignored them and drove 5 m to a distant room, leaving its own room for a teammate to claim.
4. A 3.0 s status-4 cooldown froze robots after completing a goal, allowing distant teammates to snatch frontiers right outside their doorway.

**Fix:**
In `goal_assigner.py`:
1. **Soft-cost ranking:** Replace negative bounty with positive penalties:
   ```python
   wall_cost = 2.5 if self._line_crosses_occupied(rx, ry, fx, fy) else 0.0
   teammate_penalty = max(0.0, 2.5 - d_other) * 2.0
   return d_self + gap_cost + wall_cost + teammate_penalty
   ```
2. **Teammate personal bubble rule:** Skip frontiers where `d_other < 1.2` and `d_self > 2.5` so distant robots never steal frontiers from teammates standing right next to them.
3. **Pattern 6 in-flight redirect:** If a busy robot is heading to a distant goal ($d > 3.0\text{ m}$) and a new frontier appears in the same room ($d < 1.5\text{ m}$), cancel the far goal and redirect immediately via `_redirect_goal`.
4. **Instant goal handoff:** Reduced status-4 cooldown to `now + 0.1` s.

---

### 12. Permanent obstacle shadows (trash can) & table leg collisions

**Symptom:**
1. Robots repeatedly visit the same spot (behind the round trash can) from far away, but the orange frontier marker never disappears.
2. Robot navigates between dining table legs in Gazebo, clips a table leg cylinder, and gets stuck spinning wheels. RViz path appears frozen from the previous starting point.

**Root cause:**
1. Solid obstacles in Gazebo cannot be penetrated by lidar rays. The interior remains `-1` (unknown), while the outer boundary is `0` (free). This generates an artificial, permanent frontier cluster that can never be cleared.
2. `robot_radius: 0.10` and `inflation_radius: 0.25` in Nav2 costmaps made the 0.7 m gap between table legs look completely open. But the Burger's true physical width is 17.8 cm (radius ~13 cm to corners). The robot clipped the leg in Gazebo, entering costmap collision. When replanning failed, Nav2 did not publish a new path, leaving RViz displaying the last valid plan.

**Fix:**
1. **2-Attempt visit rule:** In `_on_result`, track `_visit_counts`. First visit gets a 60 s cooldown. If visited a 2nd time without clearing, permanently blacklist for 600 s (10 min).
2. **True robot radius & inflation:** In `burger_tb3_*.yaml`, set `robot_radius: 0.13` and `inflation_radius: 0.30` under local and global costmaps. Routes paths around dining tables while keeping 0.75 m doorways passable.
3. **Goal clearance:** In `_find_goal`, raise clearance candidate threshold to `(0.35, 0.25)` m.

---

## Recreate from scratch on a new machine (years later)

### A. Bring the machine to Phase 1 baseline

Follow `P1-runbook.md` sections 0–2 (WSL, apt install Humble + Gazebo Classic + Nav2 + slam_toolbox + turtlebot3, GPU env vars). Then switch model to burger:

```bash
echo 'export TURTLEBOT3_MODEL=burger' >> ~/.bashrc
source ~/.bashrc
```

### B. Get this repo and dirs

```bash
# clone or copy ~/swarm-search onto the machine
mkdir -p ~/swarm-search/launch/tmp_sdf
```

Confirm critical files exist (see list above). If `navigation_launch.py` is missing, re-copy from apt and re-apply the `/tf` → `/tf` remap change.

### C. Terminal layout (recommended: 1 spawn + 3 SLAM + 3 Nav2)

All commands assume:

```bash
source /opt/ros/humble/setup.bash
export TURTLEBOT3_MODEL=burger
```

#### Terminal 0 — Gazebo + three robots

```bash
mkdir -p ~/swarm-search/launch/tmp_sdf
ros2 launch ~/swarm-search/launch/multi_robot_spawn.launch.py use_sim_time:=true
```

Wait until three Burgers are visible and not exploding.

Sanity:

```bash
ros2 node list | grep -E 'TB3_[123]'
ros2 run tf2_ros tf2_echo TB3_1/odom TB3_1/base_footprint
```

#### Terminals 1–3 — SLAM (one per robot)

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

Per-robot check (example TB3_1):

```bash
ros2 param get /TB3_1/slam_toolbox map_frame    # TB3_1/map
ros2 param get /TB3_1/slam_toolbox scan_topic   # /TB3_1/scan
ros2 topic list | grep map                      # /TB3_1/map, /TB3_2/map, /TB3_3/map
ros2 run tf2_ros tf2_echo TB3_1/map TB3_1/odom  # transforms after brief wait
```

#### Terminals 4–6 — Nav2 (only after that robot’s map TF exists)

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

Nav2 should configure without critics errors and without endless `TB3_N/odom` / `map` timeouts.

Check:

```bash
ros2 node list | grep controller_server
# /TB3_1/controller_server
# /TB3_2/controller_server
# /TB3_3/controller_server

ros2 topic info /tf -v | head -40
# should show Nav2 / costmap listeners among subscribers on /tf
# should NOT show Nav2 stuck only on /TB3_N/tf
```

Optional lifecycle (may print blank if a node is still blocked on TF — fix TF first):

```bash
ros2 lifecycle get /TB3_1/controller_server
# want: active (or inactive → active after bringup)
```

---

## Startup order (short)

Prefer full copy-paste in `ref-startup-after-break.md` §3.

1. `multi_robot_spawn.launch.py`
2. Three × `slam_tb3.launch.py` (params applied — verify `map_frame`)
3. Confirm `TB3_N/map → TB3_N/odom` with `tf2_echo`
4. Three × `nav2_tb3.launch.py` (**needed** for auto-explore)
5. `ros2 launch map_merge_swarm map_merge.launch.py`
6. RViz: Fixed Frame `shared_map`, Map topic `/shared_map`
7. `ros2 launch explore_swarm explore.launch.py`

Never start Nav2 before that robot’s map TF exists, or you will re-hit global_costmap timeouts (Nav2 can recover once TF appears, but a clean order avoids confusion).

Before restarting map merge, stop any old standalone `static_map_tfs` / `merge_node` so you do not get duplicate TFs or two merge publishers.

---

## Map merge (`/shared_map`)

**Status:** working. v1 spawn-offset TFs verified 2026-09-21; **superseded for alignment by Option C GT TFs** (2026-10) — `gt_map_tf_node` in `map_merge.launch.py`. History: `P2-S0-todo-map-merge-v1.md`, `P2-S1-plan-gt-shared-map.md`.

### What it does

- Each robot still runs its own SLAM (`/TB3_N/map` in frame `TB3_N/map`).
- Package `map_merge_swarm` paints each grid onto one canvas via TF `shared_map → TB3_N/map`, publishes **`/shared_map`**.
- **Alignment now:** Gazebo model states → GT SE(2) (`burger_1/2/3`). **Legacy v1:** static spawn offsets (kept in `spawn_offsets.py` / old `static_map_tfs` — do not run alongside GT launch).
- Fusion rule (v1): any occupied → occupied; else any free → free; else unknown.
- Canvas: resolution `0.05`, 400×400 cells, origin `(-10, -10)` (about 20×20 m).

This is a **shared view**, not a new single SLAM. With GT TFs, walls should stack as one house (not three spawn islands).

### Spawn offsets

Same as `launch/multi_robot_spawn.launch.py` / `src/map_merge_swarm/map_merge_swarm/spawn_offsets.py`:

| Robot | x | y | yaw |
|---|---|---|---|
| TB3_1 | -2.0 | -0.5 | 0.0 |
| TB3_2 | 0.5 | -2.0 | 0.0 |
| TB3_3 | 2.0 | 0.5 | 0.0 |

Static TFs: `shared_map` → `TB3_N/map` with that translation (identity rotation).

### Build and launch

```bash
cd ~/swarm-search
colcon build --packages-select map_merge_swarm
source install/setup.bash

# Gazebo + SLAM ×3 already running, then:
ros2 launch map_merge_swarm map_merge.launch.py
```

That one launch starts three `static_transform_publisher` nodes + `merge_node`.

### RViz

```bash
ros2 run rviz2 rviz2 --ros-args -p use_sim_time:=true
```

- Fixed Frame under **Global Options** (not Global Status): `shared_map`
- Add → Map → Topic: `/shared_map`

### Sanity checks

```bash
ros2 run tf2_ros tf2_echo shared_map TB3_1/map
# Option C (current): tracks live alignment (not fixed spawn)
# Legacy v1 only: translation ≈ (-2.0, -0.5, 0.0)

ros2 topic echo /shared_map --once | head -30
# frame_id: shared_map, width/height: 400
# leading data -1 is normal (empty corner of large grid) — trust RViz
```

### Known limits (historical v1; mostly superseded)

- Fixed canvas (~20×20 m @ 0.05 m) still applies.  
- Spawn-offset “three islands” fixed by **Option C GT TFs** (see below).  
- Robots still Nav2 in `TB3_N/map`; frontiers are planned in `shared_map` then **TF’d** into each robot frame (not AMCL-on-shared for the live explore stack).

---

## Progress after map-merge v1 (Steps 1–5) — do not lose

Living plans: `P2-checklist.md`. Commands: `ref-startup-after-break.md`.

| Step | Result | Key artifact |
|------|--------|--------------|
| 1 | One house outline on `/shared_map` | `gt_map_tf_node` — `shared_T_map = shared_T_base ∘ inv(map_T_base)`; models **`burger_*`** not waffle |
| 2 | Freeze team map | `maps/house_shared.yaml` + `.pgm`; map_saver needs Transient Local + longer timeout |
| 3 | R2 drives on frozen map | `burger_tb3_2_loc.yaml` + `nav2_tb3_localization.launch.py`; Fixed Frame `shared_map`, Map `/map` |
| 4 | Live share while exploring | T7 merge + mid-session `house_shared_live.*`; comms radius **stub later** |
| 5 | Auto explore | `frontier_detector` + greedy `goal_assigner`; `ros2 launch explore_swarm explore.launch.py` |

### Live explore launch order (summary)

Same as `ref-startup-after-break.md` §3: T0 Gazebo → T1–T3 SLAM → T4–T6 Nav2 (`burger_tb3_N.yaml`) → T7 `map_merge.launch.py` → T8 RViz (`/shared_map`) → T9 `explore.launch.py`.

```bash
ros2 launch explore_swarm explore.launch.py
```

### Explore / Nav lessons worth keeping

| Lesson | Detail |
|--------|--------|
| QoS maps | RELIABLE + **TRANSIENT_LOCAL** or late subscribers see nothing |
| Goal frame | Frontier in `shared_map` → `do_transform_pose` into `TB3_N/map` (pass `.pose`, not PoseStamped) |
| Backoff + MIN_GOAL_DIST | Pull goal into free; skip if still &lt; ~0.5 m or Nav2 instant status 4 |
| Busy + cooldown | Per-robot flags or `/frontiers` ~1 Hz spam-replaces goals |
| Multi-robot | One `ActionClient` per ns; pop claimed frontiers; `wait_for_server` or `Nav2 not ready` |
| Controller | **RPP** not DWB/RotationShim (moon-spin / tip); `inflation_radius: 0.3` |
| Tip → ghost wall | Pitch/roll → lidar sky/floor → fake walls in merge; reset that SLAM |
| No robot–robot costmap | Paths can cross / scrape (v1 OK) |
| Last frontier | Only one robot keeps moving — greedy is correct |
| `clusters_kept = 0` | Detector alive; no free↔unknown edge yet |

### Modes — never mix on one robot

| Mode | Mapping | Loc | Nav2 `global_frame` |
|------|---------|-----|---------------------|
| Live explore (§3) | SLAM on | SLAM TF | `TB3_N/map` |
| Frozen Nav Step 3 (§6) | SLAM **off** | AMCL | `shared_map` |

---

## Quick diagnostic cheat sheet

| Check | Command | Healthy result |
|---|---|---|
| Robot odom TF | `ros2 run tf2_ros tf2_echo TB3_1/odom TB3_1/base_footprint` | transforms |
| SLAM params loaded | `ros2 param get /TB3_1/slam_toolbox map_frame` | `TB3_1/map` not `map` |
| SLAM scan source | `ros2 param get /TB3_1/slam_toolbox scan_topic` | `/TB3_1/scan` |
| Laser data | `ros2 topic hz /TB3_1/scan` | ~5 Hz |
| Map topics | `ros2 topic list \| grep map` | `/TB3_N/map` ×3 + `/shared_map` |
| Map TF (live GT) | `ros2 run tf2_ros tf2_echo shared_map TB3_1/map` | tracks pose (not only spawn) |
| Gazebo models | `ros2 topic echo /gazebo/model_states --qos-reliability best_effort --once` | `burger_1/2/3` |
| Nav2 actions | `ros2 action list \| grep navigate_to_pose` | `/TB3_1`…`/TB3_3` |
| Explore nodes | `ros2 node list \| grep -E 'frontier\|goal_assign'` | both up |
| Frontiers | `ros2 topic echo /frontiers --once` | poses when map has edges |
| Nav2 using global TF | `ros2 topic info /TB3_1/tf -v` | no empty Nav2-only topic |
| Nav2 got YAML | no `No critics defined for FollowPath`; RPP FollowPath | |

---

## Still open (rest of roadmap Phase 2)

- [x] Per-robot SLAM + Nav2 bringup  
- [x] Shared occupancy `/shared_map` (GT align + fuse)  
- [x] Frontiers + greedy multi-robot NavigateToPose + explore launch  
- [ ] Coverage vs time CSV/plot (**optional** — one series from `/shared_map`)  
- [ ] **Comms radius** — **skipped for now**; may add later  
- [ ] House / demo polish — later; aerial / YOLO = later roadmap phases  

TF/param rules still apply: unique `TB3_N/map`, shared `/tf`, `/**:` SLAM params, absolute TF remaps in local Nav2 launch, `TURTLEBOT3_MODEL=burger`.
