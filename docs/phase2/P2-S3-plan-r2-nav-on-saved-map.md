> **📍 Phase 2 › Step 3 (R2 nav on saved map)** · [← Roadmap](swarm_search_rescue_roadmap_v4.md) · [← P2 todo](P2-checklist.md) · Depends on: [Step 1](P2-S1-plan-gt-shared-map.md), [Step 2](P2-S2-plan-freeze-team-map.md) · Status: ✅ Done

# Plan — Step 3: R2 navigates on the saved team map (learn + implement)

**Parent todo:** `docs/P2-checklist.md` → **Step 3** (proof: R2 uses R1’s / team map).  
**Goal:** Robot 2 **localizes** on `maps/house_shared` and **Nav2-drives** to a place that was mapped on the team map — without R2 building that map from scratch with SLAM.  
**You implement this yourself** (learning path). This file is the explanation + how-to + checklist.

**Depends on:**  
- Step 1 Option C (`P2-S1-plan-gt-shared-map.md`) — one house live  
- Step 2 freeze (`P2-S2-plan-freeze-team-map.md`) — `maps/house_shared.yaml` + `.pgm` + `maps/README.md`

**Related:** `ref-startup-after-break.md`, `maps/README.md`, `P2-checklist.md`.

---

## Big idea (plain English)

Until now:

| Mode | What R2 does |
|---|---|
| **Mapping (old Phase 2)** | SLAM draws `TB3_2/map` while driving; Nav2 plans in that private notebook |
| **This Step 3** | Load the **saved house** from disk; AMCL asks “where am I on this house?”; Nav2 plans on that same house |

So R2 is using the **team floor plan**, not only what it scanned itself.

```text
Saved files  →  map_server  →  /map  (frame: shared_map)
                                      │
                                      ▼
                               AMCL (localization)
                                      │  publishes shared_map → TB3_2/odom
                                      ▼
                               Nav2 (planner + controller)
                                      │
                                      ▼
                               TB3_2 drives to a goal on the house
```

**SLAM for TB3_2 must be OFF** during this test (two map→odom publishers fight).

---

## Mapping vs localization (one sentence each)

- **Mapping (SLAM):** build a new map while you move.  
- **Localization (AMCL):** you already have a map; estimate pose on it using lidar + odom.

Step 3 = localization + navigation on the **frozen** team map.

---

## Frames for this test (use these names)

| Frame | Role |
|---|---|
| `shared_map` | Team house (= saved map `frame_id`, Gazebo world for Option C) |
| `TB3_2/odom` | Wheel odom for robot 2 |
| `TB3_2/base_footprint` | Robot 2 body |

AMCL’s job is to publish:

```text
shared_map  →  TB3_2/odom
```

Gazebo already publishes `TB3_2/odom` → `TB3_2/base_footprint`.  
Together you get `shared_map` → … → `base_footprint` so Nav2 can plan.

**Do not** use `TB3_2/map` as the global frame for this proof — that was the SLAM notebook. The team map frame is **`shared_map`**.

---

## Important: what your repo does today

Your Phase 2 Nav2 launch (`launch/nav2_tb3.launch.py` → `navigation_launch.py`) starts **planner / controller / BT** only. It does **not** start:

- `map_server`
- `amcl`

That was OK when **SLAM** provided `TB3_N/map` → `TB3_N/odom`.  
For Step 3 you must **add** map_server + AMCL (or a localization launch), and point Nav2 global frames at **`shared_map`**.

Also: stock `burger_tb3_2.yaml` still has `global_frame: TB3_2/map` and some costmap scans on `/scan` (wrong for namespaced robots). You will make a **localization copy** of the params — do not break the old SLAM+Nav2 file if you still want mapping later.

---

## Spawn pose reminder (initial guess)

From `multi_robot_spawn.launch.py`:

| Robot | Spawn (x, y) ≈ pose in `shared_map` at start |
|---|---|
| TB3_1 | (−2.0, −0.5) |
| **TB3_2** | **(0.5, −2.0)** |
| TB3_3 | (2.0, 0.5) |

Use **(0.5, −2.0)** as the first **2D Pose Estimate** for R2 if you just spawned and did not teleop far.

---

## Terminals — keep / close / add

You already had full stack; for Step 3:

| Terminal | Action |
|---|---|
| **T0 Gazebo** | **Keep** |
| **T8 RViz** | **Keep** (Fixed Frame `shared_map`) |
| Teleop | Optional — use **TB3_2** if you nudge the robot; or use RViz goals later |
| SLAM T1–T3 | **Close** (especially TB3_2) |
| Nav2 T4–T6 | **Close** (old config) |
| T7 map merge | **Close** (use frozen files, not live `/shared_map`) |

**Add:**

| Terminal | Role |
|---|---|
| **map_server** | Load `house_shared.yaml` → `/map`, `frame_id:=shared_map` |
| **AMCL + Nav2 (TB3_2)** | Localization + navigation (after you create params/launch) |

---

## CRITICAL — every new terminal

```bash
source /opt/ros/humble/setup.bash
source ~/swarm-search/install/setup.bash
export TURTLEBOT3_MODEL=burger
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/humble/share/turtlebot3_gazebo/models
cd ~/swarm-search
```

After any edit to launch/params under the workspace that you install via colcon: **rebuild + source** if those files are installed by a package. Plain files under `~/swarm-search/launch` and `nav2_params/` are used by path — **no colcon needed** for those, but **do** source Humble.

---

## Part 1 — Bring-up checklist (sim + frozen map only)

**Why:** Prove Gazebo + saved map before touching AMCL.

1. T0 running; robots visible.  
2. No SLAM / no old Nav2 / no T7.  
3. Start map_server (from `maps/README.md`):

```bash
ros2 run nav2_map_server map_server --ros-args \
  -p yaml_filename:=$HOME/swarm-search/maps/house_shared.yaml \
  -p use_sim_time:=true \
  -p frame_id:=shared_map
```

```bash
ros2 lifecycle set /map_server configure
ros2 lifecycle set /map_server activate
```

4. RViz: Fixed Frame `shared_map`, Map topic `/map`, Durability **Transient Local**.  
5. Confirm house walls appear.

**Done when:** same as Step 2 reload proof, with Gazebo also running.

---

## Part 2 — Create a localization params file for TB3_2

**Why:** Keep `burger_tb3_2.yaml` for the old “SLAM + Nav2” mode. Step 3 needs different global frame + map path + scan topics.

```bash
cp ~/swarm-search/nav2_params/burger_tb3_2.yaml \
   ~/swarm-search/nav2_params/burger_tb3_2_loc.yaml
```

Edit **`burger_tb3_2_loc.yaml`**:

### 2a — AMCL (already in the file under `amcl:`)

Set / confirm:

```yaml
amcl:
  ros__parameters:
    base_frame_id: "TB3_2/base_footprint"
    odom_frame_id: "TB3_2/odom"
    global_frame_id: "shared_map"    # was TB3_2/map
    scan_topic: scan                 # relative → /TB3_2/scan under namespace
    map_topic: /map                  # absolute → map_server topic
    tf_broadcast: true
    set_initial_pose: false          # you set pose in RViz
```

**Why `map_topic: /map`?**  
map_server (Part 1) publishes globally on `/map`. If you leave `map`, under namespace it becomes `/TB3_2/map` and AMCL never sees the saved map.

### 2b — Nav2 global frames

Replace **`TB3_2/map`** with **`shared_map`** everywhere it means “global map frame”, including:

- `bt_navigator` → `global_frame`
- `global_costmap` → `global_frame`
- `local_costmap` stays `TB3_2/odom` (local is odom — OK)
- any other `global_frame: TB3_2/map` (e.g. behavior / waypoint sections if present)

### 2c — Scan topics on costmaps

Anywhere you see `topic: /scan`, change to:

```yaml
topic: /TB3_2/scan
```

(or relative `scan` if that node is under the `TB3_2` namespace and resolves correctly — **absolute `/TB3_2/scan` is safer**).

### 2d — map_server block in yaml (if Nav2 launch will start map_server)

```yaml
map_server:
  ros__parameters:
    yaml_filename: "/home/pruth/swarm-search/maps/house_shared.yaml"
    # frame_id may be set via launch param; map yaml has no frame — use launch/node param shared_map
```

If you keep map_server as a **separate** terminal (Part 1), you can leave this unused — but then **do not** start a second map_server from Nav2.

**Done when:** file exists; `grep -n 'TB3_2/map\|/scan\|map_topic\|yaml_filename' burger_tb3_2_loc.yaml` looks correct.

---

## Part 3 — Launch AMCL + Nav2 for TB3_2

**Why:** Your current `nav2_tb3.launch.py` has no AMCL. You need both.

### Option A (recommended learning path) — two pieces

**A1 — map_server** already running (Part 1).

**A2 — AMCL node** under namespace TB3_2:

```bash
ros2 run nav2_amcl amcl --ros-args \
  -r __ns:=/TB3_2 \
  --params-file $HOME/swarm-search/nav2_params/burger_tb3_2_loc.yaml \
  -p use_sim_time:=true
```

Note: params file structure uses key `amcl:` — with `__ns:=/TB3_2`, ROS may expect params under `/TB3_2/amcl`. If AMCL ignores params, use a small launch file (Option B) with `RewrittenYaml` / proper parameter loading like Nav2 bringup.

**A3 — existing navigation** with the loc params:

```bash
ros2 launch ~/swarm-search/launch/nav2_tb3.launch.py \
  robot_ns:=TB3_2 use_sim_time:=true \
  params_file:=$HOME/swarm-search/nav2_params/burger_tb3_2_loc.yaml
```

Then lifecycle-activate AMCL if needed:

```bash
ros2 lifecycle set /TB3_2/amcl configure
ros2 lifecycle set /TB3_2/amcl activate
```

### Option B (cleaner) — one launch file you write

Create `~/swarm-search/launch/nav2_tb3_localization.launch.py` that:

1. Optionally starts `map_server` with `yaml_filename` + `frame_id:=shared_map` **or** documents that map_server is external  
2. Starts `amcl` with `burger_tb3_2_loc.yaml` under `PushRosNamespace('TB3_2')`  
3. Includes your existing `navigation_launch.py` like `nav2_tb3.launch.py` does  
4. Lifecycle-manages `map_server` + `amcl` (if started here) + navigation nodes  

You can mirror `/opt/ros/humble/share/nav2_bringup/launch/localization_launch.py` + your `nav2_tb3.launch.py`.

**Done when:** nodes exist, e.g.:

```bash
ros2 node list | grep -E 'amcl|map_server|controller_server'
```

---

## Part 4 — Set initial pose (required)

**Why:** AMCL does not magically know where R2 is. You give a hint; lidar refines it.  
Until this succeeds, `tf2_echo shared_map TB3_2/base_footprint` will say **frame does not exist** — that is normal.

### 4a — Get current pose from Gazebo (`burger_2`)

```bash
ros2 topic echo /gazebo/model_states --qos-reliability best_effort --once
```

Find `burger_2` in `name:` — use the **same index** in `pose:` for **x, y** and **orientation** (x, y, z, w).

If you never drove R2 after spawn, use spawn defaults:

| | value |
|---|---|
| x, y | **0.5, −2.0** |
| orientation | **w: 1.0** (x=y=z=0) |

### 4b — Publish initial pose (preferred — exact numbers)

Replace `PUT_*` with Gazebo values (or spawn defaults):

```bash
ros2 topic pub --once /TB3_2/initialpose geometry_msgs/msg/PoseWithCovarianceStamped "{
  header: {frame_id: 'shared_map'},
  pose: {
    pose: {
      position: {x: PUT_X, y: PUT_Y, z: 0.0},
      orientation: {x: PUT_QX, y: PUT_QY, z: PUT_QZ, w: PUT_QW}
    }
  }
}"
```

Fresh-spawn shortcut:

```bash
ros2 topic pub --once /TB3_2/initialpose geometry_msgs/msg/PoseWithCovarianceStamped "{
  header: {frame_id: 'shared_map'},
  pose: { pose: { position: {x: 0.5, y: -2.0, z: 0.0}, orientation: {w: 1.0} } }
}"
```

Use the **full quaternion** from Gazebo when lasers don’t match walls (yaw wrong with only `w: 1.0`).

### 4c — Optional: RViz click instead

1. Fixed Frame: `shared_map`  
2. Tool **2D Pose Estimate** — click-drag near TB3_2  
3. RViz often publishes `/initialpose`; AMCL needs **`/TB3_2/initialpose`**:

```bash
ros2 topic list | grep initialpose
ros2 run topic_tools relay /initialpose /TB3_2/initialpose
```

Then click 2D Pose Estimate again. Or set the tool topic to `/TB3_2/initialpose`.

### 4d — Check

```bash
ros2 topic info /TB3_2/initialpose -v   # AMCL should be a subscriber
ros2 run tf2_ros tf2_echo shared_map TB3_2/base_footprint
```

In RViz add **LaserScan** topic `/TB3_2/scan` — beams should **line up with walls**.

**Done when:** lasers match walls; TF `shared_map` → `TB3_2/base_footprint` prints (not “frame does not exist”).

---

## Part 5 — Pick a map point + send Nav2 goal (the proof)

**Why:** R2 drives using the **saved** map. Prefer a **short** open-floor hop (~0.5–1 m), not on top of the robot and not into the center object / walls.

### 5a — Pick goal coordinates (Publish Point)

In RViz toolbar: **Publish Point** → click on the map (open floor).

In a terminal (leave running, then click once):

```bash
ros2 topic echo /clicked_point --once
```

Copy **`point.x`** and **`point.y`** (ignore z). Use those exact numbers — do not typo `0.545` as `0.055`.

### 5b — Send NavigateToPose

```bash
ros2 action send_goal /TB3_2/navigate_to_pose nav2_msgs/action/NavigateToPose \
"{pose: {
  header: {frame_id: 'shared_map'},
  pose: {
    position: {x: PUT_X, y: PUT_Y, z: 0.0},
    orientation: {w: 1.0}
  }
}}"
```

`w: 1.0` is enough unless you care about final yaw.

Watch Gazebo + Nav2 log. **Demo intent:** goal on the team/saved map (SLAM off).

**Done when:** `Goal finished with status: SUCCEEDED` (or clear drive along a path on `/map`).

---

## Part 6 — After-break test + record (Step 3 proof)

**Status:** Step 3 **works** with current stack: RPP controller, inflation 0.3, AMCL `/tf` remaps. Multiple `NavigateToPose` **SUCCEEDED** (short hops + clicked goals). Still be careful: wait for lifecycle active, lasers tight, watch Path.  
**Pause point for tonight:** demo worked; optional polish later (tick parent todo, cleaner long goals).  

**Do not chase map colors.** See **Command cheat sheet** below for all copy-paste commands.

**Every new terminal first:**

```bash
source /opt/ros/humble/setup.bash
source ~/swarm-search/install/setup.bash
export TURTLEBOT3_MODEL=burger
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/humble/share/turtlebot3_gazebo/models
cd ~/swarm-search
```

### 6A — Clean restart (full commands)

Stop Nav2/AMCL, map_server, RViz, then Gazebo. Then **in order**:

**Terminal 1 — T0 Gazebo**

```bash
mkdir -p ~/swarm-search/launch/tmp_sdf
ros2 launch ~/swarm-search/launch/multi_robot_spawn.launch.py use_sim_time:=true
```

Wait until `burger_1/2/3` spawned. **Do not** start SLAM (T1–T3), old Nav2 (T4–T6), or map merge (T7).

**Terminal 2 — map_server**

```bash
ros2 run nav2_map_server map_server --ros-args \
  -p yaml_filename:=$HOME/swarm-search/maps/house_shared.yaml \
  -p use_sim_time:=true \
  -p frame_id:=shared_map
```

**Terminal 3 — lifecycle** (same machine; can reuse after map_server is up)

```bash
ros2 lifecycle set /map_server configure
ros2 lifecycle set /map_server activate
```

**Terminal 4 — AMCL + Nav2 (TB3_2)**

```bash
ros2 launch ~/swarm-search/launch/nav2_tb3_localization.launch.py
```

**Terminal 5 — RViz**

```bash
ros2 run rviz2 rviz2 --ros-args -p use_sim_time:=true
```

RViz settings:

| Setting | Value |
|---|---|
| Fixed Frame | `shared_map` |
| Map → Topic | `/map` |
| Map → Durability | **Transient Local** |
| Map → Color Scheme | `map` |
| LaserScan → Topic | `/TB3_2/scan` |

### 6B — Clock / params check (before initial pose TF is optional)

```bash
ros2 topic echo /clock --once
ros2 param get /TB3_2/controller_server use_sim_time   # expect true
ros2 param get /TB3_2/amcl use_sim_time                # expect true
ros2 param get /map_server use_sim_time                # expect true
```

`tf2_echo shared_map TB3_2/base_footprint` will fail with **“frame does not exist”** until **6C** — that is OK here.

Optional — count free cells on `/map` (needs Transient Local QoS or the script hangs):

```bash
python3 - <<'PY'
import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import OccupancyGrid
rclpy.init()
node = rclpy.create_node('map_count')
box = {'m': None}
qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                 reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST)
node.create_subscription(OccupancyGrid, '/map', lambda m: box.update(m=m), qos)
for _ in range(40):
    if box['m'] is not None: break
    rclpy.spin_once(node, timeout_sec=0.5)
if box['m'] is None:
    print('NO /map — is map_server active?')
else:
    d = box['m'].data
    print(f"free={sum(v==0 for v in d)} occ={sum(v==100 for v in d)} unk={sum(v<0 for v in d)}")
rclpy.shutdown()
PY
```

**RViz note:** after load, free/unknown often both look gray (not bright SLAM white). Trust the count, not “white dots.”

### 6C — Localize (get pose + set initialpose)

**Get pose:**

```bash
ros2 topic echo /gazebo/model_states --qos-reliability best_effort --once
```

Copy `burger_2` **x, y** + quaternion (same index as name). Fresh spawn → `0.5, -2.0`, `w: 1.0`.

**Set pose:**

```bash
ros2 topic pub --once /TB3_2/initialpose geometry_msgs/msg/PoseWithCovarianceStamped "{
  header: {frame_id: 'shared_map'},
  pose: {
    pose: {
      position: {x: PUT_X, y: PUT_Y, z: 0.0},
      orientation: {x: PUT_QX, y: PUT_QY, z: PUT_QZ, w: PUT_QW}
    }
  }
}"
```

**Check TF times vs `/clock` (must be close):**

```bash
ros2 topic echo /clock --once
ros2 run tf2_ros tf2_echo shared_map TB3_2/base_footprint
```

Lasers on walls. If TF times are thousands of seconds apart from `/clock` → restart **6A**.

### 6D — Pick point + short goal

**Terminal — wait for click:**

```bash
ros2 topic echo /clicked_point --once
```

RViz → **Publish Point** → click open floor ~0.5–1 m from robot. Copy `point.x`, `point.y`.

**Send goal** (paste those x,y — no typos):

```bash
ros2 action send_goal /TB3_2/navigate_to_pose nav2_msgs/action/NavigateToPose \
"{pose: {
  header: {frame_id: 'shared_map'},
  pose: {
    position: {x: PUT_X, y: PUT_Y, z: 0.0},
    orientation: {w: 1.0}
  }
}}"
```

Watch the Nav2 terminal: **no** `Extrapolation Error` / `Transform data too old`.

| Result | Meaning |
|---|---|
| `SUCCEEDED` (or clear drive on a path) | **Part 5/6 pass** — go to 6E |
| ABORT + TF time errors | Restart **6A**; do not retune map colors |
| ABORT, clocks OK, spinning near obstacle | Shorter freer goal; check inflation |

### 6E — Record + tick

- Note: “R2 localized on `house_shared`, clock OK, NavigateToPose SUCCEEDED at (x, y)”  
- Tick Step 3 boxes in `P2-checklist.md`  
- Update `ref-startup-after-break.md` “Where you left off” → Step 3 done; next Step 4  

---

## Command cheat sheet (copy-paste)

Source every new terminal first (see `ref-startup-after-break.md` §1).

### Bring-up (Step 3 — no SLAM / no T7)

```bash
# T0 Gazebo
mkdir -p ~/swarm-search/launch/tmp_sdf
ros2 launch ~/swarm-search/launch/multi_robot_spawn.launch.py use_sim_time:=true

# map_server
ros2 run nav2_map_server map_server --ros-args \
  -p yaml_filename:=$HOME/swarm-search/maps/house_shared.yaml \
  -p use_sim_time:=true \
  -p frame_id:=shared_map
# other terminal:
ros2 lifecycle set /map_server configure
ros2 lifecycle set /map_server activate

# AMCL + Nav2 (TB3_2) — wait for "Managed nodes are active"
ros2 launch ~/swarm-search/launch/nav2_tb3_localization.launch.py

# RViz
ros2 run rviz2 rviz2 --ros-args -p use_sim_time:=true
```

RViz: Fixed Frame `shared_map` · Map `/map` (Durability **Transient Local**) · Laser `/TB3_2/scan` · Path `/TB3_2/plan`

### Check Gazebo pose (`burger_2` — match by **name**, not list index)

```bash
ros2 topic echo /gazebo/model_states --qos-reliability best_effort --once
```

### Set AMCL initial pose (use live x,y,qx,qy,qz,qw from above)

```bash
ros2 topic pub --once /TB3_2/initialpose geometry_msgs/msg/PoseWithCovarianceStamped "{
  header: {frame_id: 'shared_map'},
  pose: {
    pose: {
      position: {x: PUT_X, y: PUT_Y, z: 0.0},
      orientation: {x: PUT_QX, y: PUT_QY, z: PUT_QZ, w: PUT_QW}
    },
    covariance: [0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
                 0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
                 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                 0.0, 0.0, 0.0, 0.0, 0.0, 0.2]
  }
}"
```

Fresh spawn shortcut (only if robot never moved): `(0.5, -2.0)`, `orientation: {w: 1.0}`.

**Rule:** lasers must sit **tight** on walls before any goal. If lasers drift after teleop → set initialpose again.

### Check TF (must print `At time ...` before goals)

```bash
ros2 topic echo /clock --once
ros2 run tf2_ros tf2_echo shared_map TB3_2/base_footprint
ros2 param get /TB3_2/controller_server use_sim_time
ros2 param get /TB3_2/amcl use_sim_time
ros2 param get /map_server use_sim_time
```

### Pick point in RViz + send goal

```bash
ros2 topic echo /clicked_point --once
# RViz → Publish Point → copy point.x / point.y
```

```bash
ros2 action send_goal /TB3_2/navigate_to_pose nav2_msgs/action/NavigateToPose \
"{pose: {
  header: {frame_id: 'shared_map'},
  pose: {
    position: {x: PUT_X, y: PUT_Y, z: 0.0},
    orientation: {z: PUT_QZ, w: PUT_QW}
  }
}}"
```

Prefer **current yaw** from Gazebo (not always `w: 1.0`). Watch **Path** in RViz: if robot circles while Path is straight → cancel (old bug; should be fixed with RPP + AMCL `/tf` remap).

**Wait** until localization log shows `Managed nodes are active` before `send_goal` (else goal-response timeout).

### Cancel failed / spinning / overshooting nav

```bash
ros2 service call /TB3_2/navigate_to_pose/_action/cancel_goal action_msgs/srv/CancelGoal "{}"
```

### Reset tipped / stuck robot in Gazebo (keep Nav2 running)

```bash
ros2 service call /gazebo/set_entity_state gazebo_msgs/srv/SetEntityState \
"{state: {name: 'burger_2', pose: {position: {x: 0.5, y: -2.0, z: 0.01}, orientation: {w: 1.0}}}}"
```

Then re-set `/TB3_2/initialpose` from a fresh `model_states` echo.

### Restart RViz only (Gazebo/Nav2 stay up)

```bash
pkill -9 rviz2 || true
sleep 1
ros2 run rviz2 rviz2 --ros-args -p use_sim_time:=true
```

### Restart localization only (after param/launch edits)

```bash
pkill -f static_transform_publisher || true
# Ctrl+C the localization terminal, then:
ros2 launch ~/swarm-search/launch/nav2_tb3_localization.launch.py
```

### Teleop TB3_2

```bash
ros2 run turtlebot3_teleop teleop_keyboard --ros-args -r __ns:=/TB3_2
```

### Hide other robots temporarily (optional)

```bash
ros2 service call /gazebo/set_entity_state gazebo_msgs/srv/SetEntityState \
"{state: {name: 'burger_1', pose: {position: {x: 50.0, y: 50.0, z: 0.1}, orientation: {w: 1.0}}}}"
# same for burger_3
```

### Current controller note (as of late Sep 2026)

- `burger_tb3_2_loc.yaml`: **RPP** controller, inflation **0.3**, yaw goal tolerance ~2π  
- `nav2_tb3_localization.launch.py`: AMCL remaps **`/tf`** (required so controller sees `shared_map`)

---

## Suggested build order

| # | Task | Done when |
|---|---|---|
| 1 | Gazebo + map_server + RViz `/map` | House visible |
| 2 | `burger_tb3_2_loc.yaml` (`shared_map`, scans, `/map`) | Params grep OK |
| 3 | Start AMCL + Nav2 for TB3_2 | Nodes up; no SLAM |
| 4 | 2D Pose Estimate | TF + laser aligns |
| 5 | NavigateToPose goal | R2 arrives (**clocks OK first**) |
| 6 | After-break retest (Part 6) + tick Step 3 | Parent todo + startup note updated |

---

## Implement checklist (tick as you go)

### A. Prep

- [x] Step 2 files exist (`maps/house_shared.yaml` / `.pgm`)
- [x] SLAM / old Nav2 / T7 closed (for Step 3 runs)
- [x] T0 Gazebo + RViz up
- [x] map_server active; RViz shows `/map` in `shared_map`

### B. Params / launch

- [x] `nav2_params/burger_tb3_2_loc.yaml` created
- [x] `global_frame_id` / `global_frame` → `shared_map`
- [x] `map_topic: /map`
- [x] Costmap scan → `/TB3_2/scan`
- [x] AMCL + Nav2 running for TB3_2 (launch or manual)
- [x] RPP controller + AMCL `/tf` remaps (circle / no-move fixes)

### C. Localize

- [x] Initial pose from Gazebo `burger_2`
- [x] `tf2_echo shared_map TB3_2/base_footprint` OK
- [x] Laser aligns with walls in RViz

### D. Navigate proof

- [x] Goal sent in `shared_map` (short hops + clicked goals SUCCEEDED with RPP + TF fix)
- [x] R2 reaches goal using saved map (`SUCCEEDED`)
- [ ] Tick Step 3 in `P2-checklist.md` (do after you’re happy with a clean demo)

---

## Pitfalls

| Symptom | Likely cause |
|---|---|
| AMCL / costmap “waiting for map” | map_server not active; `map_topic` is `/TB3_2/map` instead of `/map` |
| TF `shared_map` → base never appears | AMCL not active; no initial pose; SLAM still publishing competing TF |
| Laser in wrong place | Bad initial pose; wrong `global_frame` still `TB3_2/map` |
| Costmap empty / no obstacles | Scan still `/scan` instead of `/TB3_2/scan` |
| “No map received” in RViz | Topic `/map`; Durability Transient Local |
| Two map_servers | Part 1 external + Nav2 also starting map_server — pick one |
| Goal ignored | Wrong goal topic/action namespace (`/TB3_2/...`); BT not active |
| Robot plans in empty space | Goal in unknown/-1; or map not loaded into costmap |
| Fight with SLAM | TB3_2 slam_toolbox still running — kill it |
| `Extrapolation Error` / `Transform data too old` / plan length 0 | **Sim clock split** — restart Gazebo→map_server→Nav2 together; all `use_sim_time:=true` |
| Gray map, “no white floor” in RViz | Normal after load (free≈unknown gray); count `free` on `/map` with Transient Local QoS |
| Map count script hangs forever | Default QoS misses latched `/map` — use Transient Local (Part 6B) |
| Goal ABORT, clocks OK, near object | Inflation / recovery — pick a short freer goal |
| Path straight, robot never moves, `shared_map` does not exist in controller | AMCL was publishing TF under `/TB3_2/tf`; launch now remaps AMCL to `/tf`. Restart localization; `pkill -f static_transform_publisher` |
| Moon-spin / large circle while Path is straight | Old DWB+RotateToGoal; now **RPP** + `use_rotate_to_heading: false` |
| `Failed to send goal response (timeout)` | Sent NavigateToPose before `Managed nodes are active` |
| `/set_entity_state` waits forever | Use **`/gazebo/set_entity_state`** (world plugin namespace) |
| Don’t force `w: 1.0` if lasers already aligned | Overwrites a good AMCL yaw — trust lasers |)

---

## After THIS plan is done — what next

| When | File | Step |
|---|---|---|
| Next | `P2-S4-plan-live-sharing.md` | **Step 4** — live sharing while exploring |
| Later | same | **Step 5** — frontiers (only after Step 3 passes) |
| Daily | `ref-startup-after-break.md` | Bring-up order (update when Step 3 launch is stable) |

**Already finished:** Step 1 (GT shared map), Step 2 (freeze `house_shared`).

---

## How this answers “is R1’s map shared to R2?”

| Layer | Status |
|---|---|
| Combined picture saved | Step 2 ✓ |
| R2 **sees** that picture (`/map`) | Part 1 ✓ |
| R2 **knows where it is** on it (AMCL) | Part 4 (re-do after restart) |
| R2 **drives** using it (Nav2) | Part 5 + **Part 6** after-break retest ← **proof** |
