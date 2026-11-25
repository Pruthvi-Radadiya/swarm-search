> **📍 Phase 2 (theory reference)** · [← Roadmap](swarm_search_rescue_roadmap_v4.md) · Covers: grids + TF → [S1](P2-S1-plan-gt-shared-map.md) · merge math → [S1](P2-S1-plan-gt-shared-map.md) · localization → [S3](P2-S3-plan-r2-nav-on-saved-map.md) · frontiers + assigner → [S5 plan](P2-S5-plan-frontiers.md) / [S5 patterns](P2-S5-patterns-wall-claims.md) · costmaps → [S5 patterns](P2-S5-patterns-wall-claims.md) · See also: [Study guide](P2-ref-study-guide.md)

# Engineering notes — swarm map & explore

Deep reference: algorithms, formulas, and design choices (not a todo / not a command cheat sheet).  
**Commands / bring-up:** `ref-startup-after-break.md`, `plan-*.md`.  
**Package wiring:** `ref-ros2-pkg-basics.md`.  
**Code:** `map_merge_swarm`, `explore_swarm`.

---

## Occupancy grid basics

Each cell in an `OccupancyGrid` is an `int8`:

| Value | Meaning |
|------:|---------|
| `0` | free (driveable / known empty) |
| `100` | occupied (wall / obstacle) |
| `-1` | unknown (not sensed yet) |

**Index ↔ world (cell center):**

```text
index = j * width + i          # row-major; i = column, j = row

x = origin_x + (i + 0.5) * resolution
y = origin_y + (j + 0.5) * resolution
```

`info.origin` is the pose of the **lower-left** corner of the grid in the map frame.  
Our shared canvas: **400×400**, `resolution = 0.05` m, `origin = (−10, −10)` → covers ~20 m × 20 m.

**QoS for maps:** RELIABLE + **TRANSIENT_LOCAL** + KEEP_LAST depth 1. Late subscribers still get the last map. Wrong QoS → “subscribed but never receives.”

---

## Two operating modes (never mix on one robot)

| Mode | Map source | Mapping | Localization | Nav2 `global_frame` |
|------|------------|---------|--------------|---------------------|
| **Live share** (Steps 1, 4, 5) | merge → `/shared_map` | SLAM on | SLAM TF | usually `TB3_N/map` |
| **Frozen Nav** (Step 3) | `map_server` → `/map` from `.yaml`/`.pgm` | SLAM **off** | **AMCL** | `shared_map` |

**Why fight:** SLAM and AMCL both try to publish `map → odom`. Only one may own that.

---

## TF tree (what each frame is)

```text
shared_map                    # team / Gazebo world sheet (Option C)
  └─ TB3_N/map                # that robot’s SLAM map origin
       └─ TB3_N/odom
            └─ TB3_N/base_footprint
```

- **Topic** `/shared_map` = occupancy picture with `header.frame_id = shared_map`.  
- **TF** `shared_map → TB3_N/map` = where SLAM’s notebook sits on the team sheet.  
Topic can publish even if TF is broken (empty/wrong paint).

**lookup convention:** `lookup_transform(target, source, time)` returns a transform that takes data **from `source` into `target`**.

Examples we use:

```text
shared_map ← TB3_1/base_footprint   # robot pose on team map
TB3_1/map  ← shared_map             # goal from team map into Nav2 frame
```

**Gazebo names:** `TURTLEBOT3_MODEL=burger` → models `burger_1/2/3`. GT node hardcodes those. `waffle_*` → no `shared_map` TF.

---

## Ground-truth map TF (Option C) — idea

We want: `shared_map` ≈ Gazebo world, and SLAM’s `TB3_N/map` placed so walls stack.

**Known:**

- From Gazebo: `shared_T_base` (robot in world)  
- From TF: `map_T_base` (robot in its SLAM map)

**Wanted:** `shared_T_map` (publish as TF parent `shared_map`, child `TB3_N/map`).

On SE(2) (x, y, yaw):

```text
shared_T_map = shared_T_base ∘ inverse(map_T_base)
```

In code (`gt_map_tf_node.py`): `se2_inverse` then `se2_compose`.  
Yaw from quaternion (2D): `atan2(2(wz + xy), 1 − 2(y² + z²))`.

`/gazebo/model_states` is often **BEST_EFFORT** — subscribe with matching QoS.

---

## Map merge — paint + fuse

**Canvas:** fixed shared OccupancyGrid; each robot’s map transformed into shared indices via TF, then fused.

**Fuse rule (v1, per cell):**

```text
if either is 100 → 100          # occupied wins
else if either is 0 → 0         # free if anyone saw free
else → -1                       # still unknown
```

**Publish rate:** timer ~1 Hz → `/shared_map`.

**Save:** `map_saver_cli -t /shared_map`. Use `map_subscribe_transient_local:=true` and `save_map_timeout:=15.0` (default ~2 s fails at ~1 Hz).

---

## Frontiers — detection

### Definition

Cell `(i,j)` is a **frontier cell** iff:

1. `data[j*width+i] == 0` (free), and  
2. at least one **4-neighbor** `(i±1,j)` or `(i,j±1)` is `-1` (unknown).

That is the **boundary of known free against unknown** — not the middle of a grey unknown blob (those cells are still `-1`, so they are not frontiers).

### Clustering (BFS / flood fill)

1. Collect all frontier cells as a set.  
2. For each unvisited cell, BFS over 4-neighbors that are also frontier cells → one **cluster**.  
3. Drop clusters smaller than `MIN_CLUSTER_SIZE` (we used ~5–15) to kill noise.  
4. **Centroid** of a cluster: average of all `(i,j)`, then convert to meters with the grid formula above → one goal candidate in `shared_map`.

### Outputs

| Topic | Type | Role |
|-------|------|------|
| `/frontiers` | `PoseArray` | centroids for code (`frame_id: shared_map`) |
| `/frontiers_markers` | `MarkerArray` | spheres for RViz; often send `DELETEALL` then ADD |

Same computation can feed both.

---

## Goal assigner — algorithms

### One-robot pipeline (v1)

```text
/frontiers
  → robot (rx,ry) via TF shared_map ← base_footprint
  → nearest frontier with dist ≥ MIN_GOAL_DIST
  → backoff toward robot
  → reject if still too close after backoff
  → transform Pose into TB3_N/map
  → NavigateToPose (if not busy / cooldown)
```

### Multi-robot greedy (Part 5)

Same math, but over **idle robots × unclaimed frontiers**:

```text
idle = robots with not busy and now ≥ cooldown
frontiers = list of centroids (mutable)

while idle and frontiers:
  pick (robot i, frontier j) with smallest d² among pairs with d ≥ MIN_GOAL_DIST
  pop both → backoff → TF → wait_for_server → NavigateToPose
  on TF / Nav2-not-ready fail: put both back (or skip that robot)
```

**Musts:**

- One `ActionClient` **per** namespace (`_nav_clients[ns]`), not a single client overwritten in a loop.  
- `wait_for_server(timeout_sec=0.0)` before send — missing Nav2 → “TB3_N: Nav2 not ready” spam.  
- Claimed frontier removed from the list so two robots never get the same centroid in one pass.  
- Last blob: only **one** idle robot should keep the goal; the other staying put is correct for greedy.

**Not in v1:** robot–robot collision / keep-out. Nav2 costmaps see static obstacles only — other burgers are invisible. Paths can cross; robots can bump.

### Euclidean distance (and squared)

```text
d² = (px − rx)² + (py − ry)²
d  = √(d²)
```

To find the **minimum**, compare **d²** only (no √ needed). Use √ only for logging / thresholds in meters.

### Nearest frontier with minimum range

```text
MIN_GOAL_DIST = 1.0 m   # (was 0.6; raise if instant SUCCEEDED)

for each frontier pose:
  if d² < MIN_GOAL_DIST²: skip
  else keep the one with smallest d²
```

If none remain → do not send a goal (“no frontier far enough”).

**Why:** Nav2 succeeds immediately if the goal is already inside `xy_goal_tolerance` of the robot (~0.2 m). Near frontiers + backoff often create that.

### Backoff (pull goal into free space)

Frontier sits on free↔unknown. Nav2 often rejects / fails goals in unknown. Move the goal a short step **from the frontier toward the robot** (into known free).

```text
vx = rx − gx          # vector frontier → robot
vy = ry − gy
length = √(vx² + vy²)

if length < 1e-3: keep (gx, gy)     # don’t divide by zero

ux, uy = vx/length, vy/length       # unit direction

# Cap step so we never walk past the robot:
step = min(0.4, max(0.0, length − 0.1))
if step < 0.05: keep (gx, gy)

gx2 = gx + step * ux
gy2 = gy + step * uy
```

**Bug we hit:** fixed backoff `0.4` while `length = 0.30` → goal past the robot → instant SUCCEEDED spam.

**Second check after backoff:**

```text
if √((gx2−rx)² + (gy2−ry)²) < 0.5: do not send
```

### Frame change for Nav2

Live stack Nav2 uses `global_frame: TB3_N/map`, not `shared_map`.

```text
Pose in shared_map (gx2, gy2)
  → lookup_transform(TB3_N/map, shared_map, …)
  → do_transform_pose(pose, tf)   # pass Pose, not PoseStamped
```

Numbers in `shared_map` and `TB3_N/map` can look **almost equal** when GT alignment makes the frames nearly coincident — still two different frames.

### NavigateToPose + busy + cooldown

- Action: `/TB3_N/navigate_to_pose` (`nav2_msgs/action/NavigateToPose`).  
- Goal: `PoseStamped` with `frame_id: TB3_N/map`.  
- **`_busy = True`** before `send_goal_async`; clear on reject or result.  
  Without busy: `/frontiers` ~1 Hz → new goal every second while navigating.  
- **Cooldown** after finish (e.g. 3 s) reduces spam when many frontiers are “close enough to succeed.”  
- Result **status 4** ≈ SUCCEEDED (`action_msgs`). Instant 4 (~50 ms) ⇒ goal was already at the robot.

Optional later: **blacklist** last goal region (ignore frontiers within ~0.8 m of last success).

---

## Controllers & costmaps (driving quality)

| Setup | Behavior |
|-------|----------|
| **DWB** + RotateToGoal / **RotationShim** | Large-radius circles (“moon spin”), wall hits, tip-overs on TB3 |
| **Regulated Pure Pursuit (RPP)** | Follows path carrot; better for this sim |

Useful RPP-related choices we settled on:

- `use_rotate_to_heading: false`  
- `yaw_goal_tolerance: 6.28` (~2π) → stop on XY, don’t insist on final yaw  
- `inflation_radius: 0.3` (was 0.5; too fat in narrow house → bad plans / stuck)

Applied in `burger_tb3_1/2/3.yaml` and `burger_tb3_2_loc.yaml`.

**Still can tip:** center pillar / sharp contact. Burger is tippy even with RPP. After tip: pitch/roll ≠ 0 → lidar points at ceiling/floor → SLAM paints **ghost walls** on `/shared_map` where nothing exists. Fix: reset that robot / restart SLAM for it; ghost cells stay until overwritten or you restart the merge session.

---

## Coverage vs time (Part 6 — optional)

**You only need one map:** subscribe to **`/shared_map`** (the team OccupancyGrid). You are not graphing three maps — the merge already fused the robots into that one grid.

**Idea:** as robots explore, unknown cells (−1) become free (0) or occupied (100). “Coverage” ≈ how much of the canvas is **known**.

Every ~5 s, count cells on that one message:

```text
unknown  = count(data == -1)
free     = count(data == 0)
occupied = count(data == 100)
N        = width * height
known_frac = (free + occupied) / N     # 0 → 1 as the house fills in
```

Log rows like `(sim_time, unknown, free, occupied, known_frac)` → CSV.

**Graph:** X = time, Y = `known_frac` (rising) **or** `unknown` (falling). One curve for one explore run. That is the “coverage vs time” plot.

(Comms on/off would be two runs → two curves — **skipped for now**; same logger if you ever add it.)

---

## Localization (Step 3) — short

- **AMCL:** particle filter; needs initial pose (`/TB3_N/initialpose`) + map + scan.  
- Publishes `shared_map → TB3_N/odom` when `global_frame_id: shared_map`.  
- Remap AMCL TF to global `/tf` (not only `/TB3_N/tf`) or controller never sees `shared_map`.  
- `map_topic: /map` (absolute) when map_server publishes globally.

---

## Packages & nodes

| Package | Node | Job |
|---------|------|-----|
| `map_merge_swarm` | `gt_map_tf_node` | broadcast `shared_map → TB3_N/map` |
| `map_merge_swarm` | `merge_node` | fuse → `/shared_map` |
| `explore_swarm` | `frontier_detector` | frontiers + markers |
| `explore_swarm` | `goal_assigner` | nearest + backoff + Nav2 |
| `explore_swarm` | launch `explore.launch.py` | both nodes + `use_sim_time` |

```bash
ros2 launch explore_swarm explore.launch.py
```

`clusters_kept = 0` ⇒ node alive, no frontier clusters yet (need free↔unknown on `/shared_map`).

Build: `colcon build --packages-select …` then `source install/setup.bash`.  
`--symlink-install` helps while editing Python. Always rebuild after changing `setup.py` entry points or `launch/`.

---

## Failure patterns (quick)

| Symptom | Likely cause |
|---------|----------------|
| No `shared_map` TF | waffle models / GT not running / SLAM TF missing |
| Frontiers = 0 | map all unknown or all known; or no free↔unknown edge |
| Instant status 4 loop | goal too close; uncapped backoff; no busy/cooldown |
| Circles / tip | DWB / RotationShim — switch to RPP |
| Ghost wall after tip | tipped lidar → bad SLAM cells in merge |
| `TB3_N: Nav2 not ready` | that Nav2 not launched / still starting; check `ros2 action list \| grep navigate` |
| Two robots scrape | no inter-robot costmap layer (expected in v1) |
| Last frontier: one stops | greedy — only one gets the blob |
| `do_transform_pose` AttributeError | passed PoseStamped; pass `.pose` |
| map_saver “Failed to spin” | timeout too short / QoS |
| RViz no map | Fixed Frame + topic + Transient Local |
| Multi Map displays / ghosts | one `/shared_map` Map display; Fixed Frame `shared_map` |
| Doorway collision / spin scrape | inflation radius 0.45m overlaps across 0.75m door; reduce to 0.25m, cost scaling 8.0 |
| `transformPose` 1ms extrapolation | `transform_tolerance: 1.0` missing under `controller_server.ros__parameters` |
| Planner aborts in doorway | NavFn Dijkstra fails potential; enable `use_astar: true` in planner |
| Robot 1 starved in starting room | `CLAIM_RADIUS: 2.5m` too large; teammates claim whole house; reduce to 1.0m |
| Endless corner frontier loop (trash can) | LIDAR occlusion shadow; raise `MIN_CLUSTER_SIZE = 12` + 300s visited cooldown |

---

## Status snapshot (Oct 2026)

- Steps 1–5 done (live share + freeze + R2 on saved map + multi-robot explore + launch).  
- Comms radius **skipped for now** (future). Part 6 coverage metric optional.  
- **Next big (roadmap):** Phase 3 aerial when ready.
