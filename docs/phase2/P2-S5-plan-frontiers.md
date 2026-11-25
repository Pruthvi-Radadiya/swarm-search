> **📍 Phase 2 › Step 5 (frontiers + explore)** · [← Roadmap](swarm_search_rescue_roadmap_v4.md) · [← P2 todo](P2-checklist.md) · Depends on: Steps 1–4 · Status: ✅ Core done · See also: [Patterns](P2-S5-patterns-wall-claims.md)

# Plan — Step 5: Frontiers on the global map (learn + implement)

**Parent todo:** `docs/P2-checklist.md` → **Step 5** (frontiers + greedy assign + Nav2 goals).  
**Goal:** Robots **automatically** go toward unexplored edges of the **team map** (`/shared_map`) so you do not teleop the whole house by hand.  
**You implement this yourself** (learning path). This file is the explanation + how-to + checklist.

**Depends on:**  

- Steps 1–4 done — one live `/shared_map`, freeze/save known, R2 can Nav on saved map, live share works  
- Full stack bring-up: `ref-startup-after-break.md`  
- Step 3 Nav proof: `P2-S3-plan-r2-nav-on-saved-map.md` (you know NavigateToPose works)

**Related:** `P2-S4-plan-live-sharing.md`, `P2-checklist.md`, `maps/README.md`, **`P2-ref-engineering-notes.md`** (concepts / terms).

---



## Big idea (plain English)

Until now **you** drove robots and watched `/shared_map` grow.

**Frontier** = boundary between **known free** space and **unknown** (−1) on the occupancy grid.  
Those edges are “doors into unexplored rooms.”

```text
/shared_map  →  frontier detector  →  list of frontier points (in shared_map)
                                              │
                                              ▼
                                    greedy assigner
                         (nearest free robot → one frontier)
                                              │
                                              ▼
                         transform goal → that robot’s Nav2 frame
                                              │
                                              ▼
                         NavigateToPose  (/TB3_N/navigate_to_pose)
                                              │
                                              ▼
                         robot drives → SLAM + merge update /shared_map
                                              │
                                              └─► repeat
```

**This is the jump** from “I can share a map” to “the swarm explores to fill the house.”

---



## Important design choice (read before coding)


| Layer                            | Frame / topic today                    |
| -------------------------------- | -------------------------------------- |
| Team map (frontiers)             | `shared_map` / `/shared_map`           |
| Each robot’s Nav2 (Step 4 stack) | `TB3_N/map` (`burger_tb3_N.yaml`)      |
| Step 3 localization Nav          | `shared_map` (`burger_tb3_2_loc.yaml`) |


**Recommended learning path for Step 5:**

1. Detect + visualize frontiers on `/shared_map` first (no Nav2 required).
2. Send goals with `frame_id: shared_map` only if that robot’s Nav2 `global_frame` is `shared_map` (Step 3 style) — **or** transform the goal into `TB3_N/map` when using the normal SLAM+Nav2 stack.

Simplest **first proof (one robot):**

- Run **TB3_1** with normal SLAM + Nav2 (`TB3_1/map`).  
- Frontier in `shared_map` → TF lookup `TB3_1/map` ← `shared_map` (or inverse) → publish goal with `frame_id: TB3_1/map`.  
- Keep T7 merge running so `/shared_map` updates as it explores.

Later you can try “all robots localize on frozen/live shared map” — that is harder; **do not block Step 5 on it.**

---



## What “done” looks like (Step 5)

1. Node(s) subscribe to `/shared_map` and publish frontier markers you can see in RViz (`shared_map` Fixed Frame).
2. At least **one** robot receives a frontier goal and **NavigateToPose** drives toward unknown space.
3. **Greedy multi-robot:** each free robot gets a different nearby frontier (no two assigned the same centroid).
4. Optional: log coverage vs time — **deferred** (Part 6; whenever you want metrics).
5. Comms radius / ablation stays **later** (same as Step 4 Part 4 deferred).
6. Package launch: `ros2 launch explore_swarm explore.launch.py` — **done** (Part 7).

---



## Terminals — Step 5 stack

Same as Step 4 live share, **plus** your frontier node, **plus** Nav2 for robots that will auto-drive.


| Terminal  | Role                                                    |
| --------- | ------------------------------------------------------- |
| **T0**    | Gazebo + 3 robots (`TURTLEBOT3_MODEL=burger`)           |
| **T1–T3** | SLAM                                                    |
| **T4–T6** | Nav2 (`burger_tb3_N.yaml`) — needed once you send goals |
| **T7**    | `map_merge.launch.py` → `/shared_map`                   |
| **T8**    | RViz: Fixed Frame `shared_map`, Map `/shared_map`       |
| **T9**    | `ros2 launch explore_swarm explore.launch.py`           |
| Teleop    | Optional fallback / unstick                             |


**Do not** run Step 3 map_server + AMCL on the same robot as SLAM.

---



## CRITICAL — every new terminal

```bash
source /opt/ros/humble/setup.bash
source ~/swarm-search/install/setup.bash
export TURTLEBOT3_MODEL=burger
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/humble/share/turtlebot3_gazebo/models
cd ~/swarm-search
```

After creating a new package / editing Python:

```bash
cd ~/swarm-search
colcon build --packages-select YOUR_PKG   # e.g. explore_swarm
source install/setup.bash
```

---



## Part 1 — Bring-up (live map ready)

**Why:** Frontiers on an empty or broken `/shared_map` teach nothing.

1. Kill old Gazebo if needed (`ref-startup-after-break.md` §2).
2. T0 → T1–T3 → T7 → T8.
3. Confirm:

```bash
ros2 topic echo /gazebo/model_states --qos-reliability best_effort --once | grep -E 'burger_|waffle_'
ros2 run tf2_ros tf2_echo shared_map TB3_1/map
ros2 topic hz /shared_map
```

Expect `burger_*`, working TF, `/shared_map` ~1 Hz.  
4. Teleop briefly so `/shared_map` has some free + unknown (not all −1).

**Done when:** one house (or partial house) visible on `/shared_map` in RViz.

---



## Part 2 — What a frontier is (algorithm sketch)

On the OccupancyGrid (`/shared_map`):


| Cell value | Meaning         |
| ---------- | --------------- |
| `0`        | free            |
| `100`      | occupied (wall) |
| `-1`       | unknown         |


A cell is a **frontier cell** if it is **free (**`0`**)** and **at least one 4- or 8-neighbor is unknown (**`-1`**)**.

Then:

1. Collect frontier cells (or cluster them).
2. Cluster nearby cells (e.g. BFS grouping, or simple grid bins).
3. Drop tiny clusters (noise).
4. For each cluster, compute a **centroid** (or nearest free cell toward unknown) → candidate goal in **meters** in `shared_map`.

**Learning stretch:** start with publishing **all** frontier cell centers as markers; then add clustering so RViz is readable.

**QoS:** `/shared_map` is **Transient Local** + Reliable (same as merge). Match that on your subscriber or you may never get a map.

---



## Part 3 — Implement frontier detector (visualize first)

**Why:** See frontiers before trusting Nav2 goals.

Suggested package name: `explore_swarm` (new) **or** a node under `map_merge_swarm` if you prefer one package. New package keeps merge code clean.

**Outputs (pick one or both):**


| Topic               | Type                             | Use                               |
| ------------------- | -------------------------------- | --------------------------------- |
| `/frontiers`        | `geometry_msgs/PoseArray`        | centroids, `frame_id: shared_map` |
| `/frontier_markers` | `visualization_msgs/MarkerArray` | colored spheres/points in RViz    |


**Node loop:**

1. Subscribe `/shared_map`.
2. Find clusters → centroids.
3. Publish markers / PoseArray.
4. Timer or “on every map” is fine (~1 Hz map already).

**RViz:**

- Fixed Frame: `shared_map`  
- Map: `/shared_map`  
- Add **MarkerArray** → `/frontier_markers` (or PoseArray display)

**Done when:** after partial explore, you see dots/spheres along free↔unknown edges — not random noise over walls.

---



## Part 4 — One-robot goal proof

**Why:** Prove “frontier → Nav2 motion” before multi-robot logic.

1. Start **T4** Nav2 for TB3_1 (wait until SLAM TF exists).
2. Pick the frontier centroid **nearest to TB3_1**.
3. Get robot pose in `shared_map` (TF: `shared_map` → `TB3_1/base_footprint`).
4. Transform goal into Nav2 global frame:

```text
Goal in shared_map  →  (TF)  →  goal in TB3_1/map
```

1. Send action goal:

```bash
# Manual smoke test shape (replace coords / use your node later)
ros2 action send_goal /TB3_1/navigate_to_pose nav2_msgs/action/NavigateToPose "{
  pose: {
    header: {frame_id: 'TB3_1/map'},
    pose: {position: {x: PUT_X, y: PUT_Y, z: 0.0}, orientation: {w: 1.0}}
  }
}"
```

Or from your assigner node with `ActionClient`.

**Safety:**

- Goal should sit in **free** space slightly **inside** known free, not deep in unknown (Nav2 often fails if start/goal invalid). Common trick: place goal at frontier centroid, then **pull back** 0.3–0.5 m toward free space / robot.  
- Cancel stuck goals:  
`ros2 service call /TB3_1/navigate_to_pose/_action/cancel_goal action_msgs/srv/CancelGoal "{}"`

**Done when:** TB3_1 moves toward unknown; `/shared_map` grows in that direction.

---



## Part 5 — Greedy assign (multi-robot)

**Why:** Match the parent todo: nearest free robot → frontier; no pile-up on one point.

**Simple greedy (good enough for v1):**

1. List frontier centroids `F`.
2. List robots that are **idle** (no active NavigateToPose / or last goal succeeded/aborted).
3. For each idle robot, compute distance in `shared_map` to each unassigned frontier.
4. Assign **closest (robot, frontier)** pair; remove that frontier; repeat.
5. Send goals (with frame transform per robot).
6. Re-run assigner every few seconds or when a robot becomes idle / map updates a lot.

**Rules of thumb:**

- Min frontier cluster size (e.g. ≥ 10–20 cells).  
- Max distance (skip unreachable far frontiers).  
- Do not re-assign the same frontier to two robots.  
- Optional: blacklist frontiers that repeatedly ABORT.

**Done when:** two+ robots leave spawn and head to **different** frontier regions without you teleoping.

---



## Part 6 — Optional coverage vs time

**Status: deferred** — Parts 1–5 + Part 7 done. Implement when you want a metric. Comms on/off ablation stays even later.

**Why (when you return):** Parent todo asks for a simple metric before fancy papers.

Ideas (pick one):

- Count cells: `unknown`, `free`, `occupied` on `/shared_map` every 5 s → CSV.  
- Plot later: unknown ↓ over wall-clock or sim time.  
- Stop when unknown change is tiny for N minutes (optional).

Comms on/off ablation = **later** (filter which robots contribute to merge / which frontiers are shared).

---



## Part 7 — Package / launch — **done**

**Status: done.** Nodes: `frontier_detector` + `goal_assigner`. Launch installs from package `launch/`.

```text
src/explore_swarm/
  explore_swarm/
    frontier_detector.py
    goal_assigner.py
  launch/
    explore.launch.py
  setup.py          # data_files → share/.../launch
  package.xml       # exec_depend launch, launch_ros; tf2_* depends
```

```bash
colcon build --packages-select explore_swarm
source install/setup.bash
ros2 launch explore_swarm explore.launch.py
```

`clusters_kept = 0` with a fresh/empty `/shared_map` means the detector is alive — not a crash. Optional later params: `map_topic`, `min_cluster_size`, `goal_backoff_m`, `robot_namespaces`.

---



## Command cheat sheet (Step 5)



### Env (every terminal)

```bash
source /opt/ros/humble/setup.bash
source ~/swarm-search/install/setup.bash
export TURTLEBOT3_MODEL=burger
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/humble/share/turtlebot3_gazebo/models
cd ~/swarm-search
```



### Stack (same as Step 4)

See `ref-startup-after-break.md` §3 — T0, T1–T3, T4–T6, T7, T8.

### Health

```bash
ros2 topic echo /gazebo/model_states --qos-reliability best_effort --once | grep burger_
ros2 run tf2_ros tf2_echo shared_map TB3_1/map
ros2 topic hz /shared_map
```



### Explore package (T9)

```bash
colcon build --packages-select explore_swarm
source install/setup.bash
ros2 launch explore_swarm explore.launch.py
# or: ros2 run explore_swarm frontier_detector
#     ros2 run explore_swarm goal_assigner
```



### Cancel Nav2 goal (example TB3_1)

```bash
ros2 service call /TB3_1/navigate_to_pose/_action/cancel_goal action_msgs/srv/CancelGoal "{}"
```



### Mid-session save (optional, from Step 4)

```bash
ros2 run nav2_map_server map_saver_cli \
  -t /shared_map \
  -f ~/swarm-search/maps/house_shared_live \
  --ros-args \
  -p use_sim_time:=true \
  -p map_subscribe_transient_local:=true \
  -p save_map_timeout:=15.0
```

---



## Suggested build order


| #   | Task                                   | Done when                      |
| --- | -------------------------------------- | ------------------------------ |
| 1   | Bring-up Part 1                        | `/shared_map` + TF OK — **done** |
| 2   | Frontier detector + RViz markers       | Edges visible — **done**         |
| 3   | One-robot NavigateToPose to a frontier | Robot moves; map grows — **done** |
| 4   | Greedy multi-robot assign              | Different frontiers — **done**   |
| 5   | Optional coverage log (Part 6)         | **Deferred**                     |
| 6   | Package / launch (Part 7)              | `explore.launch.py` — **done**   |
| 7   | Tick Step 5 in parent todo             | **done** (Part 6 optional later) |


---



## Implement checklist (tick as you go)



### A. Prep

- [x] T0 + SLAM + T7 + RViz; `burger_*` models  
- [x] `shared_map` TF works for robots you will drive  
- [x] Partial map with free + unknown visible  



### B. Detect + visualize

- [x] Subscriber matches Transient Local on `/shared_map`  
- [x] Frontier cells / clusters computed  
- [x] Markers or PoseArray in RViz look like free↔unknown edges  



### C. One-robot nav

- [x] Nav2 up for that robot (`TB3_N/map`)  
- [x] Goal transformed into Nav2 global frame  
- [x] Goal backoff into free space  
- [x] NavigateToPose SUCCEEDED or clearly progresses into unknown  



### D. Multi-robot greedy

- [x] Idle detection  
- [x] Unique frontier per robot  
- [x] At least two robots auto-assigned  



### E. Optional / later

- [ ] Coverage vs time log (**Part 6 — optional**; one `/shared_map` curve)  
- [ ] Comms radius (**skipped for now**; future)  



### F. Docs

- [x] Tick Step 5 in `P2-checklist.md`  
- [x] Update `ref-startup-after-break.md` (T9 launch + §6 Step 3)  

---



## Pitfalls


| Symptom                          | Likely cause                                                             |
| -------------------------------- | ------------------------------------------------------------------------ |
| No frontiers                     | Map all unknown or all known; thresholds; not subscribed (QoS)           |
| Markers in wrong place           | Forgot `frame_id: shared_map`; Fixed Frame wrong                         |
| Nav2 rejects / ABORT immediately | Goal in unknown/occupied; wrong `frame_id`; inflation; backoff missing   |
| Robot plans opposite way         | Goal still in `shared_map` but costmap is `TB3_N/map` without TF convert |
| Two robots same corner           | Assigner not removing claimed frontiers                                  |
| `TB3_N: Nav2 not ready`          | That robot’s Nav2 not running; check `ros2 action list \| grep navigate` |
| Tip → fake wall in RViz          | Lidar tipped; SLAM ghost cells in `/shared_map` (see engineering notes)  |
| Robots scrape each other         | No inter-robot costmap (v1 expected)                                     |
| Last blob: one robot stops       | Greedy: only one frontier left                                           |
| `shared_map` frame missing       | GT/burger issue — see Step 4 (`waffle_*` bug)                            |
| SLAM + AMCL fight                | Mixed Step 3 localization with Step 5 SLAM stack on same robot           |
| Costmap timeout at Nav2 start    | Started Nav2 before that robot’s SLAM TF existed                         |


---



## After THIS plan is done — what next


| When              | Focus                                                               |
| ----------------- | ------------------------------------------------------------------- |
| Done (Parts 1–5,7)| Multi-robot explore + `explore.launch.py`                           |
| Optional          | Part 6 — coverage vs time (one `/shared_map` curve)                 |
| Skipped for now   | Comms radius filter + on/off ablation (future)                      |
| Later             | Better assign (auction, information gain) — not required for Step 5 |
| Out of scope here | YOLO / aerial / MARL (`P2-checklist.md`)                     |


---



## How this answers “robots reconstruct the whole house together”


| Layer                          | Status after Step 5                                           |
| ------------------------------ | ------------------------------------------------------------- |
| One team map                   | Steps 1–4 ✓                                                   |
| See where to explore next      | Frontiers on `/shared_map`                                    |
| Robots go there without teleop | Greedy + NavigateToPose                                       |
| Full house                     | Emerges as unknown shrinks (you may still nudge stuck robots) |


