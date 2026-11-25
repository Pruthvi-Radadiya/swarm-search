> **📍 Phase 2 › Step 4 (live map sharing)** · [← Roadmap](swarm_search_rescue_roadmap_v4.md) · [← P2 todo](P2-checklist.md) · Depends on: Steps [1](P2-S1-plan-gt-shared-map.md)–[3](P2-S3-plan-r2-nav-on-saved-map.md) · Status: ✅ Done

# Plan — Step 4: Live sharing while exploring together (learn + implement)

**Parent todo:** `docs/P2-checklist.md` → **Step 4** (live `/shared_map` during multi-robot runs).  
**Goal:** While all robots **map with SLAM** and drive, the team keeps **one live house** on `/shared_map` that updates as they explore — without shutting merge down or falling back to three side-by-side islands.  
**You implement this yourself** (learning path). This file is the explanation + how-to + checklist.

**Depends on:**  
- Step 1 Option C (`P2-S1-plan-gt-shared-map.md`) — GT alignment, one house  
- Step 2 freeze (`P2-S2-plan-freeze-team-map.md`) — you know how to snapshot when needed  
- Step 3 proof (`P2-S3-plan-r2-nav-on-saved-map.md`) — R2 can navigate on the **saved** team map (separate mode)

**Related:** `ref-startup-after-break.md` (full stack T0–T8), `maps/README.md`, `README.md`, `P2-checklist.md`.

---

## Big idea (plain English)

| Mode | What the team sees | What each robot’s Nav2 uses |
|---|---|---|
| **Step 3** | Frozen `house_shared` via map_server → `/map` | R2: AMCL + Nav2 on **`shared_map`** (SLAM off for that robot) |
| **Step 4 (this step)** | **Live** merge → **`/shared_map`** updates every second | Each robot still uses **`TB3_N/map`** from its own SLAM (today’s `burger_tb3_N.yaml`) |

Step 4 is **not** “every robot Nav2 on live `/shared_map`” yet. It is: **run the full exploration stack** and prove the **shared picture stays one coherent house** while robots move, plus optional **periodic save** of that live picture. Nav on the live global map can come later (frontiers / Step 5 prep).

```text
Gazebo (world = shared_map)
    │
    ├── SLAM TB3_1 ──► /TB3_1/map ──┐
    ├── SLAM TB3_2 ──► /TB3_2/map ──┼──► gt_map_tf_node (shared_map ← TB3_N/map)
    └── SLAM TB3_3 ──► /TB3_3/map ──┘              │
                                                    ▼
                                            map_merge_node
                                                    │
                                                    ▼
                                            /shared_map  (RViz + optional map_saver)
```

**Do not mix Step 3 and Step 4 on the same robot without thinking:**  
Running **SLAM + AMCL on the same TB3_2** both publishing map→odom will fight. For Step 4, use **SLAM + `burger_tb3_2.yaml` Nav2**. For Step 3 retests, use **no SLAM on R2** + `nav2_tb3_localization.launch.py` + map_server.

---

## What “done” looks like (Step 4)

1. T0 + T1–T3 + **T7 merge** + RViz: Fixed Frame `shared_map`, Map topic **`/shared_map`**, **one** house outline (walls stack, not three copies).  
2. Teleop or Nav2 at least one robot into **new** space; **`/shared_map` fills in** where that robot’s SLAM sees walls (other robots’ areas still visible).  
3. Optional: run **`map_saver_cli`** once mid-session (or on a timer you design) → new snapshot under `maps/` without breaking the live merge.  
4. Optional design note: **comms radius** — which robot maps would merge if only neighbors within `R` could contribute (**skipped for now**; future if useful).

---

## Terminals — Step 4 stack (keep open)

Same order as `ref-startup-after-break.md` §3. **Do not** run Step 3 map_server or `nav2_tb3_localization` during a pure Step 4 session.

| Terminal | Role | Close for Step 3-only? |
|---|---|---|
| **T0** | Gazebo + 3 robots | Keep |
| **T1–T3** | SLAM per robot | Keep (Step 4) / **Close** (Step 3 on R2 only) |
| **T4–T6** | Nav2 per robot (`burger_tb3_N.yaml`) | Optional but useful to drive while exploring |
| **T7** | `map_merge.launch.py` → `/shared_map` | **Keep** (Step 4) / **Close** (Step 3 frozen map) |
| **T8** | RViz | Keep |
| Teleop | One namespaced robot | Optional |

**Never together:** `map_merge.launch.py` **and** old standalone `static_map_tfs` (duplicate TFs). T7 already starts `gt_map_tf_node` + `merge_node`.

---

## CRITICAL — every new terminal

```bash
source /opt/ros/humble/setup.bash
source ~/swarm-search/install/setup.bash
export TURTLEBOT3_MODEL=burger
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/humble/share/turtlebot3_gazebo/models
cd ~/swarm-search
```

After edits under `src/map_merge_swarm/`:

```bash
cd ~/swarm-search
colcon build --packages-select map_merge_swarm
source install/setup.bash
```

---

## Part 1 — Bring-up checklist (live merge only)

**Why:** Confirm the **team map** pipeline before worrying about saves or comms.

1. §2 kill old Gazebo if needed.  
2. Start **T0**; wait for spawns (`burger_1`, `burger_2`, `burger_3`).  
3. Start **T1, T2, T3** SLAM (all three, or at least two overlapping).  
4. Start **T7**:

```bash
ros2 launch map_merge_swarm map_merge.launch.py
```

5. **T8** RViz: `use_sim_time:=true`, Fixed Frame **`shared_map`**, Map → **`/shared_map`**, Durability **Transient Local**.  
6. Quick health (another terminal):

```bash
ros2 topic list | grep -E 'TB3_.*/map|/shared_map|/gazebo/model_states'
ros2 topic hz /shared_map
```

**Done when:** `/shared_map` publishes ~1 Hz; RViz shows one house (not three separated blobs). If not, revisit Step 1 / `P2-S1-plan-gt-shared-map.md` (GT node must see `model_states`).

---

## Part 2 — Explore together (proof the map “lives”)

**Why:** Step 4 is about **updates**, not a static file.

1. Start **T4** (or teleop **TB3_1**) and drive into a corridor or room **one robot** has not fully scanned yet.  
2. Watch **`/TB3_1/map`** (or that robot’s map) grow in its frame; watch **`/shared_map`** gain matching walls in the **same** world location.  
3. Repeat with **another** robot from a different spawn — merged walls should **overlap**, not slide sideways.  
4. Optional: add LaserScan displays `/TB3_N/scan` in RViz to see each robot in **`shared_map`**.

**Done when:** you can describe “R1 opened this wing; `/shared_map` updated while R2 was idle” (or the reverse).

---

## Part 3 — Optional periodic / mid-session save

**Why:** Step 2 was a one-off snapshot. Step 4 adds **“save while sim runs”** without stopping merge.

Manual save (same as `maps/README.md`, topic is **`/shared_map`** not `/map`):

```bash
ros2 run nav2_map_server map_saver_cli \
  -t /shared_map \
  -f ~/swarm-search/maps/house_shared_live_$(date +%Y%m%d_%H%M) \
  --ros-args -p use_sim_time:=true
```

**Learning stretch (you implement):**

- A small node or shell loop: every **N minutes** call `map_saver_cli` with a new filename, **or** overwrite only if you accept losing the previous backup.  
- Document chosen policy in `maps/README.md` (one line: “live session saves use prefix …”).

**Pitfall:** Saving does **not** stop SLAM or merge; it only writes disk. Reloading that file is Step 2/3 smoke test (`map_server` + `/map`), not Step 4.

**Done when:** at least one extra `.yaml`/`.pgm` pair saved **while** T7 still running and RViz still showing live `/shared_map`.

---

## Part 4 — Comms radius (design stub only)

**Why:** Real swarms do not broadcast full maps globally. Parent todo asks for a **stub** now, full logic later.

Write **half a page** (in this repo or your notes) answering:

| Question | Your stub answer |
|---|---|
| Who sends what? | e.g. each robot publishes local `/TB3_N/map` or patches; merge node fuses |
| Who listens? | merge node only vs peer-to-peer |
| Radius rule | Only fuse robot **j** if `‖pose_i − pose_j‖ < R` in **`shared_map`** |
| Where is pose from? | Gazebo GT (sim) vs AMCL later |
| What to skip when far? | Ignore that robot’s map callback for merge (filter in merge node or don’t subscribe) |

**Done when:** stub written; **no code required** for Step 4 checkbox unless you want extra credit.

---

## Part 5 — Nav2 during Step 4 (what to expect)

Per-robot Nav2 (`nav2_tb3.launch.py` + `burger_tb3_N.yaml`) plans in **`TB3_N/map`**, not on `/shared_map`. That is **expected** for Step 4.

| If you want… | Use… |
|---|---|
| Team view in RViz | `/shared_map` |
| R2 drives on **saved** team map | Step 3 stack: map_server + `nav2_tb3_localization.launch.py` + **no** R2 SLAM |
| R2 drives while **building** team map | Step 4: R2 SLAM + `burger_tb3_2.yaml` Nav2 |

Connecting Nav2 global costmaps to **live** `/shared_map` is a **later** milestone (dynamic map updates, AMCL on shared frame for all robots). Step 5 frontiers will read `/shared_map`; Step 4 ensures that topic is trustworthy during motion.

---

## Command cheat sheet (Step 4)

### Kill old sim

```bash
pkill -9 gzserver || true
pkill -9 gzclient || true
sleep 2
```

### T0 — Gazebo + 3 robots

```bash
mkdir -p ~/swarm-search/launch/tmp_sdf
ros2 launch ~/swarm-search/launch/multi_robot_spawn.launch.py use_sim_time:=true
```

### T1–T3 — SLAM

```bash
ros2 launch ~/swarm-search/launch/slam_tb3.launch.py \
  robot_ns:=TB3_1 use_sim_time:=true \
  slam_params_file:=$HOME/swarm-search/slam_params/mapper_params_tb3_1.yaml
```

(Same pattern for `TB3_2` / `mapper_params_tb3_2.yaml` and `TB3_3` / `mapper_params_tb3_3.yaml`.)

### T4–T6 — Nav2 (optional)

```bash
ros2 launch ~/swarm-search/launch/nav2_tb3.launch.py \
  robot_ns:=TB3_1 use_sim_time:=true \
  params_file:=$HOME/swarm-search/nav2_params/burger_tb3_1.yaml
```

(Repeat for TB3_2 / `burger_tb3_2.yaml`, TB3_3 / `burger_tb3_3.yaml`.)

### T7 — Live merge (required for Step 4)

```bash
ros2 launch map_merge_swarm map_merge.launch.py
```

### T8 — RViz

```bash
ros2 run rviz2 rviz2 --ros-args -p use_sim_time:=true
```

- Fixed Frame: `shared_map`  
- Map: `/shared_map`, Durability **Transient Local**

### Teleop

```bash
ros2 run turtlebot3_teleop teleop_keyboard --ros-args -r __ns:=/TB3_1
```

### “Am I up?” (30 s)

```bash
ros2 topic echo /gazebo/model_states --qos-reliability best_effort --once
ros2 run tf2_ros tf2_echo shared_map TB3_1/map
ros2 topic hz /shared_map
```

### Mid-session save (optional)

```bash
ros2 run nav2_map_server map_saver_cli \
  -t /shared_map \
  -f ~/swarm-search/maps/house_shared_live \
  --ros-args \
  -p use_sim_time:=true \
  -p map_subscribe_transient_local:=true \
  -p save_map_timeout:=15.0
```

(Default ~2 s timeout often fails with ~1 Hz `/shared_map`.)

### Switch to Step 3 retest (R2 on frozen map)

Stop R2 SLAM and T7; do **not** run two global map sources for R2. Follow **`P2-S3-plan-r2-nav-on-saved-map.md`** → Command cheat sheet.

---

## Suggested build order

| # | Task | Done when |
|---|---|---|
| 1 | Full stack T0 + SLAM + T7 + RViz | One house on `/shared_map` |
| 2 | Drive one robot; watch merge update | New walls appear on team map |
| 3 | Second robot overlap | Walls stack, not drift |
| 4 | Optional save while merge runs | New files in `maps/` |
| 5 | Comms radius stub | Short design note |
| 6 | Tick Step 4 in parent todo | `P2-checklist.md` |

---

## Implement checklist (tick as you go)

### A. Live merge

- [x] T0 + T1–T3 + T7 + RViz per Part 1  
- [x] `/shared_map` ~1 Hz; GT TFs OK (`tf2_echo shared_map TB3_N/map`)  
- [x] RViz: one house outline (Step 1 quality)

### B. Live updates

- [x] At least one robot explores new area  
- [x] `/shared_map` changes while sim runs  
- [x] No duplicate `static_map_tfs` launch

### C. Optional save

- [x] `map_saver_cli` on `/shared_map` during live session → `maps/house_shared_live.*`  
- [x] Note filename / policy in `maps/README.md`

### D. Comms stub

- [ ] Radius `R` rule documented (who merges whom) — **deferred**

### E. Parent todo

- [x] Tick Step 4 in `P2-checklist.md`  
- [x] Update `ref-startup-after-break.md` “Where you left off” if workflow changed

---

## Pitfalls

| Symptom | Likely cause |
|---|---|
| Three maps side by side | GT merge not active; old spawn-offset TFs; run only `map_merge.launch.py` |
| `No shared_map <- TB3_N/map` in merge logs | SLAM not up; GT node missing; Gazebo `model_states` not publishing |
| `/shared_map` frozen | No SLAM maps arriving; check `/TB3_N/map` with Transient Local |
| Duplicate TF / weird merge | Also launched `static_map_tfs.launch.py` with T7 |
| Nav2 costmap timeout | Nav2 started before that robot’s SLAM TF exists — start SLAM first, wait, then Nav2 |
| Ran map_server + SLAM on same robot | Two map→odom publishers — pick Step 3 **or** Step 4 mode per robot |
| Saved map empty / wrong | Saved wrong topic — use **`-t /shared_map`** for live team map |
| RViz “No map received” | Durability **Transient Local** on Map display |
| Confused with Step 3 | Step 3 = `/map` from files + localization launch; Step 4 = **`/shared_map`** + SLAM |

---

## After THIS plan is done — what next

| When | File | Step |
|---|---|---|
| Next | `P2-S5-plan-frontiers.md` | **Step 5** — frontiers on `/shared_map` |
| Later | same | Comms radius stub / filter (deferred from Part 4) |
| Step 3 retest | `P2-S3-plan-r2-nav-on-saved-map.md` | Frozen map Nav proof anytime |
| Daily | `ref-startup-after-break.md` | T0–T8 order |

**Step 4 status:** live merge + explore + `house_shared_live` save **done**; Part 4 comms **skipped for now**.

---

## How this answers “is the map shared **while** we explore?”

| Layer | Step 3 (frozen) | Step 4 (live) |
|---|---|---|
| Team picture | `house_shared` on disk | `/shared_map` streaming |
| Updates when R1 drives | No (reload/new save) | **Yes** |
| R2 Nav2 on team frame | AMCL + `shared_map` | Still `TB3_2/map` until a future step |
| RViz team view | `/map` from map_server | **`/shared_map`** |
