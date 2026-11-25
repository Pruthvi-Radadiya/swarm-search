> **📍 Phase 2 › Step 0 (v1 map merge — foundation)** · [← Roadmap](swarm_search_rescue_roadmap_v4.md) · [← P2 checklist](P2-checklist.md) · Superseded by: [Step 1 GT](P2-S1-plan-gt-shared-map.md) · Status: ✅ v1 complete

# TODO — Map merge (`/shared_map`)

**Status: v1 complete (2026-09-21).** Recreate notes live in `docs/P2-runbook.md` (Map merge section) and `docs/ref-ros2-pkg-basics.md`.

Work through these yourself. Ask anytime you’re stuck — paste the error / file / command.

**Goal:** three `/TB3_N/map` topics → one `/shared_map` in frame `shared_map`, confirmed in RViz.

**Spawn offsets** (from `launch/multi_robot_spawn.launch.py`):


| Robot | x    | y    | yaw |
| ----- | ---- | ---- | --- |
| TB3_1 | -2.0 | -0.5 | 0.0 |
| TB3_2 | 0.5  | -2.0 | 0.0 |
| TB3_3 | 2.0  | 0.5  | 0.0 |


**Already done:**

- [x] Package created: `src/map_merge_swarm/`
- [x] Stub file: `map_merge_swarm/merge_node.py` (empty)

---

## 1. Wire the package so the node can run

- [x] Add entry point in `setup.py` → `console_scripts`:
  ```
  merge_node = map_merge_swarm.merge_node:main
  ```
- [x] Add `launch/` install in `setup.py` `data_files` (so `ros2 launch` finds it later)
- [x] Optional: set a real `description` / `license` in `package.xml` and `setup.py`
- [x] `cd ~/swarm-search && colcon build --packages-select map_merge_swarm`
- [x] `source ~/swarm-search/install/setup.bash`
- [x] Confirm: `ros2 pkg list | grep map_merge_swarm`

**Ask if:** `colcon build` fails, or entry point not found after source.

---



## 2. Spawn offsets helper

- [x] Create `map_merge_swarm/spawn_offsets.py`
- [x] Export a dict, e.g. `SPAWN_OFFSETS = {"TB3_1": (-2.0, -0.5, 0.0), ...}`
- [x] Keep yaw = 0.0 for now (matches current spawn)

**Ask if:** unsure whether offsets go map→shared or shared→map (sign of transform).

---



## 3. Static TFs into `shared_map`

Pick one:

**Option A (easiest):** in launch, three `static_transform_publisher` nodes  
`shared_map` → `TB3_N/map` with translation = spawn (x, y, 0), rotation identity.

**Option B:** small Python node that publishes the same static transforms once.

- [x] Implement Option A or B
- [x] After launch with sim running, verify:
  ```bash
  ros2 run tf2_ros tf2_echo shared_map TB3_1/map
  ```
  Translation should be about `(-2, -0.5, 0)` (or the inverse — if inverted, flip parent/child; ask before guessing forever).

**Ask if:** `tf2_echo` says frame does not exist, or translation sign looks wrong vs Gazebo.

---



## 4. Merge node — structure (stubs first OK)

In `merge_node.py`:

- [x] `MergeNode(Node)` with `use_sim_time` True (param or hardcode for sim)
- [x] Subscribe to `/TB3_1/map`, `/TB3_2/map`, `/TB3_3/map` (`OccupancyGrid`)
- [x] Store latest grid per robot in a dict
- [x] Publisher: `/shared_map` (`OccupancyGrid`)
- [x] Timer ~1 Hz (or publish whenever any map updates) calling `merge_and_publish()`
- [x] Stub `merge_and_publish()` that for now publishes an empty/unknown grid with `frame_id=shared_map` — prove pub works before fusion math

Verify stub:

```bash
ros2 run map_merge_swarm merge_node
ros2 topic echo /shared_map --once
```

**Ask if:** node dies on import, no messages, or QoS mismatch with SLAM maps (transient_local / reliable).

---



## 5. Merge node — fusion logic (v1)

- [x] Choose a fixed shared canvas first (simpler than expanding):
  - e.g. resolution `0.05`
  - origin and size large enough to cover spawn poses + explored area (start generous, e.g. 20×20 m or bigger)
- [x] For each robot’s latest map:
  - For each cell that is not unknown, convert map-frame cell center → `shared_map` using spawn offset (or TF)
  - Write into shared canvas index
- [x] Fuse per cell:
  - any **100** (occupied) → 100  
  - else any **0** (free) → 0  
  - else **-1** (unknown)
- [x] Fill `OccupancyGrid.info` + `data`, set `header.frame_id = "shared_map"`, stamp with node clock
- [x] Publish

**Ask if:** maps don’t overlap correctly, everything shifted, or index/out-of-bounds errors.

---



## 6. Launch file

- [x] Create `launch/map_merge.launch.py`
  - static TFs (or TF node)
  - `merge_node`
  - `use_sim_time:=true`
- [x] Install launch via `setup.py` / rebuild
- [x] Run:
  ```bash
  ros2 launch map_merge_swarm map_merge.launch.py
  ```
  (or `ros2 launch ~/swarm-search/src/...` while iterating)

**Ask if:** launch can’t find executable or launch file path.

---



## 7. Full-system test

Bring-up order:

1. [x] Gazebo spawn (`multi_robot_spawn.launch.py`)
2. [x] SLAM ×3 (params loaded — `map_frame` = `TB3_N/map`)
3. [x] Nav2 ×3 (optional for this test; teleop is enough)
4. [x] Map merge launch (`ros2 launch map_merge_swarm map_merge.launch.py`)
5. [x] RViz2:
  - Fixed Frame: `shared_map`
  - Map display → `/shared_map`
  - Optional: Map ×3 for `/TB3_N/map` to compare

Drive robots (teleop or Nav2). Shared map should grow as one coherent picture.

Checks:

```bash
ros2 topic hz /shared_map
ros2 run tf2_ros tf2_echo shared_map TB3_1/map
ros2 run tf2_ros tf2_echo shared_map TB3_2/map
ros2 run tf2_ros tf2_echo shared_map TB3_3/map
```

- [x] Visually OK in RViz  
- [x] No constant TF errors in merge node  

**Ask if:** individual maps look fine but `/shared_map` is blank, misaligned, or flickering.

---



## 8. Document

- [x] Append a short “Map merge” section to `docs/P2-runbook.md`:
  - package name, launch command
  - spawn offsets table
  - RViz Fixed Frame + topic
  - known limit: fixed spawn offset ≈ early exploration only
- [x] Repo file tree + startup order updated; “Still open” marks merge v1 done

---



## 9. Later (do NOT start until v1 works)

- [ ] Expanding canvas instead of fixed size  
- [ ] Log-odds fusion  
- [ ] Relative alignment without spawn-only prior  
- [ ] Point Nav2 / planners at `/shared_map` so robots use areas others explored

---



## How to ask mid-task

Paste:

1. Which checkbox you’re on (e.g. “5 — fusion”)
2. Command you ran
3. Error / unexpected output
4. Relevant snippet of your code if it’s a logic bug

That’s enough to unblock without derailing the rest of the list.