# Team maps

## house_shared

- **Source topic when saved:** `/shared_map` (live merge from `map_merge_swarm`, Option C GT alignment)
- **Frame id:** `shared_map` (same as Gazebo / GT house frame)
- **Files:** `house_shared.yaml` + `house_shared.pgm`
- **Grid:** 400×400 @ 0.05 m/cell, origin `[-10, -10, 0]` (≈ 20 m × 20 m)
- **Saved:** after Step 1 one-house explore (Sep 2026)

### Save (when live `/shared_map` looks good)

```bash
source /opt/ros/humble/setup.bash
source ~/swarm-search/install/setup.bash

ros2 run nav2_map_server map_saver_cli \
  -t /shared_map \
  -f ~/swarm-search/maps/house_shared \
  --ros-args -p use_sim_time:=true
```

### Reload (smoke test — files only)

```bash
source /opt/ros/humble/setup.bash

ros2 run nav2_map_server map_server --ros-args \
  -p yaml_filename:=$HOME/swarm-search/maps/house_shared.yaml \
  -p use_sim_time:=true \
  -p frame_id:=shared_map
```

In another terminal:

```bash
source /opt/ros/humble/setup.bash
ros2 lifecycle set /map_server configure
ros2 lifecycle set /map_server activate
```

`map_server` publishes on **`/map`** by default (not `/shared_map`).

**RViz:**

- Fixed Frame: `shared_map`
- Map → Topic: `/map`
- Map → Topic → **Durability: Transient Local** (otherwise “No map received”)
- Color Scheme: `map` (walls black; free/unknown look grey — normal)

### Live session saves (Step 4)

While T7 merge runs, snapshot the **live** team map (do **not** rely on default 2 s timeout):

```bash
ros2 run nav2_map_server map_saver_cli \
  -t /shared_map \
  -f ~/swarm-search/maps/house_shared_live \
  --ros-args \
  -p use_sim_time:=true \
  -p map_subscribe_transient_local:=true \
  -p save_map_timeout:=15.0
```

- **Files:** `house_shared_live.yaml` + `house_shared_live.pgm` (Step 4 mid-session; half-house OK)
- **Policy:** use `_live` suffix so `house_shared` (Step 1/2 full freeze) is not overwritten unless you choose Option A with `-f .../house_shared`
- Details: **`docs/plan-step4-live-sharing.md`**

### Next / related

Step 5 **done** — multi-robot explore + `ros2 launch explore_swarm explore.launch.py`.  
Comms radius **skipped for now**. Coverage-vs-time optional (samples `/shared_map` only).  
Bring-up: **`docs/startup-after-break.md`**. Concepts: **`docs/engineering-notes-map-explore.md`**.
