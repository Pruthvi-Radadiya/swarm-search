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

### Next

Step 3 (`docs/todo-global-map-nav.md`): R2 AMCL + Nav2 on this saved map.
