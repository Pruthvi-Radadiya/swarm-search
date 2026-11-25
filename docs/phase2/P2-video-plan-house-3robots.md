> **📍 Phase 2 › Video (gate to Phase 3)** · [← Roadmap](swarm_search_rescue_roadmap_v4.md) · Depends on: [Patterns](P2-S5-patterns-wall-claims.md) · Status: ✅ Completed (Video recorded & posted on LinkedIn)

# Plan — 3-robot house video (before next roadmap phase)

**Goal:** Record a clean demo: **3 TurtleBots**, **house world**, live `/shared_map`, autonomous explore (T9). Keep all three robots in the video.

**Status:** ✅ Successfully recorded, demonstrated, and published on LinkedIn. Phase 2 milestone achieved.

---

## What was changed for the house run

**File:** `launch/multi_robot_spawn.launch.py`

| Setting | Turtle world (old) | House video (current) |
|--------|---------------------|------------------------|
| World | `worlds/turtlebot3_world.world` | `worlds/turtlebot3_house.world` |
| Spawns | `[[-2, -0.5], [0.5, -2], [2, 0.5]]` (commented in file) | `[[-2.0, -0.5], [0.0, 0.5], [1.5, 3.0]]` |

- **TB3_1:** (-2.0, -0.5) — open side / “R1” risk if frontiers pull outside house  
- **TB3_2:** (0.0, 0.5)  
- **TB3_3:** (1.5, 3.0)  

**Revert for turtle demos:** uncomment old `pose`, point `world` back to `turtlebot3_world.world`.

**Bring-up:** Same as README T0–T9 (`map_merge.launch.py` uses GT TFs — not the old static spawn-offset TFs).

---

## Recording checklist (when map looks good)

1. Kill old Gazebo; fresh T0 with house world + poses above  
2. T1–T3 SLAM, T4–T6 Nav2, T7 merge, T8 RViz (`shared_map`, `/shared_map`)  
3. T9 `ros2 launch explore_swarm explore.launch.py`  
4. Optional: `analysis/coverage_logger.py` for a coverage curve  
5. Record Gazebo + RViz; avoid tipped robots (messy merge / bad video)

---

## Why the house map looks messy (shifted black lines)

- **Cause:** Burger tip / flip → bad lidar → wrong SLAM cells → merge paints **offset duplicate walls**.  
- **Not fixed by** frontier tweaks alone.  
- **For video:** reduce tips (Waffle, slower Nav2, safer spawns, no wall hits) before tuning frontiers.

---

## White next to gray in RViz but no frontier markers

**Code:** `explore_swarm/frontier_detector.py` — frontier only if:

- cell value is **exactly** `0` (free), and  
- at least one **4-neighbor** (up/down/left/right) is **exactly** `-1` (unknown).

**So visually “white | gray” can still have zero frontiers when:**

- Only **corner** touch (diagonal), not edge-adjacent in the grid  
- RViz “white” is not occupancy `0` (merge / threshold / noise)  
- Door is one cell wide and merge mislabels neighbors  

**Quick check:** RViz → Map `/shared_map`, markers `/frontiers_markers` or PoseArray `/frontiers`. Empty at a door = detector logic, not RViz lying.

---

## If we “fix” frontiers (8-neighbors / doors) — will R1 run away?

**Yes, that risk is real** if you only detect more frontiers:

- Robot spawned on the **open** side of the house has a long **free | unknown** edge into the yard → endless frontiers **away** from the house.

**Do both when improving frontiers:**

1. **Better door frontiers** (e.g. 8-neighbors or morphology — implement carefully)  
2. **Goal filter** so explore stays in the house, e.g.  
   - axis-aligned **bounding box** around house (world / `shared_map` meters)  
   - drop frontiers outside box or farther than **max distance from house centroid / walls**  
   - optional **max goal distance** from each robot’s start  

Then: doors get markers; open field does not pull TB3_1 forever.

**Keep 3 robots** for the video — but all three must stay upright; one flip hurts the whole merged map.

---

## Recommended order (before pressing record)

1. **Stop flipping** (model / params / spawns / no aggressive goals near walls)  
2. **Confirm merge** looks like one house (not three shifted layers)  
3. **Frontier fix + house bounds** in assigner or detector  
4. **Test run** — markers at doors, TB3_1 not chasing yard  
5. **Record** 3-robot house explore  

---

## Future code touchpoints (when implementing)

| Piece | File | Idea |
|-------|------|------|
| Frontier rule | `frontier_detector.py` `_frontier_cells` | 4-neighbor today; optional 8-neighbor or dilate unknown — **separate from assigner Patterns 1–3** |
| Goal filter | `goal_assigner.py` | Reject frontiers outside house AABB or max radius |
| Wall-safe goals + room claims | `goal_assigner.py` | Patterns 1–3 **done**; next 4–6 in `docs/P2-S5-patterns-wall-claims.md` |
| Redirect while busy | `goal_assigner.py` | Pattern 6 in that doc (cancel far goal if closer valid frontier appears) |
| Spawn / world | `multi_robot_spawn.launch.py` | Documented above |
| Nav stability | `nav2_params/burger_tb3_*.yaml` | RPP, inflation 0.45; consider Waffle for video |

---

## Notes from testing (2026-10)

- Strict 4-neighbor frontiers explain missing markers on straight white–gray door edges.  
- Messy walls after merge tied to **flip**, not only frontier logic.  
- Comms radius / coverage plot — separate; this doc is **video + house + frontiers** only.
