> **📍 Phase 2 (master checklist)** · [← Roadmap](swarm_search_rescue_roadmap_v4.md) · Sub-plans: [S1](P2-S1-plan-gt-shared-map.md) · [S2](P2-S2-plan-freeze-team-map.md) · [S3](P2-S3-plan-r2-nav-on-saved-map.md) · [S4](P2-S4-plan-live-sharing.md) · [S5](P2-S5-plan-frontiers.md) · Status: ✅ Phase 2 100% Completed & Published (Demo on LinkedIn)

# TODO — One global map (R1 explores → R2 navigates)

**Goal (your plan):** All robots move in the **same** house and share **one** occupancy map so the team understands the surroundings and explores faster. Proof: robot 1 maps everything → robot 2 navigates using that map without remapping the whole house.

**Roadmap:** still **Phase 2** (`swarm_search_rescue_roadmap_v4.md`).  
**Depends on:** map-merge v1 done (`P2-S0-todo-map-merge-v1.md`, `/shared_map`).  
**Do this before:** frontiers / greedy assign / comms plots.

**Resume stack:** `docs/ref-startup-after-break.md`

---

## Why this comes before frontiers

Frontiers and “share exploration” only work if there is **one** global occupancy world.  
Today each robot still mostly lives in `TB3_N/map`; `/shared_map` is a v1 stitch (often 3 islands). Nav2 does **not** yet use the team map.

---



## End state

- [ ] One coherent house on `/shared_map` (or a saved team map)
- [ ] Robot 2 **localized** in that same frame
- [ ] Robot 2 **Nav2** plans on that map
- [ ] Demo: R2 reaches a place **only R1 explored**

---



## Step 1 — One coherent global map (fix merge alignment)

**Problem:** spawn-offset merge can show three side-by-side maps instead of one house.

**Chosen approach: Option C (Gazebo ground truth).**  
Full how-to + implement checklist: `docs/P2-S1-plan-gt-shared-map.md` (you implement; learning path).

- [x] Baseline: drive all 3 overlapping areas; screenshot RViz `/shared_map`
- [x] Follow `P2-S1-plan-gt-shared-map.md` (GT TF node → merge uses TF → launch without static spawn TFs)
- [x] Success target: **one** house outline (walls from different robots **stack**, not sit side-by-side)
- [x] Re-check fusion: occupied wins is OK for now
- [x] Optional: larger / expanding canvas if 20×20 m is tight

**Done when:** RViz Fixed Frame `shared_map`, topic `/shared_map` looks like one world.  
**Then:** continue this file at **Step 2** (not frontiers yet). See “After this plan” in `P2-S1-plan-gt-shared-map.md`.

---



## Step 2 — Freeze a team map file

**Full how-to + implement checklist:** `docs/P2-S2-plan-freeze-team-map.md` (you implement; learning path).

- [x] Let R1 (or the team) explore until the global map is good enough
- [x] Save with map_saver (or equivalent), e.g. `maps/house_shared.yaml` + `.pgm`
- [x] Document frame name used (`shared_map`) and how to reload (`maps/README.md`)

**Done when:** map files exist in the repo (or a known path) and reload in RViz/map_server.  
**Then:** continue this file at **Step 3**.

---



## Step 3 — Proof: R2 navigates on R1’s map

This is the milestone that matches your plan.

**Full how-to + implement checklist:** `docs/P2-S3-plan-r2-nav-on-saved-map.md` (you implement; learning path).  
**Commands:** same file → **Command cheat sheet**.

- [x] Run R2 in **localization** (AMCL) on the **saved team map** (not “mapping from scratch only” for this test)
- [x] Point R2 Nav2 / map_server / global costmap at that map and shared frame
- [x] Set R2 **initial pose** correctly in that frame
- [x] Confirm TF: team map frame → `TB3_2/base_footprint`
- [x] Send Nav2 goal to a region **R1 mapped and R2 never fully explored alone** (short + clicked goals SUCCEEDED)
- [x] Record short clip / note result (demo video recorded & published on LinkedIn)

**Done when:** R2 reaches that goal using the shared/saved map.  
**Then:** Step 4 (live sharing). Do **not** start Step 5 frontiers until this passes.

**Fixes landed during debug:** RPP controller, inflation 0.3, AMCL `/tf` remaps (see plan pitfalls / cheat sheet).

---



## Step 4 — Live sharing while exploring together

**Full how-to + implement checklist:** `docs/P2-S4-plan-live-sharing.md` (you implement; learning path).  
**Commands:** same file → **Command cheat sheet**.

- [x] Keep live `/shared_map` merge running during multi-robot runs
- [x] Optional: periodic save of team map (`maps/house_shared_live.*`; use `map_subscribe_transient_local` + longer `save_map_timeout`)
- [ ] (Later) comms radius: only accept / merge updates if `distance < R`

**Done when:** live merge stays usable; optional R filter stub designed (implement fully with frontiers or right after).  
**Status:** live share + mid-session save done; comms stub deferred. **Next: Step 5.**

---



## Step 5 — Only after Steps 1–3: frontiers on the global map

**Full how-to + implement checklist:** `docs/P2-S5-plan-frontiers.md` (you implement; learning path).  
**Commands:** same file → **Command cheat sheet**.

- [x] Frontier detector on `/shared_map` (or saved team map)
- [x] Visualize frontiers in RViz
- [x] Greedy assign nearest free robot
- [x] Transform goals into each robot’s Nav2 / shared frame and send `NavigateToPose`
- [x] Part 7: `ros2 launch explore_swarm explore.launch.py`
- [x] Coverage vs time (**Part 6** — `analysis/coverage_logger.py` + `coverage_pct.png`; 100% = end-of-run known cells)
- [ ] Comms on/off ablation — **skipped for now** (implement in future if useful)

**Do not start Step 5 until Step 3 passes.** (Steps 1–4 are done.)  
**Status:** Step 5 **done** (all 6 patterns + watchdogs + redirect implemented). House demo video recorded and published on LinkedIn. Phase 2 complete. Next: Phase 3 (aerial). Bring-up: `ref-startup-after-break.md` §3 / §6.

---



## Suggested work sessions


| Session | Focus                                         |
| ------- | --------------------------------------------- |
| 1       | Step 1 baseline + alignment until one outline |
| 2       | Step 2 save team map                          |
| 3       | Step 3 R2 AMCL + Nav2 proof                   |
| 4+      | Step 4 live share; then Step 5 frontiers      |


---



## Out of scope for this todo

- Aerial / Phase 3
- YOLO / time-to-find
- PPO / MARL
- Perfect log-odds (nice-to-have after Step 3)

---



## How to ask mid-task

Paste:

1. Which step (1–5) and checkbox
2. Command you ran
3. Error or RViz description
4. Relevant config / code snippet

---



## Status

- [x] Step 1  (Option C — `P2-S1-plan-gt-shared-map.md`)
- [x] Step 2  (`P2-S2-plan-freeze-team-map.md` — `maps/house_shared.*` + `maps/README.md`)
- [x] Step 3  (`P2-S3-plan-r2-nav-on-saved-map.md`)
- [x] Step 4  (`P2-S4-plan-live-sharing.md` — live `/shared_map` + `house_shared_live`; comms skipped for now)
- [x] Step 5  (`P2-S5-plan-frontiers.md` — explore + launch; Part 6 coverage optional; comms deferred)