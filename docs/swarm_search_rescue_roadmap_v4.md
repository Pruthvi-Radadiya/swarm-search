# Swarm Search and Rescue Robotics

**Roadmap v4 — Heterogeneous indoor search in simulation (the plan to follow)**

| | |
|---|---|
| **Status** | Active plan (v2/v3 archived). **Phase 1 ✅ done. Phase 2 ✅ 100% completed & demo published on LinkedIn** (3 TB3, `/shared_map`, frontiers, 6-pattern coordinator, stuck watchdog, redirection). Comms radius skipped for now. **Next:** Phase 3 (Aerial Drone). Day-to-day: `ref-startup-after-break.md` |
| **What this is** | All-simulation portfolio: mapping, limited-range comms, queryable object search, **time-to-find**. Aimed at sim / robotics / ML hiring, not real hardware |
| **Team** | **3× TurtleBot3 + 1 aerial** in one house-scale Gazebo world |
| **Platform** | **Gazebo + ROS2 in WSL** from week 1. No Isaac Sim (RTX 3060 6GB). No PyBullet → Gazebo policy port |
| **Brain** | Occupancy + **frontier exploration** + greedy assignment. PPO/MARL only later, and only as a comparison if the demo already works |
| **Perception** | Text **query** (`backpack`, `person`, …) + nano detector (YOLOv8n / YOLO-World). **Not** victim vs threat. **Not** LoS-tagged props as the product |
| **Metric** | **Time-to-find** (confirmed query match), plus coverage vs time, comms on/off, with/without drone |
| **Duration** | About **10 weeks**, with a recordable demo by week 6 |

---

## Why this replaces v2 / v3

v2/v3 were: one drone in PyBullet → MARL/PPO → port to Gazebo → splat as physics world → tagged hostage/threat (v3 also added thermal).

That stack fights the job you want and the GPU you have:

- Dual sim (PyBullet then Gazebo) is where the demo usually dies
- PPO as the only brain often never beats a frontier baseline
- Isaac Sim is too heavy for 6GB
- Victim/threat is a weaker, narrower story than **find what we ask**
- A raw Gaussian splat is not a collision world

**Keep from the old plan:** unknown indoor space, sensor-footprint occupancy, **communication radius**, robustness/ablations, LinkedIn + GitHub, all simulation.

**Stop:** gym-pybullet-drones as the source of truth, PettingZoo/SB3 as phase 2, thermal as a required sensor, hostage/threat classes, splat-as-physics, drone-only swarm.

Old Phase 1 work (one PyBullet drone, rays, camera, IMU drift) can stay in an `archive/` folder as a lab note. It is **not** an input to the Gazebo nodes.

---

## Roles (do not give all four the same policy)

| Robot | Job |
|--------|-----|
| **Aerial** | Fast coarse coverage, top-down RGB, drop **candidate pins** on the shared map |
| **TurtleBots** | Nav2 to pins and leftover **frontiers**, lidar mapping, closer RGB to **confirm** the query |

**Time-to-find** = first **confirmed** detection of the queried object (drone hint + one ground robot, or two independent views). A single noisy YOLO box is not the metric.

---

## Stack (locked)

| Piece | Choice |
|--------|--------|
| Sim | Gazebo Harmonic (or the ROS2-distro default) in **WSL Ubuntu** |
| Middleware | ROS2 (Humble or Jazzy — pick one and stay) |
| Ground | TurtleBot3 + **Nav2** + lidar |
| Air | Light quadrotor / Crazyflie-class SITL, **waypoints**, not vision-based flight RL |
| Map | 2D occupancy + object pins/labels (not a full 3D OctoMap on four robots) |
| Radio | One node: drop messages if `distance > R` |
| Detect | **One** YOLOv8n or YOLO-World-s process, low-res / few Hz; RGB on **drone + one TurtleBot** if 6GB is tight |
| World | Premade **house** (wide doors, single floor). Optional later: **mesh of your room + collision boxes**; splat only as a beauty-shot clip |
| GPU | 3060 6GB: no Isaac, no Grounding-DINO+SAM on three streams |

---

## Phases

### Phase 0 — Freeze the old path
**Half a day**

- Leave PyBullet / gym-pybullet-drones / SB3 installs as-is; **do not extend them**
- This document is the only active plan (v2/v3 headers say archived)
- Optional: move old Phase 1 scripts to `archive/pybullet_phase1/`

**Done when:** you are not opening the PyBullet GUI as the next task.

---

### Phase 1 — One TurtleBot in a house (Gazebo + ROS2)
**Weeks 1–2** — **DONE** (see `P1-runbook.md`)

- Install ROS2 + Gazebo in WSL; one launch that brings up a **house world** and **one TurtleBot3**
- Drive with teleop, then **Nav2** to a pose
- Lidar → occupancy in RViz
- Confirm 6GB can hold this without swapping to death

**Deliverable:** Video: robot in a house, map filling. If this fails, nothing later matters.

**Docs:**
| Doc | Purpose |
|-----|---------|  
| [P1-runbook.md](P1-runbook.md) | Full recreate guide for Phase 1 stack |

---

### Phase 2 — Three TurtleBots, shared map, comms radius
**Weeks 3–4** — **CORE DONE** (2026-10). Comms ablation **skipped for now**.

- [x] Spawn **3** TurtleBots; each runs Nav2 (`P2-runbook.md`, `ref-startup-after-break.md`)
- [x] Shared occupancy `/shared_map` (GT Option C merge — `map_merge_swarm`)
- [x] Freeze + R2 Nav on saved map (Steps 2–3)
- [x] Live share while exploring (Step 4)
- [x] **Frontiers** + greedy assign + NavigateToPose (`explore_swarm` / Step 5)
- [x] One-command explore: `ros2 launch explore_swarm explore.launch.py`
- [x] Plot **coverage vs time** (optional — one curve from `/shared_map`; see engineering notes)
- [ ] **Comms radius** — **deferred / skip for now**; implement later if useful for the portfolio story

**Deliverable (done):** Three robots auto-cover the house on `/shared_map` without teleop.  
**Optional later:** coverage plot; comms on/off ablation figure.

**Docs:**
| Doc | Purpose | Status |
|-----|---------|--------|
| [P2-runbook.md](P2-runbook.md) | Full recreate guide for Phase 2 stack | ✅ Done |
| [P2-checklist.md](P2-checklist.md) | Master checklist: Steps 1–5 | ✅ Steps 1–5 done |
| [P2-S0-todo-map-merge-v1.md](P2-S0-todo-map-merge-v1.md) | Step 0 — v1 map merge foundation (superseded by GT) | ✅ v1 complete |
| [P2-S1-plan-gt-shared-map.md](P2-S1-plan-gt-shared-map.md) | Step 1 — GT TF shared map design + implementation | ✅ Done |
| [P2-S2-plan-freeze-team-map.md](P2-S2-plan-freeze-team-map.md) | Step 2 — freeze/save shared map to disk | ✅ Done |
| [P2-S3-plan-r2-nav-on-saved-map.md](P2-S3-plan-r2-nav-on-saved-map.md) | Step 3 — R2 navigates on saved team map (AMCL) | ✅ Done |
| [P2-S4-plan-live-sharing.md](P2-S4-plan-live-sharing.md) | Step 4 — live map sharing while exploring | ✅ Done |
| [P2-S5-plan-frontiers.md](P2-S5-plan-frontiers.md) | Step 5 — frontier detection + goal assignment (core) | ✅ Core done |
| [P2-S5-patterns-wall-claims.md](P2-S5-patterns-wall-claims.md) | Step 5 — assigner pattern fixes for house (current work) | 🔧 In progress |
| [P2-video-plan-house-3robots.md](P2-video-plan-house-3robots.md) | House video plan (gate to Phase 3) | ⏳ Blocked on patterns |
| [P2-ref-engineering-notes.md](P2-ref-engineering-notes.md) | Theory reference: algorithms, formulas, design | 📖 Reference |
| [P2-ref-study-guide.md](P2-ref-study-guide.md) | Study guide: learn the code part by part | 📖 Reference |

---

### Phase 3 — Add the aerial, split roles
**Week 5**

- One UAV, conservative altitude, waypoints
- Drone writes **candidate pins**; TurtleBots take pins first, then leftover frontiers
- Fuse into the **same 2D** map (pins + occupancy)
- If Gazebo crawls: develop with **2 ground + 1 air**, record the video with 3+1

**Deliverable:** Split-screen: copter sweeping, bases driving to pins.

---

### Phase 4 — Query, detect, time-to-find
**Weeks 6–7**  
*First LinkedIn-recordable “whole stack” demo.*

- Operator query as a ROS string, e.g. `backpack`
- Detector on RGB → labels on the map (object-level, not panoptic)
- Confirm rule as above; log **seconds to confirm**
- Start with a small closed set if YOLO-World is heavy; **keep the interface as text** so you can swap models
- Do **not** implement victim vs threat

**Deliverable:** Query on screen, boxes on camera, pin on map, **time-to-find** number. This is the demo even if later phases slip.

---

### Phase 5 — Experiments the README needs
**Weeks 8–9**

Same query, randomized object pose (10–20 seeds):

| Condition | What you plot |
|-----------|----------------|
| Full team, comms on | Median time-to-find, coverage vs time |
| No drone | 3 TurtleBots only |
| No sharing (`R = 0`) | Independent explorers |
| Optional | 1 robot “fails” mid-run (robustness, from the old plan) |

**Deliverable:** Two figures: time-to-find by condition; coverage vs time. No PPO required.

---

### Phase 6 — Optional room twin + publish
**Weeks 9–10**

- **Must:** one-command launch, README, 60–90 s video (third-person + RViz + query), GitHub
- **Optional:** phone **mesh** of your room + wall/furniture **collision boxes** as a second world; splat flythrough as a separate clip, **not** the physics file
- **Skip unless Phase 4 already ships:** PPO vs greedy, thermal, procedural houses, splat-as-Gazebo-visual

**LinkedIn line:** *Simulated heterogeneous team (3 ground + 1 aerial) that shares a range-limited map and finds a queried object as fast as possible, with time-to-find vs no-comms / no-drone.*

---

## Stretch (after the video exists)

- YOLO-World / open-vocab instead of a fixed class list
- Learned frontier scoring vs greedy (one plot, same Gazebo observations — **no PyBullet train/export**)
- Procedural or second house layout
- IMU / localization noise as an ablation

---

## Explicitly out of scope

- Isaac Sim
- Continuing gym-pybullet-drones / PettingZoo as the next milestone
- Training in PyBullet and hoping Gazebo matches
- Victim vs weapon / “alive person” as product classes
- Using a Gaussian splat as the collision world
- Four full-rate cameras + heavy open-vocab on 6GB
- Real robots in a real house

---

## Repo shape (current → later)

```
launch/                 # multi spawn, slam, nav2, localization
src/map_merge_swarm/    # GT TF + /shared_map fuse
src/explore_swarm/      # frontiers + greedy assign + explore.launch.py
maps/                   # house_shared, house_shared_live
docs/                   # startup, plans, engineering notes, this roadmap
src/comms/              # NEXT — range filter
# later: aerial, perception (YOLO), analysis (time-to-find)
archive/                # old PyBullet phase 1 only
media/                  # demo video
```

---

## Next milestones (resuming after Phase 2 LinkedIn demo)

1. **Phase 2 Complete:** 3-robot house explore demo recorded, README architecture documented, and milestone posted on LinkedIn.
2. **Current step:** Phase 3 kick-off — Add aerial drone (UAV / SITL or Gazebo quadrotor) for coarse top-down mapping and candidate exploration pins.
3. **Phase 4:** Object query interface (`backpack`, `person`) and "Time-to-Find" benchmark metric.

---

## What you will have at the end

- A Gazebo/ROS2 **system**, not a notebook policy: 3 ground + 1 air, comms radius, shared map, text query, time-to-find
- Baselines a recruiter can read in one figure
- Optional mesh twin of a real room (boxes for physics, splat only for looks)
- GitHub + video aimed at **simulation / robotics / ML** roles
