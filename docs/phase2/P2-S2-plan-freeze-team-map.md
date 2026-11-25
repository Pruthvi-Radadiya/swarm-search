> **📍 Phase 2 › Step 2 (freeze team map)** · [← Roadmap](swarm_search_rescue_roadmap_v4.md) · [← P2 todo](P2-checklist.md) · Depends on: [Step 1](P2-S1-plan-gt-shared-map.md) · Status: ✅ Done

# Plan — Step 2: Freeze a team map file (learn + implement)

**Parent todo:** `docs/P2-checklist.md` → **Step 2** (freeze a team map file).  
**Goal:** Save the good `/shared_map` (or equivalent) to disk as `yaml` + `pgm`, document the frame, and prove you can reload it.  
**You implement this yourself** (learning path). This file is the explanation + how-to + checklist.

**Depends on:** Step 1 done — Option C GT merge (`P2-S1-plan-gt-shared-map.md`). You already see **one house** on `/shared_map` in RViz.  
**Related:** `ref-startup-after-break.md` (bring-up), `P2-checklist.md` (Step 3 next), `ref-ros2-pkg-basics.md` (maps / RViz).

---

## Big idea (plain English)

Right now `/shared_map` is a **live** picture. Merge keeps redrawing it while robots explore. If you shut WSL down, that picture is **gone**.

Step 2 = **take a snapshot** of that house drawing and save it as files on disk — like exporting a floor plan.

Later (Step 3), robot 2 will load that frozen floor plan and navigate on it — even without rebuilding the whole house with its own SLAM.

```text
Live (Step 1):     robots explore → merge paints → /shared_map (vanishes when you stop)
Freeze (Step 2):   /shared_map → house_shared.pgm + house_shared.yaml  (stays on disk)
Reload (Step 2):   map_server reads those files → publishes a map topic again
Nav proof (Step 3): R2 AMCL + Nav2 use that saved map
```

---

## What you are saving (two files)

ROS occupancy maps are almost always a **pair**:

| File | What it is |
|---|---|
| `house_shared.pgm` | Grayscale **image** of the grid (walls / free / unknown as pixel shades) |
| `house_shared.yaml` | **Metadata**: resolution, origin, which image file, thresholds |

**Why two files?**  
The image alone does not say “each pixel is 5 cm” or “the corner of the image is at (−10, −10) in the map frame.” The yaml holds that ruler info.

**Example yaml shape** (yours will be similar after save):

```yaml
image: house_shared.pgm
resolution: 0.05
origin: [-10.0, -10.0, 0.0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.25
```

| Field | Meaning |
|---|---|
| `image` | Path to the `.pgm` (often same folder) |
| `resolution` | Meters per pixel |
| `origin` | Pose of the image’s lower-left in the **map frame** (x, y, yaw) |
| `occupied_thresh` / `free_thresh` | How dark a pixel must be to count as wall / free |

---

## Frame name (critical for Step 3)

Your live merge publishes OccupancyGrid with:

```text
header.frame_id = "shared_map"
```

When you save with `map_saver`, the **image + yaml** describe that same grid.  
When you reload with `map_server`, the map topic’s `frame_id` must stay consistent with how you will localize later.

**For this project, document and use:**

```text
Map frame name:  shared_map
```

Do **not** silently rename it to plain `map` unless you also change every TF and Nav2 config to match. Step 3 will assume the saved map lives in **`shared_map`**.

Write this in a one-line note next to the files (Part 5 below).

---

## Why not only keep live `/shared_map`?

| Live `/shared_map` | Frozen files |
|---|---|
| Changes every second | Fixed snapshot |
| Needs Gazebo + SLAM + merge running | Can reload with only `map_server` |
| Good for exploring together | Good for “R2 drives on what R1 already mapped” proof |

Step 4 later can keep live merge; Step 2–3 need a **stable** map file for the navigation proof.

---

## Tools (what each one does)

| Tool | Job |
|---|---|
| `nav2_map_server` **map_saver_cli** | Subscribe to a map topic → write `.pgm` + `.yaml` |
| `nav2_map_server` **map_server** | Read `.yaml` → publish map on a topic (usually `/map`) |
| RViz Map display | Show either live `/shared_map` or reloaded topic |

Package name on Humble: `nav2_map_server` (executable often `map_saver_cli`).

---

## CRITICAL — before any part

1. Stack must be up with a **good** `/shared_map` (see `ref-startup-after-break.md`: Gazebo, SLAM×3, `ros2 launch map_merge_swarm map_merge.launch.py`).
2. Drive until the house looks good in RViz (Fixed Frame `shared_map`, topic `/shared_map`).
3. Every new terminal:

```bash
source /opt/ros/humble/setup.bash
source ~/swarm-search/install/setup.bash
export TURTLEBOT3_MODEL=burger
cd ~/swarm-search
```

4. Use **sim time** when talking to the sim stack (`--ros-args -p use_sim_time:=true` where needed).

---

## Part 1 — Make a place for maps

**Why:** Keep saved maps in the repo (or a known path), not `/tmp`, so they survive and Step 3 can find them.

```bash
mkdir -p ~/swarm-search/maps
```

Suggested final paths:

```text
~/swarm-search/maps/house_shared.yaml
~/swarm-search/maps/house_shared.pgm
~/swarm-search/maps/README.md    # frame name + how to reload (Part 5)
```

**Done when:** `maps/` directory exists.

---

## Part 2 — Confirm the live topic before saving

**Why:** `map_saver` only saves what it hears. Wrong topic → empty or wrong file.

```bash
ros2 topic list | grep -E 'shared_map|/map'
ros2 topic echo /shared_map --once | head -30
```

Check:

- `header.frame_id` is `shared_map`
- `info.resolution` looks right (often `0.05` — same as merge canvas)
- `info.width` / `height` non-zero
- Some cells are not all `-1` (drive first if everything is unknown)

In RViz: Fixed Frame `shared_map`, Map topic `/shared_map` — house should look good **before** you save.

**Done when:** you trust the live map visually.

---

## Part 3 — Save with map_saver_cli

**Why:** Official Nav2 tool to dump OccupancyGrid → files.

Create the output directory first (Part 1). Then, with merge still publishing `/shared_map`:

```bash
source /opt/ros/humble/setup.bash
source ~/swarm-search/install/setup.bash

ros2 run nav2_map_server map_saver_cli \
  -t /shared_map \
  -f ~/swarm-search/maps/house_shared \
  --ros-args -p use_sim_time:=true
```

| Flag / arg | Meaning |
|---|---|
| `-t /shared_map` | Topic to subscribe (your live team map) |
| `-f .../house_shared` | **Base name** (no extension) → writes `house_shared.pgm` + `house_shared.yaml` |
| `use_sim_time:=true` | Match Gazebo / merge clock |

**Why `-f` without `.yaml`?**  
The tool adds `.pgm` and `.yaml` itself.

**Expected:** log like “Map saved” / no crash. Then:

```bash
ls -la ~/swarm-search/maps/
# expect house_shared.pgm and house_shared.yaml
cat ~/swarm-search/maps/house_shared.yaml
```

**Done when:** both files exist and yaml `image:` points at the pgm (or a relative name next to it).

---

## Part 4 — Reload with map_server (prove the freeze worked)

**Why:** Saving is useless if you cannot load again. This proves the files are valid **without** needing merge.

**Important:** Stop relying on live `/shared_map` for this test (you can Ctrl+C merge, or just use a **different topic name** for the reload so they do not fight).

Simplest check — run map_server publishing on `/map` (default) from your files:

```bash
source /opt/ros/humble/setup.bash

ros2 run nav2_map_server map_server \
  --ros-args \
  -p yaml_filename:=$HOME/swarm-search/maps/house_shared.yaml \
  -p use_sim_time:=true \
  -p frame_id:=shared_map
```

If your Humble `map_server` uses a slightly different parameter name for frame, check:

```bash
ros2 param list /map_server
# or
ros2 run nav2_map_server map_server --help
```

Then configure the lifecycle (Nav2 map_server is often a lifecycle node). On Humble a common pattern is:

```bash
# In another terminal (sourced):
ros2 lifecycle set /map_server configure
ros2 lifecycle set /map_server activate
```

If your install auto-activates, you may only need to wait and then:

```bash
ros2 topic echo /map --once | head -25
```

**RViz check:**

- Fixed Frame: `shared_map` (if map `frame_id` is `shared_map`)
- Add Map → Topic: `/map` (or whatever map_server publishes)
- House outline should match what you saved

**Alternative one-liner bringup** (if you prefer launch):

```bash
ros2 launch nav2_map_server map_server.launch.py \
  yaml_filename:=$HOME/swarm-search/maps/house_shared.yaml \
  use_sim_time:=true
```

(Exact launch file name can vary by install; `ros2 pkg prefix nav2_map_server` + look under `share/nav2_map_server/launch`.)

**Done when:** reloaded map shows the same house in RViz from **files only**.

---

## Part 5 — Document frame + reload recipe

**Why:** Step 3 (and future-you after a break) must know the frame and commands without guessing.

Create `~/swarm-search/maps/README.md` with something like:

```markdown
# Team maps

## house_shared

- **Source topic when saved:** `/shared_map` (live merge from `map_merge_swarm`)
- **Frame id:** `shared_map` (same as Gazebo / GT house frame in Option C)
- **Files:** `house_shared.yaml` + `house_shared.pgm`
- **Saved when:** <date / note after a good explore>

### Reload (smoke test)

```bash
source /opt/ros/humble/setup.bash
ros2 run nav2_map_server map_server --ros-args \
  -p yaml_filename:=$HOME/swarm-search/maps/house_shared.yaml \
  -p use_sim_time:=true \
  -p frame_id:=shared_map
# lifecycle configure + activate if needed
```

RViz: Fixed Frame `shared_map`, Map topic `/map`.

### Next

Step 3 (`P2-checklist.md`): R2 AMCL + Nav2 on this map.
```

**Done when:** README exists and matches your real frame / paths.

---

## Part 6 — Optional: save again after more exploring

**Why:** First save might be half-empty. Re-run Part 3 after a better tour; overwrite the same base name or use `house_shared_v2`.

Rule of thumb: save when walls you care about for the Step 3 demo are already on `/shared_map`.

---

## Suggested build order

| # | Task | Why | Done when |
|---|---|---|---|
| 1 | `mkdir maps/` | Known path in repo | folder exists |
| 2 | Confirm `/shared_map` good | Do not save garbage | RViz house OK |
| 3 | `map_saver_cli` | Freeze to disk | yaml + pgm exist |
| 4 | `map_server` reload | Prove files work | RViz from files OK |
| 5 | `maps/README.md` | Frame + reload for Step 3 | documented |

---

## Implement checklist (tick as you go)

### A. Prep

- [x] Step 1 Option C working (one house on live `/shared_map`)
- [x] `mkdir -p ~/swarm-search/maps`
- [x] Explored enough that the house looks usable

### B. Save

- [x] `map_saver_cli -t /shared_map -f ~/swarm-search/maps/house_shared`
- [x] `house_shared.yaml` + `house_shared.pgm` on disk
- [x] Opened yaml; resolution / origin look sane

### C. Reload proof

- [x] `map_server` loads `house_shared.yaml`
- [x] RViz shows same house from reloaded topic (files only)
- [x] Confirmed map frame is `shared_map` (or documented if different)

### D. Document

- [x] `maps/README.md` with frame name + reload commands
- [x] Tick Step 2 in `P2-checklist.md`

---

## Pitfalls

| Symptom | Likely cause |
|---|---|
| map_saver hangs / no file | `/shared_map` not publishing; merge not running; wrong `-t` |
| Empty / all unknown map | Saved too early; explore first |
| Files written but RViz blank on reload | Lifecycle not activated; wrong topic (`/map` not `/shared_map`); Fixed Frame ≠ `shared_map`; Map Durability not **Transient Local** |
| TF / Nav2 confused later | Saved as `shared_map` but Step 3 configs expect `map` (or the reverse) — keep one name and document it |
| `use_sim_time` warnings / no message | Forgot `-p use_sim_time:=true` while Gazebo is master of `/clock` |
| yaml `image:` path broken | Moved pgm without updating yaml; keep both files in same folder |

---

## After THIS plan is done — what next

| When Step 2 is done | Open this file | Do this |
|---|---|---|
| Next | `P2-S3-plan-r2-nav-on-saved-map.md` | **Step 3** — R2 localizes (AMCL) + Nav2 on **saved** team map |
| Then | Step 4 | Live sharing while exploring |
| Only after Step 3 | Step 5 | Frontiers on the global map |
| Daily bring-up | `ref-startup-after-break.md` | Terminal order |

**Do not** start Step 5 (frontiers) until Step 3 passes.

**Already finished:** Step 1 / `P2-S1-plan-gt-shared-map.md` (GT shared map, one house live).

---

## How this connects to “is R1’s map shared to R2?”

| Layer | Status after Step 2 |
|---|---|
| Combined **picture** on `/shared_map` | Yes (Step 1) |
| That picture **saved on disk** | Yes (this Step 2) |
| R2’s **Nav2 driving** on that picture | **Not yet** → Step 3 |

Step 2 freezes the team knowledge. Step 3 teaches robot 2 to **use** it.
