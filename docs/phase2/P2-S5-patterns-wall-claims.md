> **📍 Phase 2 › Step 5 (assigner patterns)** · [← Roadmap](swarm_search_rescue_roadmap_v4.md) · [← Frontiers plan](P2-S5-plan-frontiers.md) · [← Video plan](P2-video-plan-house-3robots.md) · Status: ✅ 100% Completed (Patterns 1–6 validated, house demo recorded & posted on LinkedIn)

# Explore assigner: wall-safe goals + room-aware claims

**How to use this doc:** Implement assigner patterns **1 → 2 → 3 → 4** one at a time (all done). Then do **Round 2 (Steps 0, A, B, C)** before **Pattern 5**. Optional later: **6 (redirect)**. Do not jump ahead. Verify each step before the next.

**Separate work (not Patterns 1–5):** missing frontiers on free↔unknown edges / “white room not explored” → `frontier_detector.py` (see § Outside this doc below and `docs/P2-video-plan-house-3robots.md`).

**Progress (update as you go):**

| Pattern | Status |
|---------|--------|
| 1 Shared map helpers | **Done** (2026-10) |
| 2 Line clearance tool | **Done** (2026-10) |
| 3 Reject bad goals | **Done** (2026-10) — `_is_goal_valid` + skip; fix skip loop (do not re-append frontier) |
| 4 Room-aware claims | **Done** (2026-10) — `_claims`, `_frontier_claimed_by_other`, claim check before `best` update; run explored much more |
| Round 2 / Step 0 | **Done** (2026-10) — SLAM params (faster map) + Nav2 `allow_unknown: true` (retest `false` later) |
| Round 2 / Step A | **Done** (2026-10) — detector 8-neighbor + 8-connected clustering + `MIN_CLUSTER_SIZE = 5` (logs now only drop sizes 1 to 4) |
| Round 2 / Step B | **Done** (2026-10) — unknown (`-1`) goal cell accepted, reject-reason logs (part of the final `_is_goal_valid`) |
| Round 2b / N1, N2, idle log | **Done** (2026-10) — reachability check, clearance check (now a distance-transform lookup), `no pick` idle log |
| Round 2c / goal candidates, per-robot reject, escape goal | **Done** (2026-10) — `_find_goal`, `rejected` sets, `_escape_goal` |
| Round 2c / shared map relay | **Done** (2026-10) — `shared_map_relay.py` in `map_merge_swarm`, Nav2 static layer reads `/TB3_n/shared_local` |
| Round 2c / TF freeze in `gt_map_tf_node` | **Done** (2026-10-09) — transform held while a robot moves, updated after 1 s at rest; fixed the "rotated rooms" |
| Scan topic check (`/scan` vs `/TB3_n/scan` in costmap layers) | **Done** (2026-10-09) — `/scan` had 0 publishers, costmap layers now use `/TB3_n/scan` (5 subscribers each: SLAM + 4 layers) |
| Round 2 / Step C | **Done** (2026-10-12) — failed-goal blacklist, plus a 300 s visited cooldown after success |
| Round 2 / Step D | **Done** (2026-10-12) — stuck watchdog (20 s, 0.3 m, 3 strikes) |
| Round 2d | **Done** (2026-10-12) — `use_astar: true`, inflation 0.25 / 8.0, transform tolerances, `CLAIM_RADIUS` 0.8, clearance (0.35, 0.28), `MIN_CLUSTER_SIZE` 10 (see `# Round 2d`) |
| 5 Assignment loop polish | **Done** (2026-10-14) — Soft cost (`d_self + gap_cost + wall_cost + teammate_penalty`), teammate bubble check (`d_other < 1.2 and d_self > 2.5`), 2-attempt ghost shadow blacklist |
| 6 Redirect to closer frontier while busy | **Done** (2026-10-14) — In-flight redirect for busy robots (`d_current > 3.0m`, `d_new < 1.5m`, same room, cancel + immediate reassign in `_on_result`) |
| Round 2e / 3-Robot Swarm Polish | **Done** (2026-10-14) — `MIN_CLUSTER_SIZE = 7`, `now + 0.1s` success cooldown, true robot radius 0.13m, table leg collision avoidance |
| Video Recording & Milestone Demo | **Done** (2026-10-15) — Recorded full 3-robot house exploration and posted demo on LinkedIn |
| Frontier white/unknown edges | **Done** — detector work, handled by Round 2 / Step A |

**Latest status and next session notes:** search for `# Round 2e`.

**Where the new notes are:** search for `# Round 2 update` (just before Pattern 5). It has the SLAM change log, detector steps, goal fixes, blacklist, and the order.

**File to change (Patterns 1–6):** `src/explore_swarm/explore_swarm/goal_assigner.py`  
**Frontier detection (separate):** `src/explore_swarm/explore_swarm/frontier_detector.py`  
**Rebuild after each pattern:**

```bash
cd ~/swarm-search
source /opt/ros/humble/setup.bash
colcon build --packages-select explore_swarm
source install/setup.bash
```

**Do not** fix this only in Nav2 params; the assigner chooses world goals without checking `/shared_map` today (until you add these patterns).

---

## Problem you saw

1. Frontier on the **far side of a wall** → small backoff still sends Nav2 through the wall → hit → tip.
2. Two robots both chase **nearby frontiers in the same room** → collisions; meanwhile another room is ignored.
3. Pure **distance** “exclusion zone” is wrong: two robots **close through a wall** should **both** explore (one room each).

Related context: robots tip when they **hit** something (wall, furniture, other robot), not from random speed. Inflation helps wall clearance for valid paths; it does **not** fix a goal on the wrong side of a wall.

---

## Where to change

| File | Role |
|------|------|
| `src/explore_swarm/explore_swarm/goal_assigner.py` | Patterns 1–6 |
| `src/explore_swarm/explore_swarm/frontier_detector.py` | Free↔unknown frontiers (separate from 1–3) |
| Rebuild | `colcon build --packages-select explore_swarm` + re-source `install/setup.bash` |

---

## Order to implement (learn step-by-step)

1. ~~Subscribe `/shared_map` + helpers (**Pattern 1**)~~ **Done**
2. ~~`line_crosses_occupied` (**Pattern 2**)~~ **Done**
3. ~~Reject bad goals: free cell + frontier↔backoff (**Pattern 3**)~~ **Done** — do not re-append frontier on skip
4. ~~Add `_claims` + `frontier_claimed_by_other` (**Pattern 4**)~~ **Done**
5. ~~**Round 2 / Step 0:** SLAM params + Nav2 `allow_unknown: true` (see `# Round 2 update`)~~ **Done**
6. ~~**Round 2 / Step A:** frontier detector 8-neighbor + 8-connected clustering + `MIN_CLUSTER_SIZE`~~ **Done**
7. ~~**Round 2 / Step B:** allow unknown goal cell + reject-reason logs~~ **Done**
7b. ~~**Round 2b / 2c:** N1, N2, goal candidates, per-robot reject, escape goal, shared map relay, TF freeze~~ **Done**
8. **Scan topic check**, then **Round 2 / Step C:** failed-goal blacklist ← **next**
8b. **Round 2 / Step D:** stuck watchdog (after C, if robots still get stuck)
9. Refine loop if needed (**Pattern 5**)
10. Redirect while busy (**Pattern 6**, optional)
11. (Later) ROS parameters for `CLAIM_RADIUS`, occupied threshold, `MIN_CLUSTER_SIZE`, blacklist time

### Important correction (multi-room)

| Wrong idea | Right idea |
|------------|------------|
| Reject if straight line **robot → goal** crosses a wall | That blocks valid goals in **other rooms** (Nav2 should path **around** through doors) |
| Check **frontier → backoff goal** + goal cell free | Backoff stays on the **same side** as the frontier; don’t send Nav2 into a wall cell |
| Assigner must validate full Nav2 path | Planner follows costmap; assigner only sends a **sensible end point** |
| Patterns 1–3 fix missing frontiers in a room | **No** — that is `frontier_detector.py` |
---

# Pattern 1 — Shared map helper (foundation)

## Why

`/shared_map` is a grid: each cell is free (`0`), unknown (`-1`), or occupied (wall, often `100`).  
To check “does this line cross a wall?” you need that grid in the assigner.

The assigner decides “go here.” To avoid walls it must look at **`/shared_map`**.

That map is a **grid of numbers**, not meters. So you need:

1. **Get** the map into the node  
2. **Convert** robot meters → grid column/row  
3. **Read** the number in that cell  

That’s all Pattern 1 is. Nothing about frontiers or Nav2 goals yet.

## What to add

Keep latest `OccupancyGrid` from **`/shared_map`** (same QoS as `frontier_detector.py`: reliable + transient local).

- `world_to_cell(x, y, map) -> (i, j)` using `origin`, `resolution`, `width`, `height`.
- `cell_value(i, j) -> int | None` (`None` if out of bounds).
- `is_occupied(val)` → e.g. `val is not None and val >= 50` (match what you use elsewhere; 100 = wall).
- `is_free(val)` → `val == 0`.

Subscribe in `__init__`; in callback store `self._shared_map = msg`. If map is `None`, skip assignment or skip checks (log once).

## Each piece — what / why / how

### 1. `self._shared_map = None`

**What:** A box on the node that will hold the latest map message.

**Why:** Without storing it, every helper has nothing to read. Starts as `None` because merge (T7) might not be running yet.

**How:** In `__init__`, set `self._shared_map = None`. Later the callback fills it.

### 2. `map_qos` (QoS profile)

**What:** Rules for how ROS delivers `/shared_map` (reliability, “keep last message for late subscribers”).

**Why:** Your merge node publishes with **reliable + transient_local**. If the assigner uses default QoS (`depth=10`), it often **never receives** the map. Same QoS as `frontier_detector.py`.

**How:** Copy the `QoSProfile(...)` from frontier detector; pass it into `create_subscription(..., map_qos)`.

### 3. `create_subscription(OccupancyGrid, "/shared_map", self._on_shared_map, map_qos)`

**What:** “When a new `/shared_map` arrives, call `_on_shared_map`.”

**Why:** That’s how the assigner **gets** the map. No subscription → map stays `None` forever.

**How:** One line in `__init__` after you define `map_qos`.

### 4. `_on_shared_map(self, msg)`

**What:** Callback that runs every time a new map is published.

**Why:** Save `msg` into `self._shared_map` so helpers can use it later (when assigning goals).

**How:**

```python
def _on_shared_map(self, msg):
    self._shared_map = msg
```

The “log once: width/height” part is only so **you** can see it worked. Not required for the logic.

### 5. `_world_to_cell(self, x, y)`

**What:** Input: position in **meters** in `shared_map` frame (like robot pose).  
Output: grid indexes `(i, j)` — column and row — or `None` if off the map.

**Why:** The map is stored as a flat list of cells. You can’t ask “value at x=1.2 m”; you must ask “value at cell (47, 120).”

**How (idea):**

- Map has `origin` (world coords of cell 0,0) and `resolution` (meters per cell, e.g. 0.05).
- `i = floor((x - origin.x) / resolution)`  
- `j = floor((y - origin.y) / resolution)`  
- If `i`/`j` outside width/height → return `None`.

**Analogy:** GPS lat/lon → “which square on a chessboard.”

### 6. `_cell_value(self, i, j)`

**What:** Input: cell `(i, j)`. Output: occupancy number, or `None` if invalid.

Typical values:

| Value | Meaning |
|-------|---------|
| `0` | free (driveable) |
| `-1` | unknown |
| `100` (or ≥50) | occupied (wall) |

**Why:** Later: “is this cell a wall?” Pattern 1 only needs to **read** the number.

**How:** Occupancy grids are stored row-major:

```text
index = j * width + i
return data[index]
```

### 7. `_verify_shared_map` + timer (temporary only)

**What:** Every 2 seconds: get TB3_1 pose → cell → print value.

**Why:** You’re new — you need proof it works **without** building Pattern 2 yet. This is a **test flashlight**, not part of the real algorithm. Delete after you see `value=0`.

**How:**

1. `create_timer(2.0, self._verify_shared_map)` in `__init__`
2. Inside: if no map → warn; else `_get_robot_pose` → `_world_to_cell` → `_cell_value` → log

You already have `_get_robot_pose` — it returns `(x, y)` in `shared_map` meters. That’s the right input for `_world_to_cell`.

## How they chain together

```text
T7 publishes /shared_map
        ↓
_on_shared_map saves it → self._shared_map
        ↓
_verify (or later: Pattern 2) asks:
  robot at (rx, ry) meters
        ↓
_world_to_cell(rx, ry) → (i, j)
        ↓
_cell_value(i, j) → 0 / -1 / 100
```

## What is **not** necessary for Pattern 1

| Thing | Needed now? |
|-------|-------------|
| Changing `_on_frontier` / sending goals | No |
| Line through wall | No (Pattern 2) |
| Claim radius | No (Pattern 4) |
| `_verify_shared_map` forever | No — only to learn/check |

**Necessary forever for later patterns:** `_shared_map`, subscription + QoS, `_on_shared_map`, `_world_to_cell`, `_cell_value`.

## Order to write Pattern 1

1. Imports + `map_qos` + subscribe + `_on_shared_map` → rebuild → log “map received” after T7.  
2. Add `_world_to_cell` + `_cell_value`.  
3. Add `_verify_shared_map` timer → see `value=0` for TB3_1 on open floor.  
4. Remove verify timer when happy.

## Pattern 1 — code to copy

### Imports (add QoS next to your existing ones)

```python
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import (
    QoSProfile,
    QoSReliabilityPolicy,
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
)
```

### In `__init__` (subscribe block)

```python
self._shared_map = None
self._logged_map_once = False

map_qos = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
)
self.create_subscription(OccupancyGrid, "/shared_map", self._on_shared_map, map_qos)
self.get_logger().info("goal_assigner listening on /shared_map")

# Pattern 1 verify: every 2s check robot cell on shared_map
self.create_timer(2.0, self._verify_shared_map)
```

### Callbacks + helpers (add after `__init__`)

```python
def _on_shared_map(self, msg: OccupancyGrid):
    self._shared_map = msg
    if not self._logged_map_once:
        self._logged_map_once = True
        info = msg.info
        self.get_logger().info(
            f"shared_map received w={info.width} h={info.height} "
            f"res={info.resolution:.3f} "
            f"origin=({info.origin.position.x:.2f},{info.origin.position.y:.2f})"
        )


def _world_to_cell(self, x: float, y: float):
    if self._shared_map is None:
        return None
    info = self._shared_map.info
    i = int((x - info.origin.position.x) / info.resolution)
    j = int((y - info.origin.position.y) / info.resolution)
    if i < 0 or i >= info.width or j < 0 or j >= info.height:
        return None
    return (i, j)


def _cell_value(self, i: int, j: int):
    if self._shared_map is None:
        return None
    info = self._shared_map.info
    if i < 0 or i >= info.width or j < 0 or j >= info.height:
        return None
    return self._shared_map.data[j * info.width + i]


def _verify_shared_map(self):
    """Temporary — delete after Pattern 1 passes."""
    if self._shared_map is None:
        self.get_logger().warn(
            "verify: _shared_map is None (is T7 map merge running?)",
            throttle_duration_sec=5.0,
        )
        return
    try:
        rx, ry = self._get_robot_pose("TB3_1")
    except Exception as e:
        self.get_logger().warn(f"verify: TF failed: {e}", throttle_duration_sec=5.0)
        return
    cell = self._world_to_cell(rx, ry)
    if cell is None:
        self.get_logger().warn(
            f"verify: TB3_1 ({rx:.2f},{ry:.2f}) outside map",
            throttle_duration_sec=5.0,
        )
        return
    i, j = cell
    val = self._cell_value(i, j)
    self.get_logger().info(
        f"verify: TB3_1 ({rx:.2f},{ry:.2f}) -> cell ({i},{j}) value={val}",
        throttle_duration_sec=2.0,
    )
```

## Pattern 1 — verify at the end

Rebuild:

```bash
cd ~/swarm-search
source /opt/ros/humble/setup.bash
colcon build --packages-select explore_swarm
source install/setup.bash
```

Bring up T0–T7, then T9. Watch goal_assigner logs.

**Pass if all of these are true:**

| # | Check | How |
|---|--------|-----|
| 1 | Node starts | T9: `ros2 launch explore_swarm explore.launch.py` — no import/crash errors |
| 2 | Map arrives | In goal_assigner logs you see something like `shared_map received w=… h=… res=…` (after T7 is up) |
| 3 | Map stays None until T7 | Start T9 before T7 → no “received” log (or still None); start T7 → log appears |
| 4 | Cell read works | Temporarily in `_on_shared_map` or a 2 s timer: take robot pose via existing `_get_robot_pose("TB3_1")`, convert with `_world_to_cell`, `_cell_value` — expect **0** (free) if robot is standing in open floor; **None** if out of map |
| 5 | Wall cell works (optional) | Pick a known wall world point from RViz `/shared_map` (hover / estimate), convert → cell value **≥ 50** or **100** |

**Fail if:**

- Never logs map received while `/shared_map` is publishing (`ros2 topic echo /shared_map --once` works) → QoS mismatch (copy QoS from `frontier_detector.py`).
- Always `None` for in-house robot pose → wrong `origin`/`resolution` math or wrong frame (poses must be in `shared_map`).
- Crash on subscribe → missing import / wrong message type.

| Log | Meaning |
|-----|---------|
| `shared_map received w=…` | Subscribe + QoS OK |
| `verify: … value=0` | Robot on free cell — ideal Pattern 1 pass |
| `verify: … value=-1` | Helpers work; that cell is still **unknown** on merge (OK early) — see study notes |
| `verify: _shared_map is None` | Merge not up yet, or QoS wrong (often only first 1–2 s) |
| `verify: TF failed…` | TF not ready yet — normal at start if `try/except` handles it |
| `verify: … outside map` | Pose/origin mismatch |
| `value=100` or high | Robot on/near wall (or tip) — try open floor |

**Minimal “done” bar:** Pattern 1 helpers work when verify prints something like:

```text
verify: TB3_1 (-1.72,-0.47) -> cell (165,190) value=-1
```

(or `value=0`). That means map + world→cell + cell read all work. Ideal later: wait until the robot sits on white free space in RViz and see `value=0`.

Then remove the temporary debug log/timer and move to Pattern 2.

---

## Study notes — Pattern 1 (from real run, 2026-10)

### Status

- **Pattern 1: done / working** (helpers + verify log).
- **Next:** Pattern 2 (`line_crosses_occupied`), then Pattern 3 (reject goals through walls).
- Pattern 1 does **not** stop wall hits or tip-overs. Goals are still assigned the old way until Patterns 2–3.

### Naming — the #1 beginner crash

Python attribute names must match **exactly**. Underscore matters.

| Wrong | Right | Error you saw |
|-------|--------|----------------|
| Call `_world_to_cell` but def is `world_to_cell` | Same name both places | `no attribute '_world_to_cell'. Did you mean: 'world_to_cell'?` |
| Use `self.shared_map` | Use `self._shared_map` everywhere | `no attribute 'shared_map'. Did you mean: '_shared_map'?` |

**Rule:** Pick one name (`_shared_map`, `_world_to_cell`, `_cell_value`) and use it in **def + every call + every field access**.

After every edit of `goal_assigner.py`:

```bash
colcon build --packages-select explore_swarm
source install/setup.bash
```

If you forget rebuild, ROS still runs the **old** code under `install/…` and you will be confused.

### Shared map geometry (why cell 165,190 is correct)

From `map_merge_swarm` merge node:

| Constant | Value |
|----------|--------|
| Resolution | `0.05` m |
| Origin | `(-10.0, -10.0)` |
| Size | 400 × 400 cells |

Example: robot at `(-1.72, -0.47)`:

```text
i = int((-1.72 - (-10)) / 0.05) = 165
j = int((-0.47 - (-10)) / 0.05) = 190
```

If verify prints that cell for that pose, **world_to_cell math is correct**.

### Occupancy values (memorize)

| Value | Meaning |
|-------|---------|
| `0` | Free (driveable) |
| `-1` | Unknown (not mapped yet on `/shared_map`) |
| `≥ 50` / `100` | Occupied (wall / obstacle) |

### Why verify showed `value=-1` (not a Pattern 1 failure)

- Merge canvas starts filled with **unknown** (`-1`).
- Free (`0`) appears only after SLAM data is fused into `/shared_map`.
- Early in the run, the robot pose can sit on a cell still marked unknown even if Gazebo looks like open floor.
- So: **infrastructure pass** with `-1`; wait / drive / let mapping catch up if you want to see `0`. Cross-check in RViz: Fixed Frame `shared_map`, topic `/shared_map` — gray at robot → `-1` is correct.

### Startup warnings that are normal

| Log | Meaning | Action |
|-----|---------|--------|
| `verify: _shared_map is None` | Timer ran before first map | Wait; should stop after T7 |
| `"shared_map" … does not exist` | GT TF / merge not ready | Wait |
| `extrapolation into the past` | TF buffer slightly behind sim time | Handled by try/except; ignore if brief |

### Mental model (keep this)

```text
T7 publishes /shared_map
        ↓
_on_shared_map → self._shared_map
        ↓
robot pose (meters in shared_map) via TF
        ↓
_world_to_cell → (i, j)
        ↓
_cell_value → 0 / -1 / 100
```

Patterns 2–4 ask: “along **this** line, do we hit an occupied cell?” using the same helpers. Which line matters:

| Use | Line | Purpose |
|-----|------|---------|
| Pattern 3 | **frontier → backoff goal** | Goal stays same side as frontier; not in/through wall |
| Pattern 3 | (not used) robot → goal | Would block multi-room around-the-door paths |
| Pattern 4 | claim → frontier | Same room vs other side of wall for exclusion radius |

### Related (outside Pattern 1)

- Tip cause for video: **contact** (wall / furniture / other robot), not random speed.
- Nav2 `inflation_radius` raised to **0.45** in `burger_tb3_*.yaml` — helps paths stay off walls when the **goal is valid**; does **not** replace wall line-check in the assigner.
- House video plan: `docs/P2-video-plan-house-3robots.md`.

### Checklist before Pattern 2

- [ ] No AttributeError on launch
- [ ] See `verify: TB3_1 (…) -> cell (…) value=…` at least once
- [ ] Understand `-1` vs `0` vs wall
- [ ] Remove `_verify_shared_map` timer when ready (optional but cleaner)
- [ ] Do **not** expect fewer wall hits until Pattern 3 is wired into `_on_frontier`

---

# Pattern 2 — Line clearance tool (`_line_crosses_occupied`)

**Status:** **Done** (helper in `goal_assigner.py`). Implement Pattern 3 next to actually use it.

## Big picture

Pattern 1 answers: “What is at **one** point `(x, y)`?”  
Pattern 2 answers: “If I draw a **straight line** from A to B on `/shared_map`, do I hit a **wall**?”

```text
A -------------------- B
         ↓
  walk cell by cell
         ↓
  any occupied? → True
  none?         → False
```

You reuse `_world_to_cell` and `_cell_value` in a loop.

## Why

You need a reusable “does this segment cross a wall?” helper for:

- Pattern 3: line **frontier → backoff goal**
- Pattern 4: line **claim → frontier** (same room?)

You are **not** using this (in Pattern 3) as “robot must see the goal in a straight line.” Nav2 may still drive **around** walls through doors.

## What

Add one method on `GoalAssigner`:

```text
_line_crosses_occupied(start_x, start_y, end_x, end_y) -> bool
```

| Return | Meaning |
|--------|---------|
| `True` | Segment hits at least one **occupied** cell (`>= OCCUPIED_THRESH`) |
| `False` | No occupied cells on the segment |

Also add (top of file):

```python
OCCUPIED_THRESH = 50
```

## Where

**File:** `src/explore_swarm/explore_swarm/goal_assigner.py`  
**Place:** After `_cell_value`.  
**Pattern 2 only:** Do **not** call this from `_on_frontier` yet — that is Pattern 3.

## How (sampling — easiest to learn)

1. If `_shared_map is None` → return `True` (safer: treat as blocked).
2. If start and end almost the same point → return `False`.
3. Step ≈ `resolution / 2` along the segment in world meters.
4. At each sample: `_world_to_cell` → `_cell_value`.
5. If cell is `None` (off map) or `occupancy >= OCCUPIED_THRESH` → return `True`.
6. Else after all samples → return `False`.

Do **not** treat `-1` (unknown) as occupied in this helper unless you decide later; walls are `>= 50`.

## Code sketch (readable names)

```python
OCCUPIED_THRESH = 50  # cell value >= this counts as a wall


def _line_crosses_occupied(self, start_x, start_y, end_x, end_y) -> bool:
    """
    Walk in small steps from start to end on /shared_map.
    Return True if any sample lands on an occupied (wall) cell.
    """
    if self._shared_map is None:
        return True  # no map yet → treat as blocked (safe)

    delta_x = end_x - start_x
    delta_y = end_y - start_y
    distance_m = math.sqrt(delta_x * delta_x + delta_y * delta_y)

    if distance_m < 1e-6:
        return False  # same point → nothing to cross

    resolution = self._shared_map.info.resolution  # meters per cell
    step_m = resolution * 0.5  # sample every half cell
    num_steps = max(1, int(distance_m / step_m))

    for step_index in range(num_steps + 1):
        fraction_along_line = step_index / num_steps  # 0.0 = start, 1.0 = end
        sample_x = start_x + fraction_along_line * delta_x
        sample_y = start_y + fraction_along_line * delta_y

        cell = self._world_to_cell(sample_x, sample_y)
        if cell is None:
            return True  # off the map → blocked

        cell_col, cell_row = cell
        occupancy = self._cell_value(cell_col, cell_row)
        if occupancy is None or occupancy >= OCCUPIED_THRESH:
            return True  # wall (or invalid)

    return False  # never hit a wall
```

### Name cheat sheet

| Short name | Clear name | Meaning |
|------------|------------|---------|
| `x0,y0` | `start_x, start_y` | Line start (e.g. frontier) |
| `x1,y1` | `end_x, end_y` | Line end (e.g. backoff goal) |
| `dx, dy` | `delta_x, delta_y` | End − start |
| `dist` | `distance_m` | Length in meters |
| `n` | `num_steps` | How many intervals |
| `k` | `step_index` | Which sample |
| `t` | `fraction_along_line` | 0 = start, 1 = end |
| `val` | `occupancy` | Map cell value |

## Temporary verify (like Pattern 1)

Log two cases (timer or one-shot after map exists):

1. Short line in open floor → expect **`False`**
2. Line that clearly goes through a house wall in RViz → expect **`True`**

Delete verify after it works.

## Pattern 2 — verify pass / fail

| Result | Meaning |
|--------|---------|
| Open line → `False`, wall line → `True` | **Pattern 2 pass** |
| Always `False` through walls | Step too coarse, wrong thresh, or map not updating |
| Always `True` in open space | Treating unknown as wall, or map `None` |

**Done when:** helper returns sensible True/False. Robots may **still** plan through walls until Pattern 3.

## What Pattern 2 does **not** do

| Not yet | When |
|---------|------|
| Stop sending bad Nav2 goals | **Pattern 3** |
| Claim radius | Pattern 4 |

---

## Study notes — Pattern 2 (from real run, 2026-10)

### Status

- **`_line_crosses_occupied` exists** in `goal_assigner.py` and is correct as a **tool**.
- It is **not called** from `_on_frontier` yet → assignment is unchanged.
- **R2 can still hit walls** after Pattern 2 only. That is expected, not a failed Pattern 2.

### Why R2 still hits the wall after Pattern 2

```text
Pattern 2 = "I know how to test a line"
Pattern 3 = "Before send_goal, refuse bad goals using that test"
```

Without Pattern 3, the assigner still does: nearest frontier → backoff → `_send_goal`. Bad end points still go to Nav2 → path into wall → tip.

### Multi-room reminder (do not misuse the tool)

| Wrong | Right |
|-------|--------|
| Reject if **robot → goal** line crosses wall | Blocks valid other-room goals (Nav2 goes around doors) |
| Check **frontier → backoff goal** + goal cell free | Pattern 3 — keeps goal on frontier’s side / not in wall |

### Nav finished status (seen in logs)

| `status` | Meaning |
|----------|---------|
| `4` | Succeeded |
| `6` | Aborted (stuck / blocked / bad goal / etc.) |

Seeing `assign` + `goal accepted` after Pattern 2 only means the **old** loop is still running — not that the line check is filtering.

### Checklist before Pattern 3

- [x] `OCCUPIED_THRESH` + `_line_crosses_occupied` in the file
- [x] Rebuild + run without crash
- [x] Wire checks into `_on_frontier` (**Pattern 3**)
- [x] Do **not** re-append frontier on skip (avoids infinite loop)
- [ ] Pattern 4 claims when ready

---

# Pattern 3 — Reject bad Nav2 goals (fix R2 through-wall)

**Status:** **Done** (2026-10). Code: `_is_goal_valid` + skip in `_on_frontier`.

## Big picture / why your intuition is right

R2 failed because the **goal end point** was bad (frontier / backoff on wrong side or into wall), and Nav2 still made a bad plan.

Multi-room explore must still work: robot in room A, frontier in room B → Nav2 paths **around** through a door. So:

| Check | Do it? |
|-------|--------|
| Goal cell is **free** (`0`) | **Yes** |
| Line **frontier → backoff** does **not** cross occupied | **Yes** — backoff stays same side as frontier |
| Line **robot → goal** does **not** cross occupied | **No** — would block other rooms |
| Full Nav2 path validation in assigner | **No** (optional later); planner uses costmap |

```text
frontier  --0.4m backoff-->  GOAL (gx,gy)   ← check THIS segment + goal cell
robot     ~~~~~~~~ around doors ~~~~~~~~>  GOAL   ← Nav2 path, not assigner LOS
```

## Why

`_backoff_goal` only moves ~0.4 m toward the robot. It does not check the map. If the frontier is on the far side of a wall, the goal can still be wrong / in wall → hit → tip.

## Where

**File:** `goal_assigner.py`  
In `_on_frontier`, after:

```python
backoff = self._backoff_goal(frontier_xy, (rx, ry))
gx, gy = backoff
```

**before** `_transform_pose_to_robot_frame` / `_send_goal`.

## What was added

```text
def _is_goal_valid(frontier_xy, goal_xy) -> bool:
    1) goal cell free (value == 0)
    2) not _line_crosses_occupied(fx, fy, gx, gy)   # frontier → goal only
```

```text
if not self._is_goal_valid(frontier_xy, backoff):
    idle.append((ns, rx, ry))
    # do NOT frontiers.append(frontier_xy)  # critical — see study notes
    continue
```

## How — each piece

### `_is_goal_valid`

**What:** True only if the Nav2 end pose is a sensible free cell on the frontier’s side of walls.  
**Why:** Stops through-wall / in-wall goals without blocking around-the-door missions.  
**How:**

1. `cell = _world_to_cell(gx, gy)`; if `None` → False.  
2. `_cell_value(...) == 0` (strict free; optional: also reject `-1`).  
3. `not _line_crosses_occupied(fx, fy, gx, gy)`.  
4. **Do not** call `_line_crosses_occupied(rx, ry, gx, gy)` for this reject rule.

### Assignment loop change

Pick nearest → backoff → **if not `_is_goal_valid`, skip that frontier and try another** → else send.

## Pattern 3 — verify

1. Rebuild, T0–T9.  
2. Logs show `skip bad goal` for some frontiers, then later `assign` to another.  
3. Robot **can** still get a goal in another room (path goes around) — that is success.  
4. Node must **not** freeze repeating the same skip forever.

## What Pattern 3 does **not** do

- Claim / keep robots in different rooms (Pattern 4).  
- Redirect a busy robot to a newly closer frontier (Pattern 6).  
- Create frontiers where free touches unknown in a whole room (detector — separate).  
- Guarantee zero physical scrapes on long Nav2 paths (inflation + map quality still matter).

---

## Study notes — Pattern 3 (from real run, 2026-10)

### Infinite loop / “everything stuck”

**Bug:** on skip you did:

```python
idle.append((ns, rx, ry))
frontiers.append(frontier_xy)  # BAD
continue
```

Same robot + same nearest frontier go back into the lists → `while` retries forever → node stuck (logs only every 2 s due to throttle).

**Fix:**

```python
idle.append((ns, rx, ry))
# do NOT frontiers.append(frontier_xy)
continue
```

Drop that frontier for this round; try the next one.

| On skip | Put robot back? | Put frontier back? |
|---------|-----------------|--------------------|
| Try other frontiers | Yes | **No** |
| Put frontier back | — | Risk of infinite loop if still nearest |

### Why R2 skipped close frontiers and went to another room

Close frontiers in front of the robot were often **invalid** after backoff (goal cell not `0`, or frontier→goal crosses occupied), or filtered by `MIN_GOAL_DIST` / `dist_goal < 0.5`. Pattern 3 correctly skips them; the next **valid** frontier may be in another room.

Also: once `_busy`, new closer frontiers are **ignored** until nav finishes — no redirect yet (Pattern 6).

### Hit wall while going to another room

Still possible: Pattern 3 does not validate the full Nav2 path. Thin walls / map gaps can still tip the robot. Inflation helps; redirect (Pattern 6) helps keep robots local when local valid frontiers exist.

### Checklist after Pattern 3

- [x] `_is_goal_valid` wired before `_send_goal`
- [x] Skip does **not** re-append frontier
- [x] See mix of `skip` and `assign` without freeze
- [ ] Next: Pattern 4 claims **or** Pattern 6 redirect (your choice; claims planned next in table)

---

# Outside this doc — white / unknown frontiers (detector)

**This is not Pattern 3 and not fixed by Patterns 1–5.**

| Symptom | Cause | Where to fix |
|---------|--------|--------------|
| Room looks unexplored / gray; few or no frontier markers on free↔unknown edge | Detector only marks free (`0`) cells with a **4-neighbor** unknown (`-1`) | `frontier_detector.py` `_frontier_cells` |
| Straight white–gray door edge, empty markers | Corner-only / thin edge / merge labeling | Same; plan mentions optional **8-neighbor** or morphology — see `docs/P2-video-plan-house-3robots.md` |
| Robot leaves room because no local frontier passed `_is_goal_valid` | Mix of map + detector + Pattern 3 | Improve map/frontiers first; then assigner |

Do **not** expect assigner wall-checks alone to make “full room” frontiers appear.

---

# Pattern 4 — Room-aware claims (not pure radius)

**Status:** **Done** (2026-10). Code is in `goal_assigner.py`: `self._claims = {}` in `__init__`, `_frontier_claimed_by_other`, claim set in `_send_goal(ns, pose_robot, claim_xy)` (pass `backoff`, shared_map frame, not `pose_robot`), claim cleared in `_on_goal_response` (rejected) and `_on_result`.

**Gotchas we hit (keep):**

- Claim check must come **before** the `best = (d2, i, j)` update in `_nearest_frontier_for_ns`. Filters first, then pick the winner.
- Use one dict name everywhere: `_claims`. A mix of `_claimed` and `_claims` gives an `AttributeError`.
- Create `self._claims = {}` once, outside the `for robot_ns` loop.
- Pattern 4 stops robots crowding one room. It does **not** stop wall hits and does **not** create missing frontiers.

## Why

Robot already exploring a wing should “own” nearby frontiers in **that** space; another robot should work elsewhere, unless separated by a **wall**.

**Problem with pure distance claim:** two robots can be **1 m apart through a wall** (different rooms). Euclidean claim would wrongly say “that area is taken,” and the second robot would leave its own room’s frontiers alone.

**Better rule — claim only if same side of wall:**

When deciding if frontier `F` is “already covered” by robot A’s claim `C` (or A’s pose):

1. If distance(`F`, `C`) **>** `CLAIM_RADIUS` → not claimed (other robot can take it).
2. If distance **≤** `CLAIM_RADIUS`:
   - Sample cells on the line `F` → `C` on `/shared_map`.
   - If the line **crosses occupied** (a wall) → **different rooms** → **do not claim**; other robot **may** take `F`.
   - If the line is all **free** (same open space) → **claimed**; skip `F` for this robot.

**Do not** reject explore goals because robot→goal crosses a wall (that’s multi-room Nav2’s job). Claims only use line checks between **claim/frontier points**, not robot→goal LOS.

**Net effect for the house:**

| Situation | Result |
|-----------|--------|
| Same room, near another robot’s goal | Second robot gets a **different** frontier / area |
| Opposite sides of a wall, both “near” | **Both** can explore; one per room |
| Bad backoff through wall (Pattern 3) | **Reject** that goal; try another frontier |

So: **claim radius + line-of-sight on `/shared_map`**, not radius alone.

Good defaults to try: `CLAIM_RADIUS` ≈ **2.5 m**, occupied threshold ≥ **50**.

## State to add on `GoalAssigner`

```text
self._claims: dict[str, tuple[float, float]]  # ns -> (gx, gy) in shared_map
```

**When to set:** On successful `_send_goal`, store claim at **backoff goal** `(gx, gy)` for that `ns`.

**When to clear:** `_on_result`, goal rejected in `_on_goal_response`, or new goal replaces old.

## Constants (tune later, declare at top or ROS params)

```text
MIN_GOAL_DIST = 1.0      # existing: robot too close to frontier
CLAIM_RADIUS = 2.5       # meters; only matters if same open space
OCCUPIED_THRESH = 50
```

## What to implement

```text
def frontier_claimed_by_other(fx, fy, robot_ns, claims, map) -> bool:
    for other_ns, (cx, cy) in claims.items():
        if other_ns == robot_ns:
            continue
        if dist(fx, fy, cx, cy) > CLAIM_RADIUS:
            continue
        # Same "region" only if no wall between frontier and their claim
        if not line_crosses_occupied(fx, fy, cx, cy, map):
            return True
    return False
```

**Optional (stronger):** Also treat **busy** robots’ **current pose** as a claim center with the same rule (so idle robot doesn’t pick a frontier 0.5 m from a busy teammate in the same room).

**Where in loop:** In `_nearest_frontier_for_ns` (or replacement), skip frontier `j` if `frontier_claimed_by_other(fx, fy, ns, self._claims, map)`.

**Interaction with greedy loop:** After assigning robot A, add A’s claim **before** picking for robot B in the same `_on_frontier` callback so B doesn’t pick the same cluster.

---

# Round 2 update (2026-10): detector, SLAM tuning, goal fixes

**Read this before Pattern 5.** Pattern 4 is done and the run explored much more of the house. New problems showed up, and most of them are NOT assigner patterns. This section records what we found, what we changed, why, and the exact order to do things.

## What happened in the run (symptoms)

| Symptom | Real cause | Fixed by |
|---------|------------|----------|
| Map painted only small patches, stale | SLAM only integrates a scan after 0.5 m / 0.5 rad of motion and publishes every 5 s | SLAM params (Step 0) |
| Nav2 spammed "failed to create plan", spin recoveries, retries | `allow_unknown: false` while goals sit at the edge of unknown space | Nav2 param (Step 0) |
| Closer frontiers ignored, robot drives to another room | Backoff goal cell lands on `-1` (unknown) and `_is_goal_valid` demands exactly `0` | Step B |
| White straight line at a room entrance, no frontier markers there | Detector uses 4-neighbors only, so diagonal/staircase edges split into tiny clusters, and `MIN_CLUSTER_SIZE = 10` throws them away | Step A |
| R3 never leaves its room, nearest frontiers far and behind a wall | Nav2 fails (status 6), 3 s cooldown, then the same far frontier is picked again, forever | Step C |

## Order to do things

| Step | What | File | Status |
|------|------|------|--------|
| 0a | SLAM param changes (below) + restart SLAM | slam_toolbox yaml per robot | Done |
| 0b | Nav2 `allow_unknown: true` (below) | `nav2_params/burger_tb3_*.yaml` | Done |
| A | Detector: 8-neighbor edges + 8-connected clustering + smaller `MIN_CLUSTER_SIZE` | `frontier_detector.py` | Done |
| B | Allow unknown (`-1`) as goal cell + reject-reason logs | `goal_assigner.py` | Done |
| C | Failed-goal blacklist | `goal_assigner.py` | **Next** |
| D | Stuck watchdog (see Round 2c) | `goal_assigner.py` | After C, if needed |
| 5 | Pattern 5, assignment loop polish | `goal_assigner.py` | After C |
| 6 | Pattern 6, redirect while busy | `goal_assigner.py` | Later, optional |

Why detector before Pattern 5: Pattern 5 only reorders how frontiers are tried. If the door frontier does not exist, no loop order can find it.

Rebuild after every edit, same as before:

```bash
cd ~/swarm-search
source /opt/ros/humble/setup.bash
colcon build --packages-select explore_swarm
source install/setup.bash
```

Small cleanup: `goal_assigner.py` has a stray first line `from re import T`. Nothing uses it. Delete it.

---

## Step 0a: SLAM params we changed (and why)

File: your slam_toolbox yaml (the one with `solver_plugin: solver_plugins::CeresSolver`). Change these four:

```yaml
map_update_interval: 1.0        # was 5.0
minimum_travel_distance: 0.1    # was 0.5
minimum_travel_heading: 0.1     # was 0.5
minimum_time_interval: 0.2      # was 0.5
```

**Why:**

- `minimum_travel_distance` / `minimum_travel_heading`: SLAM only adds a new scan after the robot moved or turned this much. At 0.5 the map fills in patches, even though the lidar sees 360 degrees all the time.
- `map_update_interval`: how often the map is published. At 5 s the map you see is old.
- `minimum_time_interval`: minimum time between scans that get processed.
- Knock-on effect: the merge node adds its own delay, and both `frontier_detector` and `goal_assigner` read `/shared_map`. A stale, holey map means missing frontiers and bad goal checks. So this fix helps everything downstream.

**Restart SLAM** after changing, params are read at start.

**Trade-off:** more CPU with 3 robots. If the sim lags, use `0.2` for distance/heading and `2.0` for update interval.

Not the cause (checked in your file): `max_laser_range: 20.0` is fine. Separate thing to check: the simulated lidar's own `range_max` (often about 3.5 m). Test with:

```bash
ros2 topic echo /TB3_2/scan --once
```

Look at `range_max` and how many values are `inf`. Anything farther than `range_max` never gets painted, no matter what SLAM does.

## Step 0b: Nav2 `allow_unknown` (a correction)

Earlier I suggested `allow_unknown: false`. **That was wrong for this project.** Frontiers sit at the edge of unmapped space, so with `false` the planner cannot plan to them. That is exactly the log you saw: "GridBased: failed to create plan with tolerance 0.50", then spin recovery, then retry.

In `burger_tb3_*.yaml`, `planner_server`, `GridBased` section:

```yaml
allow_unknown: true
```

Earlier note, now reversed: we dropped the "same free region" (connected components) check here. That was too fast. It is back as **N1** in Round 2b, because it does not block frontiers and it stops goals that are only reachable through an unmapped wall gap.

## Why the robot still hit the wall (notes, no code change needed yet)

You expected the lidar to stop it. In theory yes. If it still happens, check in this order:

1. Local costmap not using the lidar (wrong scan topic or frame for the namespace, like `TB3_2/scan` vs `TB3_2/base_scan`). In RViz, show the TB3_2 local costmap. If the wall in front of the robot does not light up, this is it.
2. Lidar blind spots: minimum range about 0.12 m, thin horizontal slice, so low edges and door frames can be missed.
3. Speed too high for the costmap update rate. Lower max linear speed, keep inflation at 0.45.
4. Recovery behaviors: `backup` drives backwards with no rear sensing and is a common tip cause. Limit or remove it in the behavior tree.
5. Low threshold or edge that the lidar plane passes over.

Debug trick: when it gets stuck, pause and show the laser scan, the TB3_2 local costmap and the map together. Scan shows the wall but costmap does not: cause 1. Costmap shows it and the robot still hits: causes 3 to 5.

---

## Step A: Detector fix (`frontier_detector.py`)

This is the "white / unknown frontiers" item from the section above, now with exact steps.

**Why the door line is missed**

The map has a straight line of free cells (the white dots) next to unknown cells. Two things in the current code break it:

1. `_cluster_cells` joins only up/down/left/right. A diagonal or staircase edge touches only at corners, so one long edge becomes many clusters of size 1 to 3.
2. `MIN_CLUSTER_SIZE = 10` means 0.5 m of continuous edge (0.05 m per cell). Every tiny piece is under that, so all are dropped, and no marker appears.

**A1. Confirm first (optional but recommended).** In `_on_map`, replace the small-cluster skip with a logging version:

```python
if len(cluster) < MIN_CLUSTER_SIZE:
    self.get_logger().info(
        f"dropped cluster size={len(cluster)}", throttle_duration_sec=1.0
    )
    continue
```

Park a robot at the door. Lots of size 1 to 3 drops means the staircase problem. No drops and still no marker means that edge is not "free next to `-1`" in `/shared_map` (see A5).

**A2. `_frontier_cells`: check 8 neighbors for unknown.** Replace the `neighbors = (...)` tuple:

```python
neighbors = (
    cell(i - 1, j),
    cell(i + 1, j),
    cell(i, j - 1),
    cell(i, j + 1),
    cell(i - 1, j - 1),
    cell(i - 1, j + 1),
    cell(i + 1, j - 1),
    cell(i + 1, j + 1),
)
```

Why: a free cell that only touches unknown at a corner now counts as a frontier cell.

**A3. `_cluster_cells`: make it 8-connected.** Change the neighbor loop to:

```python
for ni, nj in (
    (i - 1, j), (i + 1, j), (i, j - 1), (i, j + 1),
    (i - 1, j - 1), (i - 1, j + 1), (i + 1, j - 1), (i + 1, j + 1),
):
```

Why: now a diagonal edge is one cluster, not many. Also update the docstring ("group 8 connected frontier cells"). Doing only A2 and not A3 is not enough.

**A4. Lower `MIN_CLUSTER_SIZE`.** From `10` to `5` (about 0.25 m). If you then see tiny junk markers at wall corners, go up to 6 or 7. If doors still get dropped, the A1 log tells you the sizes to tune against.

**A5. If the log shows nothing dropped but still no marker.** Then the door cells are not `0` next to `-1`. Check what the merge node writes there (values 1 to 49, or gaps). Hover over the cells in RViz or print the cell values around the door.

**Verify Step A**

| Check | Pass |
|-------|------|
| RViz `/frontiers_markers` at the door line | Marker(s) visible on the white/unknown edge |
| Total markers | More than before, but not a swarm of junk |
| Detector log | `frontier_cells=... clusters_kept=...` goes up |

Rebuild, run, look at RViz. Only then go to Step B.

---

## Step B: Goal cell may be unknown (`goal_assigner.py`)

**Why:** `_is_goal_valid` requires the goal cell value to be exactly `0`. The backoff point is 0.4 m from the frontier toward the robot, and near a door edge or in a sparsely painted map it often lands on `-1`. The goal is rejected, and the robot skips close frontiers and picks far ones in fully painted areas. That matches what you saw.

**B1. In `_is_goal_valid`, change the cell check:**

```python
# before
if value != 0:
    return False

# after: accept free (0) and unknown (-1), reject walls and off-map
if value is None or value >= OCCUPIED_THRESH:
    return False
```

The wall-line check (`_line_crosses_occupied` from frontier to goal) stays. It still protects against goals behind a wall.

**B2. Add a reason to every rejection**, so you stop guessing. Examples:

```python
# in _is_goal_valid
self.get_logger().info(f"reject: goal cell value={value}", throttle_duration_sec=1.0)
self.get_logger().info("reject: frontier->goal crosses wall", throttle_duration_sec=1.0)
```

Do the same for the `dist_goal < 0.5` branch and for the claim `continue` in `_nearest_frontier_for_ns`. These logs are temporary, remove them when behavior is good.

**B3. If many `skip bad goal` logs happen right at doors:** the frontier to backoff line may clip the door frame. Try a shorter backoff in `_backoff_goal`: change `min(0.4, ...)` to `min(0.2, ...)`.

---

## Step C: Failed-goal blacklist (`goal_assigner.py`)

**Why:** R3 sits in a room where the only frontiers are far, behind a wall. Nav2 cannot route, the goal ends with status 6 (aborted), the cooldown is 3 s, and then the assigner picks the same nearest frontier again. Forever. The robot looks stuck but is really in a retry loop. A short blacklist makes it try something else.

**C1. In `__init__`**, next to `self._claims = {}`:

```python
self._blacklist = []  # (x, y, expire_time) in shared_map frame, sim seconds
```

**C2. In `_on_result`**, keep the claim before removing it, and blacklist it on failure:

```python
claim = self._claims.pop(ns, None)
status = future.result().status
now = self.get_clock().now().nanoseconds * 1e-9
if status != 4 and claim is not None:  # 4 = succeeded
    self._blacklist.append((claim[0], claim[1], now + 60.0))
```

(Keep the rest of `_on_result` as it is: log, `_busy = False`, cooldown.)

**C3. Add a helper:**

```python
def _is_blacklisted(self, fx, fy):
    now = self.get_clock().now().nanoseconds * 1e-9
    self._blacklist = [b for b in self._blacklist if b[2] > now]  # drop expired
    return any(math.hypot(fx - bx, fy - by) < 1.0 for bx, by, _ in self._blacklist)
```

**C4. In `_nearest_frontier_for_ns`**, next to the claim check:

```python
if self._is_blacklisted(fx, fy):
    continue
```

**Why 1.0 m and not smaller:** the stored claim is the backoff point, and the real frontier is about 0.4 m away from it. A tiny radius would never match the frontier again.

**Watch out:** if everything gets blacklisted the robot idles for up to 60 s. That is fine, it is what you want while the map grows. Lower 60 to 30 if it feels too slow.

**Verify B and C**

| Check | Pass |
|-------|------|
| Close frontiers get assigned again | Yes, logs show `assign` for near ones |
| Reject logs | Show a reason, and no single reason dominates forever |
| R3 | Leaves its room after its local frontiers are done, tries other frontiers instead of the same far one |
| Failed goal | Same frontier is not picked again for about 60 s |

---

## Still true after all this

- Nav2 can still scrape or tip on long paths through unmapped areas. Inflation, speed, and recovery settings matter (notes above).
- Patterns 5 and 6 come after Steps A to C. Pattern 5 can reuse the claim and blacklist checks as part of its "first valid frontier" loop.

---

# Round 2b (2026-10): fixes from the recordings (N1, N2, detector, idle log)

**Read this after Round 2.** Pruthvi recorded runs with TF on and showed the problems one by one. This section lists every issue seen, the three root causes behind them, and the fixes. Do these before Step B, Step C, and Pattern 5 (they fold into B and C, see the order at the end).

## Issues seen in the recordings

| # | Issue | Root cause |
|---|-------|------------|
| 1 | R2 tipped on a wall that is real in Gazebo but not yet in the map | 1 |
| 2 | No frontier at the white straight door edge | 2 |
| 3 | R3 stays in its room, never leaves through the door | 2 and 3 |
| 4 | Close frontiers skipped, robot goes far away | Step B (goal cell is `-1`) |
| 5 | R3 path ends behind the trash can, tight corner, tipped | 1 and clearance (N2) |
| 6 | Junk markers at corners | `MIN_CLUSTER_SIZE` tuning |
| 7 | Map sparse compared with lidar coverage | SLAM params (done) or merge, still to verify |
| 8 | R3 tipped, its room map got rotated and speckled | Result of 5, goes away when robots stop tipping |
| 9 | R2 has closer frontiers but goes to a far room; R1 goes to the same place; about 30 fake dots in the corrupted room | 1 and 8 |
| 10 | R2 lidar fan reaches the table room in Gazebo, RViz shows only a thin diagonal white strip, no frontiers there | 2 (and painting, see "Not sure yet") |
| 11 | R2 path cuts around an unmapped wall corner into R3's room, nav status 6, new goal right away | 1 |
| 12 | R1 stays idle for long, even with frontiers in unknown space | 3 |

**Three root causes**

1. **Goals only reachable through an unmapped gap.** A frontier is a free cell, so it is only reachable if known free cells connect it to the robot. Nav2 with `allow_unknown: true` happily cuts through the unknown corner where the real wall is not mapped yet.
2. **Missing frontiers.** Door and table-room edges never become markers. The log is full of `dropped cluster size=9` while `MIN_CLUSTER_SIZE = 10`, which is just under the cutoff (0.45 m of edge).
3. **Robots idle or retrying blindly.** No reason is logged for idling, and a failed goal is picked again.

## Correction to an earlier note

Earlier in Round 2 (Step 0b) we dropped the "same free region" check, saying it fights frontier exploration. **That was too fast.** It does not block exploration, because frontier cells are free cells, and it is exactly what stops R2's path through the unmapped corner. It is back as **N1** below. The Step 0b note about `allow_unknown: true` still stands for now (see "Not sure yet").

## N1: reachability through known free cells (`goal_assigner.py`)

**Why:** R3's room and R2's strip are separate blobs of free cells, because the corner between them is still unknown. A goal in R3's room is not reachable through known free space, so it should be rejected until the gap gets mapped.

**Install check:**

```bash
python3 -c "import scipy"
# if it fails:
sudo apt install python3-scipy
```

**Imports (top of file):**

```python
import numpy as np
from scipy import ndimage
```

**In `__init__`:**

```python
self._free_labels = None
```

**Replace `_on_shared_map`:**

```python
def _on_shared_map(self, msg: OccupancyGrid):
    self._shared_map = msg
    grid = np.array(msg.data, dtype=np.int8).reshape(msg.info.height, msg.info.width)
    # every blob of connected free cells (value 0) gets its own id, walls and unknown are 0
    self._free_labels, _ = ndimage.label(grid == 0)
```

**Add helpers (near `_line_crosses_occupied`):**

```python
def _labels_near(self, cell, r=3):
    i, j = cell
    # array is indexed [row, col] = [j, i], row first
    patch = self._free_labels[max(0, j - r) : j + r + 1, max(0, i - r) : i + r + 1]
    return set(np.unique(patch)) - {0}


def _reachable_via_free(self, robot_xy, goal_xy):
    if self._free_labels is None:
        return False
    rc = self._world_to_cell(*robot_xy)
    gc = self._world_to_cell(*goal_xy)
    if rc is None or gc is None:
        return False
    return bool(self._labels_near(rc) & self._labels_near(gc))
```

**Logic:** each blob of connected free cells has an id. The robot and the goal are each checked in a small patch (3 cells, 0.15 m), because the exact cell can be `-1`. If they share an id, a known free route exists. Different ids means the route would go through unknown space.

## N2: clearance check (`goal_assigner.py`)

**Why:** R3's goal sat beside the trash can in a tight corner. The goal cell was free, but the robot got stuck and tipped.

```python
def _has_clearance(self, gx, gy, radius_m=0.3):
    cell = self._world_to_cell(gx, gy)
    if cell is None:
        return False
    ci, cj = cell
    r = int(radius_m / self._shared_map.info.resolution)  # 0.3 m = 6 cells
    for dj in range(-r, r + 1):
        for di in range(-r, r + 1):
            if di * di + dj * dj > r * r:
                continue  # circle, not square
            v = self._cell_value(ci + di, cj + dj)
            if v is not None and v >= OCCUPIED_THRESH:
                return False  # obstacle too close
    return True
```

Tuning: if door frontiers get rejected, lower `radius_m` to 0.2. If robots still brush obstacles, raise to 0.35.

## Final `_is_goal_valid` (replaces the earlier version and the Step B1 snippet)

New signature takes the robot position:

```python
def _is_goal_valid(self, frontier_xy, goal_xy, robot_xy):
    gx, gy = goal_xy
    fx, fy = frontier_xy

    cell = self._world_to_cell(gx, gy)
    if cell is None:
        return False
    value = self._cell_value(*cell)
    if value is None or value >= OCCUPIED_THRESH:  # Step B: unknown (-1) is OK
        self.get_logger().info(
            f"reject: goal cell value={value}", throttle_duration_sec=1.0
        )
        return False
    if self._line_crosses_occupied(fx, fy, gx, gy):
        self.get_logger().info(
            "reject: frontier->goal crosses wall", throttle_duration_sec=1.0
        )
        return False
    if not self._has_clearance(gx, gy):
        self.get_logger().info(
            "reject: too close to obstacle", throttle_duration_sec=1.0
        )
        return False
    if not self._reachable_via_free(robot_xy, goal_xy):  # N1
        self.get_logger().info("reject: no known free route", throttle_duration_sec=1.0)
        return False
    return True
```

In `_on_frontier`, change the call to:

```python
if not self._is_goal_valid(frontier_xy, backoff, (rx, ry)):
```

Order inside the function is cheap checks first, the heavier ones later.

## Detector size

In `frontier_detector.py`: `MIN_CLUSTER_SIZE = 5`. The recording used 10, and the log showed many `size=9` drops. Those are probably the missing door and table-room edges. If you get junk markers at wall corners, try 6 or 7.

## Idle-reason log (where to put it)

**Where:** inside `_nearest_frontier_for_ns`. Make a `stats` dict at the top, count at each `continue`, and print at the end when no pick was found. Counts are robot/frontier pairs, not single frontiers.

```python
def _nearest_frontier_for_ns(self, frontiers, idle, min_d2):
    best = None
    stats = {"close": 0, "claimed": 0}  # add "blacklisted": 0 after Step C
    for i, (ns, rx, ry) in enumerate(idle):
        for j, (fx, fy) in enumerate(frontiers):
            d2 = (fx - rx) ** 2 + (fy - ry) ** 2
            if d2 < min_d2:  # too close → ignore
                stats["close"] += 1
                continue
            if self._frontier_claimed_by_other(fx, fy, ns):
                stats["claimed"] += 1
                continue
            if best is None or d2 < best[0]:
                best = (d2, i, j)
    if best is None:
        self.get_logger().info(
            f"no pick: {stats} of {len(idle) * len(frontiers)} robot-frontier pairs",
            throttle_duration_sec=2.0,
        )
    return best
```

**How to read it:** `claimed` high means teammates' claims cover everything. `close` high means frontiers are too near. Both low with no pick means no frontiers existed. Goal rejections (wall, clearance, no route) are logged separately in `_is_goal_valid`, so together these logs show why every robot idles. This explains R1 and R3 idling. It is temporary, remove it when behavior is good.

## Not sure yet

- **`allow_unknown`:** keep `true` for now. With N1 in place, try `false` once. If plans start failing, switch back to `true`.
- **Table room not painted:** cannot tell if it is SLAM or the merge node. In RViz show `/TB3_2/map` and `/shared_map` side by side. If `/TB3_2/map` shows the room but `/shared_map` does not, it is the merge node (send its code). If both are thin, check the simulated lidar `range_max` and the SLAM params again.
- **Thin gaps split a room:** a speckled map can split a real room into two blobs, so N1 would reject goals inside it. If you see many `no known free route` logs inside one room, tell me.
- **R1 and R3 idling:** the idle log decides. It could be claims, distance, or no frontiers.

## Order for this section

1. ~~N1 and N2, and the new `_is_goal_valid` (this also does Step B1 and B2)~~ **Done**
2. ~~Detector `MIN_CLUSTER_SIZE = 5`~~ **Done**
3. Step C (blacklist), then add `"blacklisted"` to the idle log stats **Not done yet**
4. ~~Idle-reason log~~ **Done**
5. Rebuild, rerun with TF on, send screenshots plus the `assign`, `reject`, `no pick` and `nav finished` lines (repeat after Step C)
6. Then Pattern 5

**Verify**

| Check | Pass |
|-------|------|
| R2 near the unmapped corner | Log shows `reject: no known free route`, robot does not drive into the wall |
| Goal beside the trash can | `reject: too close to obstacle`, no tip |
| Door and table-room edges | Markers appear with size 5 |
| R1 or R3 idle | `no pick: {...}` log tells why |
| Robots tipping | None in a full run |

---

# Round 2c (2026-10-08): what changed after Round 2b, open issues, next session

Written at the end of a long session. Read this first when you continue.

## Implemented since Round 2b (all in `goal_assigner.py` unless noted)

1. Bug fix: `stats` in `_nearest_frontier_for_ns` needs the key `"rejected": 0` (missing key gave `KeyError` and killed the node).
2. Clearance as a lookup: `self._clearance` = distance to nearest wall in meters (`scipy.ndimage.distance_transform_edt`), built in `_on_shared_map`. `_has_clearance(gx, gy, radius_m)` is one array lookup `[row=j, col=i]`.
3. `_goal_candidates` + `_find_goal`: several candidate goals per frontier (toward robot 0.4/0.3/0.6/0.9 m, then a ring at 0.4/0.7 m sorted by distance to robot). Strict clearance 0.3 m first, then relaxed 0.2 m. `_is_goal_valid(frontier_xy, goal_xy, robot_xy, clearance_m)`.
4. Per-robot `rejected` sets: a rejected frontier is only skipped by the robot that failed it, other robots may still take it. New `_on_frontier` loop. Robots are not put back after a TF or Nav2 failure (this also removes an infinite loop).
5. Escape goal: a robot with no goal for `IDLE_ESCAPE_S = 10` s drives to the free cell with the most wall clearance within 2 m (`_escape_goal`, `_try_escape`).
6. **Shared map for Nav2** (new file `shared_map_relay.py` in `map_merge_swarm`):
   - Problem found: Nav2's global costmap static layer read each robot's own `/TB3_n/map`, so walls found by another robot never reached its planner. R2 planned through R3's wall and hit it.
   - Fix: relay resamples `/shared_map` into each robot's map frame (uses TF `shared_map <- TB3_n/map` from `gt_map_tf_node`, including yaw) and publishes `/TB3_n/shared_local` (fixed size, about 461x461, origin (-11.5, -11.5)).
   - Setup: file in `map_merge_swarm/map_merge_swarm/`, entry in `setup.py` console_scripts (`shared_map_relay = map_merge_swarm.shared_map_relay:main`), `Node` in the map_merge launch file, and in each `burger_tb3_n.yaml` under `global_costmap ... static_layer`: `map_topic: /TB3_n/shared_local`, `map_subscribe_transient_local: true`.
   - Verify: `ros2 topic hz /TB3_n/shared_local` (about 1 Hz), `ros2 param get /TB3_n/global_costmap/global_costmap static_layer.map_topic`, global costmap in RViz shows the other robots' walls.
   - Start order: SLAM, map_merge launch (with relay), Nav2, explore. Do not run `status_map_tfs.launch` together with `gt_map_tf_node`, both publish `shared_map -> TB3_n/map`.
   - Result in the last recording: paths no longer cut through walls (they follow corridors and go around rooms).

7. **TF freeze in `gt_map_tf_node.py`** (`map_merge_swarm`):
   - Problem: `tf2_echo shared_map TB3_2/map` showed the transform at rest is stable (about `[0.03, 0, yaw 0]`) but swings up to about 7 cm and 2 degrees while R2 moves, then returns. A 2 degree error moves a wall about 20 cm at 6 m, so rooms looked rotated and extra frontiers appeared around them (the merge node rebuilds the map every second with whatever the transform is at that moment).
   - Fix: keep `self._last_tf` and `self._slow_since`; use `msg.twist[i]` to decide `moving` (linear speed above 0.02 m/s or angular above 0.03 rad/s). Accept a new transform only after the robot has been slow for more than 1 s (or if none was stored yet), otherwise republish the last accepted one.
   - Trade-off: during a long drive the transform does not follow SLAM drift until the robot stops.
   - Result: worked, the rotated rooms went away.

# Round 2d (2026-10-12): Step C and D done, A* planner, tuning

What changed after the planner failures (`Failed to create a plan from potential when a legal potential was found`). The list below is as reported by Pruthvi after the change, with notes on risks to watch.

## Nav2 yaml (`nav2_params/burger_tb3_1/2/3.yaml`)

| Change | Before | After | Why / what to watch |
|--------|--------|-------|---------------------|
| Global planner | `use_astar: false` | `use_astar: true` | Observed: the planner errors stopped and R2 could go behind the bin and back. Note: NavFn still extracts the path from its potential field in both modes, so "A* can never fail" is too strong. Keep counting the error in the logs |
| Inflation (local and global costmap) | radius 0.45, scale 5.0 | radius 0.25, scale 8.0 | Doorways are about 0.75 m wide. Two 0.45 m zones overlap (0.9 m), so doors looked blocked. Watch: robots now pass closer to walls |
| `transform_tolerance` | not set | controller_server 1.0, local costmap 0.5 | Absorbs 1 ms "extrapolation into the future" errors. It does not fix a transform that is stale for many seconds |

## `frontier_detector.py`

- `MIN_CLUSTER_SIZE` set from 5 to **10**, to stop 5-cell shadow specks behind furniture from becoming frontiers.
- **Risk:** in Step A we lowered it to 5 because real door edges came in pieces of 5 to 9 cells (`dropped cluster size=9`). With 10, door frontiers may disappear again. If a door is never explored, check the `dropped cluster size=` log first. The bin pockets are better handled by the clearance checks than by this number.

## `goal_assigner.py`

| Item | Change | Notes |
|------|--------|-------|
| `CLAIM_RADIUS` | 2.5 to 0.8 (or 1.0) | A 2.5 m claim locked out 13+ frontiers and starved TB3_1. Risk: robots may pick neighboring frontiers in the same room and crowd each other. The wall line check still keeps rooms separate |
| Step C (blacklist) | **Done** | Status 5 bans the spot for 300 s (`HOLD_CANCELLED_S`), status 6 for 60 s (`HOLD_ABORTED_S`) |
| Visited cooldown (new) | success (status 4) bans the goal spot for 300 s | So robots do not bounce back to corner shadows. Risk: a real frontier next to a visited goal is hidden for 5 minutes. Try 60 to 120 s if the map seems to stall |
| Step D (stuck watchdog) | **Done** | Timer every 5 s, `STUCK_S = 20`, `STUCK_MOVE_M = 0.3`, cancel via `cancel_goal_async()`. `MAX_STUCK_COUNT = 3` in `_is_idle` takes a robot out |
| Goal clearance | `(0.3, 0.2)` to `(0.35, 0.28)` | Keeps goals out of 3-sided pockets. Risk: a goal on a 0.75 m door centerline has about 0.375 m clearance, so doors still pass, but goals near door frames may be rejected |

## Status after this round

- Done: Steps 0, A, B, C, D, N1, N2, goal candidates, per-robot reject, escape goal, scan topic, shared map relay, TF freeze, A*, inflation tuning.
- Still to verify with a full 3-robot run: stale-TF freezes (none seen in the last run), door frontiers with `MIN_CLUSTER_SIZE = 10`, crowding with `CLAIM_RADIUS = 0.8`.
- Open: the `merge_node` forward-copy lattice (holes when the alignment has a small rotation), the efficiency topics (Pattern 5 and 6), the "bin pocket" robot-size-aware reachability if the clearance change is not enough.
- Before more changes: commit and tag a known-good state, for example `git tag stable-2026-10-14`.

## Completed in Round 2e (2026-10-14)

- **Pattern 5:** Assignment loop polish with soft-cost ranking (`d_self + gap_cost + wall_cost + teammate_penalty`), teammate personal bubble exclusion (`d_other < 1.2 and d_self > 2.5`), and 2-attempt visit counting to permanently blacklist unreachable obstacle shadows.
- **Pattern 6:** In-flight redirect for busy robots (`d_current > 3.0m`, `d_new < 1.5m`, same room, cancel + immediate reassign via `_redirect_goal` in `_on_result`).
- **Doorway & Detector tuning:** `MIN_CLUSTER_SIZE = 7` (preserves 8-cell doorway clusters while filtering small noise).
- **Goal cooldown on success:** Reduced from 3.0s to 0.1s so robots immediately claim next local frontiers in their own room instead of pausing and letting distant teammates snatch them.

---

# Pattern 5 — Assignment loop with Soft-Cost Ranking

**Status:** **Done** (2026-10-14)

## Why
Greedy nearest-frontier assignment previously led to cross-path conflicts where robots swapped rooms (R1 moving North, R3 moving South through 0.75m doorways) or picked frontiers through walls.

## What was implemented
1. **Soft-cost ranking formula in `sort_key`:**
   ```python
   # Penalty if straight line crosses a wall (keeps robots in their own room)
   wall_cost = 2.5 if self._line_crosses_occupied(rx, ry, fx, fy) else 0.0

   # Positive penalty: steers robots away from where teammates are working
   other_dists = [math.hypot(fx - ox, fy - oy) for o_ns, (ox, oy) in all_poses.items() if o_ns != ns]
   d_other = min(other_dists) if other_dists else 10.0
   teammate_penalty = max(0.0, 2.5 - d_other) * 2.0

   return d_self + gap_cost + wall_cost + teammate_penalty
   ```
2. **Teammate Personal Bubble Rule:**
   If a teammate is physically standing right at a frontier (`d_other < 1.2m`) and this robot is far away (`d_self > 2.5m`), skip it (`continue`). Leave it for the local teammate.
3. **2-Attempt Ghost Frontier Blacklist:**
   Tracks `_visit_counts[(x, y)]`. On first visit, applies a 60s cooldown. If visited twice without clearing (unreachable lidar shadow, e.g. trash can cylinder), blacklists for 600s (10 min).

---

# Pattern 6 — Redirect to closer frontier while busy

**Status:** **Done** (2026-10-14)

## Why
When a robot is driving to a far goal (e.g. 5m away) and its lidar reveals fresh frontiers in its own room (e.g. 1m away), previously the assigner ignored the busy robot. The robot would abandon its room and drive across the house while another robot came to finish it.

## What was implemented
1. Evaluated at the top of `_on_frontier` before checking idle robots.
2. If `d_current > 3.0m` and a new frontier has `d_new < 1.5m` in the **same room** (`not _line_crosses_occupied`), not blacklisted, and not claimed by others:
   - Stores `_redirect_goal[ns] = (pose_robot, goal)`.
   - Records `_last_redirect[ns] = now` (10s anti-thrashing cooldown).
   - Calls `handle.cancel_goal_async()`.
3. In `_on_result`, intercepts `new_goal = self._redirect_goal.pop(ns, None)`:
   - Immediately calls `_send_goal(ns, pose_robot, goal)` without blacklisting the old far goal or waiting on cooldown.

---

# Round 2e — Swarm Polish Notes (2026-10-14)

1. **Table Leg Obstacle Clearance:**
   - Problem: Robot wheels clipped thin table leg cylinders in Gazebo when A* planned paths between 0.7m table leg gaps.
   - Fix: Set `robot_radius: 0.13` (true physical radius of Burger with corners) and `inflation_radius: 0.30` in `burger_tb3_*.yaml`. Pushes paths around tables while keeping 0.75m doorways fully open.
   - Increased goal clearance in `_find_goal` to `(0.35, 0.25)` so goals are not placed within brushing distance of furniture.
2. **Success Cooldown (`_cooldown_until`):**
   - Reduced from `3.0s` to `0.1s`. Eliminates the idle pause that previously allowed distant teammates to steal local room frontiers.
3. **Cluster Size Threshold:**
   - `MIN_CLUSTER_SIZE = 7` in `frontier_detector.py`. Preserves 8-cell doorway clusters while filtering small 1–4 cell lidar noise.

# Tests you can do without unit tests

1. **RViz** `/shared_map` + `/frontiers_markers`: goals should not sit in walls; frontier→goal backoff should not punch through walls.
2. **Two robots, two rooms:** both get goals when frontiers exist on both sides of one wall.
3. **Multi-room:** robot can still be assigned a frontier in another room; Nav2 path may go around (robot→goal chord may cross a wall — that is OK).
4. **One room:** second robot’s goal should be farther away (another cluster), not the same doorway (after Pattern 4).
5. **Regression:** fewer tips from “goal through wall” (physical scrape still possible — inflation helps).
6. **Redirect (Pattern 6):** busy robot heading far; new close valid frontier → cancels and goes local.

---

# Reference: current code hooks

Backoff still has no map check inside `_backoff_goal` itself; **Pattern 3** validates after backoff via `_is_goal_valid`.

```python
# goal_assigner.py — _backoff_goal
step = min(0.4, max(0.0, length - 0.1))
# returns point ~0.4 m toward robot
```

Assignment after Pattern 3:

```python
backoff = self._backoff_goal(frontier_xy, (rx, ry))
if not self._is_goal_valid(frontier_xy, backoff):
    idle.append((ns, rx, ry))
    # do NOT frontiers.append(frontier_xy)
    continue
# ... then _send_goal
```

`MIN_GOAL_DIST` is per-robot distance to frontier, not inter-robot:

```python
MIN_GOAL_DIST = 1.0  # meters
```

---

# Suggested implement order

1. ~~Pattern 1–4~~ **Done**
2. ~~**Round 2 / Step 0:** SLAM params + Nav2 `allow_unknown: true`~~ **Done**
3. ~~**Round 2 / Step A and B**, **Round 2b** (N1, N2, idle log), **Round 2c** (goal candidates, per-robot reject, escape goal, shared map relay, TF freeze)~~ **Done**
4. ~~**Scan topic check** (remapped `/TB3_N/scan`, removed voxel layer, disabled loop closing)~~ **Done**
5. ~~**Round 2 / Step C:** failed-goal blacklist & visited cooldown~~ **Done**
6. ~~**Round 2 / Step D:** stuck watchdog (20s / 0.3m cancel + disqualification streak)~~ **Done**
7. **Costmap & Claim Tuning:** inflation 0.25m / cost scaling 8.0, A* planner, `transform_tolerance: 1.0`, `CLAIM_RADIUS: 1.0` **Done**
8. **Pattern 5:** polish loop, and the efficiency topics in Round 2c (path-length cost, own area first, optimal assignment) ← **next**
9. **Pattern 6:** redirect while busy (optional)

Also related: Nav2 `inflation_radius` was reduced from **0.45** to **0.25** in `nav2_params/burger_tb3_*.yaml` with `cost_scaling_factor: 8.0`. This allows the robot to pass through 0.75m doorways without overlapping cost inflation from left and right doorposts.