> **📍 Phase 2 (study guide)** · [← Roadmap](swarm_search_rescue_roadmap_v4.md) · Learning companion for Phase 2 code · Parts 1–2: grids + frontiers → [S5 plan](P2-S5-plan-frontiers.md) · Parts 3–4: goal checks + assignment → [S5 patterns](P2-S5-patterns-wall-claims.md) · Parts 5–6: shared map + TF freeze → [S1](P2-S1-plan-gt-shared-map.md), [S4](P2-S4-plan-live-sharing.md) · Part 7: SLAM/Nav2 params → [S5 patterns](P2-S5-patterns-wall-claims.md) · See also: [Engineering notes](P2-ref-engineering-notes.md)

# Explore swarm study guide: 3 robots exploring a house

**Who this is for:** you, one month into robotics and ROS. Every Part has the same shape, so you can study one at a time:

1. **Why** do we need this?
2. **The idea** in simple words
3. **The math** with a worked example (numbers checked with Python)
4. **In the code** (the short, important lines only)
5. **Engineering notes** (trade-offs and mistakes we really made)
6. **Check yourself** (questions, answers are hidden under the arrow)
7. **Done when** (a test you can run)

**How to study:** read a Part, close the file, and explain it out loud in your own words. Then try the "Check yourself" questions. For Parts 1 to 8 the code already exists in your repo, so your job is to **understand it and be able to rewrite it**. Part 9 is **your turn**: skeletons with gaps that you fill in yourself.

Last updated: 2026-10-09.

---

## Part 0: The big picture

### Files and what they do

| File | Package | One-line job |
|------|---------|--------------|
| `frontier_detector.py` | `explore_swarm` | Finds places where known free space touches unknown space |
| `goal_assigner.py` | `explore_swarm` | Chooses which robot goes to which frontier, and sends the Nav2 goal |
| `merge_node.py` | `map_merge_swarm` | Paints the 3 private robot maps into one `/shared_map` |
| `gt_map_tf_node.py` | `map_merge_swarm` | Publishes where each private map sits in the world (TF) |
| `shared_map_relay.py` | `map_merge_swarm` | Gives every robot's Nav2 the shared map in that robot's own frame |
| `burger_tb3_n.yaml` | nav2 params | Nav2 settings per robot (costmaps, planner) |
| slam_toolbox yaml | slam params | SLAM settings per robot |

### Data flow

```
Gazebo: 3 robots with lidar
   |  /TB3_n/scan
   v
slam_toolbox (one per robot) --> /TB3_n/map   (a PRIVATE map in frame TB3_n/map)
   |
   v
gt_map_tf_node: "where is each private map in the world?"  (TF shared_map -> TB3_n/map)
   |
   v
merge_node: paints all private maps into ONE map --> /shared_map (frame shared_map)
   |                                      |
   v                                      v
frontier_detector --> /frontiers      shared_map_relay --> /TB3_n/shared_local
   |                                      |
   v                                      v
goal_assigner --> NavigateToPose --> Nav2 (its global costmap reads shared_local)
```

### Words you must know

| Word | Meaning |
|------|---------|
| **Occupancy grid** | A map made of square cells. Each cell holds a number: `0` free, `100` wall, `-1` unknown |
| **Frame** | A coordinate system: an origin and a direction for the x axis. `shared_map` is the world frame. `TB3_2/map` is a frame that starts where robot 2's SLAM started |
| **TF** | The ROS system that stores how frames are positioned relative to each other |
| **Frontier** | A free cell that touches unknown space. It is the edge of what we know, so it is where exploring continues |
| **Costmap** | Nav2's own grid. Cells near walls get a high cost so paths stay away from walls |
| **Goal** | The pose we send to Nav2 with the `NavigateToPose` action |
| **Claim** | "Robot A is heading there", stored so robot B does not pick the same area |

---

## Part 1: Grids and coordinates

### Why
Everything else in this project reads or writes a grid. If you mix up meters and cells, or rows and columns, every later check gives wrong answers and the error is hard to see.

### The idea
A grid is a flat list of numbers. A cell is addressed by a column `i` and a row `j`. The grid has an **origin** (where cell `(0, 0)` sits in the world) and a **resolution** (meters per cell).

### The math
```
i = floor((x - origin_x) / resolution)        column
j = floor((y - origin_y) / resolution)        row
index in the flat list = j * width + i        row first, then column
```
**Worked example** (your `/shared_map`: origin `(-10, -10)`, resolution `0.05`, size 400 x 400):

A robot at `x = -1.72`, `y = -0.47`:

```
i = (-1.72 - (-10)) / 0.05 = 8.28 / 0.05 = 165.6  -> 165
j = (-0.47 - (-10)) / 0.05 = 9.53 / 0.05 = 190.6  -> 190
```
So the robot is in cell `(165, 190)`. In the flat list it is at `190 * 400 + 165`.

To go back to meters, use the **center** of the cell: `x = origin_x + (i + 0.5) * resolution`.

### In the code
```python
def _world_to_cell(self, x, y):
    i = int((x - info.origin.position.x) / info.resolution)
    j = int((y - info.origin.position.y) / info.resolution)
    if i < 0 or i >= info.width or j < 0 or j >= info.height:
        return None  # outside the map
    return (i, j)
```

### Engineering notes
- **Row first.** In numpy arrays made from the map, you index `grid[j, i]` (row, column). Mixing this up was the "classic bug" we warned about.
- `int()` cuts toward zero, so `int(-0.4)` is `0`, not `-1`. For a point just outside the map this would look inside. `math.floor` is the safer choice.
- `-1` is "unknown". It is not "wall". We only treat `>= 50` as a wall.

### Check yourself
1. Robot at `(2.0, 3.0)`. Which cell is it in on `/shared_map`?
2. Why do we use `(i + 0.5)` when converting a cell back to meters?

<details><summary>Answers</summary>

1. `i = (2 + 10) / 0.05 = 240`, `j = (3 + 10) / 0.05 = 260`, so cell `(240, 260)`.
2. A cell covers a square. Its center is half a cell from the corner, so `+0.5` gives the center instead of the edge.
</details>

### Done when
You can explain `_world_to_cell` and `_cell_value` without looking, and you know why `[j, i]` is the numpy order.

---

## Part 2: Frontiers

### Why
A robot needs to know **where unexplored space begins**. Without frontiers it has no reason to drive anywhere new.

### The idea
A frontier cell is a **free cell (`0`) that touches an unknown cell (`-1`)**. We find all of them, group touching ones into **clusters**, and use each cluster's center as one target.

### The math and the picture
Frontier cells along a diagonal edge (`F` = frontier, `.` = free, `?` = unknown):

```
col: 0 1 2 3 4 5
r0:  . . . . F ?
r1:  . . . F ? ?
r2:  . . F ? ? ?
```
- With **4-neighbor** clustering (up, down, left, right), `F(0,4)`, `F(1,3)` and `F(2,2)` do not touch by a side. You get **3 clusters of size 1**.
- With **8-neighbor** clustering (also diagonals) you get **1 cluster of size 3**.

The size filter throws away tiny clusters (they are usually noise):

```
MIN_CLUSTER_SIZE = 5 cells,  5 * 0.05 m = 0.25 m of edge
```

The cluster center is the average of its cells' positions: `x = origin_x + (mean_i + 0.5) * resolution`.

### In the code
```python
# frontier_detector.py, _frontier_cells: 8 neighbors
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
# _cluster_cells: the same 8 directions in the BFS loop
```
BFS (breadth-first search) means: start at one cell, visit all its neighbors, then their neighbors, until nothing new touches. Everything visited is one cluster.

### Engineering notes
- This is why the door line was missing: the white line was diagonal, so every piece was a cluster of size 1 to 4 and was dropped. Your log showed `dropped cluster size=9` while the limit was 10.
- Lower `MIN_CLUSTER_SIZE` gives more frontiers but also more junk (corner noise). Higher gives fewer but safer. 5 is a compromise.
- The lidar returns `inf` for anything farther than `range_max = 3.5 m`. So a big room is only painted when a robot comes close to its walls.
- **Trash can lesson:** a small object blocks the lidar and leaves a pocket of unknown behind it. That pocket produces a frontier that is not real exploring. Three robots were attracted to it.

### Check yourself
1. Why can a straight diagonal wall of unknown cells produce no frontier at all with 4-neighbors and size 10?
2. A cluster has 8 cells. Is it kept with `MIN_CLUSTER_SIZE = 10`? With 5?

<details><summary>Answers</summary>

1. Its cells only touch at corners, so 4-connected clustering splits it into tiny pieces, and each piece is below the size limit.
2. Not kept with 10 (8 < 10). Kept with 5 (8 >= 5).
</details>

### Done when
In RViz, `/frontiers_markers` shows orange dots on the white door line, and the detector log prints only a few `dropped cluster` lines of sizes 1 to 4.

---

## Part 3: Checking a goal before sending it

### Why
A frontier is a place on the **edge** of the unknown, often right next to a wall. If we send the robot exactly there, or to a spot behind a wall, it hits things and tips over. So `goal_assigner` checks every candidate goal **before** sending it.

### The idea: a chain of checks, cheap ones first
```
candidate goal G for frontier F, robot at R

1. Goal cell: not off-map, not a wall (value < 50). Unknown (-1) is allowed.
2. Line F -> G: no wall between the frontier and the goal.
3. Clearance: G is at least 0.3 m from any wall (then 0.2 m as a fallback).
4. Reachability: R and G are connected through KNOWN free cells.
```
If one check fails, the goal is rejected and we log **why** (`reject: ...`).

### Check 2: the line check (sampling)
We walk along the segment in small steps and look at the cell under each step.
```
step = resolution / 2 = 0.025 m
steps for a 0.5 m segment = 0.5 / 0.025 = 20
sample k:  point = start + (k / 20) * (end - start)
```
If any sample lands on a cell `>= 50` (or off the map), the line "crosses a wall". Why half a cell? So a 1-cell-thick wall cannot slip between two samples.

### Check 3: clearance with a distance transform
For every cell, we pre-compute "how far is the nearest wall?" once per map. Then the check is a single lookup instead of a loop.

Tiny example: a wall in the left column, cells 0.05 m wide. The distance to the wall for each cell in a row:

```
cell:      0     1     2     3     4     5
distance:  0.00  0.05  0.10  0.15  0.20  0.25      (meters)
```
So "clearance of at least 0.3 m" simply means `distance[j, i] >= 0.3`.

```python
occupied = grid >= OCCUPIED_THRESH
self._clearance = ndimage.distance_transform_edt(~occupied) * resolution  # meters
# later:
return bool(self._clearance[j, i] >= radius_m)
```
`edt` means Euclidean distance transform: for every non-wall cell it returns the straight-line distance, in cells, to the nearest wall cell. We multiply by `resolution` to get meters.

**Why 0.3 and 0.2?** The Burger robot is about 0.1 m in radius. Nav2 inflates walls by 0.45 m, but that is a cost, not a hard stop. We try 0.3 m first and fall back to 0.2 m in narrow corridors, where 0.3 m would reject everything.

### Check 4: reachability with connected components
We label every group of connected free cells (`0`) with its own number. If the robot and the goal sit in the **same group**, a known free route exists.

Example, wall in the middle column (`#`):

```
before          labels
0 0 # 0 0       1 1 0 2 2
0 0 # 0 0       1 1 0 2 2
0 0 # 0 0       1 1 0 2 2
```
Two groups (1 and 2). A robot in group 1 and a goal in group 2 are **not** reachable. Now open a gap in the wall:

```
labels with a gap at row 2
1 1 0 1 1
1 1 0 1 1
1 1 1 1 1
1 1 0 1 1
```
One group, so now it is reachable. The code compares labels in a small patch (3 cells, 0.15 m) around each point, because the exact cell can be `-1`.

```python
self._free_labels, _ = ndimage.label(grid == 0)  # one number per blob of free cells
```

### Goal candidates (geometry)
One fixed "0.4 m backoff" landed inside walls. So `_find_goal` tries many candidates and takes the first valid one:

```
toward the robot:   0.4, 0.3, 0.6, 0.9 m from the frontier
ring around F:      8 points at 0.4 m and 8 points at 0.7 m, closest to the robot first
```
It runs all candidates with clearance 0.3 m first, then again with 0.2 m.

### Engineering notes
- **Order matters for speed.** Lookups and short lines first, labels last.
- **Why allow unknown for the goal cell?** Frontiers sit next to unknown, so a goal slightly in unknown is normal for exploring. The wall-line and reachability checks still protect us.
- **Why reachability?** One robot's wall was in `/shared_map`, but another robot's Nav2 planned straight through it. Reachability alone did not fix that (see Part 5).
- Each check adds a risk of **rejecting good goals**. That is why every reject logs a reason: so you can see which check is too strict.

### Check yourself
1. A goal at `(1.0, 1.0)` has clearance `0.25 m`. Does it pass with radius 0.3? With 0.2?
2. Why do we look at a small patch of labels around the robot instead of one cell?

<details><summary>Answers</summary>

1. Fails with 0.3 (0.25 < 0.3). Passes with 0.2 (0.25 >= 0.2).
2. The cell under the robot can be unknown (`-1`) or on a wall edge, and then it has no label. The patch finds the nearby free cells.
</details>

### Done when
Your logs show a mix of `reject: goal cell value=...`, `reject: too close to obstacle`, `reject: no known free route`, `reject: frontier->goal crosses wall`, and also `assign ...` lines.

---

## Part 4: Assigning frontiers to robots

### Why
Three robots must not chase the same frontier, and one robot's bad luck must not block the others.

### The idea (the loop)
Each time `/frontiers` arrives:

```
idle robots = robots that are free and past their cooldown
while there are idle robots and frontiers:
    pick the (robot, frontier) pair with the smallest distance, skipping:
        - too close (< 1.0 m)
        - rejected before for THIS robot in this round
        - claimed by another robot
    find a valid goal for it (Part 3)
    if none: remember "this robot rejected this frontier", put the frontier back for others
    else: send the goal, store the claim, move on
```

### Claims (room-aware)
Robot A's claim is its goal point. A frontier counts as taken **only if**:
1. it is within `CLAIM_RADIUS = 2.5 m` of the claim, **and**
2. the straight line between them has no wall.

Two robots 1 m apart on opposite sides of a wall are **not** in conflict, because the line crosses the wall.

### Why the loop always ends
Each round, every `(robot, frontier)` pair can be rejected at most once. The number of pairs is finite, so the loop must finish. Our first version put the same frontier back every time, and the node looped forever.

### The escape goal
If a robot gets no goal for `IDLE_ESCAPE_S = 10 s`, it drives to the free cell with the **most wall clearance** within 2 m, at least 0.5 m away, in the same known free region. The idea: a new viewpoint lets SLAM see new things. Note that spinning would not help, because the lidar already sees 360 degrees. Only moving adds information.

### In the code (the key idea)
```python
rejected = {ns: set() for ns in self.robot_namespaces}  # per robot, per round
...
if goal is None:
    rejected[ns].add(frontier_xy)  # only this robot skips it
    frontiers.append(frontier_xy)  # other robots may still take it
    idle.append((ns, rx, ry))  # this robot tries another frontier
```

### Engineering notes
- **Stats in the log:** `no pick: {'close': 2, 'claimed': 6, 'rejected': 4}` tells you why nobody got a goal. Remember: any new key you count must also exist in the `stats` dict, or you get a `KeyError` (this killed the node once).
- **Greedy is not optimal.** Picking the nearest pair each time can make robots cross paths. Part 10 covers better ways.
- Distance here is straight-line, not path length. A frontier "near" through a wall can be far to drive.

### Check yourself
1. Why do we put the frontier back in the list after a robot rejects it?
2. Two robots are 1.5 m apart, but a wall is between them. Can both pick frontiers in the same area?

<details><summary>Answers</summary>

1. The rejection may only be true for that robot (for example, no known route from where it stands). Another robot may reach it fine.
2. Yes. The claim line crosses the wall, so the frontier is not counted as taken.
</details>

### Done when
All three robots get goals in the first minute, and you can read a `no pick` line and explain it.

---

## Part 5: Giving Nav2 the shared map

### Why
Robot 2 planned a route through a wall that robot 3 had already found. The wall was in `/shared_map`, but Nav2's global costmap was reading robot 2's **own** map (`/TB3_2/map`). So the shared map helped only the assigner, not the planner.

### The idea
Make a new topic per robot, `/TB3_n/shared_local`, which holds the shared map **in robot n's own frame**, and point that robot's costmap static layer to it.

Why not point it straight at `/shared_map`? As far as I know, a non-rolling costmap assumes the map is already in its own frame, so a map in a different frame would land in the wrong place. Resampling it into the robot's frame removes that problem.

### The math: moving a point between frames (SE(2))
Robot map frame to world frame, with the transform `t = (tx, ty)` and yaw angle `θ`:

```
shared_x = tx + cos θ * map_x - sin θ * map_y
shared_y = ty + sin θ * map_x + cos θ * map_y
```
And back (inverse), which the relay uses to find the extent:

```
map_x =  cos θ * (shared_x - tx) + sin θ * (shared_y - ty)
map_y = -sin θ * (shared_x - tx) + cos θ * (shared_y - ty)
```
**Worked example:** `t = (2, -1)`, `θ = 90 degrees`, so `cos θ = 0`, `sin θ = 1`. The point `(1, 0)` in the robot frame becomes:

```
shared_x = 2 + 0*1 - 1*0 = 2
shared_y = -1 + 1*1 + 0*0 = 0        ->  (2, 0)
```
Go back: `(2, 0) - t = (0, 1)`, then `map_x = 0*0 + 1*1 = 1`, `map_y = -1*0 + 0*1 = 0`, which gives `(1, 0)` again. Both directions work.

### The algorithm (gather)
For every cell of the **output** map (in the robot's frame):
1. compute its center in meters,
2. transform it to the world frame,
3. look up which `/shared_map` cell is there,
4. copy that cell's value (cells outside the shared map stay `-1`).

This "gather" way never leaves holes in the output. (The opposite, copying each source cell forward, can leave holes when the grid is rotated.)

### Fixed size
Nav2 must not have to resize its costmap every second. So on the first message we compute the **bounding box** of the whole shared map's 4 corners in the robot's frame, add a margin of 1.5 m, and then keep that size forever. For you this came out as about 461 x 461 cells with origin `(-11.5, -11.5)`.

### In the code (shape of it)
```python
col = np.floor((sx - shared_origin_x) / res).astype(np.int32)
row = np.floor((sy - shared_origin_y) / res).astype(np.int32)
inside = (col >= 0) & (col < W) & (row >= 0) & (row < H)
out = np.full((h, w), -1, dtype=np.int8)
out[inside] = grid[row[inside], col[inside]]
```
Yaml for each robot (shown for TB3_1):
```yaml
static_layer:
  plugin: "nav2_costmap_2d::StaticLayer"
  map_topic: /TB3_1/shared_local
  map_subscribe_transient_local: true
```

### Engineering notes
- **Use only one static layer.** Two static layers would fight over the costmap size.
- **Delay:** a wall found by robot 1 reaches robot 2's planner after about 2 s (SLAM, merge, relay). The laser layers cover the gap.
- **Drift:** if SLAM drifts, the same wall can appear twice in the union map, and every planner then sees both. Always check the costmap in RViz.
- Costs CPU: about 600 x 600 cells per robot once a second.
- A map in the shared frame, shown in RViz with fixed frame `shared_map`, should overlap with the relay map exactly. That overlay is the best test.

### Check yourself
1. The robot map frame is rotated 90 degrees. A wall at `(0, 2)` in the robot frame is where in the world frame if `t = (0, 0)`?
2. Why is gather better than scatter here?

<details><summary>Answers</summary>

1. `shared_x = 0 + 0*0 - 1*2 = -2`, `shared_y = 0 + 1*0 + 0*2 = 0`, so `(-2, 0)`.
2. Gather visits every output cell exactly once, so there are no holes. Scatter can skip output cells when the grid is rotated.
</details>

### Done when
- `ros2 topic hz /TB3_2/shared_local` shows about 1 Hz.
- `ros2 param get /TB3_2/global_costmap/global_costmap static_layer.map_topic` prints `/TB3_2/shared_local`.
- Another robot's walls show up in `/TB3_2/global_costmap/costmap` in RViz, and paths no longer cut through them.

---

## Part 6: Transform jitter and the "freeze"

### Why
Rooms looked rotated and extra frontiers appeared, but only while robots were driving. A rotated map means every wall in it is in the wrong place.

### The idea
`gt_map_tf_node` computes where each private map sits in the world:

```
shared_T_map = shared_T_base (from Gazebo truth)  *  inverse(map_T_base) (from SLAM)
```
Both numbers describe the same robot at the same moment, so the result should be constant. But when the robot moves, the two numbers are not measured at exactly the same instant, and SLAM also corrects itself in small steps. So the result wobbles.

### Reading the evidence (`tf2_echo shared_map TB3_2/map`)
```
at rest:        translation [0.030, 0.000],  yaw 0.000 deg   (always the same)
while driving:  translation up to about 7 cm,  yaw -1.0 to +2.07 deg
after stopping: back to the same baseline
```
Because it **always returns to the same baseline**, this is a motion error and not real drift.

### The math: why a small angle matters
A yaw error `θ` moves a point at distance `r` from the frame origin by about `r * sin θ`:

```
r = 3 m,  θ = 2 deg  ->  0.105 m
r = 6 m,  θ = 2 deg  ->  0.209 m
r = 8 m,  θ = 2 deg  ->  0.279 m
```
So a 2 degree wobble moves far walls by 20 cm or more. The merge node repaints the map every second, so one robot's walls land 20 cm away from the same walls seen by another robot. Gaps appear, and the detector sees extra frontiers.

### The fix as a small state machine
```
robot is MOVING        -> keep publishing the last accepted transform
robot slow for > 1 s   -> accept the new value and store it
```
Moving means linear speed above `0.02 m/s` or turning faster than `0.03 rad/s` (from `ModelStates.twist`).

### Engineering notes
- **Trade-off:** during a long drive the transform does not follow SLAM drift until the robot stops. In a house this is fine.
- **Alternative:** static spawn offsets (`status_map_tfs.launch`) are perfectly stable but ignore drift. Never run both at once, they publish the same transform.
- **Always measure first.** We did not guess: `tf2_echo` showed the numbers.

### Check yourself
1. Why does the wobble hurt far walls more than near ones?
2. The robot is driving and never stops for 30 s. What does the map transform do?

<details><summary>Answers</summary>

1. The displacement is `r * sin θ`, so it grows with the distance `r` from the frame's origin.
2. It stays at the last value from when the robot was last at rest. It updates again after the next stop.
</details>

### Done when
`tf2_echo` shows flat values while a robot drives, and the rooms stop looking rotated.

---

## Part 7: SLAM and Nav2 settings we changed

| Setting | Before | After | What it does and why |
|---------|--------|-------|----------------------|
| `map_update_interval` | 5.0 | 1.0 | How often SLAM publishes the map. At 5 s the map is old |
| `minimum_travel_distance` | 0.5 | 0.1 | SLAM adds a scan only after the robot moved this far. At 0.5 m the map fills in patches |
| `minimum_travel_heading` | 0.5 | 0.1 | The same, for turning (radians) |
| `minimum_time_interval` | 0.5 | 0.2 | Minimum time between processed scans |
| Nav2 `allow_unknown` | false | true | Frontiers sit at the edge of unknown, so the planner must be allowed to plan toward it |
| `inflation_radius` | 0.45 | 0.45 | Cost around walls so paths keep away |

**Trade-off of faster SLAM:** more CPU with 3 robots. If the simulation lags, use 0.2 and 2.0.

**Lidar facts:** `range_min` 0.12 m, `range_max` 3.5 m. Beyond that you get `inf`. Rooms bigger than 3.5 m are painted only when a robot gets close to the walls.

**Nav2 status codes:** `4` succeeded, `5` cancelled, `6` aborted.

**Check yourself:** Why did `allow_unknown: false` cause "failed to create plan" for frontier goals?

<details><summary>Answer</summary>
A frontier goal sits next to unknown space. With `false`, the planner may not use unknown cells, so it cannot find a path ending at the edge of the unknown.
</details>

---

## Part 8: Debug playbook (the process matters more than the code)

When something goes wrong, work in this order:

1. **Look at RViz with TF on.** Where is each robot, what is each path, where are the markers?
2. **Read the log lines**, in this order: `assign`, `reject`, `no pick`, `nav finished status`.
3. **Compare two maps** for the same area: `/TB3_2/map` vs `/shared_map`. Does the wall exist in one and not the other?
4. **Measure, do not guess:** `ros2 topic hz`, `ros2 topic echo --once`, `ros2 run tf2_ros tf2_echo A B`, `ros2 param get`.
5. **Change one thing**, rebuild, run again, compare.

| Symptom | First suspect | How to check |
|---------|---------------|--------------|
| Node dies right after the first goal | Python exception | Read the traceback, it names the line (`KeyError: 'rejected'`) |
| Robot never moves | Every goal rejected | `reject:` lines and `no pick` stats |
| Robot drives through a wall | Planner map lacks the wall | `/TB3_n/global_costmap/costmap` in RViz |
| Rooms look rotated or doubled | TF wobble | `tf2_echo` while a robot drives |
| `Robot is out of bounds of the costmap` | Costmap has no map yet | `ros2 topic hz /TB3_n/shared_local` |
| A param seems ignored | Wrong file or not restarted | `ros2 param get` on the running node |
| Same frontier chosen again and again | No blacklist yet | Step C below |

---

## Part 9: Your turn (write these yourself)

These are skeletons. Read the "why" and the rules, then fill the gaps. Rebuild with `colcon build --packages-select explore_swarm` and `source install/setup.bash` after each edit.

### 9a: Scan topic check (1 minute, no code)

**Why:** Nav2's obstacle layers read the laser. Your yaml has `topic: /scan`. If the robots publish `/TB3_n/scan`, then `/scan` is a different (maybe empty) topic, the costmap never sees walls, and robots hit them.

```bash
ros2 topic list | grep scan
ros2 topic info /scan
```
**Done when:** you know the real topic names. If `/scan` is not per robot, change `topic:` to `/TB3_1/scan` (and so on) in both the obstacle and voxel layers, in the **global and the local** costmap sections.

### 9b: Step C, failed-goal blacklist

**Why:** When a goal fails, the robot waits 3 s and then picks the same frontier again, forever. A blacklist remembers failed spots for a while.

**Rules:**
- One entry = a point `(x, y)` plus the time when the ban ends.
- Radius **1.2 m** (a candidate goal can sit up to 0.9 m from its frontier).
- Hold **60 s** for status `6` (aborted), **300 s** for status `5` (cancelled).
- Status `4` (success) adds nothing.

**Skeleton:**
```python
BLACKLIST_RADIUS = 1.2

# in __init__:
self._blacklist = []  # TODO: write down what one entry holds


def _now(self):
    return self.get_clock().now().nanoseconds * 1e-9


def _is_blacklisted(self, fx, fy):
    # TODO 1: remove entries whose end time has passed
    # TODO 2: return True if any entry is closer than BLACKLIST_RADIUS to (fx, fy)
    ...


# in _on_result(self, future, ns):
#   TODO 3: pop this robot's claim BEFORE you forget it (you need its x, y)
#   TODO 4: read the status code
#   TODO 5: if status is not 4 and you have a claim, append an entry
#           (hold 300 s if status == 5, else 60 s)
#   (keep the existing busy = False and cooldown lines)

# in _nearest_frontier_for_ns:
#   TODO 6: add "blacklisted": 0 to stats, and skip blacklisted frontiers (count them)
```
**Self-check:** why do we pop the claim first? Why does the radius need to be bigger than 0.9 m?

**Done when:** after a failed goal (`status = 6`), the same frontier is not assigned again for about a minute, and `no pick` shows a `blacklisted` count.

### 9c: Step D, stuck watchdog (do it if robots still get stuck)

**Why:** a robot pushing against a wall keeps its goal, SLAM gets bad scans, and the map warps. We want to cancel a goal when the robot is not making progress. Needs Step C, because a cancelled goal ends with status `5`, and Step C then bans that spot.

**Rules:** every 2 s, for each busy robot: if it moved more than **0.3 m** since the last check, remember the new position and time. If it has not moved more than 0.3 m for **20 s**, cancel the goal. After **3 cancels in a row** without a success, treat the robot as down (probably tipped) and stop giving it goals.

**Skeleton:**
```python
STUCK_S = 20.0
STUCK_MOVE_M = 0.3

# in __init__:
self._goal_handles = {}  # ns -> running goal handle
self._last_move = {}  # ns -> (x, y, time) when it last made progress
self._stuck_count = {}  # ns -> consecutive watchdog cancels
self.create_timer(2.0, self._watchdog)

# in _on_goal_response (accepted branch):
#   TODO 1: store the goal handle, and store the robot's current pose and time

# in _on_result:
#   TODO 2: forget the handle and last_move; reset the stuck count if status == 4


def _watchdog(self):
    # TODO 3: for each robot that is busy and has a handle:
    #   - read its pose (use try/except, TF can fail)
    #   - if it moved more than STUCK_MOVE_M since last_move: update last_move
    #   - elif now - time > STUCK_S: count it, log a warning, cancel_goal_async()
    ...


# in _is_idle:
#   TODO 4: return False if the stuck count is 3 or more
```
**Self-check:** why 20 s and not 2 s? (Think about Nav2 spinning in place at the start of a goal.)

**Done when:** you can force a stuck robot and see `no progress for 20s, cancelling goal`, then `status = 5`, then a different frontier.

---

## Part 10: Later: making exploration efficient

You saw this in the recordings: a robot leaves its own room while frontiers remain nearby, and robots swap areas. These are ideas for later, in this order of effort:

| Idea | Why | Rough approach |
|------|-----|----------------|
| **Path length instead of straight line** | A frontier "near" through a wall is far to drive | Breadth-first search over free cells of the shared map from each robot |
| **Own area first** | Less crossing, more local work | Bonus for frontiers in the same free region as the robot, or near its last goal |
| **Optimal assignment** | Greedy nearest-pair can cross paths | The Hungarian method on a robot-by-frontier cost table |
| **Redirect while busy (Pattern 6)** | A much closer frontier appears while driving far | Cancel and resend, with a cooldown so it does not flip-flop |
| **Frontier quality** | Tiny pockets (trash can) waste trips | Weight by cluster size, skip tight pockets |

Before building any of these, look at the `reject:` lines. Many "nearby frontiers skipped" cases are really **rejections** by the checks in Part 3, not a bad ordering.

**Reading, when you want theory:** Yamauchi (1997) on frontier-based exploration, Burgard and others (2005) on coordinated multi-robot exploration, and the Nav2 documentation for the costmap static layer. I have not re-checked the details of these, so treat them as starting points.

---

## Appendix A: Mistakes we really made

| Mistake | What happened | Fix |
|---------|---------------|-----|
| `self._claimed` vs `self._claims` | `AttributeError` | One name everywhere |
| Claim check after the `best` update | Claimed frontiers still chosen | Filters first, then pick the winner |
| Dict created inside the `for` loop | Reset 3 times | Create it once in `__init__` |
| Stat key missing (`"rejected"`) | `KeyError`, node died | Add every counted key to `stats` |
| Frontier put back, same robot again | Infinite loop | Remember per-robot rejections |
| Mixing `[row, col]` and `[x, y]` | Wrong cells | Row (`j`) first in numpy |
| `allow_unknown: false` | No plan to frontier goals | Keep `true` |
| Costmap read the robot's own map | Planned through a wall | Relay, Part 5 |
| Frame wobble while driving | Rotated rooms | Freeze, Part 6 |
| Two TF publishers for one frame | Transform fights | Run only one (GT node or static offsets) |

## Appendix B: Commands you will reuse

```bash
cd ~/swarm-search && source /opt/ros/humble/setup.bash
colcon build --packages-select explore_swarm      # or map_merge_swarm
source install/setup.bash

ros2 topic hz /TB3_2/shared_local
ros2 topic echo /TB3_2/scan --once
ros2 param get /TB3_2/global_costmap/global_costmap static_layer.map_topic
ros2 run tf2_ros tf2_echo shared_map TB3_2/map
ros2 node list | grep TB3_1
ros2 lifecycle get /TB3_1/planner_server
```