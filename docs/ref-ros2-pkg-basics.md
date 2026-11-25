> **📍 Reference** · [← Roadmap](swarm_search_rescue_roadmap_v4.md) · ROS 2 package patterns used in this project

# ROS2 Package Setup, Explained

Reference for the `map_merge_swarm` package setup, same pattern applies to every future node.

---

## The pieces and what each one does

### The package folder itself

A ROS2 package is just a folder with a specific structure that ROS2's tools know how to recognize. `ros2 pkg create` scaffolds it: the package folder, `package.xml`, `setup.py`, `resource/` marker file. Without this structure, ROS2 has no idea your code even exists.

### `package.xml`

Describes the package to ROS2 itself, name, version, dependencies (like `rclpy`), description, license. This is metadata, not code. Other packages and tools read this to know what your package needs.

### `setup.py`

Python's own packaging file. This is what actually tells the system how to install your code as a runnable program.

**`entry_points`**, the critical one:

```python
entry_points={
    'console_scripts': [
        'merge_node = map_merge_swarm.merge_node:main',
    ],
},
```

Maps a command name (`merge_node`) to an actual Python function (`map_merge_swarm.merge_node:main`). Without this, `main()` is just a function sitting in a file, nothing knows it's meant to be a runnable program. This line is literally what makes `ros2 run map_merge_swarm merge_node` mean anything.

**`data_files`**, tells the installer which non-Python files to copy too:

```python
import os
from glob import glob

data_files=[
    ('share/ament_index/resource_index/packages',
        ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    (os.path.join('share', package_name, 'launch'),
        glob(os.path.join('launch', '*.launch.py'))),
],
```

Python packaging only handles `.py` files by default, so launch files (or any other non-Python assets) need to be explicitly listed or they get left behind.

---

## The build pipeline

### `colcon build --packages-select map_merge_swarm`

Reads `setup.py` and `package.xml`, and produces an `install/` folder: a clean, self-contained copy of your package in the exact layout ROS2 expects at runtime, executables in one place, launch files in another. Your source code in `src/` isn't directly runnable, this step is what turns it into something ROS2 can find and execute.

### `source install/setup.bash`

Adds that install folder to your shell's search paths, specifically the ones ROS2 tools check. Before sourcing it, your terminal has no idea the package exists, that's exactly what an early "No executable found" error usually means, the terminal genuinely can't see it yet.

### `ros2 run map_merge_swarm merge_node`

Looks up `merge_node` in that installed package's entry points, and runs the function it points to, `main()` in this case.

---

## The full chain

```
write Python code
  → tell setup.py which function is the entry point (entry_points)
  → colcon build packages it into install/
  → source install/setup.bash makes ROS2 aware of that install folder
  → ros2 run executes it by name
```

Every ROS2 node from here follows this exact pipeline. Only the code inside `main()` changes.

---

## Two ways to launch a file, and why it matters for where launch files live

This is the part that trips people up: you now have two `launch/` folders in the repo, doing two genuinely different jobs.

### Way 1: by direct path, no package involved

```bash
ros2 launch /full/path/to/file.launch.py
```

ROS2 just opens and runs that exact Python file, wherever it happens to sit on disk. No package structure required, no `colcon build`, no install step, nothing registered anywhere. This is what `~/swarm-search/launch/` (spawn, `slam_tb3`, `nav2_tb3`) has been using the whole time, full path, every time. Those files were never meant to be found "by name," so they never needed any of the packaging machinery.

### Way 2: by package name, needs the full pipeline

```bash
ros2 launch map_merge_swarm static_map_tfs.launch.py
```

Here ROS2 doesn't get a path at all, only a package name and a filename. To turn that into an actual file, it has to:

1. **Find where the package is installed.** This lookup is powered by ament's resource index, which is exactly what the first `data_files` entry registers:
   ```python
   ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
   ```
2. **Look inside that package's installed `launch/` folder** for the requested filename, specifically `<install_location>/share/map_merge_swarm/launch/`.

If the file isn't sitting in that exact install location, `ros2 launch` fails, "package not found" or "file not found," even if the file genuinely exists somewhere else on your machine.

### How the file actually gets into that install folder

This is the part `data_files` controls:

```python
(os.path.join('share', package_name, 'launch'),
    glob(os.path.join('launch', '*.launch.py'))),
```

Read as an instruction to colcon: "look in a folder called `launch/` sitting right next to `setup.py` (that's `src/map_merge_swarm/launch/`), grab every `*.launch.py` file there, copy each one into `share/map_merge_swarm/launch/` inside `install/`."

So there are two distinct locations, and nothing connects them except this glob line:

- **Source** (where you save the file): `src/map_merge_swarm/launch/static_map_tfs.launch.py`
- **Destination after `colcon build`** (where ROS2 actually finds it): `install/map_merge_swarm/share/map_merge_swarm/launch/static_map_tfs.launch.py`

Save the file one folder off, and `colcon build` still succeeds with no warning, it just silently doesn't copy it, and `ros2 launch map_merge_swarm ...` fails later with a confusing "not found" even though the file is right there on disk.

### The full picture, side by side

Package-based (`map_merge_swarm`):
```
src/map_merge_swarm/launch/static_map_tfs.launch.py
        │  colcon build  (reads setup.py's data_files glob)
        ▼
install/map_merge_swarm/share/map_merge_swarm/launch/static_map_tfs.launch.py
        │  source install/setup.bash  (registers this install prefix)
        ▼
ros2 launch map_merge_swarm static_map_tfs.launch.py
        │  (ament index finds the package, then finds the file by name)
        ▼
file runs
```

Direct-path (everything in `~/swarm-search/launch/`):
```
~/swarm-search/launch/nav2_tb3.launch.py
        │  ros2 launch ~/swarm-search/launch/nav2_tb3.launch.py  (no lookup at all)
        ▼
file runs
```

Same end result, but one skips the entire packaging chain and the other depends on every link in it being wired correctly.

---

## OccupancyGrid maps (what `/TB3_N/map` and `/shared_map` are)

A map in ROS2 is usually `nav_msgs/OccupancyGrid`: a 2D grid of cells.

| Cell value | Meaning |
|---|---|
| `-1` | unknown (not explored yet) |
| `0` | free |
| `100` | occupied (obstacle) |

Also on each message:

- `info.resolution` — meters per cell (we use `0.05`)
- `info.width` / `info.height` — grid size in cells
- `header.frame_id` — which TF frame the map lives in  
  (`TB3_1/map` for one robot’s SLAM map, `shared_map` for the merged map)

### Import the message class, not the module

```python
from nav_msgs.msg import OccupancyGrid   # correct — the message type
# import nav_msgs.msg as OccupancyGrid  # WRONG — that is the whole module
```

If you import the module by mistake, `create_subscription` / `create_publisher` fail with something like:

```
AttributeError: ... no attribute '_TYPE_SUPPORT'
This might be a ROS 1 message type but it should be a ROS 2 message type.
```

That message is misleading here: you are not mixing ROS 1. You passed a **module** where ROS2 expected a **message class**.

---

## QoS for map topics

**QoS** (Quality of Service) is the contract between a publisher and a subscriber: how reliable delivery is, and whether a late joiner still gets the last message.

slam_toolbox map topics typically use:

- **reliability:** `RELIABLE`
- **durability:** `TRANSIENT_LOCAL` (latched — if your node starts after SLAM, you still get the latest map)
- **history:** `KEEP_LAST`, **depth:** `1` (only the newest grid matters)

Your merge node must use a matching profile on both subscribe and publish, for example:

```python
map_qos = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
)
```

If QoS does not match, you often get **no messages at all**, with no loud error — subscriptions look fine in code but stay silent.

---

## Local map vs shared map (the idea in plain English)

Each robot’s SLAM has its **own** notebook:

- `/TB3_1/map`, `/TB3_2/map`, `/TB3_3/map`
- In that notebook, the robot treats “where I started mapping” as about **(0, 0)**

**Shared / global** (`shared_map`) is one common world sheet. Spawn positions matter there, e.g. TB3_1 at `(-2, -0.5)` in the world — not `(0, 0)`.

Simple rule used by the v1 merge:

```text
shared position ≈ local position + spawn offset
```

Example: robot spawned at `(1, 1)` thinks it is at local `(0, 0)`; on the shared sheet that same place is `(1, 1)`.

### What `/shared_map` is (and is not)

| `/shared_map` is | `/shared_map` is not |
|---|---|
| One **topic** that draws all three maps on one canvas (after spawn shifts) | A new single SLAM that all robots update together |
| A **shared view** for RViz (and later for nav / coordination) | Proof that the three maps already melt into one perfect world map |

So in RViz you often see **three patches side by side**, slight overlap near spawn places. That is expected for v1: three sources, one picture. Later steps can improve alignment so robot 2 can **use** areas robot 1 already explored.

Static TFs match the same offsets: `shared_map` → `TB3_N/map` with translation = spawn `(x, y, 0)`.

---

## Stub `/shared_map` vs real merge

**Stub (wiring check):**

- Subscribe to the three `/TB3_N/map` topics, store latest
- Timer publishes a tiny fake `/shared_map` (e.g. 1×1, all `-1`)

Proves: hear three maps → speak on `/shared_map`.

**Real merge (v1):**

- Big canvas in frame `shared_map` (e.g. 20×20 m at 0.05 m/cell, origin `(-10, -10)`)
- For each non-unknown cell: shift by spawn → write into canvas
- Fuse: occupied (100) wins, else free (0), else unknown (-1)
- Publish `/shared_map`

Do stub before fusion so a blank output is clearly “wiring” vs “math.”

### Why `ros2 topic echo /shared_map` can look like “all -1”

A 400×400 map has **160 000** cells. Echo prints from the **start** of `data` — that corner is often world `(-10, -10)`, far from the robots, so many leading `-1`s are normal. Use **RViz** (or count `0` / `100` in the full array) to judge if merge works. Sitting still also leaves mostly unknown until someone drives and SLAM fills free/occupied cells.

---

## RViz for `/shared_map` (things that wasted time)

Start RViz with sim time when Gazebo is running:

```bash
ros2 run rviz2 rviz2 --ros-args -p use_sim_time:=true
```

**Global Options** (editable) ≠ **Global Status** (read-only error text).  
Change **Fixed Frame** under **Global Options**, not under Global Status.

Typical setup:

1. Fixed Frame = `shared_map` (needs `static_map_tfs` running or Fixed Frame stays red)
2. Add → Map → Topic = `/shared_map`

Keep these running while viewing merge: Gazebo, SLAM ×3, static TFs, `merge_node`, RViz.

### Before restarting merge / a combined map-merge launch

**Stop** (avoid duplicate TFs / two merge nodes):

- `static_map_tfs.launch.py`
- `ros2 run map_merge_swarm merge_node`

**Keep** (usually):

- Gazebo spawn, SLAM ×3, Nav2 if any, RViz optional

---

## Common failure and what it means

| Symptom | Cause |
|---|---|
| `No executable found` | `entry_points` missing/wrong, or built before adding it, or forgot to rebuild after editing `setup.py` |
| `ModuleNotFoundError: No module named 'package.node_name'` | Typo in the command name, or entry point points at a file/function that doesn't exist |
| Package not found by `ros2 pkg list` | Build failed silently, or `install/setup.bash` not sourced in this terminal |
| `ros2 launch <package> <file>.launch.py` says file/package not found | Launch file wasn't saved inside the package's own `launch/` folder (e.g. `src/map_merge_swarm/launch/`), or forgot to rebuild after adding it |
| `_TYPE_SUPPORT` / “might be a ROS 1 message type” | Wrong import: used `import nav_msgs.msg as OccupancyGrid` instead of `from nav_msgs.msg import OccupancyGrid` |
| Map topic subscribed but never receives data | QoS mismatch (use RELIABLE + TRANSIENT_LOCAL for maps), or wrong topic name / SLAM not running |
| RViz Fixed Frame red / “Frame [map] does not exist” | Wrong frame, or static TFs not running; edit under **Global Options**, not Global Status |
| RViz Map empty / no `/shared_map` shape | Topic not set to `/shared_map`, or RViz missing `use_sim_time:=true`, or robots never explored |
| Echo shows only `-1` at the top | Looking at the empty corner of a large grid — check RViz or count non-`-1` cells |
| Three map islands side by side on `/shared_map` | Expected for v1 spawn-offset merge — not one shared SLAM yet |

If `entry_points` or `data_files` change, a clean rebuild is safest:

```bash
rm -rf build/<package_name> install/<package_name>
colcon build --packages-select <package_name>
source install/setup.bash
```

---

## See also

- **Map / frontiers / Nav2 concepts:** `docs/P2-ref-engineering-notes.md` (design ideas, not package wiring)
- **Explore:** `ros2 launch explore_swarm explore.launch.py` (`frontier_detector` + `goal_assigner`)
- **Bring-up:** `docs/ref-startup-after-break.md` — §3 live (T0…T9), §6 Step 3 frozen map
