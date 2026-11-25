> **📍 Phase 2 › Step 1 (GT shared map)** · [← Roadmap](swarm_search_rescue_roadmap_v4.md) · [← P2 todo](P2-checklist.md) · Status: ✅ Done

# Plan — Step 1 Option C: Ground-truth shared map (learn + implement)

**Parent todo:** `docs/P2-checklist.md` → **Step 1** (one coherent global map).  
**Goal:** `/shared_map` shows **one** house (walls stack), using Gazebo truth instead of fixed spawn offsets.  
**You implement this yourself** (learning path). This file is the explanation + how-to + checklist (includes material from chat so you can re-learn without scrolling history).

**Related:** `ref-startup-after-break.md` (bring-up — custom world, 8 s spawn delay, ALSA ignore, `model_states` QoS), `P2-S0-todo-map-merge-v1.md` (v1 merge already done), `ref-ros2-pkg-basics.md` (maps / TF / RViz), `P2-checklist.md` (steps after this).

---



## Big idea

Gazebo already has **one true world**. Each robot’s SLAM map lives in its own frame `TB3_N/map` (“private notebook,” start ≈ local 0,0).

Compute for each robot:

> Where does `TB3_N/map` sit inside the Gazebo world?

Treat `shared_map` **= Gazebo world** (same origin/axes for this project).  
Then paint each OccupancyGrid into `/shared_map` with that transform — **not** with fixed `SPAWN_OFFSETS` (v1 only slid by spawn; that can leave three islands).

Model names (spawn): `burger_1` / `burger_2` / `burger_3` ↔ namespaces `TB3_1` / `TB3_2` / `TB3_3`.

---



## Frames glossary (use only these names)


| Name                   | What it is                                     |
| ---------------------- | ---------------------------------------------- |
| `shared_map`           | One global house frame (= Gazebo world for us) |
| `TB3_1/map`            | Robot 1’s SLAM map frame (“my notebook”)       |
| `TB3_1/base_footprint` | Robot 1’s body on the floor                    |


Same pattern for TB3_2 / TB3_3.

A transform `A → B` (written `A_T_B` or `shared_T_base`) means:  
**where frame B sits when you look from frame A**, or how to take a point written in B and rewrite it in A.

---



## The three transforms

For one robot (example TB3_1):

### 1) `shared_T_base`

- **From:** Gazebo `/gazebo/model_states` for model `burger_1`
- **Meaning:** Where is the **robot body** in the **global house**?
- Example: body at house (−1.5, −0.4) with some yaw



### 2) `map_T_base`

- **From:** normal TF lookup: `TB3_1/map` → `TB3_1/base_footprint`
- **Meaning:** Where is the **robot body** in **its own SLAM map**?
- SLAM + odom already publish this chain (`map` → `odom` → `base_footprint`)



### 3) `shared_T_map` ← what we compute and broadcast

- **Meaning:** Where does **robot 1’s SLAM map** sit inside the **global house**?
- This is the missing link for correct merging

---



## Picture

```text
shared_map  (global house)
    │
    │  shared_T_map   ← we compute and broadcast this
    ▼
TB3_1/map  (robot’s private map)
    │
    │  map_T_base     ← TF already has this
    ▼
TB3_1/base_footprint  (robot body)
    ▲
    │  shared_T_base  ← Gazebo already has this
    │
shared_map  (same global house)
```

Two paths from house → body must agree:

```text
shared_T_base  =  shared_T_map  ∘  map_T_base
```

(“∘” = apply the right-hand transform first, then the left — same as chaining matrices / TF.)

Solve for the unknown:

```text
shared_T_map  =  shared_T_base  ∘  inverse(map_T_base)
```

**In words:**  
Take “body in house,” **undo** “body in private map” → what’s left is “private map in house.”

---



## Why merge needs `shared_T_map`

A wall cell in the SLAM message is a point in `TB3_1/map`, e.g. `(map_x, map_y)`.

To draw it on the global canvas you need `shared_map`:

```text
(shared_x, shared_y)  =  shared_T_map  applied to  (map_x, map_y)
```

- **v1 (spawn only):** `shared ≈ map + spawn` (translation only, fixed) — often three side-by-side maps  
- **Option C:** full `shared_T_map` from Gazebo + SLAM TF — walls should stack on one house

---



## Tiny numeric intuition (no rotation)

Suppose at one moment:

- Gazebo: body at house `(10, 0)` → `shared_T_base` translation = 10  
- SLAM: body at private map `(2, 0)` → `map_T_base` translation = 2

Then private map’s origin in the house is about:

```text
10 − 2 = 8
```

So `shared_T_map` translation ≈ 8.  
A cell at map `(2, 0)` (where the robot is) lands at house `(10, 0)` — correct.

With yaw, same idea with rotate + translate (SE2). Inverse = “undo that pose.”

---



## Same math as AV / multi-lidar (different words)

In AV / multi-lidar calibration you often do:

```text
point_in_lidar_A
  → T_world_lidarA
  → T_lidarB_world
  = point_in_lidar_B
```

Or: lidar → ref → camera. Under the hood: multiply / invert **4×4 (or SE2) extrinsics**.

Multi-robot maps here:

```text
TB3_1/map → shared_map ← TB3_2/map
```

Same chain rule. ROS publishes those extrinsics as a **TF tree** (time-stamped edges) instead of a folder of named matrices — each TF edge still *is* a rigid transform.


| AV / calibration language    | This project                                          |
| ---------------------------- | ----------------------------------------------------- |
| world / global               | `shared_map`                                          |
| lidar / sensor / ego vehicle | `TB3_N/base_footprint` (or `base_scan` for the laser) |
| local map / slam map         | `TB3_N/map`                                           |
| `T_world_ego`                | `shared_T_base`                                       |
| `T_map_ego`                  | `map_T_base`                                          |
| `T_world_map`                | `shared_T_map`                                        |


AV form of the same formula:

```text
T_world_localMap = T_world_ego * inv(T_localMap_ego)
```

Why robotics docs say “TF / map / odom / base” instead of “lidar / world”: ROS **REP-105** naming (`map` → `odom` → `base_link`) is the mobile-robot standard; AV stacks prefer ego/world/lidar. **Math is the same.**

---



## Does Gazebo give world poses via IMU?

**For Option C we do not use IMU for** `shared_T_base`**.**


| Source                               | What it is                                                                                            |
| ------------------------------------ | ----------------------------------------------------------------------------------------------------- |
| **Gazebo model state** (what we use) | Simulator’s **true** pose of `burger_N` in the world — like perfect GT `T_world_ego`, not an estimate |
| **IMU** on TB3                       | Noisy sensor for realism / real robots; **not** what locks the shared map in this plan                |
| **Wheel odom TF**                    | Estimated `odom` → `base`; drifts                                                                     |
| **SLAM**                             | Estimated `map` → `odom`                                                                              |


Gazebo runs physics and knows object poses. Reading `/gazebo/model_states` is reading that **ground truth**, analogous to a perfect ego pose in a logged AV dataset — not an IMU fusion output.

---



## What the GT node does each tick

1. Read Gazebo → `shared_T_base`
2. Read TF → `map_T_base`
3. Compute `shared_T_map = shared_T_base ∘ inv(map_T_base)`
4. Broadcast TF: parent `shared_map`, child `TB3_N/map`

Merge then asks TF: transform each cell from `TB3_N/map` into `shared_map`.

---



## Architecture


| Piece                        | Role                                                     |
| ---------------------------- | -------------------------------------------------------- |
| New `gt_map_tf_node.py`      | Gazebo poses + TF → broadcast `shared_map` → `TB3_N/map` |
| Update `merge_node.py`       | Paint using TF lookup, **not** `SPAWN_OFFSETS`           |
| Update `map_merge.launch.py` | Remove static spawn TFs; start GT node + merge           |


Do **not** leave static `shared_map`→`TB3_N/map` publishers running — they conflict with GT.

---



## Implementation guide with code blocks

Follow **in order**. Each part has **Why** (for beginners) then **What to write**.

### CRITICAL — when you must rebuild (read this before Part 1)

`ros2 run` / `ros2 launch` use the **installed** copy under `~/swarm-search/install/`, **not** the file you just edited in `src/`.

**Rule for beginners (memorize this):**  
After **every** change to Python under `src/map_merge_swarm/`, or to `setup.py` / `package.xml`, run these **two** commands in the **same terminal** before `ros2 run` / `ros2 launch`:

```bash
cd ~/swarm-search
colcon build --packages-select map_merge_swarm
source install/setup.bash
```

Also `source` again in **every new terminal** before testing.

Do this after:

- Adding/changing an **entry point** in `setup.py` (e.g. `gt_map_tf_node`)
- Adding a **depend** in `package.xml`
- **Any edit** to `gt_map_tf_node.py` or `merge_node.py` (yes, even one line)
- Seeing `No executable found`, old log text, or “I changed code but nothing happened”


| If you skip…                | What happens                                                                                         |
| --------------------------- | ---------------------------------------------------------------------------------------------------- |
| `colcon build`              | `install/` has no new executable / still has **old** `.py` → `No executable found` or stale behavior |
| `source install/setup.bash` | This terminal still points at an old install → same errors                                           |


**Part 2 / Part 3 trap:** Editing `gt_map_tf_node.py` in the editor does **nothing** for `ros2 run` until build + source. The plan repeats this after each part on purpose.

**Shortcut later (optional):** some people use `colcon build --symlink-install` so Python edits apply without rebuild — until you use that, **always rebuild**.

---



### Part 1 — Package wiring

**Why:**  
Your new node will import `gazebo_msgs` (Gazebo’s pose messages) and use `tf2` helpers. ROS must know those packages exist (`package.xml`).  
`setup.py` **entry_points** register a command name (`gt_map_tf_node`) so `ros2 run map_merge_swarm gt_map_tf_node` knows which Python `main()` to call — same idea as `merge_node`.

`package.xml` — add (keep existing depends):

```xml
  <depend>gazebo_msgs</depend>
  <depend>tf2</depend>
```

`setup.py` — entry points:

```python
entry_points = (
    {
        "console_scripts": [
            "merge_node = map_merge_swarm.merge_node:main",
            "gt_map_tf_node = map_merge_swarm.gt_map_tf_node:main",
        ],
    },
)
```

Create empty/stub file: `src/map_merge_swarm/map_merge_swarm/gt_map_tf_node.py`

**Then immediately (do not skip):**

```bash
cd ~/swarm-search
colcon build --packages-select map_merge_swarm
source install/setup.bash
```

Check the executable exists:

```bash
ros2 pkg executables map_merge_swarm
# should list both: merge_node and gt_map_tf_node
```

---



### Part 2 — GT node: stub (spin only)

**Why:**  
A ROS **node** is one running program that can talk on topics/TF.  
`rclpy.spin(node)` keeps it alive and processes callbacks. Without spin, the node starts and exits immediately.  
`use_sim_time: True` means “use Gazebo’s clock (`/clock`), not your laptop wall clock.” If you forget this while Gazebo runs, TF and timestamps disagree and lookups fail in confusing ways.

```python
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter


class GtMapTfNode(Node):
    def __init__(self):
        super().__init__(
            "gt_map_tf_node",
            parameter_overrides=[
                Parameter("use_sim_time", Parameter.Type.BOOL, True),
            ],
        )
        self.get_logger().info("gt_map_tf_node started")


def main(args=None):
    rclpy.init(args=args)
    node = GtMapTfNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
```

**Before the test:** if you just finished Part 1 or edited `setup.py`, run build + source again in this terminal (see CRITICAL above).

Test:

```bash
ros2 run map_merge_swarm gt_map_tf_node
# expect log: gt_map_tf_node started  (or your log text)
```

If you see `No executable found`: you did not rebuild/source after adding the entry point, or this terminal was never sourced. Fix with the two commands in CRITICAL, then retry.

**Why this test first?** Proves packaging works before you debug math or Gazebo.

---



### Part 3 — Subscribe Gazebo model states

**Why:**  
Gazebo knows the true position of every model in the world. It publishes that on `/gazebo/model_states`.  
That list uses **model names** like `burger_1` (from spawn), while the rest of your stack uses **namespaces** like `TB3_1`. The dict below is only a translation table between those two naming systems.

```python
ROBOTS = {
    "TB3_1": "burger_1",
    "TB3_2": "burger_2",
    "TB3_3": "burger_3",
}
```

**Why subscribe (not a one-shot service)?**  
The robot moves continuously. You need a fresh pose every time you update TF — a subscription delivers a stream.

**Why BEST_EFFORT QoS?**  
Gazebo’s publisher often uses “best effort” (OK to drop a message if busy). If you subscribe with only the default “reliable,” you may get **no messages** with no obvious error. Matching QoS is a ROS beginner trap — same idea as map QoS for SLAM.

```python
from gazebo_msgs.msg import ModelStates
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

qos = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)
self.create_subscription(
    ModelStates, "/gazebo/model_states", self._on_model_states, qos
)
```

**Why print first?**  
Confirm you found the right model and that coordinates move when you teleop — before writing transform math.

```python
def _on_model_states(self, msg: ModelStates):
    try:
        i = msg.name.index("burger_1")
    except ValueError:
        self.get_logger().warn("burger_1 not in model_states")
        return
    p = msg.pose[i].position
    self.get_logger().info(f"burger_1 xyz=({p.x:.2f}, {p.y:.2f}, {p.z:.2f})")
```

With Gazebo spawn running:

```bash
# 1) rebuild + source AFTER every code edit (required)
cd ~/swarm-search
colcon build --packages-select map_merge_swarm
source install/setup.bash

# 2) confirm topic (note QoS!)
ros2 topic echo /gazebo/model_states --qos-reliability best_effort --once | head -40
# confirm burger_1 / burger_2 / burger_3 exist

# 3) run node
ros2 run map_merge_swarm gt_map_tf_node
# drive TB3_1 — numbers should change
```

That pose is `shared_T_base`: “where is the robot body in the house?”

**After editing Part 3 code — always:**

```bash
cd ~/swarm-search
colcon build --packages-select map_merge_swarm
source install/setup.bash
```

**Before testing Part 3 — Gazebo must be running** (spawn launch). Without Gazebo there is no `/gazebo/model_states`.

**Echo this topic correctly** (Gazebo uses best-effort; plain echo often prints **nothing**):

```bash
ros2 topic echo /gazebo/model_states --qos-reliability best_effort --once
```

Then run your node:

```bash
ros2 run map_merge_swarm gt_map_tf_node
# expect burger_1 xyz: ...  (or warn if name missing)
```

---



### Part 4 — TF listener for `map_T_base`

**Why:**  
Gazebo told you body-in-house. You also need body-in-private-map. That already lives in the **TF tree** (SLAM publishes `map`→`odom`, Gazebo diff-drive publishes `odom`→`base_footprint`).  
`Buffer` + `TransformListener` = “listen to `/tf` and remember recent transforms so I can ask questions later.”

```python
from tf2_ros import Buffer, TransformListener

self.tf_buffer = Buffer()
self.tf_listener = TransformListener(self.tf_buffer, self)
```

**Why lookup** `map` **→** `base_footprint`**?**  
That single query asks TF to **chain** whatever links exist between those frames. You get `map_T_base` without manually multiplying map→odom→base yourself.

```python
from rclpy.time import Time

# inside loop for each robot_ns, after you have Gazebo pose:
try:
    map_to_base = self.tf_buffer.lookup_transform(
        f"{robot_ns}/map",
        f"{robot_ns}/base_footprint",
        Time(),  # latest available
    )
except Exception as e:
    self.get_logger().warn(
        f"TF {robot_ns}/map -> base failed: {e}", throttle_duration_sec=2.0
    )
    continue
```

**Why try/except and skip?**  
If SLAM is not up yet, `TB3_N/map` does not exist. Crashing the whole node is worse than waiting. `throttle_duration_sec` avoids flooding the log every cycle.

`map_to_base` is `map_T_base`.

**After editing Part 4 code:**

```bash
cd ~/swarm-search
colcon build --packages-select map_merge_swarm
source install/setup.bash
```

Need SLAM running or lookup fails — that is OK; skip that robot until ready.

---



### Part 5 — Compose `shared_T_map` and broadcast

**Why:**  
You have two facts about the same body. Combining them answers: “where does the private map sit in the house?”

```text
shared_T_map = shared_T_base ∘ inverse(map_T_base)
```

**Why inverse?**  
`map_T_base` is “body expressed in map.” Inverse is “map expressed relative to body.” Chaining with “body in house” yields “map in house.” Same trick as in multi-lidar: undo one extrinsic, apply another.

**Why 2D helpers (x, y, yaw) instead of full 4×4?**  
TurtleBot drives on a flat floor. Roll/pitch are ~0. SE2 is enough, easier to print and debug than full quaternions/matrices. (AV stacks often use 4×4; math is the same idea.)

```python
import math
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


def yaw_from_quat(x, y, z, w):
    # ROS poses use quaternions; we only need heading (yaw) on flat ground
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def se2_inverse(x, y, yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    # Undo the pose: opposite rotation, then opposite translation
    return (-c * x - s * y, s * x - c * y, -yaw)


def se2_compose(x1, y1, yaw1, x2, y2, yaw2):
    # result = T1 ∘ T2  (apply T2 first, then T1)
    c, s = math.cos(yaw1), math.sin(yaw1)
    return (
        x1 + c * x2 - s * y2,
        y1 + s * x2 + c * y2,
        yaw1 + yaw2,
    )
```

From Gazebo pose + `map_to_base` transform:

```python
# shared_T_base from Gazebo
gx = msg.pose[i].position.x
gy = msg.pose[i].position.y
gq = msg.pose[i].orientation
gyaw = yaw_from_quat(gq.x, gq.y, gq.z, gq.w)

# map_T_base from TF
t = map_to_base.transform.translation
r = map_to_base.transform.rotation
mx, my = t.x, t.y
myaw = yaw_from_quat(r.x, r.y, r.z, r.w)

ix, iy, iyaw = se2_inverse(mx, my, myaw)
sx, sy, syaw = se2_compose(gx, gy, gyaw, ix, iy, iyaw)
# (sx, sy, syaw) = shared_T_map
```

**Why broadcast TF?**  
Other nodes (your merge, RViz, later Nav2) should not re-do this math. Publishing `shared_map` → `TB3_N/map` puts the answer on the shared TF tree — same idea as publishing an extrinsic so every tool can reuse it.

In `__init__`: `self.tf_broadcaster = TransformBroadcaster(self)`

```python
out = TransformStamped()
out.header.stamp = self.get_clock().now().to_msg()
out.header.frame_id = "shared_map"  # parent = global house
out.child_frame_id = f"{robot_ns}/map"  # child = private map
out.transform.translation.x = sx
out.transform.translation.y = sy
out.transform.translation.z = 0.0
# yaw -> quaternion (roll=pitch=0); ROS expects quaternion, not yaw alone
out.transform.rotation.z = math.sin(syaw * 0.5)
out.transform.rotation.w = math.cos(syaw * 0.5)
self.tf_broadcaster.sendTransform(out)
```

Do that for all three robots each callback (or on a timer).

**After editing Part 5 code:**

```bash
colcon build --packages-select map_merge_swarm && source install/setup.bash
```

**Why stop static spawn TFs?**  
Two publishers for the same parent→child TF fight each other. Only GT should own `shared_map` → `TB3_N/map`.

```bash
ros2 run map_merge_swarm gt_map_tf_node
ros2 run tf2_ros tf2_echo shared_map TB3_1/map
```

You want a live transform (may change slightly as SLAM updates `map→odom`).

---



### Part 6 — Change merge to use TF (not spawn)

**Why:**  
Merge’s job is: take cells from each robot’s OccupancyGrid and stamp them onto one canvas.  
Those cells are coordinates in `TB3_N/map`. After Part 5, TF knows how to convert them into `shared_map`.  
Using TF (instead of `SPAWN_OFFSETS`) means merge automatically follows the GT alignment — one house, not three fixed slides.

**Why Buffer in merge too?**  
Merge is a **different process**. It does not see variables inside `gt_map_tf_node`. It only sees what is on `/tf`. So it needs its own listener.

In `merge_node.py` `__init__`:

```python
from tf2_ros import Buffer, TransformListener

self.tf_buffer = Buffer()
self.tf_listener = TransformListener(self.tf_buffer, self)
```

**Why lookup** `shared_map` **←** `TB3_N/map`**?**  
Ask: “How do I express a point from the private map in the global house?” That is exactly `shared_T_map` that GT published.

**Why skip unknown cells (**`value == -1`**)?**  
Unknown means “not explored.” Painting them would wipe free/occupied from other robots with empty fog.

**Why rotate then translate?**  
That is applying SE2: `shared = R * map_point + t`, same as multiplying by the extrinsic matrix.

```python
import math
from rclpy.time import Time


def _paint_robot_map(self, canvas, robot_ns, msg):
    try:
        tf = self.tf_buffer.lookup_transform("shared_map", f"{robot_ns}/map", Time())
    except Exception as e:
        self.get_logger().warn(
            f"No shared_map <- {robot_ns}/map: {e}",
            throttle_duration_sec=2.0,
        )
        return

    tx = tf.transform.translation.x
    ty = tf.transform.translation.y
    q = tf.transform.rotation
    yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
    c, s = math.cos(yaw), math.sin(yaw)

    res = msg.info.resolution
    width = msg.info.width
    height = msg.info.height
    origin_x = msg.info.origin.position.x
    origin_y = msg.info.origin.position.y

    for row in range(height):
        for col in range(width):
            value = int(msg.data[row * width + col])
            if value == -1:
                continue
            # cell center in the robot's private map frame
            map_x = origin_x + (col + 0.5) * res
            map_y = origin_y + (row + 0.5) * res
            # apply shared_T_map
            shared_x = tx + c * map_x - s * map_y
            shared_y = ty + s * map_x + c * map_y
            idx = self._shared_to_index(shared_x, shared_y)
            if idx is not None:
                canvas[idx] = self._fuse_maps(canvas[idx], value)
```

Remove `from map_merge_swarm.spawn_offsets import SPAWN_OFFSETS` if unused.  
Keep fuse / canvas / publisher as they are.

**Why keep fuse?**  
When two robots see the same wall, both write the same cell. Occupied-wins (and free over unknown) merges their knowledge on one canvas.

**After editing Part 6 (**`merge_node.py`**):**

```bash
colcon build --packages-select map_merge_swarm && source install/setup.bash
```

---



### Part 7 — Launch file

**Why a launch file?**  
So you start GT + merge with one command, both with `use_sim_time`, instead of remembering two terminals and args.

**Why remove static_transform_publisher?**  
Those published a **fixed** spawn guess. GT publishes the **live** correct `shared_map`→`map`. Only one should exist.

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            Node(
                package="map_merge_swarm",
                executable="gt_map_tf_node",
                name="gt_map_tf_node",
                output="screen",
                parameters=[{"use_sim_time": use_sim_time}],
            ),
            Node(
                package="map_merge_swarm",
                executable="merge_node",
                name="map_merge_node",
                output="screen",
                parameters=[{"use_sim_time": use_sim_time}],
            ),
        ]
    )
```

**After editing Part 7 launch file:**

```bash
colcon build --packages-select map_merge_swarm && source install/setup.bash
# launch files are installed via setup.py data_files — rebuild required
```

---



### Part 8 — Full verify

**Why this order?**  
Gazebo must exist before poses. SLAM must exist before `map`→`base`. GT must exist before merge can lookup `shared_map`→`map`. RViz is only for your eyes.

1. Stop old static TF / old merge if running
2. Spawn → SLAM ×3 → `ros2 launch map_merge_swarm map_merge.launch.py`
3. RViz: Fixed Frame `shared_map`, Map `/shared_map`
4. Drive overlapping areas → **one** house (vs your baseline screenshot)

```bash
ros2 run tf2_ros tf2_echo shared_map TB3_1/map
ros2 topic echo /shared_map --once | head -25
```

**Why RViz > echo for success?**  
Echo shows the start of a huge grid (often all `-1` in the empty corner). RViz shows whether walls form one house.

---



### Suggested build order


| #   | Task                         | Why this step                          | Done when              |
| --- | ---------------------------- | -------------------------------------- | ---------------------- |
| 1   | Stub GT node                 | Prove ROS packaging before hard bugs   | runs + log             |
| 2   | Print `burger_1` pose        | Prove Gazebo name + QoS                | moves when robot moves |
| 3   | Lookup `map`→`base`          | Prove SLAM TF available                | works with SLAM up     |
| 4   | Broadcast `shared_map`→`map` | Publish the extrinsic everyone needs   | `tf2_echo` OK          |
| 5   | Merge paints via TF          | Use that extrinsic to stamp one canvas | no `SPAWN_OFFSETS`     |
| 6   | Launch without static TFs    | One-command bring-up, no TF fights     | one house in RViz      |


---



## Implement checklist (tick as you go)



### A. Package wiring

- [x] `gazebo_msgs` / `tf2` in `package.xml`
- [x] `gt_map_tf_node.py` created
- [x] Entry point in `setup.py`
- [x] `colcon build` **+** `source install/setup.bash` (required or `No executable found`)
- [x] `ros2 pkg executables map_merge_swarm` shows `gt_map_tf_node`



### B. GT node

- [x] Stub
- [x] Gazebo pose print
- [x] TF `map`→`base`
- [x] Broadcast `shared_T_map`
- [x] `tf2_echo shared_map TB3_1/map` OK without static TFs



### C. Merge

- [x] TF buffer/listener
- [x] Paint via TF; no spawn offsets
- [x] Lookup fail → skip + throttle warn



### D. Launch

- [x] No static spawn TFs
- [x] `gt_map_tf_node` + `merge_node`



### E. Verify

- [x] One house on `/shared_map` in RViz
- [x] Tick Step 1 in `P2-checklist.md`

---



## Pitfalls


| Symptom                                                   | Likely cause                                                                                               |
| --------------------------------------------------------- | ---------------------------------------------------------------------------------------------------------- |
| `No executable found` **for** `gt_map_tf_node`            | Added entry point / new file but **did not** `colcon build` + `source install/setup.bash` in this terminal |
| **Code changed but node behavior / logs unchanged**       | Edited `src/` but forgot rebuild + source — still running old `install/` copy                              |
| `ros2 topic echo /gazebo/model_states` **prints nothing** | (1) Gazebo not running, or (2) need `--qos-reliability best_effort`                                        |
| No `burger_N` in model_states                             | Check names: `ros2 topic echo /gazebo/model_states --qos-reliability best_effort --once`                   |
| `map`→`base` lookup fails                                 | SLAM not up — skip until available                                                                         |
| Two publishers for `shared_map`→`TB3_N/map`               | Static TF still running                                                                                    |
| Still three islands                                       | Merge still using spawn offsets                                                                            |
| TF time errors                                            | `use_sim_time` not set on GT/merge                                                                         |
| `ModuleNotFoundError: gazebo_msgs`                        | Missing depend / rebuild / source                                                                          |


---



## After THIS plan is done — what next (which file)

This Option C plan **only finishes Step 1** of `P2-checklist.md`.


| When Option C / Step 1 is done | Open this file                      | Do this step                                                                        |
| ------------------------------ | ----------------------------------- | ----------------------------------------------------------------------------------- |
| Next                           | `P2-S2-plan-freeze-team-map.md`           | **Step 2** — Freeze team map (`map_saver` → `yaml` + `pgm`)                         |
| Then                           | `P2-checklist.md`            | **Step 3** — Proof: R2 localizes + Nav2 on that map (R1 explored → R2 drives there) |
| Then                           | `P2-checklist.md`            | **Step 4** — Live sharing (optional comms radius stub)                              |
| Only after Step 3 passes       | `P2-checklist.md`            | **Step 5** — Frontiers on the global map                                            |
| Daily bring-up                 | `ref-startup-after-break.md`            | Terminal order to run the stack                                                     |
| Overall phase plan             | `swarm_search_rescue_roadmap_v4.md` | Still **Phase 2** until frontiers + comms + coverage plots exist                    |


**Do not** start frontiers (`P2-checklist.md` Step 5) or roadmap Phase 3 (aerial) until Step 3 of `P2-checklist.md` passes.

**Already finished (do not redo):** `P2-S0-todo-map-merge-v1.md` (merge v1 with spawn offsets) — you are upgrading alignment via this Option C plan.