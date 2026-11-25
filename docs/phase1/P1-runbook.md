> **📍 Phase 1** · [← Roadmap](swarm_search_rescue_roadmap_v4.md) · Status: ✅ Done

# Phase 1 Runbook — Multi-Robot Search & Rescue Sim

One TurtleBot3 in a Gazebo house world, mapped live with SLAM, driven autonomously with Nav2. This is the checkpoint v4 of the roadmap calls "Phase 1" — everything below gets you back to a fully working state on a fresh machine without re-discovering any of tonight's bugs.

## Final working stack

- **OS:** Ubuntu 22.04 (WSL2)
- **ROS2 distro:** Humble
- **Simulator:** Gazebo Classic (11), not Gazebo Harmonic/Jetty/Sim
- **Robot:** TurtleBot3 Waffle
- **Nav stack:** Nav2 + slam_toolbox (online async), controller is DWB wrapped in RotationShimController

This combination was landed on after two other combinations failed for reasons that are worth remembering (see "Why not Lyrical or Jazzy" below), so don't be tempted to chase the newest ROS2/Gazebo release on a future machine without expecting the same class of pain.

---

## 0. WSL setup

One distro only, not multiple side by side — disk space runs out fast with more than one full ROS2 + Gazebo install (each is roughly 15–40GB depending on what you install).

```powershell
# PowerShell
wsl -l -v
wsl --install -d Ubuntu-22.04
```

Sanity check that WSLg can actually open a window before installing anything heavy:

```bash
sudo apt update
sudo apt install -y mesa-utils x11-apps
xclock
```

If `xclock` shows a taskbar icon but no window ever appears, that's a known WSLg stale-session issue on a fresh distro, not something wrong with your setup:

```powershell
wsl --shutdown
wsl --update
```

Wait a few seconds, reopen, try again. Closing a terminal window does **not** shut down WSL, the VM keeps running in the background — this is why a stale process can survive what looks like a fresh terminal.

---

## 1. Install ROS2 Humble, Gazebo Classic, Nav2, SLAM, TurtleBot3

```bash
sudo apt install curl -y
curl -L -o /tmp/ros2-apt-source.deb \
  https://github.com/ros-infrastructure/ros-apt-source/releases/download/1.2.0/ros2-apt-source_1.2.0.jammy_all.deb
sudo dpkg -i /tmp/ros2-apt-source.deb
sudo apt update && sudo apt upgrade -y

sudo apt install ros-humble-ros-base ros-humble-rviz2 -y
sudo apt install gazebo ros-humble-gazebo-ros-pkgs -y
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup -y
sudo apt install ros-humble-slam-toolbox -y
sudo apt install ros-humble-turtlebot3 ros-humble-turtlebot3-msgs \
  ros-humble-turtlebot3-simulations ros-humble-turtlebot3-teleop -y

echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

## 2. Environment variables — put these in `.bashrc` permanently

```bash
echo 'export TURTLEBOT3_MODEL=waffle' >> ~/.bashrc
echo 'export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/humble/share/turtlebot3_gazebo/models' >> ~/.bashrc

# GPU rendering fix — without this, Gazebo runs on CPU software rendering (llvmpipe),
# which shows as near-0% GPU usage and heavy frame drops even with a real GPU present
echo 'export GALLIUM_DRIVER=d3d12' >> ~/.bashrc
echo 'export MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA' >> ~/.bashrc

source ~/.bashrc
```

Confirm the GPU fix actually took:

```bash
glxinfo | grep "OpenGL renderer"
```

Should mention D3D12 and your GPU name, not `llvmpipe`.

---

## 3. WSL memory allocation

Without this, WSL caps itself at roughly half your physical RAM by default, which silently makes Nav2 fail to bring up its ~11 nodes with no obvious error (they just never activate, or costmaps time out).

```powershell
notepad %UserProfile%\.wslconfig
```

```
[wsl2]
memory=10GB
swap=32GB
```

(Adjust `memory` down if your machine has less than 16GB total — leave Windows itself at least 4–6GB.)

```powershell
wsl --shutdown
```

---

## 4. Repo structure

```
swarm-search/
├── archive/
│   └── pybullet_phase1/       # old single-drone PyBullet/MARL work, frozen, not extended
├── launch/
├── worlds/
├── src/
│   ├── comms/
│   ├── map/
│   ├── assign/
│   └── perception/
├── analysis/
├── media/
└── nav2_params/
    └── waffle.yaml            # our own copy, see section 5
```

Clone or copy your repo back in before starting:

```bash
git clone https://github.com/<your-username>/swarm-search.git
```

---

## 5. `nav2_params/waffle.yaml` — required changes from the stock file

Start from TurtleBot3's own model-matched params, not the generic `nav2_bringup` defaults (generic ones don't know the robot's real footprint size and cause constant wall collisions):

```bash
mkdir -p ~/swarm-search/nav2_params
cp /opt/ros/humble/share/turtlebot3_navigation2/param/waffle.yaml \
   ~/swarm-search/nav2_params/waffle.yaml
```

This file needs several edits before it actually works. Each one is explained in the postmortem section below — this is just the final result.

### 5a. `use_sim_time` — add explicitly to every single node section

The `use_sim_time:=true` launch argument does **not** reliably propagate to every node in `navigation_launch.py`. This was the single biggest time sink tonight — it silently stayed `False` on `controller_server`, both costmaps, `planner_server`, and `bt_navigator` at different points, each one needing to be caught individually. Don't trust the launch flag — set it in the YAML directly, everywhere:

```yaml
controller_server:
  ros__parameters:
    use_sim_time: True
    ...

local_costmap:
  local_costmap:
    ros__parameters:
      use_sim_time: True
      ...

global_costmap:
  global_costmap:
    ros__parameters:
      use_sim_time: True
      ...

planner_server:
  ros__parameters:
    use_sim_time: True
    ...

bt_navigator:
  ros__parameters:
    use_sim_time: True
    ...

behavior_server:
  ros__parameters:
    use_sim_time: True
    ...
```

RViz also needs it — but as a node, not through the params file (see section 6, terminal 3).

### 5b. Plugin class name format — inconsistent across sections, don't blanket-replace

Different plugin families use different separators between package and class name in this Humble build. Getting this wrong throws a `FATAL` error naming the plugin that failed and lists the correct "Declared types" right in the error — always copy that exact string rather than guessing or regex-replacing broadly:

| Plugin family | Correct format |
|---|---|
| `nav2_navfn_planner` (planner) | `nav2_navfn_planner/NavfnPlanner` (slash) |
| `nav2_behaviors` (spin, backup, wait, etc.) | `nav2_behaviors/Spin` etc. (slash) |
| `nav2_controller` (progress/goal checkers) | `nav2_controller::SimpleProgressChecker` etc. (double colon) |
| `nav2_costmap_2d` (obstacle/static/inflation layers) | `nav2_costmap_2d::ObstacleLayer` etc. (double colon) |
| `dwb_core` | `dwb_core::DWBLocalPlanner` (double colon) |

### 5c. Controller — RotationShimController wrapping DWB

Plain DWB blends rotating and driving forward at once, which produces wide swinging arcs instead of turning in place, and fights itself trying to match final heading at the goal (constant small left-right corrections that never settle). `RotationShimController` splits rotation and driving into two clean separate steps:

```yaml
controller_server:
  ros__parameters:
    use_sim_time: True
    controller_frequency: 10.0
    min_x_velocity_threshold: 0.001
    min_y_velocity_threshold: 0.001
    min_theta_velocity_threshold: 0.05   # was 0.001 — too sensitive, caused twitching near goal
    failure_tolerance: 0.3
    progress_checker_plugins: ["progress_checker"]
    goal_checker_plugins: ["goal_checker"]
    controller_plugins: ["FollowPath"]

    progress_checker:
      plugin: "nav2_controller::SimpleProgressChecker"
      required_movement_radius: 0.1
      movement_time_allowance: 10.0

    goal_checker:
      stateful: true
      plugin: "nav2_controller::SimpleGoalChecker"
      xy_goal_tolerance: 0.25
      yaw_goal_tolerance: 0.5

    FollowPath:
      plugin: "nav2_rotation_shim_controller::RotationShimController"
      primary_controller: "dwb_core::DWBLocalPlanner"   # required — omitting this is a FATAL error
      angular_dist_threshold: 0.785
      forward_sampling_distance: 0.5
      rotate_to_heading_angular_vel: 1.8
      max_angular_accel: 3.2
      simulate_ahead_time: 1.0
      rotate_to_goal_heading: true    # lets the shim handle final-heading rotation cleanly
      debug_trajectory_details: true
      min_vel_x: 0.0
      min_vel_y: 0.0
      max_vel_x: 0.3
      max_vel_y: 0.0
      max_vel_theta: 1.0
      min_speed_xy: 0.0
      max_speed_xy: 0.3
      min_speed_theta: 0.0
      acc_lim_x: 3.0
      acc_lim_y: 0.0
      acc_lim_theta: 3.2
      decel_lim_x: -2.5
      decel_lim_y: 0.0
      decel_lim_theta: -3.2
      vx_samples: 20
      vy_samples: 0
      vtheta_samples: 40
      sim_time: 1.5
      linear_granularity: 0.05
      angular_granularity: 0.025
      transform_tolerance: 0.5
      trans_stopped_velocity: 0.25
      short_circuit_trajectory_evaluation: true
      stateful: true
      # RotateToGoal removed — the shim's rotate_to_goal_heading now owns final rotation,
      # keeping both active caused them to fight each other
      critics: ["Oscillation", "BaseObstacle", "GoalAlign", "PathAlign", "PathDist", "GoalDist"]
      BaseObstacle.scale: 0.02
      PathAlign.scale: 32.0
      PathAlign.forward_point_distance: 0.1
      GoalAlign.scale: 24.0
      GoalAlign.forward_point_distance: 0.1
      PathDist.scale: 32.0
      GoalDist.scale: 24.0
```

---

## 6. Launch sequence — five terminals, in order

Each `source /opt/ros/humble/setup.bash` is only needed if it's not already in `.bashrc`.

**Terminal 1 — Gazebo, house world**
```bash
ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py
```

**Terminal 2 — SLAM**
```bash
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=true
```

**Terminal 3 — RViz, with sim time forced (not through the launch file argument — it doesn't reliably take)**
```bash
ros2 run rviz2 rviz2 -d /opt/ros/humble/share/nav2_bringup/rviz/nav2_default_view.rviz \
  --ros-args -p use_sim_time:=true
```

**Terminal 4 — Nav2, pointed at our own params file**
```bash
ros2 launch nav2_bringup navigation_launch.py use_sim_time:=true \
  params_file:=$HOME/swarm-search/nav2_params/waffle.yaml
```

**Terminal 5 — verification, run this before touching RViz at all**
```bash
ros2 param get /rviz use_sim_time
ros2 param get /controller_server use_sim_time
ros2 param get /local_costmap/local_costmap use_sim_time
ros2 param get /global_costmap/global_costmap use_sim_time
ros2 param get /planner_server use_sim_time
ros2 param get /bt_navigator use_sim_time
ros2 param get /controller_server FollowPath.plugin
```

Every `use_sim_time` must say `True`, and the last line must say `nav2_rotation_shim_controller::RotationShimController`. Skipping this check and testing navigation anyway is how tonight's confusion mostly happened — several "fixes" got tested against a stale, unfixed process without realizing it.

**Drive a bit first**, teleop, so SLAM has something built before asking Nav2 to plan anywhere:
```bash
ros2 run turtlebot3_teleop teleop_keyboard
```
`s` to stop cleanly. **Then actually kill this terminal (Ctrl+C), don't just press `s`** — a "stopped" teleop process is still a live publisher on `/cmd_vel` and will fight Nav2's own commands later, causing erratic circling that looks like a planning bug but isn't.

In RViz: add a **Map** display if not already present (Add → By topic → `/map`), click **Nav2 Goal**, click a point in the mapped gray area.

---

## Why not Lyrical or Jazzy (context for future you)

Three distro/Gazebo combinations were tried in order tonight. Worth knowing why the first two were abandoned, so a future setup doesn't repeat the same hours:

**Ubuntu 26.04 + ROS2 Lyrical Luth + Gazebo Jetty** — the newest possible combination at the time. Failed because Nav2's own binary packages weren't fully published for Lyrical yet (`apt install ros-lyrical-navigation2` returned "unable to locate package"). Nav2 releases its binaries somewhat after each new ROS distro, not simultaneously — this is a recurring pattern, not a one-off.

**Ubuntu 24.04 + ROS2 Jazzy + Gazebo Harmonic** — Nav2 binaries existed here, but Jazzy-era Nav2 partially migrated to publishing `TwistStamped` on `/cmd_vel` instead of plain `Twist`, and this migration wasn't fully wired through every node — `collision_monitor` and `docking_server` kept publishing the old `Twist` type even with `enable_stamped_cmd_vel:=true` set, while `ros_gz_bridge` (the thing that forwards commands into Gazebo) only accepted `TwistStamped`. Mismatched types never connect in ROS2, so velocity commands were silently going nowhere. This surfaced as: Nav2 reporting active and planning fine, but zero physical robot movement, even for a plain in-place `spin` behavior. Also hit a separate, unrelated bug on this combination: a legacy leading-slash frame name (`/base_link` instead of `base_link`) in the robot's static transform tree, worked around with a `static_transform_publisher` identity bridge — this was specific to that install and not seen again after switching to Humble.

**Ubuntu 22.04 + ROS2 Humble + Gazebo Classic** — the combination that actually worked. Gazebo Classic's ROS integration is a native plugin, no separate bridge process translating between message types, so the whole TwistStamped category of bug doesn't exist here. Years of TurtleBot3 + Humble + Classic in production use elsewhere.

**Lesson for next time:** don't default to the newest release of everything. For a time-bound deliverable, one version behind the bleeding edge is usually the faster path, not the slower one.

---

## Every other bug hit tonight, symptom → root cause → fix

**Xclock window never appears, only a taskbar icon** — stale WSLg compositor session, common right after a fresh distro install or a `wsl --update`. Fix: `wsl --shutdown` from PowerShell (closing the terminal does *not* stop the underlying VM), then reopen fresh.

**Gazebo running at high CPU, ~0% GPU, heavy frame drops** — WSLg silently falling back to Mesa's software renderer (`llvmpipe`) instead of the real GPU. Fix: `GALLIUM_DRIVER=d3d12` + `MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA` env vars.

**RTF (real time factor) sitting well under 1.0, everything sluggish including teleop control** — genuine CPU/physics load, gets worse as more of the map is explored (more occupancy cells, more lidar raycasting per tick). Not something to eliminate entirely on a modest machine — for demo footage, just record and speed the clip up in editing afterward.

**System memory near 100%, Nav2 nodes failing to activate with no clear error** — WSL2 defaults to capping itself at ~50% of host RAM. Fix: explicit `memory=` line in `.wslconfig`, sized to leave the Windows host a few GB of its own headroom.

**`lifecycle_manager` stuck forever on "Waiting for service controller_server/get_state"** — usually the node was silently killed under memory pressure before finishing startup, or a stale process from an earlier run never fully exited. Check `ros2 node list | grep <name>` — a truly hung wait (minutes, not seconds) after a `free -h` showing near-zero available memory means a process didn't survive.

**`ros2 topic info /cmd_vel -v` shows two different message types on one topic** — two different processes (e.g. Nav2 and a still-running `teleop_keyboard`, or a Twist/TwistStamped mismatch) both publishing to the same topic name. ROS2 requires matching types between publisher and subscriber — a mismatch means the subscriber silently never receives anything. Always check this when commands are being computed but nothing moves.

**Robot won't spawn in Gazebo Classic at all** — `GAZEBO_MODEL_PATH` wasn't pointing at TurtleBot3's model files. Needed explicitly for Classic; not an issue on newer Gazebo (Harmonic/Jetty).

**"Transform data too old when converting from map to odom" (`tf_help` error), spamming continuously** — this one had multiple layered causes across the night, worth understanding the actual mechanism: the error compares the *stamp already baked into whichever pose is being transformed* ("Data time") against the *latest transform actually in the tf buffer* ("Transform time"). It is not a live clock reading. Anywhere a node stamps a pose using real wall-clock time instead of sim time, this error follows for as long as that stamp is in use. Every node that touches poses needs `use_sim_time: True` set explicitly — the launch-level flag does not reliably cascade to all of them. Caught missing, one at a time, on: `controller_server`, `local_costmap`, `global_costmap`, `planner_server`, `bt_navigator`. RViz needed it passed directly as a `-p` ros-arg since its own launch file's argument didn't take either.

**Duplicate/stale `controller_server` process running alongside a freshly launched one** — caused log timestamps to visibly jump backward (a strong tell — a single process's logs never go backward in time). Always confirm a full clean stop (`ros2 node list | grep <name>` returns empty) before relaunching, `Ctrl+C` and wait for it to actually print shutdown messages rather than assuming it's gone.

**Robot reaches goal position but shakes/oscillates or spins a full circle instead of stopping** — `RotateToGoal` critic actively fighting a tight `yaw_goal_tolerance`, worsened by any residual sim-time lag. Fixed by moving final-heading handling entirely to `RotationShimController`'s `rotate_to_goal_heading: true` and removing `RotateToGoal` from DWB's own critics list, so only one thing owns that job.

**Wide swinging-arc turns instead of rotating in place when a new goal is behind the robot** — DWB blends rotating and driving forward simultaneously by default. Fixed by `RotationShimController`, which does a clean in-place rotation first, then hands off to DWB once roughly facing the right way.

**Robot moves erratically, ignoring the drawn path, sometimes circling into walls** — a `teleop_keyboard` process from earlier testing was still alive and still a registered publisher on `/cmd_vel`, fighting Nav2's own commands. Pressing `s` only zeroes velocity, it does not kill the node. Always fully `Ctrl+C` teleop once Nav2 takes over.
