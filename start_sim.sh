#!/usr/bin/env bash
# ==============================================================================
# One-Click Swarm Bringup: Gazebo + 3x Burger + 3x SLAM + 3x Nav2 + Map Merge
# ==============================================================================
set -e

WORKSPACE_DIR="$HOME/swarm-search"
cd "$WORKSPACE_DIR"

source /opt/ros/humble/setup.bash
source "$WORKSPACE_DIR/install/setup.bash"
export TURTLEBOT3_MODEL=burger
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/humble/share/turtlebot3_gazebo/models
mkdir -p "$WORKSPACE_DIR/launch/tmp_sdf"

echo "=== [1/5] Cleaning up old processes... ==="
pkill -9 gzserver 2>/dev/null || true
pkill -9 gzclient 2>/dev/null || true
pkill -f "slam_toolbox" 2>/dev/null || true
pkill -f "nav2_" 2>/dev/null || true
pkill -f "map_merge" 2>/dev/null || true
pkill -f "gt_map_tf_node" 2>/dev/null || true
sleep 2

PIDS=()
cleanup() {
    echo -e "\n=== Shutting down all nodes and Gazebo... ==="
    for pid in "${PIDS[@]}"; do
        kill -SIGINT "$pid" 2>/dev/null || true
    done
    sleep 2
    pkill -9 gzserver 2>/dev/null || true
    pkill -9 gzclient 2>/dev/null || true
    pkill -f "slam_toolbox" 2>/dev/null || true
    pkill -f "nav2_" 2>/dev/null || true
    pkill -f "map_merge" 2>/dev/null || true
    echo "=== All processes stopped cleanly ==="
    exit 0
}
trap cleanup SIGINT SIGTERM

echo "=== [2/5] Starting Gazebo & 3 Robots... ==="
ros2 launch launch/multi_robot_spawn.launch.py use_sim_time:=true &
PIDS+=($!)
sleep 9

echo "=== [3/5] Starting SLAM for TB3_1, TB3_2, TB3_3... ==="
ros2 launch launch/slam_tb3.launch.py robot_ns:=TB3_1 use_sim_time:=true slam_params_file:=slam_params/mapper_params_tb3_1.yaml &
PIDS+=($!)
ros2 launch launch/slam_tb3.launch.py robot_ns:=TB3_2 use_sim_time:=true slam_params_file:=slam_params/mapper_params_tb3_2.yaml &
PIDS+=($!)
ros2 launch launch/slam_tb3.launch.py robot_ns:=TB3_3 use_sim_time:=true slam_params_file:=slam_params/mapper_params_tb3_3.yaml &
PIDS+=($!)
sleep 3

echo "=== [4/5] Starting Nav2 for TB3_1, TB3_2, TB3_3... ==="
ros2 launch launch/nav2_tb3.launch.py robot_ns:=TB3_1 use_sim_time:=true params_file:=nav2_params/burger_tb3_1.yaml &
PIDS+=($!)
ros2 launch launch/nav2_tb3.launch.py robot_ns:=TB3_2 use_sim_time:=true params_file:=nav2_params/burger_tb3_2.yaml &
PIDS+=($!)
ros2 launch launch/nav2_tb3.launch.py robot_ns:=TB3_3 use_sim_time:=true params_file:=nav2_params/burger_tb3_3.yaml &
PIDS+=($!)
sleep 4

echo "=== [5/5] Starting Map Merge... ==="
ros2 launch map_merge_swarm map_merge.launch.py &
PIDS+=($!)

echo -e "\n========================================================"
echo "  ALL 3 ROBOTS, SLAM, NAV2 & MAP MERGE ARE RUNNING!"
echo "  Terminal 2: ros2 run rviz2 rviz2 --ros-args -p use_sim_time:=true"
echo "  Terminal 3: ros2 launch explore_swarm explore.launch.py"
echo "  Press Ctrl+C here anytime to stop everything."
echo "========================================================"

wait

