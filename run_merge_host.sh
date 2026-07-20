#!/usr/bin/env bash
# Live two-LiDAR merge feeding GLIM. Run this on the HOST (one terminal).
# Publishes /merged_scan + /merged_imu (fused), then plays the bag.
# Usage: ./run_merge_host.sh [bag_dir] [extra ros2 bag play args...]
# Downsample knobs (see merge_clouds.py): GLIM_MERGE_STRIDE=N, GLIM_MERGE_VOXEL=0.1
set -euo pipefail

export FASTRTPS_DEFAULT_PROFILES_FILE=/home/roser/repos/glim/fastdds_udp.xml
source /opt/ros/jazzy/setup.bash
source /home/roser/workspaces/ros2_ws/install/setup.bash

BAG="${1:-/home/roser/Documents/bags/nora/rosbag2_2026_05_15-08_55_48}"
shift || true

pids=()
cleanup() { kill "${pids[@]}" 2>/dev/null || true; }
trap cleanup EXIT INT TERM

# Cloud+IMU merger -> /merged_scan, /merged_imu (left-LiDAR frame; extrinsics
# are baked into merge_clouds.py, so no TF lookup is needed -- this replaces
# the old scans_merger C++ node, which also silently dropped the per-point
# timestamp field GLIM needs for deskewing).
python3 /home/roser/repos/glim/merge_clouds.py & pids+=($!)

sleep 2
echo "Playing bag: $BAG"
# Tip: add  --rate 0.5  to slow down, or  --start-offset N  to skip ahead.
ros2 bag play "$BAG" --clock "$@"
