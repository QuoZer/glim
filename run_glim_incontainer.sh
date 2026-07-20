#!/usr/bin/env bash
# All-in-one-container two-LiDAR GLIM run (bag play + merger + GLIM, single DDS).
# Usage:  args: [DURATION_s] [RATE] [START_OFFSET_s]
#   ./run_glim_incontainer.sh                 # full bag from the start
#   ./run_glim_incontainer.sh 40              # only first 40 s (quick test)
#   ./run_glim_incontainer.sh 0 0.5           # full bag at 0.5x speed
#   ./run_glim_incontainer.sh 0 1.0 24        # from the 23-28s still stop -> clean IMU init
#   ./run_glim_incontainer.sh 0 1.0 24 1      # LEFT-LiDAR-ONLY baseline (merge-warp diagnostic)
#   GLIM_MERGE_STRIDE=2 ./run_glim_incontainer.sh   # pre-merge decimation, keep every 2nd pt
#   GLIM_MERGE_VOXEL=0.1 ./run_glim_incontainer.sh  # pre-merge 10cm voxel dedup
# Ctrl-C makes GLIM save the map to ~/glim_out, then exits.
set -euo pipefail

DURATION="${1:-0}"          # seconds; 0 = to end of bag (relative to START)
RATE="${2:-1.0}"           # bag playback rate
START="${3:-0}"            # start-offset into the bag [s]; 0 = from beginning
LEFT_ONLY="${4:-0}"        # 1 = republish only the left sensor (single-LiDAR baseline)
GLIM_MERGE_STRIDE="${GLIM_MERGE_STRIDE:-1}"   # pre-merge decimation: keep every Nth point
GLIM_MERGE_VOXEL="${GLIM_MERGE_VOXEL:-0.0}"   # pre-merge voxel dedup size in meters (0 = off)
BAG=/home/roser/Documents/bags/nora/rosbag2_2026_05_15-08_55_48_mcap
CFG=/home/roser/repos/glim/config_merged
NODE=/home/roser/repos/glim/merge_clouds.py
OUT="$HOME/glim_out"
IMAGE=koide3/glim_ros2:jazzy_cuda13.1

mkdir -p "$OUT"
xhost +local:docker >/dev/null 2>&1 || true

DUR_ARG=""
[ "$DURATION" != "0" ] && DUR_ARG="--playback-duration $DURATION"
START_ARG=""
[ "$START" != "0" ] && START_ARG="--start-offset $START"

docker run --rm -it --gpus all \
  -e NVIDIA_DRIVER_CAPABILITIES=all \
  -e HOST_UID="$(id -u)" -e HOST_GID="$(id -g)" \
  -e GLIM_LEFT_ONLY="$LEFT_ONLY" \
  -e GLIM_MERGE_STRIDE="$GLIM_MERGE_STRIDE" \
  -e GLIM_MERGE_VOXEL="$GLIM_MERGE_VOXEL" \
  -e DISPLAY="$DISPLAY" -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v "$BAG":/data:ro \
  -v "$CFG":/glim_config \
  -v "$NODE":/node/merge_clouds.py:ro \
  -v "$OUT":/out \
  "$IMAGE" bash -lc '
    source /opt/ros/jazzy/setup.bash; source /root/ros2_ws/install/setup.bash
    python3 /node/merge_clouds.py >/tmp/merger.log 2>&1 &
    ros2 run glim_ros glim_rosnode --ros-args -p config_path:=/glim_config -p dump_path:=/out &
    GLIM=$!
    trap "kill -INT $GLIM 2>/dev/null" INT TERM
    sleep 5
    echo "=== playing bag (start '"$START_ARG"' '"$DUR_ARG"' rate '"$RATE"') ==="
    ros2 bag play /data --rate '"$RATE"' '"$START_ARG"' '"$DUR_ARG"' || true
    sleep 2
    echo "=== stopping GLIM (saving map to /out) ==="
    kill -INT $GLIM 2>/dev/null || true
    wait $GLIM 2>/dev/null || true
    # Hand the dump back to the host user (container runs as root otherwise).
    chown -R "$HOST_UID:$HOST_GID" /out 2>/dev/null || true
    chmod -R u+rwX /out 2>/dev/null || true
  '
echo "Done. Map saved to $OUT"
