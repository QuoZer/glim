#!/usr/bin/env bash
# GLIM offline pass over the pre-merged two-LiDAR bag (no live merger needed).
# glim_rosbag reads the bag directly and auto-paces playback, then saves the
# map to ~/glim_out and exits (auto_quit). Ctrl-C also saves before exiting.
# Usage: ./run_glim_merged.sh [merged_bag_dir]
set -euo pipefail

BAG="${1:-/home/roser/Documents/bags/nora/nora_merged_150_750_mcap}"
[[ -f "$BAG/metadata.yaml" ]] || { echo "not a rosbag2 dir: $BAG" >&2; exit 1; }

xhost +local:docker >/dev/null 2>&1 || true
mkdir -p "$HOME/glim_out"

docker run --rm -it --gpus all --network host --ipc=host \
  -e DISPLAY="$DISPLAY" -v /tmp/.X11-unix:/tmp/.X11-unix \
  -e FASTRTPS_DEFAULT_PROFILES_FILE=/cfg/fastdds_udp.xml \
  -v /home/roser/repos/glim/fastdds_udp.xml:/cfg/fastdds_udp.xml:ro \
  -v /home/roser/repos/glim/config_merged:/glim_config \
  -v "$BAG":/bag:ro \
  -v "$HOME/glim_out":/out \
  koide3/glim_ros2:jazzy_cuda13.1 \
  bash -lc 'source /opt/ros/jazzy/setup.bash; source /root/ros2_ws/install/setup.bash; \
    ros2 run glim_ros glim_rosbag --ros-args \
      -p config_path:=/glim_config -p dump_path:=/out -p auto_quit:=true \
      -- /bag'
