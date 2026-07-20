#!/usr/bin/env bash
# Open a saved GLIM map dump in the offline viewer (Iridescence GUI).
# Usage:
#   ./view_glim_map.sh            # latest dump in ~/glim_out, lightweight pose-graph load
#   ./view_glim_map.sh /some/dump # a specific dump directory
#   FULL=1 ./view_glim_map.sh     # load with the ORIGINAL gpu config (rebuilds
#                                 #   VGICP factors -> heavy; this is what hangs)
#   DEBUG=1 ./view_glim_map.sh    # verbose load logging
#
# By default the viewer loads a POSE-GRAPH-patched copy of config_merged:
# libglobal_mapping_pose_graph.so just loads submap points + poses and skips the
# GPU matching-factor rebuild that stalls the offline viewer at 0%.
set -euo pipefail

DUMP="${1:-$HOME/glim_out}"
CFG=/home/roser/repos/glim/config_merged
IMAGE=koide3/glim_ros2:jazzy_cuda13.1
DEBUG_ARG=""; [ "${DEBUG:-0}" != "0" ] && DEBUG_ARG="--debug"

[ -f "$DUMP/graph.bin" ] || { echo "No graph.bin in $DUMP — not a GLIM dump."; exit 1; }
xhost +local:docker >/dev/null 2>&1 || true

# Build the config the viewer will use.
VIEWCFG="$(mktemp -d)"
trap 'rm -rf "$VIEWCFG"' EXIT
cp "$CFG"/*.json "$VIEWCFG"/
if [ "${FULL:-0}" = "0" ]; then
  sed -i 's#config_global_mapping_gpu.json#config_global_mapping_pose_graph.json#' "$VIEWCFG/config.json"
  echo "Viewer using LIGHTWEIGHT pose-graph global mapping (set FULL=1 for the gpu config)."
else
  echo "Viewer using the ORIGINAL gpu config (may stall rebuilding VGICP factors)."
fi

docker run --rm -it --gpus all \
  -e NVIDIA_DRIVER_CAPABILITIES=all \
  -e DISPLAY="$DISPLAY" -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v "$DUMP":/map \
  -v "$VIEWCFG":/glim_config:ro \
  "$IMAGE" bash -lc '
    source /opt/ros/jazzy/setup.bash; source /root/ros2_ws/install/setup.bash
    ros2 run glim_ros offline_viewer --map_path /map --config_path /glim_config '"$DEBUG_ARG"
