#!/usr/bin/env bash
# Run HBA (hku-mars) on a converted GLIM dump (pcd/ + pose.json).
# HBA OVERWRITES pose.json in place with optimized poses expressed RELATIVE TO
# scan 0, so we back up the GLIM input to pose_input.json first.
# Usage:
#   ./run_hba.sh                 # ~/glim_hba, 1 global BA layer, 8 threads
#   ./run_hba.sh ~/glim_hba 2 8  # DATA  LAYERS  THREADS
set -euo pipefail

DATA="${1:-$HOME/glim_hba}"
LAYERS="${2:-1}"
THREADS="${3:-8}"
IMAGE=hba:noetic

[ -f "$DATA/pose.json" ] || { echo "No $DATA/pose.json — run glim_to_hba.py first."; exit 1; }
cp -f "$DATA/pose.json" "$DATA/pose_input.json"
echo "Backed up initial poses -> $DATA/pose_input.json"

docker run --rm -it \
  -e HOST_UID="$(id -u)" -e HOST_GID="$(id -g)" \
  -v "$DATA":/data \
  "$IMAGE" bash -lc '
    source /opt/ros/noetic/setup.bash; source /catkin_ws/devel/setup.bash
    roscore >/tmp/roscore.log 2>&1 &
    sleep 3
    HBA_BIN=/catkin_ws/src/bin/hba
    [ -x "$HBA_BIN" ] || HBA_BIN="$(find /catkin_ws -type f -name hba -perm -111 | head -1)"
    echo "=== running $HBA_BIN (layers='"$LAYERS"' threads='"$THREADS"') ==="
    "$HBA_BIN" __name:=hba \
      _data_path:=/data/ \
      _total_layer_num:='"$LAYERS"' \
      _pcd_name_fill_num:=0 \
      _thread_num:='"$THREADS"'
    chown -R "$HOST_UID:$HOST_GID" /data 2>/dev/null || true
  '
echo "Done. Optimized poses -> $DATA/pose.json (relative to scan 0); input kept in pose_input.json"
