#!/usr/bin/env bash
# Gather the SLAM map and the VLN runs into two folders for a presentation.
#
#   bash scripts/collect_results.sh ~/Workspace/G1-isaacsim-slam ~/Workspace/G1-result
#   MAP=maps/campus RUNS=outputs bash scripts/collect_results.sh <slam folder> <vln folder>
#
# Runs scripts/collect_results.py in the g1-sim-ros2 image with both folders mounted (they may be outside the
# repository). Files with the same names are overwritten; nothing else in the folders is deleted.
set -euo pipefail
cd "$(dirname "$0")/.."
[[ $# -eq 2 ]] || { sed -n '2,8p' "$0"; exit 2; }
mkdir -p "$1" "$2"
SLAM_OUT=$(cd "$1" && pwd)
VLN_OUT=$(cd "$2" && pwd)
G1_GPU=0 G1_DOCKER_ARGS="-v $SLAM_OUT:/collect/slam -v $VLN_OUT:/collect/vln" bash scripts/g1_docker.sh \
  python3 scripts/collect_results.py --map "${MAP:-maps/campus}" --runs "${RUNS:-outputs}" \
  --slam-out /collect/slam --vln-out /collect/vln
