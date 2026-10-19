#!/usr/bin/env bash
# Run a command in the g1-sim-ros2:humble image with the repository at /ws.
#
#   bash scripts/g1_docker.sh python3 scripts/fetch_assets.py
#   bash scripts/g1_docker.sh python3 scripts/check_hvln_env.py
#   bash scripts/g1_docker.sh ros2 launch launch/g1_hvln.launch.py task:=inspect_conveyor   # with Isaac Sim running
#
# * Runs as the calling user, so files written into the repository (external/, outputs/) stay yours.
# * Host network and IPC: ROS 2 reaches other containers (Isaac Sim) and the VLM server at 127.0.0.1.
# * GPU (--gpus all) when nvidia-smi exists on the host; G1_GPU=0 turns it off.
# * Fast DDS with configs/fastdds.xml (default): Isaac Sim's bundled ROS 2 uses Fast DDS, so the whole graph
#   does. G1_DDS=cyclonedds only for the MuJoCo development graph.
# * ROS_DOMAIN_ID is passed through (default 0).
set -euo pipefail
cd "$(dirname "$0")/.."
[[ $# -gt 0 ]] || { sed -n '2,13p' "$0"; exit 2; }
IMAGE="${G1_IMAGE:-g1-sim-ros2:humble}"
args=(docker run --rm --network host --ipc host -v "$PWD:/ws" --user "$(id -u):$(id -g)"
      -e HOME=/tmp -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}")
if [[ "${G1_GPU:-1}" == "1" ]] && command -v nvidia-smi >/dev/null 2>&1; then
  args+=(--gpus all)
fi
if [[ "${G1_DDS:-fastdds}" == "fastdds" ]]; then
  args+=(-e RMW_IMPLEMENTATION=rmw_fastrtps_cpp -e FASTRTPS_DEFAULT_PROFILES_FILE=/ws/configs/fastdds.xml)
fi
if [[ -t 0 && -t 1 ]]; then
  args+=(-it)
fi
exec "${args[@]}" "$IMAGE" "$@"
