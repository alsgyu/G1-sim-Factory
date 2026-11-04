#!/usr/bin/env bash
# Run a command in G1-sim-Factory's g1-sim-ros2:humble image: G1-sim-Factory at /ws (where the image expects it),
# this repository at /pp (the working directory).
#
#   bash scripts/g1_docker.sh python3 -m pytest -q
#   bash scripts/g1_docker.sh ros2 launch launch/naive_pp.launch.py initial_pose:=0,0,0
#   bash scripts/g1_docker.sh python3 /ws/scripts/send_instruction.py "Go to the pump room"
#
# * G1-sim-Factory is the submodule (git submodule update --init), or the checkout G1_SIM_ROOT names.
# * Everything else is G1-sim-Factory/scripts/g1_docker.sh: G1_GPU, G1_DDS, ROS_DOMAIN_ID, G1_IMAGE, G1_DOCKER_ARGS.
set -euo pipefail
cd "$(dirname "$0")/.."
[[ $# -gt 0 ]] || { sed -n '2,10p' "$0"; exit 2; }
SIM="${G1_SIM_ROOT:-G1-sim-Factory}"
[[ -f "$SIM/scripts/g1_docker.sh" ]] || {
  echo "G1-sim-Factory not found at $SIM: git submodule update --init, or set G1_SIM_ROOT" >&2; exit 1; }
# G1-sim-Factory's script splits G1_DOCKER_ARGS at spaces.
[[ "$PWD" != *[[:space:]]* ]] || { echo "the path of this repository must not contain spaces: $PWD" >&2; exit 1; }
G1_DOCKER_ARGS="-v $PWD:/pp -w /pp -e G1_SIM_ROOT=/ws ${G1_DOCKER_ARGS:-}" exec bash "$SIM/scripts/g1_docker.sh" "$@"
