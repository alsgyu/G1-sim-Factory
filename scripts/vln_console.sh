#!/usr/bin/env bash
# Type instructions yourself: the VLN stack runs in the background, the console runs here.
#
#   bash scripts/run_isaac_sim.sh --gui --spawn -12 3.8 0     # terminal 1: Isaac Sim + G1 (start pose)
#   bash scripts/vln_console.sh                               # terminal 2: type instructions
#   VLM_MODEL=qwen3-vl:4b-instruct bash scripts/vln_console.sh
#   VLN_MAP_FILE=maps/campus/map_world.yaml bash scripts/vln_console.sh   # on the robot's SLAM map
#
# Each instruction is saved under outputs/console/<time>_<words>/ (feedback, decision images, inspection
# image, route.png). The stack log: docker logs -f g1_vln_stack. Leaving the console stops the stack.
set -euo pipefail
cd "$(dirname "$0")/.."
NAME="${VLN_STACK_NAME:-g1_vln_stack}"
MODEL="${VLM_MODEL:-qwen3-vl:8b-instruct}"
docker rm -f "$NAME" >/dev/null 2>&1 || true
mkdir -p outputs/console
G1_DOCKER_ARGS="-d --name $NAME" bash scripts/g1_docker.sh ros2 launch launch/g1_hvln.launch.py \
  agent:=false monitor:=false vlm_model:="$MODEL" output_dir:=/ws/outputs/console/stack \
  ${VLN_MAP_FILE:+map_file:=$VLN_MAP_FILE} >/dev/null
trap 'docker rm -f "$NAME" >/dev/null 2>&1 || true' EXIT
echo "VLN stack started in the background (model $MODEL; log: docker logs -f $NAME)"
bash scripts/g1_docker.sh python3 scripts/vln_console.py
