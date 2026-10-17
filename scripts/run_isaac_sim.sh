#!/usr/bin/env bash
# Run the G1 Isaac Sim node (g1_factory/isaac/isaac_sim_node.py) in the Isaac Sim 5.1 container.
#
#   bash scripts/run_isaac_sim.sh --smoke-test 8                 # bring-up check, no ROS
#   bash scripts/run_isaac_sim.sh --spawn -12 3.8 0              # publish /g1_sim topics for the VLN stack
#   bash scripts/run_isaac_sim.sh --gui --spawn -12 3.8 0        # same, with the Isaac Sim window (follow camera)
#   ISAAC_DETACH=1 ISAAC_NAME=g1_isaac bash scripts/run_isaac_sim.sh --spawn ...   # background (runner)
#
# Needs: NVIDIA driver + nvidia-container-toolkit, `docker pull nvcr.io/nvidia/isaac-sim:5.1.0`,
# outputs/isaac/campus.usd from scripts/export_campus_usd.py, and the pinned G1 assets in external/.
# The repository is mounted read-only at /ws. ROS 2 uses Isaac's bundled Humble libraries with Fast DDS
# (configs/fastdds.xml); run the VLN stack with RMW_IMPLEMENTATION=rmw_fastrtps_cpp and the same ROS_DOMAIN_ID.
set -euo pipefail
cd "$(dirname "$0")/.."
IMAGE="${ISAAC_IMAGE:-nvcr.io/nvidia/isaac-sim:5.1.0}"
CACHE="${ISAAC_CACHE:-$HOME/docker/isaac-sim}"
# Isaac Sim 5.x containers run as uid 1234: its cache folders must belong to that user (NVIDIA container guide).
if [[ ! -d "$CACHE/cache/main" || "$(stat -c %u "$CACHE/cache/main" 2>/dev/null)" != "1234" ]]; then
  echo "Create the Isaac Sim cache owned by the container user (once):" >&2
  echo "  mkdir -p $CACHE/{cache/main/ov,cache/main/warp,cache/computecache,config,data/documents,data/Kit,logs,pkg}" >&2
  echo "  sudo chown -R 1234:1234 $CACHE" >&2
  exit 1
fi
[[ -f outputs/isaac/campus.usd ]] || { echo "Missing outputs/isaac/campus.usd: run scripts/export_campus_usd.py first" >&2; exit 1; }
[[ -f external/unitree_rl_gym/resources/robots/g1_description/g1_12dof.urdf ]] || { echo "Missing G1 URDF: run scripts/fetch_assets.py" >&2; exit 1; }
# The container user (uid 1234) reads the repository: make our files world-readable (assets fetched
# by older versions of fetch_assets.py were owner-only).
chmod -R a+rX external outputs/isaac configs g1_factory 2>/dev/null || true
RUN=(docker run --rm --gpus all --network host --ipc host
  -e ACCEPT_EULA=Y -e PRIVACY_CONSENT="${PRIVACY_CONSENT:-N}"
  -e ROS_DISTRO=humble -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp
  -e LD_LIBRARY_PATH=/isaac-sim/exts/isaacsim.ros2.bridge/humble/lib
  -e FASTRTPS_DEFAULT_PROFILES_FILE=/ws/configs/fastdds.xml
  -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
  -v "$CACHE/cache/main:/isaac-sim/.cache:rw"
  -v "$CACHE/cache/computecache:/isaac-sim/.nv/ComputeCache:rw"
  -v "$CACHE/logs:/isaac-sim/.nvidia-omniverse/logs:rw"
  -v "$CACHE/config:/isaac-sim/.nvidia-omniverse/config:rw"
  -v "$CACHE/data:/isaac-sim/.local/share/ov/data:rw"
  -v "$CACHE/pkg:/isaac-sim/.local/share/ov/pkg:rw"
  -v "$PWD:/ws:ro" -e PYTHONDONTWRITEBYTECODE=1 -u 1234:1234
  --entrypoint /isaac-sim/python.sh)
if [[ " $* " == *" --gui "* ]]; then
  # Isaac Sim window on this desktop (X11): allow local containers to use the display once per login.
  [[ -n "${DISPLAY:-}" ]] || { echo "--gui needs a desktop session (DISPLAY is empty)" >&2; exit 1; }
  xhost +local: >/dev/null
  RUN+=(-e DISPLAY="$DISPLAY" -v /tmp/.X11-unix:/tmp/.X11-unix:ro)
fi
if [[ "${ISAAC_DETACH:-0}" == "1" ]]; then
  RUN+=(-d --name "${ISAAC_NAME:-g1_isaac}")
fi
exec "${RUN[@]}" "$IMAGE" /ws/g1_factory/isaac/isaac_sim_node.py \
  --campus /ws/outputs/isaac/campus.usd --meta /ws/outputs/isaac/campus_meta.json \
  --upstream /ws/external/unitree_rl_gym "$@"
