#!/usr/bin/env bash
# Record the raw head camera, LiDAR, odometry, TF and clock topics of a running simulator into a rosbag2
# folder (default sqlite3 storage, no compression), then check it with scripts/check_bag.py.
#
#   bash scripts/record_bag.sh g1bag_conveyor        # Ctrl-C to stop -> outputs/bags/g1bag_conveyor/
#   bash scripts/record_bag.sh g1bag_conveyor 300    # or stop by itself after 300 wall-clock seconds
#
# * Start it in its own terminal after the simulator is up and before the VLN launch, stop it after the task.
# * Stopping with Ctrl-C (or the time limit) lets ros2 bag write metadata.yaml; never close the terminal instead.
# * About 51 MB per simulated second at 640x480 30 Hz RGB-D and 10 Hz LiDAR: 5 simulated minutes take about 15 GB.
# * Same container, network and DDS settings as scripts/g1_docker.sh (Fast DDS for Isaac Sim by default).
set -euo pipefail
cd "$(dirname "$0")/.."
[[ $# -ge 1 && $# -le 2 ]] || { sed -n '2,11p' "$0"; exit 2; }
name="$1"
seconds="${2:-}"
out="outputs/bags/$name"
[[ ! -e "$out" ]] || { echo "$out already exists; pick another name" >&2; exit 1; }
mkdir -p outputs/bags
free_gb=$(df -Pk outputs/bags | awk 'NR == 2 {print int($4/1048576)}')
if (( free_gb < 20 )); then
  echo "warning: only $free_gb GB free under outputs/bags; 5 simulated minutes take about 15 GB" >&2
fi
topics=(/camera/camera/color/image_raw /camera/camera/aligned_depth_to_color/image_raw /livox/lidar
        /camera/camera/color/camera_info /odom /tf /tf_static /clock)
limit=()
[[ -z "$seconds" ]] || limit=(timeout -s INT -k 60 "$seconds")
# Ctrl-C belongs to ros2 bag inside the container; this script keeps going to check the bag afterwards.
trap : INT
echo "recording ${#topics[@]} topics to $out (Ctrl-C to stop)"
bash scripts/g1_docker.sh ${limit[@]+"${limit[@]}"} ros2 bag record -o "$out" "${topics[@]}" || true
trap - INT
G1_GPU=0 bash scripts/g1_docker.sh python3 scripts/check_bag.py "$out"
