#!/usr/bin/env bash
# Native Ubuntu setup for the ROS 2 VLN stack (alternative to docker/Dockerfile.ros2).
#   Ubuntu 22.04 + ROS 2 Humble (Python 3.10) or Ubuntu 24.04 + ROS 2 Jazzy (Python 3.12).
# Install ROS 2 first (docs.ros.org), then:  bash scripts/setup_ros2_native.sh
# Creates .venv-ros with access to the system rclpy, installs the simulator
# dependencies and the pinned G1 assets. Activate with:
#   source /opt/ros/$ROS_DISTRO/setup.bash && source .venv-ros/bin/activate
set -euo pipefail
cd "$(dirname "$0")/.."
DISTRO="${ROS_DISTRO:-}"
if [[ -z "$DISTRO" ]]; then
  for candidate in jazzy humble; do [[ -d /opt/ros/$candidate ]] && DISTRO=$candidate && break; done
fi
[[ -n "$DISTRO" && -f /opt/ros/$DISTRO/setup.bash ]] || { echo "Install ROS 2 Humble or Jazzy first." >&2; exit 1; }
sudo apt-get update
sudo apt-get install -y python3-venv python3-pip libegl1 libgl1 libosmesa6 fonts-dejavu-core \
  ros-$DISTRO-rmw-cyclonedds-cpp ros-$DISTRO-sensor-msgs-py ros-$DISTRO-tf2-ros-py
# rclpy lives in the system interpreter: the venv must use it and see its packages.
/usr/bin/python3 -m venv --system-site-packages .venv-ros
.venv-ros/bin/python -m pip install --upgrade pip
.venv-ros/bin/python -m pip install torch==2.7.1 --index-url https://download.pytorch.org/whl/cpu
.venv-ros/bin/python -m pip install "mujoco==3.5.0" "numpy>=1.26,<2" "PyYAML>=6.0,<7" "Pillow>=10,<13" "pytest==8.4.2"
.venv-ros/bin/python scripts/fetch_assets.py
set +u; source /opt/ros/$DISTRO/setup.bash; set -u
.venv-ros/bin/python -c "import rclpy, mujoco, torch; print('rclpy OK | mujoco', mujoco.__version__, '| torch', torch.__version__)"
echo "Ready. Every new shell:"
echo "  source /opt/ros/$DISTRO/setup.bash && source .venv-ros/bin/activate"
echo "  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp CYCLONEDDS_URI=file://$PWD/configs/cyclonedds.xml MUJOCO_GL=egl"
