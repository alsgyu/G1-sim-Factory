#!/bin/bash
set -e
source /opt/ros/humble/setup.bash
[[ -f /opt/g1_ws/install/setup.bash ]] && source /opt/g1_ws/install/setup.bash
exec "$@"
