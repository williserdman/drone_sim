#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/jazzy/setup.bash
source /ros_ws/install/setup.bash
export PYTHONPATH="/opt/drone_sim/artifacts/src${PYTHONPATH:+:${PYTHONPATH}}"

exec python3 /opt/drone_sim/foundation_node.py "$@"
