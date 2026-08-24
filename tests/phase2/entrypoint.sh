#!/usr/bin/env bash
set -Eeuo pipefail

source /opt/ros/jazzy/setup.bash
source /ros_ws/install/setup.bash

export PYTHONPATH="/opt/drone_sim/phase2:/opt/drone_sim/artifacts/src:/ros_ws/install/simulation_interfaces/lib/python3.12/site-packages${PYTHONPATH:+:${PYTHONPATH}}"

if (($# == 0)); then
    echo "phase2 image requires a command" >&2
    exit 64
fi

exec "$@"
