#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/jazzy/setup.bash
source /ros_ws/install/setup.bash
export PYTHONPATH="/opt/drone_sim/artifacts/src${PYTHONPATH:+:${PYTHONPATH}}"

python3 /opt/drone_sim/foundation_observer.py &
observer_pid=$!

cleanup() {
    kill "$observer_pid" 2>/dev/null || true
    wait "$observer_pid" 2>/dev/null || true
}
trap cleanup EXIT

python3 /opt/drone_sim/foundation_node.py "$@"
wait "$observer_pid"
trap - EXIT
