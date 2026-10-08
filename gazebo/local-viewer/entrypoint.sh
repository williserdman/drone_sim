#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/jazzy/setup.bash
source \
    /opt/drone_sim/ros_ws/install/simulation_interfaces/share/simulation_interfaces/local_setup.bash
set -u

: "${SIM_RUN_ID:?SIM_RUN_ID is required}"
if [[ ! "${SIM_RUN_ID}" =~ ^[0-9a-f]{8}-[0-9a-f]{4}-4[0-9a-f]{3}-[89ab][0-9a-f]{3}-[0-9a-f]{12}$ ]]; then
    printf 'invalid SIM_RUN_ID: %s\n' "${SIM_RUN_ID}" >&2
    exit 2
fi

GZ_PARTITION="drone_sim_${SIM_RUN_ID//-/_}"
export GZ_PARTITION
export DISPLAY="${DISPLAY:-:99}"

children=()
stop_children() {
    local child
    for child in "${children[@]}"; do
        kill "${child}" 2>/dev/null || true
    done
    wait "${children[@]}" 2>/dev/null || true
}
trap stop_children EXIT INT TERM

Xvfb "${DISPLAY}" -screen 0 1600x900x24 -nolisten tcp &
children+=("$!")

for _attempt in $(seq 1 100); do
    if xdpyinfo -display "${DISPLAY}" >/dev/null 2>&1; then
        break
    fi
    sleep 0.1
done
xdpyinfo -display "${DISPLAY}" >/dev/null

dbus-run-session -- openbox &
children+=("$!")

world_seen=false
for _attempt in $(seq 1 120); do
    if gz topic -l 2>/dev/null | grep --quiet '^/world/'; then
        world_seen=true
        break
    fi
    sleep 1
done
if [[ "${world_seen}" != "true" ]]; then
    printf 'Gazebo server did not appear on partition %s\n' "${GZ_PARTITION}" >&2
    exit 1
fi

gz sim -g --force-version 8 --gui-config "${GZ_GUI_CONFIG_PATH}" &
children+=("$!")

window_seen=false
for _attempt in $(seq 1 120); do
    if xwininfo -display "${DISPLAY}" -root -tree 2>/dev/null \
        | grep --quiet '"Gazebo Sim"'; then
        window_seen=true
        break
    fi
    sleep 1
done
if [[ "${window_seen}" != "true" ]]; then
    printf 'Gazebo GUI did not become ready on partition %s\n' \
        "${GZ_PARTITION}" >&2
    exit 1
fi

x11vnc -display "${DISPLAY}" -localhost -rfbport 5900 -forever -shared -nopw &
children+=("$!")
websockify --web=/usr/share/novnc 6080 localhost:5900 &
children+=("$!")

wait -n "${children[@]}"

