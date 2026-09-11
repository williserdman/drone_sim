FROM scratch AS context_contract

# Exercise every repository path currently consumed by a root-context
# Dockerfile. Docker itself applies the repository's .dockerignore before
# resolving these sources.
COPY pyproject.toml uv.lock conftest.py /context/
COPY config /context/config
COPY artifacts /context/artifacts
COPY orchestration /context/orchestration
COPY gazebo /context/gazebo
COPY ros_ws/src/simulation_interfaces /context/ros_ws/src/simulation_interfaces
COPY companion/pyproject.toml /context/companion/pyproject.toml
COPY companion/src /context/companion/src
COPY companion/comp2026/src/drone/timebase.py /context/companion/comp2026/src/drone/timebase.py
COPY companion/comp2026/src/gc/prepare_attempt.py /context/companion/comp2026/src/gc/prepare_attempt.py
COPY companion/comp2026/src/gc/custom_missions.json /context/companion/comp2026/src/gc/custom_missions.json
COPY companion/comp2026/src/drone/control /context/companion/comp2026/src/drone/control
COPY companion/comp2026/src/drone/missions /context/companion/comp2026/src/drone/missions
COPY companion/comp2026/src/drone/sensors/camera /context/companion/comp2026/src/drone/sensors/camera
COPY companion/comp2026/src/drone/sensors/lidar/__init__.py /context/companion/comp2026/src/drone/sensors/lidar/__init__.py
COPY companion/comp2026/src/drone/sensors/lidar/lidar.py /context/companion/comp2026/src/drone/sensors/lidar/lidar.py
COPY companion/comp2026/src/drone/sensors/lidar/clearance.py /context/companion/comp2026/src/drone/sensors/lidar/clearance.py
COPY companion/comp2026/src/drone/sensors/servo /context/companion/comp2026/src/drone/sensors/servo
COPY ardupilot_sitl/src /context/ardupilot_sitl/src
COPY ardupilot_sitl/params /context/ardupilot_sitl/params
COPY ardupilot_sitl/provenance /context/ardupilot_sitl/provenance
COPY electromagnet/pyproject.toml /context/electromagnet/pyproject.toml
COPY electromagnet/src /context/electromagnet/src
COPY scorekeeper/pyproject.toml /context/scorekeeper/pyproject.toml
COPY scorekeeper/src /context/scorekeeper/src
COPY scorekeeper/rules /context/scorekeeper/rules
COPY scorekeeper/bin /context/scorekeeper/bin
COPY tests/phase2 /context/tests/phase2
COPY tests/foundation /context/tests/foundation
