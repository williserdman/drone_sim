FROM ros:jazzy-ros-base

RUN apt-get update \
    && apt-get install --yes --no-install-recommends python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ros_ws
COPY ros_ws/src/simulation_interfaces src/simulation_interfaces
RUN . /opt/ros/jazzy/setup.sh \
    && colcon build --packages-select simulation_interfaces

COPY artifacts/src /opt/drone_sim/artifacts/src
COPY tests/foundation/foundation_node.py /opt/drone_sim/foundation_node.py
COPY tests/foundation/foundation_observer.py /opt/drone_sim/foundation_observer.py
COPY tests/foundation/entrypoint.sh /opt/drone_sim/entrypoint.sh
RUN chmod +x /opt/drone_sim/entrypoint.sh

ENTRYPOINT ["/opt/drone_sim/entrypoint.sh"]
