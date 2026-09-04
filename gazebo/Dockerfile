FROM ros:jazzy-ros-base@sha256:2589a8fba5257307857890173c069852c2abf913a0be7970f172478baecb09e4 AS runtime

ARG ROS_GZ_PACKAGE=ros-jazzy-ros-gz=1.0.22-1noble.20260616.074726
ARG ROS_GZ_BRIDGE_PACKAGE=ros-jazzy-ros-gz-bridge=1.0.22-1noble.20260615.142443
ARG ROS_GZ_IMAGE_PACKAGE=ros-jazzy-ros-gz-image=1.0.22-1noble.20260615.145009
ARG ROS_GZ_INTERFACES_PACKAGE=ros-jazzy-ros-gz-interfaces=1.0.22-1noble.20260615.112415
ARG ROS_GZ_SIM_PACKAGE=ros-jazzy-ros-gz-sim=1.0.22-1noble.20260615.173223
ARG GZ_SIM_VENDOR_PACKAGE=ros-jazzy-gz-sim-vendor=0.0.10-1noble.20260604.111001
ARG SDFORMAT_VENDOR_PACKAGE=ros-jazzy-sdformat-vendor=0.0.11-1noble.20260604.104102
ARG ZSTD_PACKAGE=zstd=1.5.5+dfsg2-2build1.1
ARG ARDUPILOT_GAZEBO_COMMIT=082a0fe231f6e63bc8d1598f1cba461d9e2ea7f5

COPY gazebo/harmonic-packages.lock /etc/drone_sim/harmonic-packages.lock
RUN dpkg-query -W -f='${binary:Package}=${Version}\n' \
        | sed 's/:amd64=/=/' \
        | sort > /tmp/packages-before-harmonic \
    && apt-get update \
    && xargs --arg-file=/etc/drone_sim/harmonic-packages.lock \
        apt-get install --yes --no-install-recommends \
    && for direct_package in \
        "$ROS_GZ_PACKAGE" \
        "$ROS_GZ_BRIDGE_PACKAGE" \
        "$ROS_GZ_IMAGE_PACKAGE" \
        "$ROS_GZ_INTERFACES_PACKAGE" \
        "$ROS_GZ_SIM_PACKAGE" \
        "$GZ_SIM_VENDOR_PACKAGE" \
        "$SDFORMAT_VENDOR_PACKAGE" \
        "$ZSTD_PACKAGE"; do \
        grep --fixed-strings --line-regexp "$direct_package" \
            /etc/drone_sim/harmonic-packages.lock; \
    done \
    && dpkg-query -W -f='${binary:Package}=${Version}\n' \
        | sed 's/:amd64=/=/' \
        | sort > /tmp/packages-after-harmonic \
    && comm -13 /tmp/packages-before-harmonic /tmp/packages-after-harmonic \
        | sort > /tmp/harmonic-installed-delta \
    && sort /etc/drone_sim/harmonic-packages.lock \
        > /tmp/harmonic-expected-delta \
    && diff --unified \
        /tmp/harmonic-expected-delta /tmp/harmonic-installed-delta \
    && rm -f \
        /tmp/packages-before-harmonic \
        /tmp/packages-after-harmonic \
        /tmp/harmonic-installed-delta \
        /tmp/harmonic-expected-delta \
    && rm -rf /var/lib/apt/lists/*

COPY gazebo/plugin /tmp/ardupilot_gazebo-build
RUN test "${ARDUPILOT_GAZEBO_COMMIT}" = "082a0fe231f6e63bc8d1598f1cba461d9e2ea7f5" \
    && git init /tmp/ardupilot_gazebo \
    && git -C /tmp/ardupilot_gazebo remote add origin https://github.com/ArduPilot/ardupilot_gazebo.git \
    && git -C /tmp/ardupilot_gazebo -c http.version=HTTP/1.1 \
        fetch --depth=1 origin "${ARDUPILOT_GAZEBO_COMMIT}" \
    && git -C /tmp/ardupilot_gazebo checkout --detach FETCH_HEAD \
    && test "$(git -C /tmp/ardupilot_gazebo rev-parse HEAD)" = "${ARDUPILOT_GAZEBO_COMMIT}" \
    && git -C /tmp/ardupilot_gazebo apply \
        /tmp/ardupilot_gazebo-build/0001-paused-initial-json.patch \
    && bash -lc 'source /opt/ros/jazzy/setup.bash \
        && export GZ_VERSION=harmonic \
        && cmake -S /tmp/ardupilot_gazebo-build \
            -B /tmp/ardupilot_gazebo/build \
            -DCMAKE_BUILD_TYPE=RelWithDebInfo \
            -DBUILD_TESTING=ON \
            -DARDUPILOT_GAZEBO_SOURCE=/tmp/ardupilot_gazebo \
        && cmake --build /tmp/ardupilot_gazebo/build --parallel 2 \
        && ctest --test-dir /tmp/ardupilot_gazebo/build \
            --output-on-failure' \
    && mkdir -p /opt/drone_sim/gazebo/plugins \
    && cp /tmp/ardupilot_gazebo/build/libArduPilotPlugin.so \
        /opt/drone_sim/gazebo/plugins/libArduPilotPlugin.so \
    && cp /tmp/ardupilot_gazebo/build/libcwru_payload_command_coordinator.so \
        /opt/drone_sim/gazebo/plugins/libcwru_payload_command_coordinator.so \
    && cp /tmp/ardupilot_gazebo/build/libdrone_sim_detachable_joint_system.so \
        /opt/drone_sim/gazebo/plugins/libdrone_sim_detachable_joint_system.so \
    && printf '%s\n' "${ARDUPILOT_GAZEBO_COMMIT}" \
        > /opt/drone_sim/gazebo/plugins/ardupilot_gazebo-revision.txt \
    && rm -rf /tmp/ardupilot_gazebo

COPY artifacts orchestration gazebo ros_ws/src/simulation_interfaces /opt/drone_sim/source/
# These cross-package runtime modules must invalidate independently when an
# isolated worktree changes shared configuration or paused-flight readiness.
COPY orchestration/src/orchestration/config.py /opt/drone_sim/source/src/orchestration/config.py
COPY gazebo/src/drone_sim_gazebo/ros_adapter/topics.py /opt/drone_sim/source/src/drone_sim_gazebo/ros_adapter/topics.py
COPY gazebo/src/drone_sim_gazebo/ros_adapter/node.py /opt/drone_sim/source/src/drone_sim_gazebo/ros_adapter/node.py
COPY gazebo/src/drone_sim_gazebo/runtime/runtime_node.py /opt/drone_sim/source/src/drone_sim_gazebo/runtime/runtime_node.py
RUN mkdir -p /opt/drone_sim/gazebo /etc/drone_sim \
    && cp -a /opt/drone_sim/source/resources /opt/drone_sim/gazebo/resources \
    && cp -a /opt/drone_sim/source/provenance /opt/drone_sim/gazebo/provenance \
    && cp /opt/drone_sim/source/config/bridge.yaml /etc/drone_sim/gazebo-bridge.yaml \
    && cp /opt/drone_sim/source/config/bridge-flight.yaml /etc/drone_sim/gazebo-bridge-flight.yaml \
    && cp /opt/drone_sim/source/config/bridge-competition.yaml /etc/drone_sim/gazebo-bridge-competition.yaml \
    && python3 -m venv --system-site-packages /opt/drone_sim/venv \
    && /opt/drone_sim/venv/bin/pip install \
        --no-cache-dir \
        --no-index \
        --no-build-isolation \
        --no-deps \
        /opt/drone_sim/source \
    && bash -lc 'source /opt/ros/jazzy/setup.bash \
    && cmake -S /opt/drone_sim/source \
        -B /opt/drone_sim/ros_ws/build/simulation_interfaces \
        -DCMAKE_INSTALL_PREFIX=/opt/drone_sim/ros_ws/install/simulation_interfaces \
    && cmake --build /opt/drone_sim/ros_ws/build/simulation_interfaces --parallel 2 \
    && cmake --install /opt/drone_sim/ros_ws/build/simulation_interfaces'

ENV VIRTUAL_ENV=/opt/drone_sim/venv \
    PATH="/opt/drone_sim/venv/bin:${PATH}" \
    GZ_SIM_RESOURCE_PATH=/opt/drone_sim/gazebo/resources \
    SDF_PATH=/opt/drone_sim/gazebo/resources/models \
    AMENT_PREFIX_PATH="/opt/drone_sim/ros_ws/install/simulation_interfaces:/opt/ros/jazzy" \
    CMAKE_PREFIX_PATH="/opt/drone_sim/ros_ws/install/simulation_interfaces:/opt/ros/jazzy" \
    LD_LIBRARY_PATH="/opt/drone_sim/ros_ws/install/simulation_interfaces/lib:/opt/ros/jazzy/lib" \
    PYTHONPATH="/opt/drone_sim/ros_ws/install/simulation_interfaces/lib/python3.12/site-packages"

LABEL org.opencontainers.image.ardupilot_gazebo.revision="082a0fe231f6e63bc8d1598f1cba461d9e2ea7f5"

# Source ROS before replacing PID 1 with the caller's argv. Runtime code is
# added in Task 5; this stage already owns the immutable runtime supply chain.
ENTRYPOINT ["bash", "-lc", "source /opt/ros/jazzy/setup.bash && source /opt/drone_sim/ros_ws/install/simulation_interfaces/share/simulation_interfaces/local_setup.bash && export PYTHONPATH=/opt/drone_sim/ros_ws/install/simulation_interfaces/lib/python3.12/site-packages:${PYTHONPATH} && export LD_LIBRARY_PATH=/opt/drone_sim/ros_ws/install/simulation_interfaces/lib:${LD_LIBRARY_PATH} && exec \"$@\"", "--"]

FROM runtime AS test

COPY gazebo/tests/test_gazebo_image /opt/drone_sim/image-tests/test_gazebo_image
CMD ["/opt/drone_sim/image-tests/test_gazebo_image"]
