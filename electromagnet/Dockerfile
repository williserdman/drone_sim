FROM ros:jazzy-ros-base@sha256:2589a8fba5257307857890173c069852c2abf913a0be7970f172478baecb09e4 AS runtime

RUN apt-get update \
    && apt-get install --yes --no-install-recommends \
        python3-venv=3.12.3-0ubuntu2.1 \
        python3.12-venv=3.12.3-1ubuntu0.15 \
        python3-pip-whl=24.0+dfsg-1ubuntu1.3 \
        python3-setuptools-whl=68.1.2-2ubuntu1.2 \
        python3-wheel=0.42.0-2 \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ros_ws
COPY ros_ws/src/simulation_interfaces src/simulation_interfaces
RUN . /opt/ros/jazzy/setup.sh \
    && colcon build --packages-select simulation_interfaces

COPY artifacts/pyproject.toml /opt/drone_sim/artifacts/pyproject.toml
COPY artifacts/src /opt/drone_sim/artifacts/src
COPY electromagnet/pyproject.toml /opt/drone_sim/electromagnet/pyproject.toml
COPY electromagnet/src /opt/drone_sim/electromagnet/src
RUN python3 -m venv --system-site-packages /opt/drone_sim/venv \
    && /opt/drone_sim/venv/bin/pip install \
        --no-cache-dir --no-index --no-build-isolation --no-deps \
        /opt/drone_sim/artifacts \
    && /opt/drone_sim/venv/bin/pip install \
        --no-cache-dir --no-index --no-build-isolation --no-deps \
        /opt/drone_sim/electromagnet

ENV VIRTUAL_ENV=/opt/drone_sim/venv \
    PATH="/opt/drone_sim/venv/bin:${PATH}" \
    AMENT_PREFIX_PATH="/ros_ws/install/simulation_interfaces:/opt/ros/jazzy" \
    CMAKE_PREFIX_PATH="/ros_ws/install/simulation_interfaces:/opt/ros/jazzy" \
    LD_LIBRARY_PATH="/ros_ws/install/simulation_interfaces/lib:/opt/ros/jazzy/lib" \
    PYTHONPATH="/ros_ws/install/simulation_interfaces/lib/python3.12/site-packages"

ENTRYPOINT ["bash", "-lc", "source /opt/ros/jazzy/setup.bash && source /ros_ws/install/setup.bash && exec \"$@\"", "--"]
CMD ["drone-sim-electromagnet-runtime"]
