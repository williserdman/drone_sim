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

COPY artifacts/src /opt/drone_sim/artifacts/src
COPY tests/phase2/module_stub.py /opt/drone_sim/phase2/module_stub.py
COPY tests/phase2/synthetic_gazebo.py /opt/drone_sim/phase2/synthetic_gazebo.py
COPY tests/phase2/synthetic_electromagnet.py /opt/drone_sim/phase2/synthetic_electromagnet.py
COPY tests/phase2/synthetic_scorekeeper.py /opt/drone_sim/phase2/synthetic_scorekeeper.py
COPY tests/phase2/scoring.json /opt/drone_sim/phase2/scoring.json
COPY tests/phase2/entrypoint.sh /opt/drone_sim/entrypoint.sh
RUN chmod 0755 /opt/drone_sim/entrypoint.sh

ENV AMENT_PREFIX_PATH="/ros_ws/install/simulation_interfaces:/opt/ros/jazzy"
ENV CMAKE_PREFIX_PATH="/ros_ws/install/simulation_interfaces:/opt/ros/jazzy"
ENV LD_LIBRARY_PATH="/ros_ws/install/simulation_interfaces/lib:/opt/ros/jazzy/lib"
ENV PYTHONPATH="/opt/drone_sim/phase2:/opt/drone_sim/artifacts/src:/ros_ws/install/simulation_interfaces/lib/python3.12/site-packages"

ENTRYPOINT ["/opt/drone_sim/entrypoint.sh"]

FROM runtime AS test

RUN apt-get update \
    && apt-get install --yes --no-install-recommends python3-pytest \
    && rm -rf /var/lib/apt/lists/*

CMD ["python3", "-m", "pytest", "tests/phase2"]
