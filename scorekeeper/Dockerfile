FROM ros:jazzy-ros-base@sha256:2589a8fba5257307857890173c069852c2abf913a0be7970f172478baecb09e4 AS runtime

COPY ros_ws/src/simulation_interfaces /opt/drone_sim/source/simulation_interfaces

RUN bash -lc 'source /opt/ros/jazzy/setup.bash \
    && cmake -S /opt/drone_sim/source/simulation_interfaces \
        -B /opt/drone_sim/ros_ws/build/simulation_interfaces \
        -DCMAKE_INSTALL_PREFIX=/opt/drone_sim/ros_ws/install/simulation_interfaces \
    && cmake --build /opt/drone_sim/ros_ws/build/simulation_interfaces --parallel 2 \
    && cmake --install /opt/drone_sim/ros_ws/build/simulation_interfaces'

COPY artifacts/pyproject.toml /opt/drone_sim/source/artifacts/pyproject.toml
COPY artifacts/src /opt/drone_sim/source/artifacts/src
COPY scorekeeper/pyproject.toml /opt/drone_sim/source/scorekeeper/pyproject.toml
COPY scorekeeper/src /opt/drone_sim/source/scorekeeper/src
COPY scorekeeper/rules /opt/drone_sim/scorekeeper/rules
COPY scorekeeper/bin/drone-sim-scorekeeper-runtime /usr/local/bin/drone-sim-scorekeeper-runtime

RUN chmod 0755 /usr/local/bin/drone-sim-scorekeeper-runtime

ENV SIM_SCORE_RULES_PATH=/opt/drone_sim/scorekeeper/rules \
    AMENT_PREFIX_PATH="/opt/drone_sim/ros_ws/install/simulation_interfaces:/opt/ros/jazzy" \
    CMAKE_PREFIX_PATH="/opt/drone_sim/ros_ws/install/simulation_interfaces:/opt/ros/jazzy" \
    LD_LIBRARY_PATH="/opt/drone_sim/ros_ws/install/simulation_interfaces/lib:/opt/ros/jazzy/lib" \
    PYTHONPATH="/opt/drone_sim/source/scorekeeper/src:/opt/drone_sim/source/artifacts/src:/opt/drone_sim/ros_ws/install/simulation_interfaces/lib/python3.12/site-packages"

ENTRYPOINT ["bash", "-lc", "source /opt/ros/jazzy/setup.bash && source /opt/drone_sim/ros_ws/install/simulation_interfaces/share/simulation_interfaces/local_setup.bash && exec \"$@\"", "--"]
CMD ["drone-sim-scorekeeper-runtime"]

FROM runtime AS test

CMD ["python3", "-m", "drone_sim_scorekeeper.self_test"]
