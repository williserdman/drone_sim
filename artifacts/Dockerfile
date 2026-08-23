FROM ros:jazzy-ros-base@sha256:2589a8fba5257307857890173c069852c2abf913a0be7970f172478baecb09e4 AS runtime

RUN apt-get update \
    && apt-get install --yes --no-install-recommends \
        python3-venv=3.12.3-0ubuntu2.1 \
        python3.12-venv=3.12.3-1ubuntu0.15 \
        python3-pip-whl=24.0+dfsg-1ubuntu1.3 \
        python3-setuptools-whl=68.1.2-2ubuntu1.2 \
        python3-wheel=0.42.0-2 \
        ros-jazzy-rosbag2=0.26.11-1noble.20260616.084050 \
        ros-jazzy-rosbag2-storage-mcap=0.26.11-1noble.20260616.074830 \
    && rm -rf /var/lib/apt/lists/*

COPY artifacts/ffmpeg-packages.lock /etc/drone_sim/ffmpeg-packages.lock
RUN dpkg-query -W -f='${binary:Package}=${Version}\n' \
        | sed 's/:amd64=/=/' \
        | sort > /tmp/packages-before-ffmpeg \
    && apt-get update \
    && xargs --arg-file=/etc/drone_sim/ffmpeg-packages.lock \
        apt-get install --yes --no-install-recommends \
    && dpkg-query -W -f='${binary:Package}=${Version}\n' \
        | sed 's/:amd64=/=/' \
        | sort > /tmp/packages-after-ffmpeg \
    && comm -13 /tmp/packages-before-ffmpeg /tmp/packages-after-ffmpeg \
        | sort > /tmp/ffmpeg-installed-delta \
    && sort /etc/drone_sim/ffmpeg-packages.lock > /tmp/ffmpeg-expected-delta \
    && diff --unified /tmp/ffmpeg-expected-delta /tmp/ffmpeg-installed-delta \
    && ffmpeg -hide_banner -encoders 2>/dev/null \
        | awk '$2 == "libx264" { found = 1 } END { exit !found }' \
    && ffprobe -version >/dev/null \
    && rm -f /tmp/packages-before-ffmpeg /tmp/packages-after-ffmpeg \
        /tmp/ffmpeg-installed-delta /tmp/ffmpeg-expected-delta \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ros_ws
COPY ros_ws/src/simulation_interfaces src/simulation_interfaces
RUN . /opt/ros/jazzy/setup.sh \
    && colcon build --packages-select simulation_interfaces

COPY config/recording-qos.yaml /etc/drone_sim/recording-qos.yaml
COPY artifacts/pyproject.toml /opt/drone_sim/artifacts/pyproject.toml
COPY artifacts/src /opt/drone_sim/artifacts/src
RUN python3 -m venv --system-site-packages /opt/drone_sim/venv \
    && /opt/drone_sim/venv/bin/pip install \
        --no-cache-dir \
        --no-build-isolation \
        /opt/drone_sim/artifacts

ENV VIRTUAL_ENV=/opt/drone_sim/venv
ENV PATH="/opt/drone_sim/venv/bin:${PATH}"
ENV AMENT_PREFIX_PATH="/ros_ws/install/simulation_interfaces:/opt/ros/jazzy"
ENV CMAKE_PREFIX_PATH="/ros_ws/install/simulation_interfaces:/opt/ros/jazzy"
ENV LD_LIBRARY_PATH="/ros_ws/install/simulation_interfaces/lib:/opt/ros/jazzy/lib"
ENV PYTHONPATH="/ros_ws/install/simulation_interfaces/lib/python3.12/site-packages"

FROM runtime AS test

ENV UV_PROJECT_ENVIRONMENT=/opt/drone_sim/venv
ENV DRONE_SIM_REQUIRE_ROS_TESTS=1
ENV DRONE_SIM_APT_PACKAGE_LOCK="python3-venv=3.12.3-0ubuntu2.1,python3.12-venv=3.12.3-1ubuntu0.15,python3-pip-whl=24.0+dfsg-1ubuntu1.3,python3-setuptools-whl=68.1.2-2ubuntu1.2,python3-wheel=0.42.0-2,ros-jazzy-rosbag2=0.26.11-1noble.20260616.084050,ros-jazzy-rosbag2-storage-mcap=0.26.11-1noble.20260616.074830"
ENV DRONE_SIM_FFMPEG_APT_PACKAGE_LOCK=/etc/drone_sim/ffmpeg-packages.lock

RUN /opt/drone_sim/venv/bin/pip install \
    --no-cache-dir \
    "uv==0.12.1"

WORKDIR /workspace
COPY pyproject.toml uv.lock ./
RUN uv sync --frozen --inexact --no-install-project
COPY artifacts/tests artifacts/tests
COPY artifacts/schemas artifacts/schemas

CMD ["uv", "run", "pytest", "artifacts/tests", "-v"]
