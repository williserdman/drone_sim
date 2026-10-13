FROM drone-sim-gazebo-runtime:phase3 AS viewer

ARG TARGETARCH

COPY gazebo/local-viewer/viewer-packages.arm64.lock \
    /etc/drone_sim/viewer-packages.arm64.lock
COPY gazebo/local-viewer/viewer-packages.amd64.lock \
    /etc/drone_sim/viewer-packages.amd64.lock
RUN case "${TARGETARCH}" in \
        amd64|arm64) ;; \
        *) printf 'unsupported target architecture: %s\n' "${TARGETARCH}" >&2; exit 2 ;; \
    esac \
    && cp "/etc/drone_sim/viewer-packages.${TARGETARCH}.lock" \
        /etc/drone_sim/viewer-packages.lock \
    && dpkg-query -W -f='${binary:Package}=${Version}\n' \
        | sed -E 's/:(amd64|arm64)=/=/g' \
        | sort > /tmp/packages-before-viewer \
    && apt-get update \
    && xargs --arg-file=/etc/drone_sim/viewer-packages.lock \
        apt-get install --yes --no-install-recommends \
    && dpkg-query -W -f='${binary:Package}=${Version}\n' \
        | sed -E 's/:(amd64|arm64)=/=/g' \
        | sort > /tmp/packages-after-viewer \
    && comm -13 /tmp/packages-before-viewer /tmp/packages-after-viewer \
        | sort > /tmp/viewer-installed-delta \
    && sort /etc/drone_sim/viewer-packages.lock \
        > /tmp/viewer-expected-delta \
    && diff --unified \
        /tmp/viewer-expected-delta /tmp/viewer-installed-delta \
    && rm -f \
        /tmp/packages-before-viewer \
        /tmp/packages-after-viewer \
        /tmp/viewer-installed-delta \
        /tmp/viewer-expected-delta \
    && rm -rf /var/lib/apt/lists/*

COPY gazebo/local-viewer /tmp/drone-sim-local-viewer
RUN bash -lc 'source /opt/ros/jazzy/setup.bash \
    && cmake -S /tmp/drone-sim-local-viewer \
        -B /tmp/drone-sim-local-viewer/build \
        -DCMAKE_BUILD_TYPE=Release \
    && cmake --build /tmp/drone-sim-local-viewer/build --parallel 2' \
    && mkdir -p /opt/drone_sim/gazebo/viewer/plugins /etc/drone_sim \
    && cp /tmp/drone-sim-local-viewer/build/libReadOnlyInspector.so \
        /opt/drone_sim/gazebo/viewer/plugins/libReadOnlyInspector.so \
    && cp /tmp/drone-sim-local-viewer/gui.config \
        /etc/drone_sim/gazebo-viewer.config \
    && cp /tmp/drone-sim-local-viewer/entrypoint.sh \
        /usr/local/bin/drone-sim-gazebo-viewer \
    && chmod 0755 /usr/local/bin/drone-sim-gazebo-viewer \
    && rm -rf /tmp/drone-sim-local-viewer

COPY gazebo/local-viewer/operator.html /usr/share/novnc/operator.html

ENV DISPLAY=:99 \
    GZ_GUI_CONFIG_PATH=/etc/drone_sim/gazebo-viewer.config \
    GZ_GUI_PLUGIN_PATH=/opt/drone_sim/gazebo/viewer/plugins

EXPOSE 6080

ENTRYPOINT ["/usr/local/bin/drone-sim-gazebo-viewer"]
