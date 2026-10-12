FROM ardupilot/ardupilot-dev-base@sha256:576cd622957308469d6e72528befe5de5862457b264e61d60aaa4b8f29de85b6 AS source

ARG ARDUPILOT_COMMIT=1511f27194f1dcc3728270883047bdf022b3fd53
COPY ardupilot_sitl/patches/0001-sitl-json-rangefinder-bitmasks.patch /tmp/0001-sitl-json-rangefinder-bitmasks.patch
COPY ardupilot_sitl/tests/check_sim_json_rangefinders.py /tmp/check_sim_json_rangefinders.py
RUN test "${ARDUPILOT_COMMIT}" = "1511f27194f1dcc3728270883047bdf022b3fd53" \
    && git init /opt/ardupilot-src \
    && cd /opt/ardupilot-src \
    && git remote add origin https://github.com/ArduPilot/ardupilot.git \
    && git fetch --depth=1 origin "${ARDUPILOT_COMMIT}" \
    && git checkout --detach FETCH_HEAD \
    && test "$(git rev-parse HEAD)" = "${ARDUPILOT_COMMIT}" \
    && git apply /tmp/0001-sitl-json-rangefinder-bitmasks.patch \
    && python3 /tmp/check_sim_json_rangefinders.py --source-root /opt/ardupilot-src \
    && git submodule update --init --recursive \
    && git submodule status --recursive > /opt/ardupilot-submodules.txt \
    && printf '%s\n' "${ARDUPILOT_COMMIT}" > /opt/ardupilot-revision.txt

FROM source AS build
WORKDIR /opt/ardupilot-src
RUN ./waf configure --board sitl \
    && ./waf copter -j8 \
    && test -x build/sitl/bin/arducopter

FROM ardupilot/ardupilot-dev-base@sha256:576cd622957308469d6e72528befe5de5862457b264e61d60aaa4b8f29de85b6 AS runtime
COPY --from=build /opt/ardupilot-src/build/sitl/bin/arducopter /opt/ardupilot/bin/arducopter
COPY --from=source /opt/ardupilot-src/COPYING.txt /opt/ardupilot/COPYING.txt
COPY --from=source /opt/ardupilot-revision.txt /opt/ardupilot/revision.txt
COPY --from=source /opt/ardupilot-submodules.txt /opt/ardupilot/submodules.txt
COPY --from=source /tmp/0001-sitl-json-rangefinder-bitmasks.patch /opt/ardupilot/patches/0001-sitl-json-rangefinder-bitmasks.patch
COPY ardupilot_sitl/src /opt/drone_sim/ardupilot_sitl/src
COPY artifacts/src /opt/drone_sim/artifacts/src
COPY ardupilot_sitl/params /opt/drone_sim/ardupilot/params
COPY ardupilot_sitl/provenance /opt/drone_sim/ardupilot/provenance

ENV PYTHONPATH=/opt/drone_sim/ardupilot_sitl/src:/opt/drone_sim/artifacts/src
LABEL org.opencontainers.image.source="https://github.com/ArduPilot/ardupilot.git" \
      org.opencontainers.image.revision="1511f27194f1dcc3728270883047bdf022b3fd53" \
      org.opencontainers.image.licenses="GPL-3.0-or-later"

ENTRYPOINT ["python3", "-m", "drone_sim_ardupilot.runtime_node"]
