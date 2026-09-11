FROM ardupilot/ardupilot-dev-base@sha256:576cd622957308469d6e72528befe5de5862457b264e61d60aaa4b8f29de85b6 AS source

ARG ARDUPILOT_COMMIT=2a3dc4b7bf2507120f7378a7b2fde73185e0c325
RUN test "${ARDUPILOT_COMMIT}" = "2a3dc4b7bf2507120f7378a7b2fde73185e0c325" \
    && git init /opt/ardupilot-src \
    && cd /opt/ardupilot-src \
    && git remote add origin https://github.com/ArduPilot/ardupilot.git \
    && git fetch --depth=1 origin "${ARDUPILOT_COMMIT}" \
    && git checkout --detach FETCH_HEAD \
    && test "$(git rev-parse HEAD)" = "${ARDUPILOT_COMMIT}" \
    && grep -Fqx '#define THISFIRMWARE "ArduCopter V4.5.7"' ArduCopter/version.h \
    && grep -Fqx '#define FIRMWARE_VERSION 4,5,7,FIRMWARE_VERSION_TYPE_OFFICIAL' ArduCopter/version.h \
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
COPY ardupilot_sitl/src /opt/drone_sim/ardupilot_sitl/src
COPY artifacts/src /opt/drone_sim/artifacts/src
COPY ardupilot_sitl/params /opt/drone_sim/ardupilot/params
COPY ardupilot_sitl/provenance /opt/drone_sim/ardupilot/provenance

ENV PYTHONPATH=/opt/drone_sim/ardupilot_sitl/src:/opt/drone_sim/artifacts/src
LABEL org.opencontainers.image.source="https://github.com/ArduPilot/ardupilot.git" \
      org.opencontainers.image.revision="2a3dc4b7bf2507120f7378a7b2fde73185e0c325" \
      org.opencontainers.image.licenses="GPL-3.0-or-later"

ENTRYPOINT ["python3", "-m", "drone_sim_ardupilot.runtime_node"]
