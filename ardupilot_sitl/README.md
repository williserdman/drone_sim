# ArduPilot SITL

[Project overview](../README.md) · [Architecture](../docs/architecture.md) ·
[Runbook](../docs/runbook.md)

This module runs the simulated flight controller: estimation, navigation, and
vehicle-control loops backed by ArduPilot Copter SITL. It connects companion
mission commands to the Gazebo sensor/actuator lockstep exchange.

It does **not** own mission or vision decisions, authoritative physics or
ground truth, scenario behavior, scoring, or aggregate lifecycle state. It is
an ArduPilot SITL wrapper, not a Pixhawk simulator.

## Entry points and owned files

- [pyproject.toml](pyproject.toml) exposes `drone-sim-ardupilot-runtime`.
- [runtime_node.py](src/drone_sim_ardupilot/runtime_node.py) is the production
  process wrapper and durable readiness/failure/quiescence boundary.
- [config.py](src/drone_sim_ardupilot/config.py) validates run inputs, resolves
  the Gazebo service once, and constructs the shell-free ArduCopter command.
- [runtime.py](src/drone_sim_ardupilot/runtime.py) supervises SITL, interprets
  readiness output, writes events, and inventories diagnostics.
- [state.py](src/drone_sim_ardupilot/state.py) contains the pure lifecycle model.
- [json_peer.py](src/drone_sim_ardupilot/json_peer.py) is a bounded test peer for
  the upstream UDP protocol; production does not use it.
- [descent.parm](params/descent.parm) is the image-baked parameter overlay, and
  [Dockerfile](Dockerfile) pins the upstream build and runtime layout.

## Interfaces

The companion connects over Compose-only MAVLink TCP at
`tcp://ardupilot-sitl:5760`. ArduPilot consumes flight commands and provides
telemetry, modes, state, acknowledgements, and `STATUSTEXT` diagnostics.

The upstream JSON backend exchanges servo outputs and simulated sensor/dynamics
data with `gazebo-runtime:9002` over UDP. Sensor replies retain
`no_time_sync=true` and `no_lockstep=false`; loss of the Gazebo exchange must
prevent free-running simulation progress. Broader ownership and ordering are in the
[architecture guide](../docs/architecture.md).

The module provides its own structured child output, readiness fact, DataFlash
logs, SITL storage, failure evidence, and quiescence marker. A bound MAVLink
listener is not mission readiness: orchestration also waits for the companion
to observe a real heartbeat and healthy prearm status.

### Launch origin

The runtime requires `SIM_LAUNCH_ORIGIN_JSON`. Its value must be a JSON object
with exactly these four fields:

- `latitude_deg`: a finite number in `[-90, 90]`
- `longitude_deg`: a finite number in `[-180, 180]`
- `amsl_m`: a finite number
- `heading_deg`: a finite number in `[0, 360)`

The runtime has no built-in home location. It rejects a missing or malformed
object, extra fields, booleans, non-finite numbers, and out-of-range values
before it resolves Gazebo DNS, creates the work directory, constructs the SITL
process, or emits startup status. Orchestration supplies the validated QGC
runtime-policy origin for QGC runs and the fixed diagnostic origin
`37.4003371,-122.0800351,0,0` for controlled descent, roll AutoTune, and roll
hover runs. Caller process environment cannot override either value.

## Constraints worth knowing

- The source build freezes ArduPilot `Copter-4.5.7` at commit
  `2a3dc4b7bf2507120f7378a7b2fde73185e0c325`, verifies the official version
  declaration, and builds only `waf copter`. Existing local image tags are not
  changed by this source update. Supply-chain details live in
  [ardupilot.json](provenance/ardupilot.json) and
  [LICENSE.ArduPilot.txt](provenance/LICENSE.ArduPilot.txt).
- The pinned JSON backend accepts numeric IPv4 addresses, so startup resolves
  the Compose service name once. Name resolution does not advance simulation.
- Keep `no_time_sync=true`, `no_lockstep=false`, and `--speedup 1`; wall
  deadlines bound unavailable infrastructure, never mission time.
- Copter 4.5.7's native TCP serial argument is `tcp:PORT`. The runtime keeps the
  configured port in that spelling and does not invoke a shell.
- The parameter overlay deliberately preserves normal prearm checks. Its small
  accelerometer calibration offsets come from upstream SITL defaults; do not
  replace them with force-arm behavior.
- Copter 4.5.7 uses `LAND_SPEED` in cm/s, so the overlay's `50` retains the
  0.50 m/s final descent target. `SR0_EXT_STAT=1` requests the channel 0 extended
  status stream at 1 Hz for passive readiness observation. Parameter edits
  require rebuilding the image because the overlay is copied at build time.
- Copter 4.7's `ATC_ACC_R_MAX=2547.76` in deg/s² becomes
  `ATC_ACCEL_R_MAX=254776` in Copter 4.5.7's centidegrees/s² units. This keeps
  the existing physical roll acceleration request. It exceeds both releases'
  published metadata ranges and is retained tuning evidence, not a
  flight-approved tune.
- `FS_THR_ENABLE=0` and `FLTMODE_CH=0` are retained legacy simulator diagnostics.
  They are not an aircraft/QGC failsafe or takeover profile and do not lift the
  `comp2026_auto` quarantine.
- ArduPilot's JSON resend message is a recoverable upstream retry diagnostic,
  not by itself peer-loss evidence.

## Focused tests

Run from the project root:

```bash
uv run --locked pytest ardupilot_sitl/tests -q
```

The suite exercises configuration, lifecycle, process supervision, output
recognition, and the bounded JSON test peer. It does not build ArduPilot or
prove a live Gazebo flight; follow the [runbook](../docs/runbook.md) for those
checks.

## Isolated QGC listener probe

[`tests/qgc_sitl`](../tests/qgc_sitl) is a separate Copter 4.5.7 transport test.
It uses a real DroneKit listener connection and a separate pymavlink injector on
one internal Docker network. The only admitted test action records an envelope
in the result directory. The probe does not run a mission, publish host ports,
join a production network, or verify QGroundControl or flight behavior.

Run its offline checks first:

```bash
uv run --locked pytest \
  ardupilot_sitl/tests/test_json_peer.py \
  tests/qgc_sitl/test_listener_sitl.py -q
```

The Docker build and run are opt-in. Choose a new lowercase project name for
each run, and preserve the immutable image IDs in the environment and result.
For the bounded telemetry diagnosis, keep the Copter image, commit, parameters,
and companion dependency image unchanged. Build only the listener probe under
the new `drone-sim-qgc-listener-probe:4.5.7-2a3dc4b7-diag1` tag. The retained
`qgc-sitl-20260906-r2` evidence used the earlier
`drone-sim-qgc-listener-probe:4.5.7-2a3dc4b7` image and remains historical; do
not relabel it as a diagnostic run.

These commands intentionally do not use `compose down` or a prune operation:

```bash
export QGC_SITL_PROJECT=qgc-sitl-20260906-a1
export QGC_SITL_RESULT_DIR="$PWD/runs/qgc_sitl/$QGC_SITL_PROJECT"
export QGC_SITL_UID="$(id -u)"
export QGC_SITL_GID="$(id -g)"
mkdir -p "$PWD/runs/qgc_sitl"
umask 077
mkdir -m 700 "$QGC_SITL_RESULT_DIR"
mkdir -m 700 "$QGC_SITL_RESULT_DIR/arducopter"
test -z "$(docker ps -aq \
  --filter label=com.docker.compose.project="$QGC_SITL_PROJECT")"
if docker network inspect "${QGC_SITL_PROJECT}_default" >/dev/null 2>&1; then
  echo "refusing existing test network" >&2
  exit 1
fi

export QGC_SITL_ARDUCOPTER_IMAGE_ID="sha256:26d603213a1271ab3248834f810a1fd5ec0768e013cc37315d0915ac95a8c7ae"
export QGC_SITL_DEPENDENCY_IMAGE_ID="sha256:907d481abadfb4a4f54c7a5ea4597ab9ae58a4b24a8c39a0efab2e550812c9fe"
if [ "$(docker image inspect --format '{{.Id}}' \
  "$QGC_SITL_ARDUCOPTER_IMAGE_ID")" != "$QGC_SITL_ARDUCOPTER_IMAGE_ID" ] \
  || [ "$(docker image inspect --format '{{.Id}}' \
  "$QGC_SITL_DEPENDENCY_IMAGE_ID")" != "$QGC_SITL_DEPENDENCY_IMAGE_ID" ] \
  || [ "$(docker image inspect --format '{{.Id}}' \
  drone-sim-qgc-listener-arducopter:4.5.7-2a3dc4b7)" \
  != "$QGC_SITL_ARDUCOPTER_IMAGE_ID" ]; then
  echo "refusing changed or unavailable retained images" >&2
  exit 1
fi
export QGC_SITL_COMPANION_IMAGE_REF="$QGC_SITL_DEPENDENCY_IMAGE_ID"

timeout 600 docker build \
  --build-arg COMPANION_BASE_IMAGE="$QGC_SITL_COMPANION_IMAGE_REF" \
  -f tests/qgc_sitl/Dockerfile.listener-probe \
  -t drone-sim-qgc-listener-probe:4.5.7-2a3dc4b7-diag1 .

export QGC_SITL_PROBE_IMAGE_ID="$(docker image inspect \
  --format '{{.Id}}' drone-sim-qgc-listener-probe:4.5.7-2a3dc4b7-diag1)"

set +e
python3 tests/qgc_sitl/host_runner.py \
  "$QGC_SITL_PROJECT" "$QGC_SITL_RESULT_DIR"
QGC_SITL_HOST_EXIT=$?
set -e

QGC_SITL_RUN_INTEGRATION=1 \
QGC_SITL_RESULT_PATH="$QGC_SITL_RESULT_DIR/result.json" \
  uv run --locked pytest tests/qgc_sitl/test_listener_sitl.py \
  -m qgc_sitl_integration -q
test "$QGC_SITL_HOST_EXIT" -eq 0
```

Compose runs both services as the numeric result owner supplied in
`QGC_SITL_UID:QGC_SITL_GID`; it does not grant either service extra write
permissions.
`host_runner.py` rejects missing or non-numeric IDs and result mounts that are
not private directories owned by those IDs before calling Docker. It then
installs SIGINT and SIGTERM handling and its status record before `compose up`.
Every Docker call has a wall timeout. Its cleanup path ignores repeated
termination while it attempts log capture, service-status capture, removal of
the two named services, and removal of only the named test network. Each output
write and status write is independent. Any command or capture failure makes the
host runner exit nonzero, while later cleanup still runs.

The result records raw `AUTOPILOT_VERSION` fields, packet and ACK identities,
handler count, source byte hashes including the mounted launcher, exact
ArduCopter arguments, and in-process cleanup. Missing native routing, telemetry,
metadata, or cleanup stays a failed result. The constant JSON sensor response
is only a lockstep transport aid. Its clock accumulates the period from every
accepted positive servo header rate; retransmitted frames reuse the prior JSON
timestamp and do not advance the application clock.

The diagnostic result's `passive_telemetry_receipts` records source-filtered
arrival counts and up to four recent accepted-frame-clock timestamps per
requested message ID around startup collection. These receipts do not prove
collector cadence, freshness, distinctness, firmware metadata, or valid payload
fields. Preserve a telemetry-validation failure as diagnostic evidence. The
probe admits no listener commands after collector failure. No actual `diag1`
image build or run has occurred yet.
