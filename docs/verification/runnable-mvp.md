# Runnable MVP handoff

## What this delivers

The production profile integrates the original nested
`companion/comp2026` mission into the seven-service Docker Compose runtime:
orchestration, artifacts, companion, ArduPilot SITL, Gazebo, electromagnet,
and scorekeeper. ArduPilot flies the Iris through MAVLink while Gazebo owns
simulation time, vehicle physics, payload attachment/release, cameras, range,
and ground truth.

The automatic competition attempt completes the full physical mission:

1. Fly from home to the landing site, land, and disarm.
2. Fly to field 2 and release marker 2 for the `80/150` checkpoint.
3. Acquire and release marker 3 for `145/150`.
4. Acquire and release marker 4 for `150/150`.
5. Return home, land, and disarm before 600 simulated seconds.

Mission waits, timeouts, stability windows, freshness, and the deadline use
elapsed simulation time through the existing clock adapter. QGC command
handling and broader determinism/clock cleanup remain deferred.

## Run from a clean checkout

Requirements are Docker with Compose v2 and `uv`; no credentials are needed.
The nested `companion/comp2026` checkout must be present at that exact path.

```bash
uv sync
docker compose --profile phase3 build
uv run drone-sim start --config config/default-run.json
```

The command performs one automatic mission attempt, prints the run ID, and
exits only after evidence finalization and Compose teardown. Finalization may
take several wall minutes because it semantically scans the full MCAP bag.

```bash
uv run drone-sim status RUN_ID
uv run drone-sim collect-results RUN_ID
```

Outputs are under `runs/RUN_ID/`. Independent host inspection additionally
requires FFmpeg/ffprobe and the ROS Jazzy `rosbag2_py` environment used by the
artifacts runtime:

```bash
uv run python scripts/inspect_competition_run.py runs/RUN_ID
```

## Acceptance contract

- Terminal state is `COMPLETED` with reason `mission_complete`.
- The physical score is exactly `150/150` under `competition_v1`.
- Home landing/disarm occurs before the 600-second mission deadline.
- Public `/clock` covers exactly 0 through 600,000,000,000 ns.
- Each frame-metadata, range, and ground-truth stream in the MCAP contains
  exactly 12,000 samples on a contiguous 50 ms grid. Raw camera images are
  intentionally omitted from MCAP.
- Both MP4s are H.264/yuv420p at 640x480 and 20 FPS with exactly 12,000 frames.
- The MCAP contains the configured competition streams, physical payload and
  mission evidence, and score evidence.
- The manifest binds source revisions, dirty states, image digests,
  configuration, scores, and SHA-256 records for every inventoried artifact.
- The run-specific Compose project has no remaining containers or networks.

## Preserved accepted evidence

Run `8c47f8a1-7823-464b-98f3-894dfbc043ac` was executed on an ephemeral Vast
VM, retrieved to the control server, and accepted at `150/150`. The mission
returned home and disarmed at about 315.18 simulated seconds, with mission
completion at about 315.24 seconds and the required evidence tail through
exactly 600 seconds. Its manifest SHA-256 is
`423820264c6a4d587d9f44f614c241842cec53fa763421b120aa3e778e3857bc`.

See [`comp2026-mvp.md`](comp2026-mvp.md) for immutable evidence paths, source
revisions, the scoring checksum, and checkpoint summary. Run directories are
ignored evidence and do not travel with a Git clone; transfer this preserved
directory separately when the recipient needs the original videos and bag.

## Deferred work

- QGC command handling.
- Broader deterministic scheduling and clock cleanup.
- Hardening and optimization beyond evidence-backed MVP corrections.
