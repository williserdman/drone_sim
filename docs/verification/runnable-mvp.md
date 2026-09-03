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

The explicit pacing experiment uses the same mission and artifact contract but
asks Gazebo and SITL to target 1x real time:

```bash
uv run drone-sim start --config config/realtime-run.json
```

The target is a pacing ceiling, not an acceptance threshold or performance
guarantee. Mission behavior and the 600-second evidence horizon remain based on
simulation time when the host runs slower than the requested factor.

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
- Gazebo native state is stored as an integrity-checked
  `gazebo/state/state.tlog.zst`; consumers can stream-decompress it without
  expanding the artifact bundle in place.
- The manifest binds source revisions, dirty states, image digests,
  configuration, scores, and SHA-256 records for every inventoried artifact.
- The run-specific Compose project has no remaining containers or networks.

## Preserved accepted evidence

Run `3dc895c3-a5aa-4651-a893-d21884fa43f5` used the explicit 1x profile on an
ephemeral Vast VM, was retrieved to the control server, and was independently
accepted at `150/150`. The mission returned Home and disarmed at 511.122
simulated seconds, completed at 511.2 seconds, and retained evidence through
exactly 600 seconds. Its manifest SHA-256 is
`5b88d1ca377e85b12aabb083bc5f24c66c8724c41248cd1d7a7982b8ab519653`.
The full bundle is 373,552,227 bytes, including a 251,710,608-byte compressed
Gazebo state file. The public 600-second interval sustained about 0.272x real
time, demonstrating correctness at the requested 1x configuration without
claiming the host achieved that pace.

See [`comp2026-mvp.md`](comp2026-mvp.md) for immutable evidence paths, source
revisions, the scoring checksum, and checkpoint summary. Run directories are
ignored evidence and do not travel with a Git clone; transfer this preserved
directory separately when the recipient needs the original videos and bag.

## Deferred work

- QGC command handling.
- Broader deterministic scheduling and clock cleanup.
- Hardening and optimization beyond evidence-backed MVP corrections.
