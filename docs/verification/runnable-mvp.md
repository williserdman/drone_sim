# Runnable MVP handoff

## What this delivers

The Phase 3 production profile starts exactly seven Docker Compose services:
orchestration, artifacts, companion, ArduPilot SITL, Gazebo, electromagnet, and
scorekeeper. ArduPilot controls an Iris in Gazebo through normal MAVLink
`GUIDED -> arm -> takeoff to 1.5 m -> LAND -> disarm`. Gazebo owns simulation
time and physical truth; the companion never controls Gazebo directly.

The yellow circle is the start and landing target. The vehicle intentionally
takes off vertically, stabilizes briefly, and lands on the same marker. The
nested `companion/comp2026` repository is not part of this runtime.

## Run from a clean checkout

Requirements are Docker with Compose v2 and `uv`. No credentials are required.

```bash
uv sync
docker compose --profile phase3 build
uv run drone-sim start --config config/default-run.json
```

The default run may take substantially longer than its 60 seconds of public
simulation time because ArduPilot boots during private lockstep warmup and the
configured real-time factor is `0.1`. The command prints the run ID and exits
only after finalization. Inspect it with:

```bash
uv run drone-sim status RUN_ID
uv run drone-sim collect-results RUN_ID
```

Outputs are under `runs/RUN_ID/`. A completed bundle contains both MP4s, the
ten-topic MCAP bag, Gazebo native state and server log, seven structured module
logs, configuration snapshots, scoring files, and `manifest.json`.

## Acceptance contract

- Terminal state is `COMPLETED` with reason `mission_complete`.
- Public `/clock` is exactly 0 through 60,000,000,000 ns.
- Each camera and matching ground-truth stream contains exactly 1,200 samples
  on a contiguous 50 ms grid; both MP4s are H.264/yuv420p at 320x240 and 20 FPS.
- The mission reaches landed/disarmed state through positive MAVLink command
  acknowledgements.
- `descent_v1` awards 100/100 for airborne/contact, touchdown precision, safe
  pre-impact speed, and stable contact.
- `make inspect-phase3` performs independent semantic inspection when supplied
  the source revision/dirty flag and seven image digests captured at launch.

## Preserved local evidence

These ignored run directories accompany the handoff on this machine but are
not part of Git clones:

| Run | Condition | Result | Manifest SHA-256 |
| --- | --- | --- | --- |
| `06c87df1-92ab-48eb-a4bc-abed25a4004b` | Baseline | Accepted 100/100 | `02c90288baf27da49d18d77278066c21abc6ded8f62c62a8664679f35dd34705` |
| `40b5d3aa-c1ac-49f6-bced-3f9c12247381` | Gazebo and SITL limited to 0.75 CPU through source completion | Accepted 100/100 | `21707fb7195a3edb71e3642ec0525c1fc066be7c5b08c9a007ef0254871663a2` |

## Scope and next step

This is a vertical-descent infrastructure MVP, not the complete competition
mission. The next intended integration is the nested `companion/comp2026`
vision/autonomy code. Active electromagnet physics, payload/dropper behavior,
LiDAR, precision navigation, and broader scoring remain explicitly deferred in
`docs/technical-debt/vertical-slice-hardening.md`.
