# Gazebo Asset Provenance External Interface

This directory exposes an audit record, not a runtime service. Consumers may
verify the selectively imported Iris assets through
`ardupilot_gazebo-assets.json`. Each file record names its original upstream
path, its repository import path, its SHA-256 digest, and its SPDX license
identifier.

The immutable upstream identity is:

- origin: `https://github.com/ArduPilot/ardupilot_gazebo`
- revision: `082a0fe231f6e63bc8d1598f1cba461d9e2ea7f5`
- license: `LGPL-3.0-only`

The original model files and all four meshes are preserved under `upstream/`.
The license text is preserved as `LICENSE.ardupilot_gazebo.md`. The Phase 3
model's copied meshes are byte-identical imports and therefore have their own
manifest records. Its `model.config` and `model.sdf` are repository-authored
derived files and are deliberately not represented as upstream bytes.
