# Gazebo Asset Provenance Internal Interface

Provenance review is closed over the exact `files` array in
`ardupilot_gazebo-assets.json`. An import is valid only when its bytes hash to
the digest for its `source_path`, its `imported_path` is repository-relative,
and its `license` is `LGPL-3.0-only`. Every physical copy has a separate record,
including the upstream snapshot and the Phase 3 model copy of each mesh.

To refresh an asset, choose and review a new immutable upstream commit, fetch
the raw file from that commit, verify it outside the repository, then update
the revision, every affected digest, and every imported path together. Never
use a generated, dirty, or neighboring worktree as an import source.

No compiled library, ArduPilot system plugin, world, payload, gimbal, camera
pipeline, or remote model reference belongs in this provenance scope.
