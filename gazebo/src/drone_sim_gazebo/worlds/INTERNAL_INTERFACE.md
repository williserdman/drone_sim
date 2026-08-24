# Worlds Internal Interface

`resolve_world(WorldConfig, package_root=None)` accepts only
`phase3_foundation/iris` and returns a frozen `ResolvedWorld` containing the
absolute world path, stable identities, the absolute `resources/models`
Gazebo resource path, the world SHA-256, and a sorted tuple of SHA-256 values
for every regular file below the full `resources` root.

Resolution fails closed on an escaping world, symlink, special file,
multiply-linked regular file, or tree mutation. Hash reads reuse the artifact
validator's retained, no-follow descriptors and pre/post identity checks. The
optional `package_root` exists for controlled tests; production uses the
image-owned `gazebo/resources` tree.

The later server layer replaces any ambient `GZ_SIM_RESOURCE_PATH` with the
returned absolute models directory. This makes `model://iris_phase3` local and
prevents a remote resource fallback.
