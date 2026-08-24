# Worlds Internal Interface

`resolve_world(WorldConfig, package_root=None)` accepts exactly
`phase3_foundation/iris` and `vertical_descent/iris_flight`, and returns a frozen `ResolvedWorld` containing the
absolute world path, stable identities, the absolute `resources/models`
Gazebo resource path, the world SHA-256, and a sorted tuple of SHA-256 values
for every regular file below the full `resources` root.

Resolution fails closed on an escaping world, symlink, special file,
multiply-linked regular file, or tree mutation. Resolution opens one
identity-stable snapshot of the full resource inventory: no-follow descriptors
for the root, every directory, and every single-link regular file remain open
through hashing and final verification. Hashes are read only from those
retained file descriptors. Before return, every path must still name the same
device and inode with the same metadata, and every directory inventory must be
unchanged. All descriptors close on success or failure. The optional
`package_root` exists for controlled tests; production uses the image-owned
`gazebo/resources` tree.

The later server layer replaces any ambient `GZ_SIM_RESOURCE_PATH` with the
returned absolute models directory. This makes `model://iris_phase3` local and
prevents a remote resource fallback. The flight selection resolves
`model://iris_flight`; the model's physical entity remains `iris` so its private
transport children have stable names.
