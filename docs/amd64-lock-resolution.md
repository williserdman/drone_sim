# AMD64 lock resolution supplement

Resolution date: 2026-10-12

This supplement replaces the earlier audit's conclusion that a native AMD64
machine is required to generate complete replacement lock candidates. Exact
candidate metadata can be resolved on ARM64 without executing AMD64 code. A
native AMD64 build is still required for acceptance.

## Result

`resolve_amd64_locks.py` resolved the Dockerfile install deltas from the pinned
AMD64 ROS base status. It used the ARM64 variant of the same pinned image only
as an apt utility, set `APT::Architecture=amd64`, used an isolated dpkg status,
and requested `--no-install-recommends --no-remove` in every simulation and
download pass. It then materialized each selected delta as synthetic installed
metadata and reran the same request. All four second simulations converged with
no install, upgrade, or removal proposal.

The run downloaded every selected package and matched its file SHA-256 against
the value in the signed apt metadata. The package files were deleted after
verification. The resolver writes package manifests with the repository filename, size,
architecture, index SHA-256, and verified downloaded filename for every row.

| Layer | Parent state | Selected rows | Download bytes | Candidate SHA-256 |
| --- | --- | ---: | ---: | --- |
| Artifacts direct install | Pinned ROS AMD64 base | 15 | 17,696,872 | `5dc0c3ef68106fe3d075e5826c38a335c414a79ffd8c222b99359e2447209623` |
| Artifacts FFmpeg install | Synthetic artifacts-direct state | 146 | 107,778,832 | `2048c831d649fde6dd167f20709706cddf6c93bb1d09dcbb7ccafa452b9de71d` |
| Gazebo Harmonic install | Pinned ROS AMD64 base | 803 | 556,722,278 | `6bb600d901816d3733a0e5e46eca413edc715aff93a6ede40c4fbcd5849c3192` |
| Local viewer install | Synthetic Gazebo state | 90 | 45,921,782 | `d29308041de14bb20d97208c0a647de1277655139ab292beadf69a1b327e3cbb` |

The per-architecture counts were 12 AMD64 and 3 `all` packages for artifacts
direct, 138 AMD64 and 8 `all` for FFmpeg, 752 AMD64 and 51 `all` for Gazebo,
and 47 AMD64 and 43 `all` for the viewer.

## Candidate changes

The artifacts direct pins change only the two architecture-specific ROS build
timestamps:

```diff
-ros-jazzy-rosbag2=0.26.11-1noble.20261006.034453
-ros-jazzy-rosbag2-storage-mcap=0.26.11-1noble.20260911.095159
+ros-jazzy-rosbag2=0.26.11-1noble.20261006.034645
+ros-jazzy-rosbag2-storage-mcap=0.26.11-1noble.20260911.143733
```

The Dockerfile selects these refreshed rows only for AMD64. ARM64 retains the
previous `.034453` and `.095159` rows, which are architecture-specific ROS
builds already present in the accepted ARM64 image.

The FFmpeg lock changes from 144 to 146 rows. Relative to the sorted current
lock, it removes seven rows and adds nine. Seven changes are repository version
updates; `libx11-6` and `libx11-data` become part of the install delta because
the pinned base does not already contain the newer versions. The exact change
is in `ffmpeg-packages.amd64.candidate.diff`.

The Gazebo lock changes from 795 to 803 rows. Relative to the sorted current
lock, it removes 105 rows and adds 113. The direct pins become:

```text
ros-jazzy-ros-gz=1.0.24-1noble.20261006.041805
ros-jazzy-ros-gz-bridge=1.0.24-1noble.20260920.234644
ros-jazzy-ros-gz-image=1.0.24-1noble.20260921.023911
ros-jazzy-ros-gz-interfaces=1.0.24-1noble.20260911.110630
ros-jazzy-ros-gz-sim=1.0.24-1noble.20261006.035056
ros-jazzy-gz-sim-vendor=0.0.13-1noble.20260905.073557
ros-jazzy-sdformat-vendor=0.0.14-1noble.20260905.033543
zstd=1.5.5+dfsg2-2build1.1
```

The exact sorted change is in `harmonic-packages.amd64.candidate.diff`.

The 90-row AMD64 viewer candidate is byte-for-byte identical to
`viewer-packages.arm64.lock`. Its empty diff has the SHA-256 of an empty file,
`e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855`.
Resolving it against the refreshed 803-row AMD64 Gazebo state still selects all
90 rows, so the parent state has no overlap that changes the expected viewer
delta.

## Exact inputs

The resolver verified Docker Hub's OCI bytes before extracting the AMD64 dpkg
status:

| Input | Digest |
| --- | --- |
| ROS multi-architecture index | `sha256:2589a8fba5257307857890173c069852c2abf913a0be7970f172478baecb09e4` |
| AMD64 child manifest | `sha256:bab7e640bf79cd84957e4e18fcba7d87efc3385b4e3f36a32eeca01638e43206` |
| AMD64 config | `sha256:3786be5ed1130691ee8813979e86da478531ff25e9e5bbdad85783a222716eba` |
| Layer supplying `/var/lib/dpkg/status` | `sha256:e8411d72f4b8d1a32d36a409d255c8e7f71951e6eecb2cbb35b4d27a5ca35348` |
| Extracted status | `bd99079deff3da2ae452b7a45871630ac4b799195efb39964d88f5e2b6cfe3ea` |

Source file SHA-256 values were:

| Input | SHA-256 |
| --- | --- |
| `artifacts/Dockerfile` | `280c56d02d41e5d3489e35f4b3359b0565f8688dd85c574092146f9224c83947` |
| `artifacts/ffmpeg-packages.lock` | `72d3275639a1486b0b243ea585ab823c5f5a118e764595ecfdafcd08df040b88` |
| `gazebo/Dockerfile` | `adf74d994649f677de304d10039c02e6c9d8f8d5d78761d31b88488528aa5517` |
| `gazebo/harmonic-packages.lock` | `c900f123e628d29c3ca5193f5767ef2f9084b60da6a18425aef66dc5214956b1` |
| `gazebo/local-viewer/Dockerfile` | `b5df0ddd7b7a94235a5bdc2631717a524fbfbbdca5ae1020655aa15a66b104b7` |
| `gazebo/local-viewer/viewer-packages.arm64.lock` | `d29308041de14bb20d97208c0a647de1277655139ab292beadf69a1b327e3cbb` |

The tracked `amd64-lock-resolution-summary.json` retains the complete signed
index inventory. The resolver recreates the per-package manifests for audit.
The tracked resolver SHA-256 is
`bf17353aef22a20216231a48a9bc39f96a6884fd1d520fcbdfc9fa9917928a05`;
the audited run used the same resolver logic with outputs under the former SDD
workspace instead of `.local-runtime`.

The signed metadata came from these configured sources:

- [Ubuntu Noble archive](http://archive.ubuntu.com/ubuntu/dists/noble/InRelease)
- [Ubuntu Noble updates](http://archive.ubuntu.com/ubuntu/dists/noble-updates/InRelease)
- [Ubuntu Noble backports](http://archive.ubuntu.com/ubuntu/dists/noble-backports/InRelease)
- [Ubuntu Noble security](http://security.ubuntu.com/ubuntu/dists/noble-security/InRelease)
- [ROS 2 Noble](http://packages.ros.org/ros2/ubuntu/dists/noble/InRelease)

The successful run saved all 22 signed and compressed index files with their
sizes and SHA-256 values in `resolution-summary.json`. The ROS source definition
and its embedded signing key came from the pinned ROS utility image. The
resolver did not add a key or trust an unsigned index.

## Reproduction

From the repository root:

```sh
python3 scripts/resolve-amd64-locks.py
```

The script creates a uniquely named run directory under
`.local-runtime/amd64-lock-resolution/runs/`. It does not depend on `/tmp`
state. It records the base manifest,
apt sources and indexes, simulations, selected roots, synthetic status files,
per-package checksum evidence, final summary, and candidate files. Its Docker
containers have unique `drone-sim-amd64-lock-*` names, use `--rm`, and execute
only the pinned ARM64 utility image. The script compares the complete image-tag
and running-container inventories before and after the run and fails if either
changes.

The successful evidence is in `runs/20261012T221228Z-50964`. The utility
reported `aarch64`; every apt request reported AMD64 selection. The final
Docker inventory comparison passed, and the eight original containers remained
running.

## Failure and retry record

The first full download pass returned apt exit 100 late in the 803-package
Gazebo download after populating nearly the entire cache. Retrying that exact
request fetched the remaining 551 kB and completed, identifying a transient
transport failure rather than an unavailable package or checksum mismatch. The
resolver now sets five acquire retries and a 30-second HTTP timeout.

The first resumed pass then downloaded all 90 viewer archives but apt returned
exit 100 for `python3-pyasyncore_1.0.2-2_all.deb`: the `_apt` user could not
open the bind-mounted `cache/archives/partial` path (`EACCES`). The bind-mounted directory
retained mode 0700 between utility containers on this Colima host. The
resolver now sets `APT::Sandbox::User=root` inside the disposable utility
container. Repeating the complete four-layer pass then verified every archive,
cleaned the cache, and passed the Docker inventory check. This override affects
only apt's cache writer; package selection and signed-index verification are
unchanged.

## Acceptance limit

This method proves that apt's AMD64 solver accepts the exact direct pins and
candidate deltas against the pinned base metadata, and that every selected
archive is downloadable with the signed-index checksum. It does not execute
AMD64 maintainer scripts, compile the Gazebo or viewer sources, link native
objects, start the images, or verify runtime behavior. Synthetic dpkg status
also cannot reproduce side effects created by maintainer scripts.

After Task 7 applies and reviews the candidate locks and direct pins, the
smallest remaining native AMD64 acceptance commands are:

```sh
scripts/local-build.sh --build
scripts/local-build.sh --check
```

Those commands must run on a real AMD64 host. Emulation is not acceptance.
