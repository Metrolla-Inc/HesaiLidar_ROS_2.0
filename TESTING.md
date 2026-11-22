# TESTING.md — Hesai driver image

CI (`.github/workflows/ci.yml`) runs three separately named jobs on every pull request and every
push to `master`, on `ubuntu-24.04`. The release workflow (`docker-build.yml`) also builds the
image for amd64 and arm64 on every pull request, without pushing.

| Tier | Job | What | Needs | Time |
|---|---|---|---|---|
| unit | `unit` | `tests/unit`: the environment contract (every accepted value, every refusal and its message, `run`/`check`/`selfcheck`), the contract against the C++ and CMake it relies on (`test_vendor_contract.py`, including the "node ignores the rendered config" mechanism), the synthetic XT32 against the SDK's protocol header, the recorded JT16 cloud layout; `tests/legacy`; shellcheck; hadolint | Python, pytest, pyyaml, the SDK submodule | seconds |
| regression | `regression` | `tests/legacy` (fleet environments → the v1.0.5 entrypoint's goldens), `tests/regression` (contract renders and refusal messages → goldens; past bugs) | Python, pytest, pyyaml, the SDK submodule | seconds |
| smoke | `smoke` | build the image (amd64), `docker/smoke/smoke.sh`: legacy fleet environments in the real image, contract refusals, and a contract run with a synthetic PandarXT-32 | Docker, sudo (one loopback address) | ~20-40 min cold |

Run locally:

```sh
git submodule update --init
python3 -m pip install pytest pyyaml shellcheck-py
python3 -m pytest                       # unit + legacy + regression
shellcheck docker/contract-entrypoint.sh docker/smoke/smoke.sh
docker build -t hesai-driver:ci . && docker/smoke/smoke.sh hesai-driver:ci [ghcr.io/metrolla-inc/hesai_driver:v1.0.5]
```

## Legacy goldens (the fleets' behaviour)

`tests/legacy/harness.py` runs an entrypoint in a sandbox laid out like the image (per-model
configs in the install space, the source `config/config.yaml`, empty ROS setup files), with
absolute paths rewritten into the sandbox and the exec'd command replaced by a recorder. A record
holds exit code, stdout, stderr, the exec'd argv, its environment, every file under `/hesai_ws`
with its hash, the rendered `config.yaml` and the `config.yaml` the node reads.

`tests/legacy/cases/*.env` are the fleets' environments, taken from the balena-fleets compose files
(partner1-dev XT32 and JT128, flood_dev AT128E2X, partner1-mirror, campus1 with the model unset),
plus the switch at `0` and empty, the unknown-model fallback, and a command with launch
arguments. The goldens were generated from `entrypoint-v1.0.5.sh` (the tag's blob
`9aee44f`) **before** the contract was added (commit "Pin the legacy entrypoint"); the test runs
both that file and the current `entrypoint.sh` and requires both to equal the golden. `REGEN=1`
regenerates from the v1.0.5 file only.

The smoke tier repeats this in the built image: same entrypoint output, same rendered and node-read
`config.yaml`, same library path. When the run's token can pull `hesai_driver:v1.0.5`, it also
diffs the old and the new image's output for every case.

## Contract goldens

`tests/regression/test_goldens.py` runs `hesai_env.py check` in-process with the image's paths for
each `tests/regression/cases/<name>.env` and compares exit code, stdout and stderr with
`golden/<name>.golden` (`refused-*` must exit 1). The smoke tier runs the same `check` in the image
and requires byte-identical output. `REGEN=1 python3 -m pytest tests/regression` regenerates; read
the diff.

## Past bugs (tests/regression/test_past_bugs.py)

1. The node ignored the rendered config (legacy mechanism pinned; contract: the node's
   `config_path` is the written file, through the entrypoint dispatch too).
2. An unknown `LIDAR_MODEL` became PandarXT-16 (300 random names: never a fallback).
3. Calibration paths that do not exist in the image (every rendered path exists; a missing
   firetime file refuses to start instead of the SDK's log line).

## What the smoke test does with packets

There is **no recording of raw Hesai packets**: the only Hesai recording (the JT16 bag in
`Metrolla-Inc/HesaiDriver`, `jt16-r2`) holds decoded `/lidar_points` and `/lidar_imu` only, and the
JT16 is a serial sensor this contract refuses. Two clouds of it (500 kB,
`tests/fixtures/jt16-r2-lidar_points.cdr`, provenance in the `.json`) pin the PointCloud2 layout a
real sensor produced through this driver (fields x y z intensity ring timestamp, 26-byte points,
frame `hesai_lidar`, 16 rings); the smoke checker requires the same layout.

The packets are synthetic: `docker/smoke/fake_xt32.py` (first written for C13) sends deterministic
PandarXT-32 UDP protocol 6.1 packets from `192.168.1.201` (a loopback alias) and answers the
driver's PTC connection with an error code, so the SDK loads the calibration files from the
configuration. The smoke run (uid 97000, read-only root, tmpfs `/tmp`, cap-drop ALL,
no-new-privileges, host network and IPC) requires: `/tmp/hesai/config.yaml` in the container equal
to `check`'s render; 3 clouds on `/east/lidar_points` with frame `hesai_east`, the recorded layout,
at least 60,000 points, every point 2.000-7.156 m away, rings 0-31; the log lines "Parser
correction file success!" and "Open firetime file success!"; a clean stop on SIGTERM.

**Run locally (partial, not the image):** on 2026-11-21 the driver was built with colcon in a
conda RoboStack Jazzy environment (not Ubuntu's packages, not Docker), a contract config rendered
by `hesai_env.py` (SDK paths pointed at the build tree) was given to the node with
`-r __ns:=/east -p config_path:=...`, and `fake_xt32.py` + `check_cloud.py` passed: 64,000 points
per cloud, ranges 2.000-7.156 m, rings 0-31, frame `hesai_east`, correction and firetime files
loaded. The image itself, uid 97000, read-only root and the legacy-in-image comparison run in CI
only.

## Not exercised (no sensor)

- No real Hesai sensor, no real packets. Only the PandarXT-32 (protocol 6.1) path is decoded; the
  other models' configurations are rendered and their calibration files checked, never decoded.
- PTC answers are synthetic refusals: calibration download from a sensor, standby/speed settings
  and PTP status are not exercised. `TIME_MODE` other than the default is rendered only.
- Firetime correction on vs off (contract vs legacy) is not compared on hardware.
- arm64 is built (release workflow) but not run.

## Rules

- Assert values; a pass with zero points or zero cases is a failure. No network, Docker, ROS or
  wall clock in unit and regression tests. Check once that a new test fails when the behaviour is
  broken on purpose.
