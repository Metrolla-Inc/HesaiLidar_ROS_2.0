#!/usr/bin/env bash
# Smoke tier of the Hesai driver image (TESTING.md). Needs Docker, python3 and sudo (one loopback
# address). Usage: docker/smoke/smoke.sh IMAGE [OLD_IMAGE]
#
#   1. Image facts: entrypoint, command and user are what the fleets run today.
#   2. Legacy mode: each fleet environment of tests/legacy/cases gives, in the real image, the
#      entrypoint output, rendered config.yaml and node-read config.yaml of its golden. With
#      OLD_IMAGE (the released image, e.g. hesai_driver:v1.0.5) the same is compared with it.
#   3. Contract refusals and checks: stdout/stderr byte-identical to tests/regression/golden.
#   4. Contract run as the product does it (uid 97000, read-only root, tmpfs /tmp, cap-drop ALL,
#      no-new-privileges, host network and IPC) with a synthetic PandarXT-32 (fake_xt32.py):
#      PointCloud2 on /east/lidar_points, calibration and firetime files loaded, clean stop.
set -euo pipefail

IMAGE=${1:?usage: smoke.sh IMAGE [OLD_IMAGE]}
OLD_IMAGE=${2:-}
HERE=$(cd "$(dirname "$0")" && pwd)
REPO=$(cd "$HERE/../.." && pwd)
SENSOR_IP=192.168.1.201
WORK=$(mktemp -d)
NAME=hesai-smoke-$$
FAILED=0

fail() { echo "SMOKE FAIL: $*" >&2; FAILED=1; }
ok() { echo "ok: $*"; }
# expect MESSAGE COMMAND...: ok when the command succeeds, a failure otherwise.
expect() { local msg=$1; shift; if "$@"; then ok "$msg"; else fail "$msg"; fi; }
cleanup() {
    docker rm -f "$NAME" >/dev/null 2>&1 || true
    sudo ip addr del "$SENSOR_IP/32" dev lo >/dev/null 2>&1 || true
    rm -rf "$WORK"
}
trap cleanup EXIT

# Read KEY=VALUE lines of a case file into docker -e arguments (skips comments and "argv:").
env_args() {
    local line
    ENV_ARGS=()
    while IFS= read -r line; do
        [[ -z "$line" || "$line" == \#* || "$line" == argv:* ]] && continue
        ENV_ARGS+=(-e "$line")
    done < "$1"
}

# ---------------------------------------------------------------------------------------------
echo "== 1. image facts"
facts=$(docker image inspect "$IMAGE" --format '{{json .Config.Entrypoint}} {{json .Config.Cmd}} user={{.Config.User}}')
echo "$facts"
[[ "$facts" == '["/entrypoint.sh"] ["ros2","launch","hesai_ros_driver","start.py"] user=' ]] \
    || fail "entrypoint, command or user changed: $facts"

# ---------------------------------------------------------------------------------------------
echo "== 2. legacy mode, fleet environments"
SHARE=/hesai_ws/install/hesai_ros_driver/share/hesai_ros_driver/config/config.yaml
# shellcheck disable=SC2016  # expanded by the shell inside the container
DUMP='echo "@@@ exec: $0 $*"; echo "@@@ LD_LIBRARY_PATH=$LD_LIBRARY_PATH"; echo "@@@ live"; cat '"$SHARE"'; echo "@@@ node-reads"; cat /hesai_ws/src/config/config.yaml'
legacy_dump() {  # IMAGE CASE -> stdout of entrypoint + dump
    env_args "$2"
    docker run --rm "${ENV_ARGS[@]}" "$1" bash -c "$DUMP" ros2 launch hesai_ros_driver start.py
}
for case in "$REPO"/tests/legacy/cases/*.env; do
    stem=$(basename "$case" .env)
    legacy_dump "$IMAGE" "$case" > "$WORK/$stem.new" 2>&1 || fail "$stem: exit $?"
    python3 - "$WORK/$stem.new" "$REPO/tests/legacy/golden/$stem.json" "$stem" <<'PY' || FAILED=1
import json, sys
out = open(sys.argv[1]).read()
golden = json.load(open(sys.argv[2]))
stem = sys.argv[3]
head, _, rest = out.partition("@@@ exec: ")
live = rest.split("@@@ live\n", 1)[1].split("@@@ node-reads\n", 1)
problems = []
if head != golden["stdout"] + golden["stderr"]:
    problems.append(f"entrypoint output {head!r} != golden {golden['stdout'] + golden['stderr']!r}")
if live[0] != golden["live_config_yaml"]:
    problems.append("rendered config.yaml differs from the golden")
if live[1] != golden["node_reads_config_yaml"]:
    problems.append("the config.yaml the node reads differs from the golden")
ld = rest.split("@@@ LD_LIBRARY_PATH=", 1)[1].split("\n", 1)[0]
if not ld.endswith(golden["exec"]["env"]["LD_LIBRARY_PATH"].lstrip(":")):
    problems.append(f"LD_LIBRARY_PATH {ld!r} does not end like the golden's")
for p in problems:
    print(f"SMOKE FAIL: legacy {stem}: {p}", file=sys.stderr)
if not problems:
    print(f"ok: legacy {stem}: entrypoint output and both config.yaml files equal the golden")
sys.exit(1 if problems else 0)
PY
    if [[ -n "$OLD_IMAGE" ]]; then
        legacy_dump "$OLD_IMAGE" "$case" > "$WORK/$stem.old" 2>&1 || true
        if diff -u "$WORK/$stem.old" "$WORK/$stem.new"; then
            ok "legacy $stem identical to $OLD_IMAGE"
        else
            fail "legacy $stem differs from $OLD_IMAGE (diff above)"
        fi
    fi
done

# ---------------------------------------------------------------------------------------------
echo "== 3. contract: check and refusals in the image equal the host-side goldens"
for case in "$REPO"/tests/regression/cases/*.env; do
    stem=$(basename "$case" .env)
    env_args "$case"
    set +e
    docker run --rm "${ENV_ARGS[@]}" "$IMAGE" check > "$WORK/$stem.out" 2> "$WORK/$stem.err"
    code=$?
    set -e
    {
        printf 'exit %s\n--- stdout\n' "$code"
        cat "$WORK/$stem.out"
        printf -- '--- stderr\n'
        cat "$WORK/$stem.err"
    } > "$WORK/$stem.golden"
    if diff -u "$REPO/tests/regression/golden/$stem.golden" "$WORK/$stem.golden"; then
        ok "contract $stem (exit $code)"
    else
        fail "contract $stem differs from its golden (diff above)"
    fi
done

set +e
docker run --rm -e METROLLA_DRIVER_CONTRACT=1 -e LIDAR_MODEL=xt32 -e "SENSOR_IP=$SENSOR_IP" "$IMAGE" bash -l \
    > /dev/null 2> "$WORK/cmd.err"
code=$?
set -e
expect "unknown command refused (exit $code)" grep -q "unknown command" "$WORK/cmd.err"
expect "unknown command exits 1" test "$code" = 1

set +e
docker run --rm --read-only --user 97000:97000 -e METROLLA_DRIVER_CONTRACT=1 -e LIDAR_MODEL=xt32 \
    -e "SENSOR_IP=$SENSOR_IP" "$IMAGE" > /dev/null 2> "$WORK/ro.err"
code=$?
set -e
expect "read-only root without a writable /tmp refused: $(cat "$WORK/ro.err")" grep -q "cannot write" "$WORK/ro.err"
expect "read-only root without a writable /tmp exits 1" test "$code" = 1

# ---------------------------------------------------------------------------------------------
echo "== 4. contract run with a synthetic PandarXT-32"
CONTRACT_ENV=(-e METROLLA_DRIVER_CONTRACT=1 -e LIDAR_MODEL=hesai-xt32 -e "SENSOR_IP=$SENSOR_IP"
              -e SENSOR_ID=east -e HOST_IP=192.168.1.100/24 -e FRAME_ID=hesai_east
              -e ROS_LOCALHOST_ONLY=1)
RESTRICT=(--network host --ipc host --user 97000:97000 --read-only --tmpfs /tmp
          --cap-drop ALL --security-opt no-new-privileges:true
          -e ROS_HOME=/tmp/ros -e ROS_LOG_DIR=/tmp/ros/log)
sudo ip addr add "$SENSOR_IP/32" dev lo
docker run -d --name "$NAME" "${RESTRICT[@]}" "${CONTRACT_ENV[@]}" "$IMAGE" > /dev/null
sleep 3
[[ "$(docker inspect -f '{{.State.Running}}' "$NAME")" == true ]] || fail "driver not running"
uid=$(docker exec "$NAME" sh -c 'stat -c %u /proc/1')
expect "driver runs as uid $uid (97000)" test "$uid" = 97000
comm=$(docker exec "$NAME" cat /proc/1/comm)
expect "PID 1 is the driver ($comm)" test "$comm" = hesai_ros_drive
docker run --rm "${CONTRACT_ENV[@]}" "$IMAGE" check | sed '$d' > "$WORK/expected.yaml"
docker exec "$NAME" cat /tmp/hesai/config.yaml > "$WORK/actual.yaml"
expect "/tmp/hesai/config.yaml in the container equals check's render" \
    diff -u "$WORK/expected.yaml" "$WORK/actual.yaml"

python3 "$HERE/fake_xt32.py" --sensor-ip "$SENSOR_IP" --data-port 2368 --ptc-port 9347 \
    --host 127.0.0.1 --seconds 40 > "$WORK/fake.log" 2>&1 &
FAKE=$!
set +e
docker run --rm "${RESTRICT[@]}" -e ROS_LOCALHOST_ONLY=1 -v "$HERE:/smoke:ro" --entrypoint bash "$IMAGE" \
    -c 'source /opt/ros/jazzy/setup.bash && python3 /smoke/check_cloud.py /east/lidar_points hesai_east --clouds 3 --timeout 60' \
    > "$WORK/check.json"
code=$?
set -e
cat "$WORK/check.json"
expect "3 PointCloud2 on /east/lidar_points decoded from synthetic packets (exit $code)" test "$code" = 0
wait "$FAKE" || true
cat "$WORK/fake.log"
docker logs "$NAME" > "$WORK/driver.log" 2>&1
grep -E "hesai-driver:|correction|firetime|ptc" "$WORK/driver.log" | head -20
expect "angle correction file loaded" grep -q "Parser correction file success!" "$WORK/driver.log"
expect "firetime file loaded" grep -q "Open firetime file success!" "$WORK/driver.log"
if grep -q "load firetime error\|Open correction file failed" "$WORK/driver.log"; then
    fail "calibration load error in the log"
fi
docker stop -t 10 "$NAME" > /dev/null
code=$(docker inspect -f '{{.State.ExitCode}}' "$NAME")
expect "stopped by SIGTERM, not SIGKILL (exit $code)" test "$code" != 137

if [[ $FAILED != 0 ]]; then
    echo "== driver log"; cat "$WORK/driver.log"
    echo "SMOKE FAILED"
    exit 1
fi
echo "SMOKE PASSED"
