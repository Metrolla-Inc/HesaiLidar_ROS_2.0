"""Run an entrypoint script of this image outside the image, in a sandbox, and record everything it
does: exit code, stdout, stderr, the command it execs (argv), the environment that command gets,
and every file under /hesai_ws afterwards (path and content).

The sandbox is a temporary directory laid out like the image: the per-model configs installed in
the share directory, the source copy of config/config.yaml (the file the node reads), empty ROS
and workspace setup.bash files. The script's absolute paths are rewritten into the sandbox, the
exec'd command is a recorder on PATH, and every sandbox path is mapped back in the record, so a
record reads as if taken in the image.

Used by tests/legacy (goldens of the fleet environments) and tests/regression.
"""

from __future__ import annotations

import hashlib
import json
import shutil
import subprocess
import tempfile
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
SHARE = "/hesai_ws/install/hesai_ros_driver/share/hesai_ros_driver"
REWRITES = ("/hesai_ws", "/opt/ros/jazzy", "/opt/metrolla/hesai")
# Variables of the runner that must not leak into a record.
BASE_ENV = {"PATH": "/usr/bin:/bin", "LANG": "C", "LC_ALL": "C"}
RECORDED_ENV = ("LD_LIBRARY_PATH", "LIDAR_MODEL", "DEVICE_IP", "UDP_PORT", "HOST_IP", "CONFIG_PATH",
                "SOURCE_CONFIG_DIR", "SELECTED_CONFIG", "METROLLA_DRIVER_CONTRACT")

RECORDER = """#!/usr/bin/env python3
import json, os, sys
with open(os.environ["HARNESS_RECORD"], "w") as f:
    json.dump({"argv": [os.path.basename(sys.argv[0])] + sys.argv[1:], "env": dict(os.environ)}, f)
"""


def _layout(root: Path, config_dir: Path) -> None:
    share = root / SHARE.lstrip("/") / "config"
    share.mkdir(parents=True)
    for f in sorted(config_dir.glob("*.yaml")):
        shutil.copyfile(f, share / f.name)
    src = root / "hesai_ws/src/config"
    src.mkdir(parents=True)
    shutil.copyfile(config_dir / "config.yaml", src / "config.yaml")
    for setup in ("opt/ros/jazzy/setup.bash", "hesai_ws/install/setup.bash"):
        (root / setup).parent.mkdir(parents=True, exist_ok=True)
        (root / setup).write_text("# empty in the sandbox\n")
    bindir = root / "bin"
    bindir.mkdir()
    for name in ("ros2", "contract-entrypoint.sh"):
        (bindir / name).write_text(RECORDER)
        (bindir / name).chmod(0o755)
    # The contract block execs /opt/metrolla/hesai/contract-entrypoint.sh: record that too.
    (root / "opt/metrolla/hesai").mkdir(parents=True)
    shutil.copyfile(bindir / "contract-entrypoint.sh", root / "opt/metrolla/hesai/contract-entrypoint.sh")
    (root / "opt/metrolla/hesai/contract-entrypoint.sh").chmod(0o755)


def run(script: Path, env: dict[str, str], argv: list[str], config_dir: Path | None = None) -> dict:
    config_dir = config_dir or REPO / "config"
    with tempfile.TemporaryDirectory(prefix="hesai-legacy-") as tmp:
        root = Path(tmp)
        _layout(root, config_dir)
        text = script.read_text()
        for prefix in REWRITES:
            text = text.replace(prefix, str(root) + prefix)
        sandboxed = root / "entrypoint.sh"
        sandboxed.write_text(text)
        record_file = root / "record.json"
        run_env = dict(BASE_ENV)
        run_env["PATH"] = f"{root}/bin:" + run_env["PATH"]
        run_env["HARNESS_RECORD"] = str(record_file)
        run_env.update(env)
        proc = subprocess.run(["bash", str(sandboxed), *argv], env=run_env, capture_output=True,
                              text=True, timeout=60, check=False)

        def back(value: str) -> str:
            return value.replace(str(root), "")

        record = None
        if record_file.exists():
            raw = json.loads(record_file.read_text())
            exec_env = {k: back(v) for k, v in raw["env"].items()
                        if k in RECORDED_ENV or k in env}
            record = {"argv": [back(a) for a in raw["argv"]], "env": dict(sorted(exec_env.items()))}
        files = {}
        for path in sorted((root / "hesai_ws").rglob("*")):
            if path.is_file():
                data = path.read_bytes()
                files["/" + str(path.relative_to(root))] = {
                    "sha256": hashlib.sha256(data).hexdigest(),
                    "bytes": len(data),
                }
        return {
            "exit": proc.returncode,
            "stdout": back(proc.stdout),
            "stderr": back(proc.stderr),
            "exec": record,
            "files": files,
            "live_config_yaml": (root / SHARE.lstrip("/") / "config/config.yaml").read_text(),
            "node_reads_config_yaml": (root / "hesai_ws/src/config/config.yaml").read_text(),
        }


def read_case(path: Path) -> tuple[dict[str, str], list[str]]:
    """KEY=VALUE lines (# comments); a line 'argv: a b c' sets the command."""
    env: dict[str, str] = {}
    argv = ["ros2", "launch", "hesai_ros_driver", "start.py"]
    for line in path.read_text().splitlines():
        line = line.strip()
        if not line or line.startswith("#"):
            continue
        if line.startswith("argv:"):
            argv = line[len("argv:"):].split()
            continue
        key, sep, value = line.partition("=")
        assert sep, f"{path}: not KEY=VALUE: {line!r}"
        env[key] = value
    return env, argv


def dump(record: dict) -> str:
    return json.dumps(record, indent=2, sort_keys=True) + "\n"

