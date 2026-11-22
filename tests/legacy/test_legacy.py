"""Legacy mode stays exactly the v1.0.5 image's behaviour (the image the fleets run).

Goldens: for each fleet environment in cases/*.env (taken from the balena-fleets compose files),
the record of running the entrypoint (exit code, stdout, stderr, the exec'd command and its
environment, every file under /hesai_ws with its hash, the rendered and the node-read
config.yaml) as produced by the **v1.0.5 entrypoint** (entrypoint-v1.0.5.sh, byte-identical to
git tag v1.0.5). The current entrypoint.sh must produce the same record.

REGEN=1 rewrites the goldens from the v1.0.5 script, never from the current one.
"""

from __future__ import annotations

import hashlib
import os
from pathlib import Path

import pytest

from harness import REPO, dump, read_case, run

HERE = Path(__file__).resolve().parent
CASES = sorted((HERE / "cases").glob("*.env"))
REFERENCE = HERE / "entrypoint-v1.0.5.sh"
CURRENT = REPO / "entrypoint.sh"
V105_BLOB = "9aee44f57a7a59fba34c82da6e51c258e5fffcaf"  # git rev-parse v1.0.5:entrypoint.sh
SHEBANG = "#!/bin/bash\n"
BLOCK_END = "# --- end of the Metrolla contract block; the v1.0.5 entrypoint follows ---\n"


def git_blob_id(data: bytes) -> str:
    return hashlib.sha1(b"blob %d\0" % len(data) + data).hexdigest()


def test_reference_is_the_v1_0_5_entrypoint():
    assert git_blob_id(REFERENCE.read_bytes()) == V105_BLOB


def test_entrypoint_is_v1_0_5_plus_the_opt_in_block_only():
    current = CURRENT.read_text()
    reference = REFERENCE.read_text()
    assert current.startswith(SHEBANG) and reference.startswith(SHEBANG)
    if current == reference:  # before the contract block was added
        return
    head, sep, tail = current.partition(BLOCK_END)
    assert sep, "entrypoint.sh differs from v1.0.5 and has no contract block end marker"
    assert tail == reference[len(SHEBANG):], "the legacy part of entrypoint.sh changed"
    block = [line for line in head[len(SHEBANG):].splitlines() if not line.startswith("#")]
    assert block == [
        'if [ -n "${METROLLA_DRIVER_CONTRACT:-}" ] && [ "${METROLLA_DRIVER_CONTRACT}" != "0" ]; then',
        '    exec /opt/metrolla/hesai/contract-entrypoint.sh "$@"',
        "fi",
    ]


def test_there_are_fleet_cases():
    assert len(CASES) >= 8


@pytest.mark.parametrize("case", CASES, ids=[c.stem for c in CASES])
def test_legacy_golden(case: Path):
    env, argv = read_case(case)
    golden = HERE / "golden" / (case.stem + ".json")
    reference = dump(run(REFERENCE, env, argv))
    if os.environ.get("REGEN"):
        golden.write_text(reference)
    expected = golden.read_text()
    assert reference == expected, "the v1.0.5 entrypoint no longer gives its golden (harness changed?)"
    assert dump(run(CURRENT, env, argv)) == expected, "legacy behaviour changed"


def test_legacy_render_is_not_what_the_node_reads():
    """Documents the v1.0.5 mechanism (AGENTS.md, "Legacy mode"): with OVERWRITE_CONFIG the
    entrypoint renders into the install space, while the node reads the source copy
    (/hesai_ws/src/config/config.yaml, PROJECT_PATH) and start.py passes no config_path. Kept as
    it is for the fleets; the contract mode fixes it."""
    env, argv = read_case(HERE / "cases" / "partner1-dev-jt128.env")
    record = run(CURRENT, env, argv)
    assert "JT128" in record["stdout"]
    assert record["live_config_yaml"] != record["node_reads_config_yaml"]
    assert record["node_reads_config_yaml"] == (REPO / "config" / "config.yaml").read_text()
    assert record["exec"]["argv"] == ["ros2", "launch", "hesai_ros_driver", "start.py"]
