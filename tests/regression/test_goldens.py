"""Regression tier: goldens of `check` (rendered configuration and exec line) and of the refusal
messages, for the environments in cases/*.env. Names starting refused- must exit 1, others 0.

`check` runs in-process with the image's paths (the SDK at /hesai_ws/src/src/...); only the
calibration-file existence check is pointed at this checkout's SDK submodule.

REGEN=1 rewrites the goldens; read the diff: a changed golden is a changed contract.
"""

from __future__ import annotations

import importlib
import os
from pathlib import Path

import pytest

import hesai_env

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[1]
CASES = sorted((HERE / "cases").glob("*.env"))


def read_env(path: Path) -> dict[str, str]:
    env = {}
    for line in path.read_text().splitlines():
        if line.strip() and not line.startswith("#"):
            key, sep, value = line.partition("=")
            assert sep, line
            env[key] = value
    return env


@pytest.fixture
def image_paths(monkeypatch, needs_sdk):
    fresh = importlib.reload(hesai_env)
    image_sdk = fresh.SDK_DIR
    repo_sdk = str(REPO / "src" / "driver" / "HesaiLidar_SDK_2.0")
    original = fresh.missing_files
    monkeypatch.setattr(fresh, "MODEL_CONFIG_DIR", str(REPO / "config"))
    monkeypatch.setattr(fresh, "missing_files",
                        lambda files: original([f.replace(image_sdk, repo_sdk, 1) for f in files]))
    return fresh


def test_there_are_cases():
    assert len([c for c in CASES if c.stem.startswith("refused-")]) >= 8
    assert len([c for c in CASES if not c.stem.startswith("refused-")]) >= 4


@pytest.mark.parametrize("case", CASES, ids=[c.stem for c in CASES])
def test_golden(case, image_paths, capsys):
    code = image_paths.main(["check"], read_env(case))
    out = capsys.readouterr()
    text = f"exit {code}\n--- stdout\n{out.out}--- stderr\n{out.err}"
    assert code == (1 if case.stem.startswith("refused-") else 0)
    golden = HERE / "golden" / (case.stem + ".golden")
    if os.environ.get("REGEN"):
        golden.write_text(text)
    assert text == golden.read_text()
