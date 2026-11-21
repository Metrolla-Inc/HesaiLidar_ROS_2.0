"""Shared setup: point hesai_env.py's in-image paths at this checkout, so the unit and regression
tiers render from the repo's config/ and check calibration files in the SDK submodule."""

from __future__ import annotations

import sys
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / "docker"))
sys.path.insert(0, str(REPO / "tests" / "legacy"))

import hesai_env  # noqa: E402

SDK = REPO / "src" / "driver" / "HesaiLidar_SDK_2.0"


def sdk_checked_out() -> bool:
    return (SDK / "correction").is_dir()


@pytest.fixture(autouse=True)
def repo_paths(monkeypatch):
    monkeypatch.setattr(hesai_env, "MODEL_CONFIG_DIR", str(REPO / "config"))
    monkeypatch.setattr(hesai_env, "SDK_DIR", str(SDK))
    yield


@pytest.fixture
def needs_sdk():
    if not sdk_checked_out():
        pytest.fail("the SDK submodule is not checked out (git submodule update --init): the "
                    "calibration-file checks cannot run, which is an ABORT, not a pass")
