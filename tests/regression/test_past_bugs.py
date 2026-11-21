"""Past bugs of this image, each pinned so it cannot come back in contract mode.

1. The driver ignored its rendered configuration: the legacy entrypoint renders
   config/<LIDAR_MODEL>.yaml into the install space, the node reads the compiled-in source path
   (/hesai_ws/src/config/config.yaml), and start.py passes no config_path. So LIDAR_MODEL,
   SENSOR_IP, HOST_IP, SENSOR_PORT, FRAME_ID and POINT_CLOUD_TOPIC never reached the node.
2. An unknown or empty LIDAR_MODEL silently became PandarXT-16.
3. The per-model configurations name calibration files under /hesai_ws/src/driver/..., a path
   that does not exist in the image (the SDK is at /hesai_ws/src/src/driver/...), so the SDK ran
   without its firetime correction and only logged "Open firetime file failed".
"""

from __future__ import annotations

import random
import string
from pathlib import Path

import pytest

import hesai_env as h
from harness import read_case, run

REPO = Path(__file__).resolve().parents[2]
LEGACY = REPO / "tests" / "legacy"


def config_the_node_reads(argv: list[str]) -> str:
    """The rule of node/hesai_ros_driver_node.cc: the config_path ROS parameter, else
    PROJECT_PATH + /config/config.yaml (PROJECT_PATH = /hesai_ws/src in the image)."""
    for i, arg in enumerate(argv):
        if arg == "-p" and argv[i + 1].startswith("config_path:="):
            return argv[i + 1][len("config_path:="):]
        if arg.startswith("config_path:="):  # a launch argument; start.py declares none
            pytest.fail("start.py has no config_path argument; this would be ignored")
    return h.PACKAGE_SOURCE + "/config/config.yaml"


def test_bug1_legacy_mechanism_is_as_documented():
    env, argv = read_case(LEGACY / "cases" / "mirasys-dev-jt128.env")
    record = run(REPO / "entrypoint.sh", env, argv)
    assert config_the_node_reads(record["exec"]["argv"]) == "/hesai_ws/src/config/config.yaml"
    assert "/hesai_ws/src/config/config.yaml" in record["files"]
    assert "JT128" not in record["node_reads_config_yaml"]


def test_bug1_contract_node_reads_the_rendered_file(monkeypatch, tmp_path, needs_sdk):
    calls = []
    monkeypatch.setattr(h.os, "execve", lambda path, argv, envp: calls.append(argv))
    env = {"METROLLA_DRIVER_CONTRACT": "1", "LIDAR_MODEL": "JT128", "SENSOR_IP": "192.168.1.201",
           "FRAME_ID": "jt128_frame", "HESAI_RUN_DIR": str(tmp_path / "run"), "ROS_HOME": str(tmp_path)}
    h.main(["run"], env)
    (argv,), = [calls]
    read = Path(config_the_node_reads(argv))
    assert read == tmp_path / "run" / "config.yaml"
    text = read.read_text()
    assert "config/JT128.yaml" in text and 'ros_frame_id: "jt128_frame"' in text


def test_bug1_contract_through_the_entrypoint_dispatch():
    """METROLLA_DRIVER_CONTRACT=1 makes /entrypoint.sh exec the contract entrypoint with the same
    arguments, before the legacy code runs (no legacy render, no legacy output)."""
    env = {"METROLLA_DRIVER_CONTRACT": "1", "LIDAR_MODEL": "xt32", "SENSOR_IP": "192.168.1.201",
           "OVERWRITE_CONFIG": "true"}
    record = run(REPO / "entrypoint.sh", env, ["ros2", "launch", "hesai_ros_driver", "start.py"])
    assert record["exit"] == 0 and record["stdout"] == "" and record["stderr"] == ""
    assert record["exec"]["argv"] == ["contract-entrypoint.sh", "ros2", "launch", "hesai_ros_driver",
                                      "start.py"]
    assert record["live_config_yaml"] == (REPO / "config" / "config.yaml").read_text()


@pytest.mark.parametrize("value", ["2", "true", "yes"])
def test_any_non_zero_switch_value_goes_to_the_contract_which_refuses_unknown_versions(value):
    record = run(REPO / "entrypoint.sh", {"METROLLA_DRIVER_CONTRACT": value}, [])
    assert record["exec"]["argv"] == ["contract-entrypoint.sh"]
    with pytest.raises(h.Refused) as exc:
        h.parse({"METROLLA_DRIVER_CONTRACT": value, "LIDAR_MODEL": "xt32", "SENSOR_IP": "192.168.1.201"})
    assert exc.value.problems[0].startswith("METROLLA_DRIVER_CONTRACT=")


def test_bug2_no_unknown_name_ever_becomes_pandarxt16():
    rng = random.Random(20261121)
    names = ["", "xt-32", "XT32M", "PandarXT", "Pandar XT-32", "pandarxt-16 ", "hesai", "hesai-",
             "PandarXT-16.yaml", "../PandarXT-16"]
    names += ["".join(rng.choice(string.ascii_letters + string.digits + "-_") for _ in range(rng.randint(1, 12)))
              for _ in range(300)]
    accepted = 0
    for name in names:
        try:
            s = h.parse({"METROLLA_DRIVER_CONTRACT": "1", "LIDAR_MODEL": name, "SENSOR_IP": "192.168.1.201"})
        except h.Refused:
            continue
        accepted += 1
        # Accepted only when the name is that model (or its alias), never as a fallback.
        key = h.normalise_model(name)
        assert h.MODEL_ALIASES.get(key, key) == s.model.key
    assert accepted < len(names)


def test_bug3_every_rendered_calibration_file_exists(needs_sdk):
    for model in h.MODELS.values():
        s = h.parse({"METROLLA_DRIVER_CONTRACT": "1", "LIDAR_MODEL": model.config, "SENSOR_IP": "192.168.1.201"})
        text, files = h.render(s)
        assert h.CONFIG_SDK_PREFIX not in text
        assert all(f.startswith(h.SDK_DIR + "/correction/") for f in files)
        assert not h.missing_files(files), model.config


def test_bug3_a_missing_firetime_file_fails_loudly(monkeypatch, tmp_path, capsys):
    """The SDK only logs a missing firetime file and runs on; the contract refuses to start."""
    sdk = tmp_path / "sdk"
    angle = sdk / "correction" / "angle_correction"
    angle.mkdir(parents=True)
    (angle / "PandarXT_Angle Correction File.csv").write_text("x")
    monkeypatch.setattr(h, "SDK_DIR", str(sdk))
    assert h.main(["check"], {"METROLLA_DRIVER_CONTRACT": "1", "LIDAR_MODEL": "xt32",
                              "SENSOR_IP": "192.168.1.201"}) == 1
    err = capsys.readouterr().err
    assert "PandarXT-32_Firetime Correction File.csv" in err and "is not in this image" in err
