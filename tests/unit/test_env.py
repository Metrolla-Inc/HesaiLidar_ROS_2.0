"""Unit tier: the environment contract of docker/hesai_env.py (METROLLA_DRIVER_CONTRACT=1)."""

from __future__ import annotations

import json
import os

import pytest
import yaml

import hesai_env as h

BASE = {"METROLLA_DRIVER_CONTRACT": "1", "LIDAR_MODEL": "PandarXT-32", "SENSOR_IP": "192.168.1.201"}


def env(**kw):
    out = dict(BASE)
    for k, v in kw.items():
        if v is None:
            out.pop(k, None)
        else:
            out[k] = v
    return out


def refused(e) -> list[str]:
    with pytest.raises(h.Refused) as exc:
        h.parse(e)
    assert exc.value.problems, "refused without a reason"
    return exc.value.problems


def rendered(e) -> dict:
    s = h.parse(e)
    text, _ = h.render(s)
    return yaml.safe_load(text)["lidar"][0]


# ---------------------------------------------------------------- accepted values and defaults


def test_minimal_environment_defaults():
    s = h.parse(BASE)
    assert (s.model.config, s.sensor_ip, s.sensor_id) == ("PandarXT-32", "192.168.1.201", None)
    assert (s.data_port, s.device_port) == (2368, 9347)
    assert (s.frame_id, s.topic, s.time_mode) == ("hesai_lidar", "lidar_points", "sensor")
    assert s.run_dir == "/tmp/hesai" and s.config_path == "/tmp/hesai/config.yaml"
    assert s.node_fqn == "/hesai_ros_driver_node"


@pytest.mark.parametrize("raw,config", [
    ("PandarXT-32", "PandarXT-32"), ("pandarxt-32", "PandarXT-32"), ("hesai-xt32", "PandarXT-32"),
    ("XT32", "PandarXT-32"), ("pandar_xt_32", "PandarXT-32"), ("xt16", "PandarXT-16"),
    ("hesai-jt128", "JT128"), ("JT128", "JT128"), ("at128p", "AT128E2X"), ("AT128E2X", "AT128E2X"),
    ("at128e3x", "AT128E3X"), ("OT128", "OT128"), ("xt32m2x", "XT32M2X"), ("Pandar90E3X", "Pandar90E3X"),
])
def test_model_names(raw, config):
    assert h.parse(env(LIDAR_MODEL=raw)).model.config == config


def test_every_model_file_is_a_model_and_renders(needs_sdk):
    files = {p[:-5] for p in os.listdir(h.MODEL_CONFIG_DIR) if p.endswith(".yaml")}
    singles = {f for f in files if f != "config" and not f.startswith("dual_") and f != "JT16"}
    assert singles == {m.config for m in h.MODELS.values()}
    for model in h.MODELS.values():
        s = h.parse(env(LIDAR_MODEL=model.config))
        text, cal = h.render(s)
        doc = yaml.safe_load(text)
        assert len(doc["lidar"]) == 1
        assert doc["lidar"][0]["driver"]["source_type"] == 1
        assert not h.missing_files(cal), (model.config, cal)


def test_full_environment_renders_each_key():
    d = rendered(env(LIDAR_MODEL="hesai-jt128", SENSOR_IP="10.1.2.3", SENSOR_ID="east", DATA_PORT="2370",
                     DEVICE_PORT="9400", HOST_IP="10.1.2.100/24", FRAME_ID="hesai_east",
                     POINTCLOUD_TOPIC="/east/points", TIME_MODE="PTP"))
    udp = d["driver"]["lidar_udp_type"]
    assert udp["device_ip_address"] == "10.1.2.3"
    assert udp["udp_port"] == 2370 and udp["ptc_port"] == 9400
    assert udp["host_ip_address"] == "10.1.2.100"
    assert d["driver"]["use_timestamp_type"] == 0
    assert d["ros"]["ros_frame_id"] == "hesai_east"
    assert d["ros"]["ros_send_point_cloud_topic"] == "/east/points"


def test_side_topics_become_relative_so_they_follow_the_namespace():
    ros = rendered(env(SENSOR_ID="east"))["ros"]
    assert ros["ros_send_packet_topic"] == "lidar_packets"
    assert ros["ros_send_imu_topic"] == "lidar_imu"
    assert ros["ros_send_packet_loss_topic"] == "lidar_packets_loss"
    assert ros["ros_send_point_cloud_topic"] == "lidar_points"


@pytest.mark.parametrize("mode,stamp", [("host", 1), ("HOST", 1), ("sensor", 0), ("gps", 0), ("ptp", 0),
                                        ("ntp", 0)])
def test_time_mode(mode, stamp):
    assert rendered(env(TIME_MODE=mode))["driver"]["use_timestamp_type"] == stamp


def test_host_ip_unset_keeps_the_model_files_value():
    assert rendered(BASE)["driver"]["lidar_udp_type"]["host_ip_address"] == "192.168.1.100"


def test_calibration_paths_point_into_the_sdk_that_is_in_the_image(needs_sdk):
    s = h.parse(BASE)
    text, files = h.render(s)
    udp = yaml.safe_load(text)["lidar"][0]["driver"]["lidar_udp_type"]
    assert udp["firetimes_path"].startswith(h.SDK_DIR + "/correction/firetime_correction/")
    assert udp["correction_file_path"].startswith(h.SDK_DIR + "/correction/angle_correction/")
    assert h.CONFIG_SDK_PREFIX not in text.replace(h.SDK_DIR, "<sdk>")
    assert files == [udp["correction_file_path"], udp["firetimes_path"]]
    assert not h.missing_files(files)


def test_image_paths_are_the_layout_of_the_dockerfile():
    """The module's constants are the image's paths (the autouse fixture patches them per test)."""
    import importlib
    fresh = importlib.reload(h)
    assert fresh.SDK_DIR == "/hesai_ws/src/src/driver/HesaiLidar_SDK_2.0"
    assert fresh.PACKAGE_SOURCE == "/hesai_ws/src"
    assert fresh.MODEL_CONFIG_DIR == "/hesai_ws/install/hesai_ros_driver/share/hesai_ros_driver/config"
    assert fresh.DRIVER_NODE == "/hesai_ws/install/hesai_ros_driver/lib/hesai_ros_driver/hesai_ros_driver_node"


def test_dual_lidar_false_values_accepted():
    for v in ("", "0", "false", "FALSE", "no", "off"):
        h.parse(env(DUAL_LIDAR=v))


def test_publish_scan_false_accepted():
    h.parse(env(PUBLISH_SCAN="false"))


def test_metrolla_variables_pass_through():
    h.parse(env(METROLLA_SITE="x", ROS_LOCALHOST_ONLY="1", RMW_IMPLEMENTATION="rmw_fastrtps_cpp", TZ="UTC"))


# ---------------------------------------------------------------- refusals


def test_nothing_set():
    problems = refused({"METROLLA_DRIVER_CONTRACT": "1"})
    assert [p.split(" ")[0] for p in problems] == ["LIDAR_MODEL", "SENSOR_IP"]


@pytest.mark.parametrize("raw,needle", [
    ("PandarXT-31", "is not a Hesai model"), ("", "set but empty"), ("at128", "is ambiguous"),
    ("JT16", "serial"), ("dual_JT128", "two-sensor"), ("config", "is not a model"),
    ("xt32 ", "is not a Hesai model"),
])
def test_unknown_or_unusable_model_is_refused_never_pandarxt16(raw, needle):
    problems = refused(env(LIDAR_MODEL=raw))
    assert any(p.startswith("LIDAR_MODEL") and needle in p for p in problems), problems


@pytest.mark.parametrize("value", ["192.168.1", "192.168.1.256", "192.168.001.1", "0.0.0.0", "224.0.0.1",
                                   "255.255.255.255", "240.0.0.1", "lidar.local", " 192.168.1.201"])
def test_bad_sensor_ip(value):
    assert any(p.startswith(f"SENSOR_IP={h.shown(value)}") for p in refused(env(SENSOR_IP=value)))


@pytest.mark.parametrize("name", ["DATA_PORT", "DEVICE_PORT"])
@pytest.mark.parametrize("value", ["0", "80", "1023", "65536", "02368", "x", "2368.0", "-1"])
def test_bad_ports(name, value):
    problems = refused(env(**{name: value}))
    assert any(p.startswith(f"{name}=") and "1024-65535" in p for p in problems)


def test_host_ip_subnet_and_self():
    assert any("not in the host's subnet" in p for p in refused(env(HOST_IP="10.0.0.1/24")))
    assert any("sensor's own address" in p for p in refused(env(HOST_IP="192.168.1.201")))
    assert any("prefix length" in p for p in refused(env(HOST_IP="192.168.1.100/33")))
    h.parse(env(HOST_IP="10.0.0.1"))  # no prefix: no subnet check


@pytest.mark.parametrize("name,value", [
    ("SENSOR_ID", "East"), ("SENSOR_ID", "1east"), ("SENSOR_ID", "a__b"), ("SENSOR_ID", "a" * 32),
    ("FRAME_ID", "9lidar"), ("FRAME_ID", "hesai/lidar"), ("POINTCLOUD_TOPIC", "/a//b"),
    ("POINTCLOUD_TOPIC", "a b"), ("POINTCLOUD_TOPIC", "/a__b"), ("TIME_MODE", "utc"),
    ("TIME_MODE", "tai"), ("PUBLISH_SCAN", "true"), ("PUBLISH_SCAN", "maybe"), ("DUAL_LIDAR", "true"),
    ("DUAL_LIDAR", "2"), ("HESAI_RUN_DIR", "tmp/hesai"), ("HESAI_RUN_DIR", "/tmp/../etc"),
])
def test_bad_values(name, value):
    assert any(p.startswith(name) for p in refused(env(**{name: value})))


@pytest.mark.parametrize("name", ["SENSOR_ID", "DATA_PORT", "DEVICE_PORT", "HOST_IP", "FRAME_ID",
                                  "POINTCLOUD_TOPIC", "TIME_MODE", "PUBLISH_SCAN", "HESAI_RUN_DIR"])
def test_empty_optional_values_are_refused(name):
    assert f"{name} is set but empty" in refused(env(**{name: ""}))[0]


@pytest.mark.parametrize("name,hint", [
    ("SENSOR_PORT", "use DATA_PORT"), ("POINT_CLOUD_TOPIC", "use POINTCLOUD_TOPIC"),
    ("OVERWRITE_CONFIG", "remove it"), ("CONFIG_PATH", "remove it"), ("DEVICE_IP", "use SENSOR_IP"),
    ("PACKET_TOPIC", "remove it"), ("UDP_PORT", "use DATA_PORT"), ("IMU_TOPIC", "remove it"),
])
def test_legacy_variables_are_refused_in_contract_mode(name, hint):
    problems = refused(env(**{name: "x"}))
    assert any(p.startswith(f"{name} is a legacy variable") and hint in p for p in problems)


@pytest.mark.parametrize("name,hint", [
    ("SENSOR_IPS", "SENSOR_IP"), ("LIDAR_IP", "SENSOR_IP"), ("DATA_PROT", "DATA_PORT"),
    ("HESAI_RUNDIR", "HESAI_RUN_DIR"), ("FRAME", None), ("TIME_MOD", "TIME_MODE"),
    ("SENSOR_MODEL", "LIDAR_MODEL"), ("SENSOR_FRAME_ID", "FRAME_ID"), ("PTC_PORT", "DEVICE_PORT"),
])
def test_typos_are_refused_with_a_hint(name, hint):
    if hint is None:  # FRAME has no strict prefix (FRAME_ has), so it passes through
        h.parse(env(**{name: "x"}))
        return
    problems = refused(env(**{name: "x"}))
    assert any(p.startswith(f"{name} is not a variable") and f"did you mean {hint}?" in p
               for p in problems), problems


@pytest.mark.parametrize("value", ["2", "true", "yes", "on", " 1"])
def test_unknown_contract_version_is_refused(value):
    problems = refused(env(METROLLA_DRIVER_CONTRACT=value))
    assert problems[0].startswith("METROLLA_DRIVER_CONTRACT=")


def test_all_problems_reported_at_once():
    problems = refused({"METROLLA_DRIVER_CONTRACT": "1", "LIDAR_MODEL": "x", "SENSOR_IP": "x",
                        "DATA_PORT": "1", "TIME_MODE": "x", "LIDAR_IPS": "x"})
    assert len(problems) == 5


# ---------------------------------------------------------------- argv, check, run


def test_argv_without_and_with_sensor_id():
    assert h.node_argv(h.parse(BASE)) == [h.DRIVER_NODE, "--ros-args", "-p",
                                          "config_path:=/tmp/hesai/config.yaml"]
    s = h.parse(env(SENSOR_ID="east", HESAI_RUN_DIR="/run/hesai"))
    assert h.node_argv(s) == [h.DRIVER_NODE, "--ros-args", "-r", "__ns:=/east", "-p",
                              "config_path:=/run/hesai/config.yaml"]
    assert s.node_fqn == "/east/hesai_ros_driver_node"


def test_check_prints_config_and_exec(capsys, needs_sdk):
    assert h.main(["check"], BASE) == 0
    out = capsys.readouterr().out
    assert out.splitlines()[-1].startswith("# exec: " + h.DRIVER_NODE)
    assert yaml.safe_load(out)["lidar"][0]["driver"]["lidar_udp_type"]["device_ip_address"] == "192.168.1.201"


def test_check_json(capsys, needs_sdk):
    assert h.main(["check", "--json"], env(SENSOR_ID="east")) == 0
    out = json.loads(capsys.readouterr().out)
    assert out["node"] == "/east/hesai_ros_driver_node"
    assert out["config_path"] == "/tmp/hesai/config.yaml"
    assert out["argv"][-1] == "config_path:=/tmp/hesai/config.yaml"
    assert len(out["calibration_files"]) == 2
    assert out["sources"]["SENSOR_ID"] == "env" and out["sources"]["DATA_PORT"] == "default"


def test_refusal_exit_and_stderr(capsys):
    assert h.main(["check"], env(LIDAR_MODEL="nope")) == 1
    err = capsys.readouterr().err.splitlines()
    assert len(err) == 1 and err[0].startswith("hesai-driver: refusing to start: LIDAR_MODEL='nope'")


def test_unknown_command_refused(capsys):
    assert h.main(["bash"], BASE) == 1
    assert "unknown command" in capsys.readouterr().err
    assert h.main(["ros2", "launch", "hesai_ros_driver", "start.py", "frame_id:=x"], BASE) == 1


def test_missing_calibration_file_refuses_to_start(capsys, monkeypatch, tmp_path):
    monkeypatch.setattr(h, "SDK_DIR", str(tmp_path / "no-sdk"))
    assert h.main(["check"], BASE) == 1
    err = capsys.readouterr().err
    assert "calibration file" in err and "is not in this image" in err


def test_run_writes_the_file_the_node_is_given_and_execs(monkeypatch, tmp_path, needs_sdk):
    calls = []
    monkeypatch.setattr(h.os, "execve", lambda path, argv, envp: calls.append((path, argv, envp)))
    run_dir = tmp_path / "run"
    e = env(HESAI_RUN_DIR=str(run_dir), ROS_HOME=str(tmp_path / "ros"), SENSOR_ID="east")
    assert h.main([], e) == 1  # returns only because execve was replaced
    (path, argv, envp), = calls
    assert path == h.DRIVER_NODE and argv[0] == h.DRIVER_NODE
    written = run_dir / "config.yaml"
    assert argv[-1] == f"config_path:={written}"
    assert written.read_text() == h.render(h.parse(e))[0]
    assert envp["ROS_LOG_DIR"] == str(tmp_path / "ros" / "log")
    assert (tmp_path / "ros" / "log").is_dir()


def test_legacy_default_cmd_means_run(monkeypatch, tmp_path, needs_sdk):
    calls = []
    monkeypatch.setattr(h.os, "execve", lambda *a: calls.append(a))
    h.main(h.LEGACY_DEFAULT_CMD, env(HESAI_RUN_DIR=str(tmp_path), ROS_HOME=str(tmp_path)))
    assert len(calls) == 1


def test_run_unwritable(monkeypatch, capsys, needs_sdk):
    monkeypatch.setattr(h.os, "execve", lambda *a: pytest.fail("must not exec"))
    assert h.main(["run"], env(HESAI_RUN_DIR="/proc/hesai")) == 1
    assert "cannot write" in capsys.readouterr().err


def test_selfcheck_on_the_checkout(needs_sdk):
    assert h.cmd_selfcheck(node="/bin/true") == 0


def test_selfcheck_fails_without_sdk(tmp_path):
    assert h.cmd_selfcheck(sdk_dir=str(tmp_path), node="/bin/true") == 1


def test_selfcheck_fails_without_node(needs_sdk):
    assert h.cmd_selfcheck(node="/nonexistent/hesai_ros_driver_node") == 1


def test_render_refuses_a_model_file_of_another_shape():
    s = h.parse(BASE)
    with pytest.raises(h.BrokenImage):
        h.render_config(s, "lidar:\n  - driver:\n      source_type: 4\n")
    with pytest.raises(h.BrokenImage):
        h.render_config(s, "lidar:\n  - driver:\n      source_type: 1\n")  # keys missing
    two = open(os.path.join(h.MODEL_CONFIG_DIR, "dual_JT128.yaml")).read()
    with pytest.raises(h.BrokenImage):
        h.render_config(s, two)
