"""sensor-tool (docker/sensor_tool.py) against a fake Hesai sensor on loopback: the interface's
rules, each with its test. No real sensor is contacted."""

from __future__ import annotations

import io
import json
import struct
import subprocess
import sys
from pathlib import Path

import pytest

import sensor_tool
from fake_hesai_sensor import FakeHesai

TOOL = Path(__file__).resolve().parents[2] / "docker" / "sensor_tool.py"
A, B = "127.0.50.1", "127.0.50.2"
KEYS = {"sensor_tool", "vendor", "command", "ip", "outcome", "exit_code", "message", "identity", "before", "wanted", "after",
        "changed", "unsupported", "reboot", "verified", "answers_at", "sessions", "warnings"}
NET = ["--new-ip", B, "--netmask", "255.0.0.0", "--gateway", "127.0.0.254"]
DEST = ["--dest-ip", "127.0.0.77", "--data-port", "2369"]
PTP = ["--transport", "L2", "--domain", "3", "--delay-mechanism", "E2E", "--timescale", "TAI", "--profile", "1588v2", "--clock-source", "ptp"]
SETS = {"set-net": NET, "set-dest": DEST, "set-ptp": PTP}


@pytest.fixture(autouse=True)
def quick(monkeypatch):
    monkeypatch.setattr(sensor_tool, "PAUSE", 0.05)
    monkeypatch.setattr(sensor_tool, "LIVE_GRACE", 0.4)
    monkeypatch.setattr(sensor_tool, "ALLOW_LOOPBACK", True)


def run(fake, *argv, expect=None):
    """One tool run against `fake`: (answer, exit status). The answer's shape is checked every time."""
    sensor_tool.PORTS.update(ptc=fake.ptc_port, http=fake.http_port)
    printed = io.StringIO()
    status = sensor_tool.main([str(word) for word in argv] + ["--timeout", "1"], stdout=printed)
    lines = printed.getvalue().splitlines()
    assert len(lines) == 1, lines
    answer = json.loads(lines[0])
    assert set(answer) == KEYS and answer["sensor_tool"] == 1 and answer["vendor"] == "hesai" and answer["command"] == argv[0]
    assert answer["exit_code"] == status == sensor_tool.EXIT[answer["outcome"]]
    assert answer["sessions"]["max_concurrent"] <= 1 and fake.max_open <= 1  # never two PTC sessions at once
    assert answer["sessions"]["ptc"] >= fake.ptc_sessions - getattr(fake, "_seen", 0)
    fake._seen = fake.ptc_sessions
    if expect is not None:
        assert answer["outcome"] == expect, answer
    return answer, status


def writes(fake):
    return [code for code in fake.codes if code not in sensor_tool.PTC_READS], [q for q in fake.queries if "action=get" not in q]


# --- reading ----------------------------------------------------------------------------------------


def test_probe_and_get_net_over_ptc():
    with FakeHesai() as fake:
        answer, status = run(fake, "probe", "--ip", A, "--driver-stopped", expect="reported")
        assert status == 0
        assert answer["identity"] == {"vendor": "hesai", "model": None, "serial": "XT0000000000001", "mac": "ec:9f:0d:a1:28:01", "firmware": "2.1.7"}
        answer, _ = run(fake, "get-net", "--ip", A, "--driver-stopped", expect="reported")
        assert answer["after"] == {
            "net": {"ip": A, "netmask": "255.0.0.0", "gateway": "127.0.0.254"},
            "dest": {"ip": "127.0.0.9", "data_port": 2368, "gps_port": 10110},
            "ptp": {"clock_source": "gps", "profile": "1588v2", "transport": "UDPv4", "domain": 0, "status": "listening"},
            "channel": "ptc"}
        assert answer["sessions"] == {"ptc": 1, "http": 1, "max_concurrent": 1}  # one HTTP try, refused; one PTC session
        assert fake.ptc_sessions == 2 and writes(fake) == ([], [])


def test_the_at128p_inventory_layout():
    with FakeHesai(layout="at128p") as fake:
        answer, _ = run(fake, "probe", "--ip", A, "--driver-stopped", "--channel", "ptc", expect="reported")
        assert answer["identity"]["serial"] == "XT0000000000001" and answer["identity"]["mac"] == "ec:9f:0d:a1:28:01"
        assert answer["sessions"]["http"] == 0  # --channel ptc: HTTP is not even tried


def test_a_ptc_read_needs_the_driver_stopped():
    with FakeHesai() as fake:
        for command in ("probe", "get-net"):
            answer, status = run(fake, command, "--ip", A, expect="refused")
            assert status == 1 and "--driver-stopped" in answer["message"]
        assert fake.ptc_sessions == 0 and fake.codes == []


def test_no_answer():
    with FakeHesai() as fake:
        answer, status = run(fake, "probe", "--ip", B, "--driver-stopped", expect="no-answer")  # the sensor is at A
        assert status == 1 and B in answer["message"]
        fake.alive = False
        run(fake, "get-net", "--ip", A, "--driver-stopped", expect="no-answer")


def test_reads_and_dry_runs_send_no_write():
    """Read-only code cannot write: over every read and every dry run, on both kinds of sensor,
    only read codes and `action=get` reach the sensor."""
    for http in (False, True):
        with FakeHesai(http=http) as fake:
            for flags in ((), ("--driver-stopped",)):
                run(fake, "probe", "--ip", A, *flags)
                run(fake, "get-net", "--ip", A, *flags)
                for command, values in SETS.items():
                    answer, status = run(fake, command, "--ip", A, *values, "--dry-run", *flags)
                    assert answer["outcome"] in ("changes", "refused"), answer
            assert writes(fake) == ([], [])
            assert set(fake.codes) <= set(sensor_tool.PTC_READS)
            assert (fake.ip, fake.dest_ip, fake.clock_source, fake.network) == (A, "127.0.0.9", 0, 0)


def test_the_read_functions_refuse_to_write():
    with FakeHesai(http=True) as fake:
        sensor_tool.PORTS.update(ptc=fake.ptc_port, http=fake.http_port)
        args = sensor_tool.parse_arguments(["get-net", "--ip", A, "--driver-stopped"], sensor_tool._extra)
        tool_run = sensor_tool.Run(args, sensor_tool.blank("hesai"), sensor_tool.Sessions(("ptc",), ("http",)))
        with sensor_tool.Ptc(tool_run, A) as session:
            for code in list(sensor_tool.PTC_WRITES) + list(sensor_tool.PTC_NEVER) + [0x99]:
                with pytest.raises(ValueError):
                    session.read(code)
            for code in sensor_tool.PTC_NEVER:  # the factory reset and the firmware upgrade among them
                with pytest.raises(ValueError):
                    session._exchange(code, b"")
        with pytest.raises(ValueError):
            sensor_tool.http_get(tool_run, A, "lidar&key=clock_source&value=1")
        with pytest.raises(sensor_tool.Stop):  # a run that reads cannot make a Writer
            sensor_tool.Writer(tool_run)
        assert fake.codes == [] and fake.queries == []
        assert {0x25, 0x83} <= set(sensor_tool.PTC_NEVER) and not set(sensor_tool.PTC_NEVER) & (set(sensor_tool.PTC_READS) | set(sensor_tool.PTC_WRITES))


# --- writing ----------------------------------------------------------------------------------------


@pytest.mark.parametrize("http", [False, True])
def test_every_write_needs_the_driver_stopped(http):
    with FakeHesai(http=http) as fake:
        for command, values in SETS.items():
            answer, status = run(fake, command, "--ip", A, *values, expect="refused")
            assert status == 1 and "--driver-stopped" in answer["message"]
        assert fake.ptc_sessions == 0 and fake.codes == [] and fake.queries == []  # nothing was sent at all


def test_set_dest_changes_only_the_destination_and_is_idempotent():
    with FakeHesai() as fake:
        answer, status = run(fake, "set-dest", "--ip", A, *DEST, "--dry-run", "--driver-stopped", expect="changes")
        assert status == 2 and answer["changed"] == ["dest.ip", "dest.data_port"] and fake.dest_ip == "127.0.0.9"
        answer, status = run(fake, "set-dest", "--ip", A, *DEST, "--driver-stopped", expect="set")
        assert status == 0 and answer["verified"] is True and answer["answers_at"] == A and answer["reboot"] == "none"
        assert answer["before"]["dest"]["ip"] == "127.0.0.9" and answer["after"]["dest"] == {"ip": "127.0.0.77", "data_port": 2369, "gps_port": 10110}
        assert fake.payloads[0x20] == [bytes([127, 0, 0, 77]) + struct.pack(">HH", 2369, 10110)]  # the GPS port is kept
        assert answer["sessions"]["ptc"] == 1
        answer, status = run(fake, "set-dest", "--ip", A, *DEST, "--driver-stopped", expect="unchanged")
        assert status == 0 and answer["changed"] == [] and fake.codes.count(0x20) == 1  # nothing sent the second time


def test_set_net_taken_live():
    with FakeHesai() as fake:
        answer, status = run(fake, "set-net", "--ip", A, *NET, "--verify-timeout", "5", "--driver-stopped", expect="set")
        assert status == 0 and answer["changed"] == ["net.ip"] and answer["answers_at"] == B and answer["reboot"] == "none"
        assert answer["after"]["net"] == {"ip": B, "netmask": "255.0.0.0", "gateway": "127.0.0.254"} and answer["verified"] is True
        assert fake.payloads[0x21] == [bytes([127, 0, 50, 2, 255, 0, 0, 0, 127, 0, 0, 254]) + b"\x01\x00\x07"]  # the VLAN is kept
        assert 0x10 not in fake.codes
        run(fake, "set-net", "--ip", B, *NET, "--driver-stopped", expect="unchanged")
        assert fake.codes.count(0x21) == 1


def test_set_net_restarts_a_sensor_that_does_not_take_the_address_live():
    with FakeHesai(live=False) as fake:
        answer, _ = run(fake, "set-net", "--ip", A, *NET, "--verify-timeout", "5", "--driver-stopped", expect="set")
        assert answer["reboot"] == "sent" and answer["answers_at"] == B and fake.codes.count(0x10) == 1 and fake.ip == B


def test_set_net_without_the_restart_is_unverified():
    with FakeHesai(live=False) as fake:
        answer, status = run(fake, "set-net", "--ip", A, *NET, "--no-reboot", "--verify-timeout", "5", "--driver-stopped", expect="unverified")
        assert status == 5 and answer["reboot"] == "needed" and answer["answers_at"] == A and answer["verified"] is False
        assert 0x10 not in fake.codes


def test_set_net_of_the_netmask_alone_stays_in_one_session():
    with FakeHesai() as fake:
        answer, _ = run(fake, "set-net", "--ip", A, "--new-ip", A, "--netmask", "255.255.0.0", "--gateway", "127.0.0.254",
                        "--driver-stopped", expect="set")
        assert answer["changed"] == ["net.netmask"] and answer["sessions"]["ptc"] == 1 and fake.netmask == "255.255.0.0"


def test_a_sensor_that_vanishes_after_the_change_is_unverified():
    with FakeHesai(vanish=0x21) as fake:
        answer, status = run(fake, "set-net", "--ip", A, *NET, "--verify-timeout", "1", "--driver-stopped", expect="unverified")
        assert status == 5 and answer["answers_at"] is None and "neither address" in answer["message"]


def test_another_sensor_at_the_new_address_fails():
    with FakeHesai() as fake:
        original = fake._inventory

        def other():
            return original() if fake.ip == A else original().replace(b"XT0000000000001", b"XT9999999999999")
        fake._inventory = other
        answer, status = run(fake, "set-net", "--ip", A, *NET, "--verify-timeout", "3", "--driver-stopped", expect="failed")
        assert status == 1 and "another sensor" in answer["message"]


def test_an_address_the_host_could_not_reach_is_refused_before_anything_is_sent():
    with FakeHesai() as fake:
        for values in (["--new-ip", "10.99.99.9", "--netmask", "255.255.255.0", "--gateway", "0.0.0.0"],  # no host address in that network
                       ["--new-ip", "127.255.255.255", "--netmask", "255.0.0.0", "--gateway", "0.0.0.0"],  # the broadcast address
                       ["--new-ip", B, "--netmask", "255.0.0.0", "--gateway", "10.0.0.1"]):  # a gateway outside the network
            answer, status = run(fake, "set-net", "--ip", A, *values, "--driver-stopped", expect="refused")
            assert status == 1
            run(fake, "set-net", "--ip", A, *values, "--dry-run", "--driver-stopped", expect="refused")
        assert writes(fake) == ([], []) and fake.ip == A


def test_a_refused_write_is_failed():
    with FakeHesai(refuse={0x21: 1}) as fake:
        answer, status = run(fake, "set-net", "--ip", A, *NET, "--driver-stopped", expect="failed")
        assert status == 1 and "0x21" in answer["message"] and "invalid input parameter" in answer["message"]


def test_a_write_the_sensor_does_not_take_is_unverified():
    with FakeHesai(deaf=(0x20,)) as fake:
        answer, status = run(fake, "set-dest", "--ip", A, *DEST, "--verify-timeout", "0.3", "--driver-stopped", expect="unverified")
        assert status == 5 and answer["answers_at"] == A and answer["verified"] is False and "dest.ip" in answer["message"]


@pytest.mark.parametrize("extended", [False, True])
def test_set_ptp_over_ptc(extended):
    with FakeHesai(extended_ptp=extended) as fake:
        fake.intervals = (0, -3, -1)
        answer, status = run(fake, "set-ptp", "--ip", A, *PTP, "--driver-stopped", expect="set")
        assert status == 0 and answer["changed"] == ["ptp.clock_source", "ptp.transport", "ptp.domain"] and answer["unsupported"] == []
        assert answer["before"]["ptp"]["clock_source"] == "gps"  # the clock source before, for the caller's record
        assert answer["after"]["ptp"] == {"clock_source": "ptp", "profile": "1588v2", "transport": "L2", "domain": 3, "status": "slave"}
        assert fake.payloads[0x1F] == [b"\x01"]
        # profile 0, domain 3, network 1 (L2), the sensor's own intervals kept, the reserved byte where the sensor has one
        assert fake.payloads[0x24] == [struct.pack(">BBBbbb", 0, 3, 1, 0, -3, -1) + (b"\0" if extended else b"")]
        assert answer["sessions"]["ptc"] == 1
        run(fake, "set-ptp", "--ip", A, *PTP, "--driver-stopped", expect="unchanged")
        assert fake.codes.count(0x24) == 1 and fake.codes.count(0x1F) == 1


def test_peer_to_peer_delay_is_said_and_the_rest_is_set():
    with FakeHesai() as fake:
        values = [word if word != "E2E" else "P2P" for word in PTP]
        answer, status = run(fake, "set-ptp", "--ip", A, *values, "--driver-stopped", expect="set")
        assert status == 0 and [u["setting"] for u in answer["unsupported"]] == ["ptp.delay_mechanism"] and fake.clock_source == 1


def test_a_sensor_without_a_readable_ptp_configuration_gets_the_clock_source_only():
    with FakeHesai(no_ptp_config=True) as fake:
        answer, _ = run(fake, "set-ptp", "--ip", A, *PTP, "--driver-stopped", expect="set")
        assert answer["changed"] == ["ptp.clock_source"] and [u["setting"] for u in answer["unsupported"]] == ["ptp.transport", "ptp.domain"]
        assert 0x24 not in fake.codes and answer["warnings"]


# --- the XT32 class: HTTP first ------------------------------------------------------------------------


def test_an_http_sensor_is_read_over_http_alone_with_the_driver_running():
    with FakeHesai(http=True) as fake:
        answer, _ = run(fake, "probe", "--ip", A, expect="reported")
        assert answer["identity"] == {"vendor": "hesai", "model": "XT32M2X", "serial": "XT0000000000001", "mac": "ec:9f:0d:a1:28:01", "firmware": "2.1.7"}
        answer, _ = run(fake, "get-net", "--ip", A, expect="reported")
        state = answer["after"]
        assert state["channel"] == "http" and state["net"] == {"ip": A, "netmask": None, "gateway": None}
        assert state["dest"] == {"ip": "127.0.0.9", "data_port": 2368, "gps_port": 10110}
        assert state["ptp"]["clock_source"] == "gps" and state["ptp"]["status"] == "Free Run" and state["ptp"]["transport"] == "UDPv4"
        assert "ClockSource" in state["http_keys"] and any("net.netmask" in w for w in answer["warnings"])
        for command in ("set-dest", "set-ptp"):  # their dry runs need no PTC session either
            run(fake, command, "--ip", A, *SETS[command], "--dry-run", expect="changes")
        assert fake.ptc_sessions == 0 and answer["sessions"]["ptc"] == 0
        assert fake.queries and all(query.startswith("action=get&object=") for query in fake.queries)


def test_an_http_sensor_with_the_driver_stopped_gets_one_ptc_session_for_netmask_and_gateway():
    with FakeHesai(http=True) as fake:
        answer, _ = run(fake, "get-net", "--ip", A, "--driver-stopped", expect="reported")
        assert answer["after"]["channel"] == "http+ptc" and answer["after"]["net"] == {"ip": A, "netmask": "255.0.0.0", "gateway": "127.0.0.254"}
        assert fake.ptc_sessions == 1 and fake.codes == [0x08]
        run(fake, "get-net", "--ip", A, "--driver-stopped", "--channel", "http", expect="reported")
        assert fake.ptc_sessions == 1  # --channel http: never a PTC session


def test_set_ptp_on_an_http_sensor_is_http_only():
    with FakeHesai(http=True) as fake:
        answer, status = run(fake, "set-ptp", "--ip", A, *PTP, "--driver-stopped", expect="set")
        assert status == 0 and answer["changed"] == ["ptp.clock_source", "ptp.transport", "ptp.domain"]
        assert answer["before"]["ptp"]["clock_source"] == "gps" and answer["after"]["ptp"]["clock_source"] == "ptp"
        assert fake.ptc_sessions == 0 and fake.codes == []
        assert [q for q in fake.queries if "action=set" in q] == [
            "action=set&object=lidar&key=clock_source&value=1",
            "action=set&object=lidar&key=ptp_configuration&value={%22Profile%22:0,%22Domain%22:3,%22Network%22:1,"
            "%22LogAnnounceInterval%22:1,%22LogSyncInterval%22:1,%22LogMinDelayReqInterval%22:0,%22tsn_switch%22:0}"]
        assert (fake.clock_source, fake.domain, fake.network) == (1, 3, 1)
        run(fake, "set-ptp", "--ip", A, *PTP, "--driver-stopped", expect="unchanged")
        assert len([q for q in fake.queries if "action=set" in q]) == 2


def test_an_http_sensor_that_does_not_show_its_ptp_configuration_gets_the_clock_source_only():
    with FakeHesai(http=True, http_ptp=False) as fake:
        answer, _ = run(fake, "set-ptp", "--ip", A, *PTP, "--driver-stopped", expect="set")
        assert answer["changed"] == ["ptp.clock_source"] and [u["setting"] for u in answer["unsupported"]] == ["ptp.transport", "ptp.domain"]
        assert [q for q in fake.queries if "action=set" in q] == ["action=set&object=lidar&key=clock_source&value=1"]


def test_an_http_write_the_sensor_does_not_take_is_unverified():
    with FakeHesai(http=True, deaf=("http:clock_source",)) as fake:
        fake.network, fake.domain = 1, 3
        answer, status = run(fake, "set-ptp", "--ip", A, *PTP, "--verify-timeout", "0.3", "--driver-stopped", expect="unverified")
        assert status == 5 and answer["answers_at"] == A


def test_set_dest_on_an_http_sensor_uses_one_ptc_session():
    with FakeHesai(http=True) as fake:
        answer, _ = run(fake, "set-dest", "--ip", A, *DEST, "--driver-stopped", expect="set")
        assert fake.ptc_sessions == 1 and answer["sessions"]["ptc"] == 1 and fake.dest_ip == "127.0.0.77"
        assert not [q for q in fake.queries if "action=set" in q]  # no HTTP key for the destination is known: none is tried
        run(fake, "set-dest", "--ip", A, *DEST, "--driver-stopped", expect="unchanged")
        assert fake.ptc_sessions == 1  # already right by HTTP: no PTC session at all


def test_set_dest_dry_run_needs_ptc_when_http_does_not_show_the_destination():
    with FakeHesai(http=True, http_dest=False) as fake:
        run(fake, "set-dest", "--ip", A, *DEST, "--dry-run", expect="refused")
        run(fake, "set-dest", "--ip", A, *DEST, "--dry-run", "--driver-stopped", expect="changes")
        assert fake.ptc_sessions == 1


@pytest.mark.parametrize("live", [True, False])
def test_set_net_on_an_http_sensor_uses_one_ptc_session(live):
    with FakeHesai(http=True, live=live) as fake:
        answer, status = run(fake, "set-net", "--ip", A, *NET, "--verify-timeout", "5", "--driver-stopped", expect="set")
        assert status == 0 and answer["answers_at"] == B and fake.ip == B
        assert fake.ptc_sessions == 1 and answer["sessions"]["ptc"] == 1  # also when it had to be restarted
        assert answer["reboot"] == ("none" if live else "sent")
        assert not [q for q in fake.queries if "action=set" in q]
        assert answer["after"]["net"]["ip"] == B and answer["after"]["channel"] == "http+ptc"


def test_channel_http_refuses_what_needs_ptc():
    with FakeHesai(http=True) as fake:
        for command in ("set-net", "set-dest"):
            answer, _ = run(fake, command, "--ip", A, *SETS[command], "--driver-stopped", "--channel", "http", expect="refused")
        assert fake.ptc_sessions == 0


# --- the program ------------------------------------------------------------------------------------


def test_it_prints_one_json_object_also_for_a_usage_error():
    for argv in (["set-net", "--ip", "1.2.3.4"], ["probe", "--ip", "not-an-address"], ["format-disk"], [],
                 ["set-ptp", "--ip", "1.2.3.4", "--transport", "carrier-pigeon"], ["probe", "--ip", "1.2.3.4", "--help"]):
        done = subprocess.run([sys.executable, "-I", str(TOOL), *argv], capture_output=True, text=True, timeout=30)
        answer = json.loads(done.stdout)
        assert done.returncode == 1 and answer["outcome"] == "refused" and answer["exit_code"] == 1 and set(answer) == KEYS
        assert len(done.stdout.splitlines()) == 1 and answer["sessions"]["ptc"] == 0


def test_a_fault_of_the_tool_is_one_json_object_too(monkeypatch):
    with FakeHesai() as fake:
        monkeypatch.setitem(sensor_tool.HANDLERS, "probe", lambda _run: 1 / 0)
        answer, status = run(fake, "probe", "--ip", A, expect="failed")
        assert status == 1 and "ZeroDivisionError" in answer["message"]


def test_a_second_control_connection_is_refused_by_the_tool_itself():
    sessions = sensor_tool.Sessions(("ptc",), ("http",))
    sessions.opening("ptc")
    sessions.opening("http")  # HTTP is not a control session
    with pytest.raises(RuntimeError):
        sessions.opening("ptc")
    sessions.closed("ptc")
    sessions.opening("ptc")
    assert sessions.report() == {"ptc": 2, "http": 1, "max_concurrent": 1}
