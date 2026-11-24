#!/usr/bin/python3 -I
"""sensor-tool for Hesai lidars: reads a sensor's own settings and sets them. Interface version 1.

    sensor-tool probe    --ip A [--model M] [--timeout S] [--driver-stopped]
    sensor-tool get-net  --ip A [--model M] [--timeout S] [--driver-stopped]
    sensor-tool set-net  --ip A --new-ip B --netmask M --gateway G [--no-reboot] [--dry-run] [--verify-timeout S] --driver-stopped
    sensor-tool set-dest --ip A --dest-ip H --data-port P [--dry-run] [--verify-timeout S] --driver-stopped
    sensor-tool set-ptp  --ip A --transport L2|UDPv4 --domain N --delay-mechanism E2E|P2P --timescale UTC|TAI
                         [--profile 1588v2] [--clock-source ptp] [--dry-run] [--verify-timeout S] --driver-stopped

It prints exactly one JSON object, also for a usage error, and exits 0 (reported, set, unchanged),
2 (changes: a dry run that would change the sensor), 1 (refused, failed, no-answer), 3 (unsupported)
or 5 (unverified: a change was sent and did not read back in time). Standard library only.

The rules it keeps (tests/unit/test_sensor_tool.py has a test for each):

- Reading and writing are separate code. `Ptc.read` sends only the codes of PTC_READS and
  `http_get` sends only `action=get`; the codes of PTC_WRITES and `action=set` go through
  `Writer`, which exists only in a run that was told `--driver-stopped` and is not a dry run.
  The codes of PTC_NEVER (factory reset, firmware upgrade, registers) cannot be sent at all.
- Every write needs `--driver-stopped`, and so does every PTC session: the driver holds one, and
  extra PTC sessions to an XT32 dropped its Ethernet link in the fleet.
- HTTP first. A sensor that answers `pandar.cgi` (the XT32 class) is read over HTTP alone, with
  the driver running; `set-ptp` on it is HTTP alone. PTC is used only for what HTTP does not
  give: at most one PTC session at a time, and on an HTTP sensor at most one per run.
- A set command reads first, changes only what differs, reads back, and is idempotent.
- What cannot be set is said in `unsupported`, never skipped silently.

Where each fact comes from:

- PTC framing (0x47 0x74, command, return code, big-endian payload length) and port 9347:
  the vendor SDK in this repository, src/driver/HesaiLidar_SDK_2.0/libhesai/PtcParser/include/
  ptc_1_0_parser.h (PTCHeader_1_0) and libhesai/PtcClient/include/ptc_client.h (kTcpPort).
- PTC 0x06, 0x07, 0x09, 0x3a, 0x10, 0x20, 0x21 and the payloads of 0x20 and 0x21: the same SDK,
  libhesai/PtcClient/include/ptc_client.h (kPTC...) and src/ptc_client.cc (SetNet,
  SetDesIpandPort, RebootLidar, GetPTPDiagnostics).
- PTC 0x08 (the configuration and its layout), 0x1f (clock source), 0x24 and 0x26 (PTP
  configuration), the layout of the inventory (0x07), the return codes, and the value of the HTTP
  key `ptp_configuration`: tier4/nebula (Apache-2.0), nebula_hesai_hw_interfaces, files
  hesai_hw_interface.hpp, hesai_hw_interface.cpp and hesai_cmd_response.hpp, read at commit
  a152b0a. Cited as a source of facts; no code is copied from it.
- `clock_source` over HTTP (0 GPS, 1 PTP), `action=get&object=lidar_config` with ClockSource,
  PTPStatus, PTPProfile and PTPConfig, the encoding of a `ptp_configuration` write, and the rule
  about PTC sessions on an XT32: the fleet's PTP script (run_ptp4l.sh, "Hesai normalisation").
- `action=get&object=device_info` (SN, Mac, Model, "SW Ver", "FW Ver"): Hesai's developer guide,
  hesaitechnology.github.io/dev/docs/how_to_guides/HTTP_api.
- No HTTP key for a sensor's address or its destination is in any of these sources, so `set-net`
  and `set-dest` use PTC, also on an XT32. A guessed key is never tried.

Not possible, or not known: peer-to-peer delay (said as unsupported); the timescale is the
grandmaster's and is not a sensor setting. The netmask and the gateway are read over PTC only.
What `lidar_config` holds beyond the fleet script's four keys has no first-hand source here (see
`http_state`): the answer lists the keys the sensor gave (`http_keys`), and what it did not give
is read over PTC or said to be unreadable. None of this has been run against a real sensor.
"""

from __future__ import annotations

import argparse
import ipaddress
import json
import socket
import struct
import sys
import time
import urllib.error
import urllib.parse
import urllib.request

VENDOR = "hesai"

# ---- the interface: this part is the same text in every driver repository's sensor-tool ----------

INTERFACE = 1
COMMANDS = ("probe", "get-net", "set-net", "set-dest", "set-ptp")
STEP = {"set-net": "net", "set-dest": "dest", "set-ptp": "ptp"}
EXIT = {"reported": 0, "set": 0, "unchanged": 0, "changes": 2, "refused": 1, "failed": 1, "no-answer": 1, "unsupported": 3, "unverified": 5}
PAUSE = 1.0  # seconds between two reads while a change is awaited; tests shorten it


class Stop(Exception):
    """The run ends with this outcome and message."""

    def __init__(self, outcome: str, message: str) -> None:
        super().__init__(message)
        self.outcome, self.message = outcome, message


class NoAnswer(Exception):
    """Nothing answered, or what answered is not this vendor's sensor."""


class Sessions:
    """Counts the connections of a run. A second control connection while one is open is a fault
    of this program, and is refused here before it is opened."""

    def __init__(self, control: tuple, others: tuple = ()) -> None:
        self.control, self.open_now, self.max_concurrent = control, 0, 0
        self.counts = {kind: 0 for kind in (*control, *others)}

    def opening(self, kind: str) -> None:
        if kind in self.control:
            if self.open_now:
                raise RuntimeError(f"a second control connection ({kind}) while one is open")
            self.open_now += 1
            self.max_concurrent = max(self.max_concurrent, self.open_now)
        self.counts[kind] += 1

    def not_opened(self, kind: str) -> None:
        """The connection just announced was never made."""
        self.counts[kind] -= 1
        self.closed(kind)

    def closed(self, kind: str) -> None:
        if kind in self.control:
            self.open_now -= 1

    def report(self) -> dict:
        return {**self.counts, "max_concurrent": self.max_concurrent}


class _Parser(argparse.ArgumentParser):
    def error(self, message: str) -> None:  # argparse would print usage and exit 2
        raise Stop("refused", f"usage: {message}")


def ipv4(text: str) -> str:
    try:
        return str(ipaddress.IPv4Address(text))
    except ValueError:
        raise argparse.ArgumentTypeError(f"{text!r} is not an IPv4 address") from None


def netmask(text: str) -> str:
    text = ipv4(text)
    bits = int(ipaddress.IPv4Address(text))
    if bits == 0 or (~bits & 0xFFFFFFFF) & ((~bits & 0xFFFFFFFF) + 1):
        raise argparse.ArgumentTypeError(f"{text} is not a netmask")
    return text


def number(low: int, high: int):
    def parse(text: str) -> int:
        if not text.isascii() or not text.isdigit() or not low <= int(text) <= high:
            raise argparse.ArgumentTypeError(f"{text!r} is not a number from {low} to {high}")
        return int(text)
    return parse


def seconds(text: str) -> float:
    try:
        value = float(text)
    except ValueError:
        value = -1.0
    if not 0 <= value <= 3600:
        raise argparse.ArgumentTypeError(f"{text!r} is not a number of seconds from 0 to 3600")
    return value


def parse_arguments(argv: list, extra=None) -> argparse.Namespace:
    """The arguments of interface version 1. `extra(parser, command)` adds a vendor's own options."""
    parser = _Parser(prog="sensor-tool", add_help=False, allow_abbrev=False)
    commands = parser.add_subparsers(dest="command", parser_class=_Parser, required=True)
    for name in COMMANDS:
        sub = commands.add_parser(name, add_help=False, allow_abbrev=False)
        sub.add_argument("--ip", type=ipv4, required=True)
        sub.add_argument("--model", default="")
        sub.add_argument("--dev-port", type=number(1, 65535), default=None)
        sub.add_argument("--timeout", type=seconds, default=5.0)
        sub.add_argument("--driver-stopped", action="store_true")
        if name == "set-net":
            sub.add_argument("--new-ip", type=ipv4, required=True)
            sub.add_argument("--netmask", type=netmask, required=True)
            sub.add_argument("--gateway", type=ipv4, required=True)
            sub.add_argument("--no-reboot", action="store_true")
        elif name == "set-dest":
            sub.add_argument("--dest-ip", type=ipv4, required=True)
            sub.add_argument("--data-port", type=number(1, 65535), required=True)
        elif name == "set-ptp":
            sub.add_argument("--transport", choices=("L2", "UDPv4"), required=True)
            sub.add_argument("--domain", type=number(0, 127), required=True)
            sub.add_argument("--delay-mechanism", choices=("E2E", "P2P"), required=True)
            sub.add_argument("--timescale", choices=("UTC", "TAI"), required=True)
            sub.add_argument("--profile", choices=("1588v2",), default="1588v2")
            sub.add_argument("--clock-source", choices=("ptp",), default="ptp")
        if name in STEP:
            sub.add_argument("--dry-run", action="store_true")
            sub.add_argument("--verify-timeout", type=seconds, default=60.0)
        if extra is not None:
            extra(sub, name)
    args = parser.parse_args(argv)
    args.dry_run = bool(getattr(args, "dry_run", False))
    args.writes = args.command in STEP and not args.dry_run
    return args


def blank(vendor: str) -> dict:
    return {"sensor_tool": INTERFACE, "vendor": vendor, "command": None, "ip": None, "outcome": None, "exit_code": None,
            "message": "", "identity": None, "before": None, "wanted": None, "after": None, "changed": [], "unsupported": [],
            "reboot": "none", "verified": None, "answers_at": None, "sessions": None, "warnings": []}


def blank_state() -> dict:
    return {"net": {"ip": None, "netmask": None, "gateway": None}, "dest": {"ip": None, "data_port": None},
            "ptp": {"clock_source": None, "profile": None, "transport": None, "domain": None, "status": None}, "channel": None}


def blank_identity(vendor: str) -> dict:
    return {"vendor": vendor, "model": None, "serial": None, "mac": None, "firmware": None}


def source_address(ip: str):
    """The address of this host that a packet to `ip` would leave from, or None. Sends nothing."""
    probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        probe.connect((ip, 9))
        return probe.getsockname()[0]
    except OSError:
        return None
    finally:
        probe.close()


def check_new_address(new_ip: str, mask: str, gateway: str) -> None:
    """Refuse an address that is no host address of its network, a gateway outside it, and an address
    this host could not reach afterwards: it has no address of its own in that network."""
    network = ipaddress.IPv4Network(f"{new_ip}/{mask}", strict=False)
    address = ipaddress.IPv4Address(new_ip)
    if address.is_multicast or address.is_unspecified or address.is_loopback and not ALLOW_LOOPBACK:
        raise Stop("refused", f"{new_ip} is not an address a sensor can have")
    if network.prefixlen < 31 and address in (network.network_address, network.broadcast_address):
        raise Stop("refused", f"{new_ip} is the network or broadcast address of {network}")
    if gateway != "0.0.0.0" and (ipaddress.IPv4Address(gateway) not in network or gateway == new_ip):
        raise Stop("refused", f"the gateway {gateway} is not another address of {network}")
    source = source_address(new_ip)
    if source is None or ipaddress.IPv4Address(source) not in network:
        raise Stop("refused", f"this host has no address in {network}, so the sensor could not be reached at {new_ip} afterwards; nothing was sent")
    if source == new_ip and not ALLOW_LOOPBACK:
        raise Stop("refused", f"{new_ip} is this host's own address")


ALLOW_LOOPBACK = False  # tests set it: their fake sensors are on loopback addresses


def differing(step: str, wanted: dict, have: dict) -> list:
    """The settings of `wanted` that `have` does not already have, as "step.key"."""
    return [f"{step}.{key}" for key, value in wanted.items() if (have or {}).get(key) != value]


def await_state(deadline: float, read, good):
    """Read until `good(state)` or the deadline: (the last state read or None, whether it was good).
    `read` returns None when nothing answered."""
    last = None
    while True:
        state = read()
        if state is not None:
            last = state
            if good(state):
                return state, True
        if time.monotonic() + PAUSE > deadline:
            return last, False
        time.sleep(PAUSE)


class Run:
    """One run of the tool: its arguments, its answer and its connections."""

    def __init__(self, args: argparse.Namespace, out: dict, sessions: Sessions) -> None:
        self.args, self.out, self.sessions = args, out, sessions

    def warn(self, text: str) -> None:
        if text not in self.out["warnings"]:
            self.out["warnings"].append(text)

    def cannot(self, setting: str, reason: str) -> None:
        self.out["unsupported"].append({"setting": setting, "reason": reason})

    def need_stopped(self, what: str) -> None:
        if not self.args.driver_stopped:
            raise Stop("refused", f"{what} needs --driver-stopped: the driver's own connection to the sensor must be closed first; nothing was sent")


def run_tool(argv, vendor: str, sessions: Sessions, handlers: dict, extra=None, stdout=None) -> int:
    """Parse, run one command, print the one JSON object, return the exit status."""
    out = blank(vendor)
    words = list(sys.argv[1:] if argv is None else argv)
    if words and words[0] in COMMANDS:
        out["command"] = words[0]
    try:
        args = parse_arguments(words, extra)
        out["ip"] = args.ip
        if args.writes and not args.driver_stopped:
            raise Stop("refused", f"{args.command} writes to the sensor and needs --driver-stopped; nothing was sent")
        handlers[args.command](Run(args, out, sessions))
        raise RuntimeError("the command ended without an outcome")
    except Stop as stop:
        out["outcome"], out["message"] = stop.outcome, stop.message
    except NoAnswer as exc:  # after a change was sent, silence is "not read back", not "nothing there"
        out["outcome"], out["message"] = "unverified" if out.get("_wrote") else "no-answer", str(exc)
    except Exception as exc:  # noqa: BLE001 - the answer is always one JSON object
        out["outcome"] = "unverified" if out.get("_wrote") else "failed"
        out["message"] = f"{type(exc).__name__}: {exc}"
    out.pop("_wrote", None)
    out["exit_code"], out["sessions"] = EXIT[out["outcome"]], sessions.report()
    print(json.dumps(out), file=stdout or sys.stdout, flush=True)
    return out["exit_code"]


# ---- Hesai: what is read and what is written ------------------------------------------------------

PORTS = {"ptc": 9347, "http": 80}  # ptc_client.h kTcpPort; the sensor's web server
LIVE_GRACE = 15.0  # seconds a sensor gets to take a new address by itself before it is restarted

# PTC codes that only read. Sources in the module text.
PTC_READS = {0x06: "PTP diagnostics", 0x07: "inventory", 0x08: "configuration", 0x09: "status", 0x26: "PTP configuration",
             0x3A: "PTP lock offset"}
# PTC codes that change the sensor: sent only by Writer.
PTC_WRITES = {0x10: "restart", 0x1F: "clock source", 0x20: "destination", 0x21: "address", 0x24: "PTP configuration"}
# Codes this tool can never send, whoever asks: ptc_client.h (kPTCUpgradeLidar 0x83, kPTCSetFpgaRegister
# 0x0D, kPTCSetSpinSpeed 0x17, kPTCSetSyncAngle 0x18, kPTCSetStandbyMode 0x1C, kPTCSetReturnMode 0x1E)
# and nebula's hesai_hw_interface.hpp (reset 0x25, trigger method 0x1b, range 0x22, lock offset 0x39,
# rotation 0x2a, high resolution 0x29, blockage 0x58).
PTC_NEVER = {0x25: "factory reset", 0x83: "firmware upgrade", 0x0D: "FPGA register", 0x17: "spin rate", 0x18: "sync angle",
             0x1B: "trigger method", 0x1C: "standby", 0x1E: "return mode", 0x22: "range", 0x29: "high resolution",
             0x2A: "rotation", 0x39: "PTP lock offset", 0x58: "blockage detection"}
# nebula hesai_hw_interface.hpp, g_ptc_error_code_*
PTC_ERRORS = {1: "invalid input parameter", 2: "server connection failed", 3: "invalid data", 4: "out of memory",
              5: "unsupported command", 6: "FPGA communication failed", 7: "other"}
# nebula hesai_cmd_response.hpp, get_str_network: 0 UDP/IP, 1 L2; get_str_profile: 0 IEEE 1588v2
NETWORK = {0: "UDPv4", 1: "L2"}
PROFILE = {0: "1588v2", 1: "802.1AS", 2: "802.1AS automotive", 3: "802.1AS AUTOSAR"}
CLOCK = {0: "gps", 1: "ptp"}  # nebula g_hesai_lidar_gps_clock_source / _ptp_clock_source; the fleet's PTP script
PORT_STATE = {1: "initializing", 2: "faulty", 3: "disabled", 4: "listening", 5: "pre-master", 6: "master", 7: "passive",
              8: "uncalibrated", 9: "slave"}  # IEEE 1588 port states, as 0x06 type 1 gives them (nebula HesaiPtpDiagStatus)
# Models whose 0x24 takes a reserved byte after the three intervals (nebula set_ptp_config, "is_new_gen"),
# by the driver's configuration names. Used only when the sensor's own 0x26 answer does not show it.
RESERVED_BYTE_MODELS = ("AT128", "QT128", "Pandar128", "OT128")
INTERVALS = (1, 1, 0)  # announce, sync, minimum delay request (log2 s): nebula g_ptp_log_*; used when the sensor reports none


class PtcRefused(Exception):
    def __init__(self, code: int, status: int) -> None:
        name = PTC_READS.get(code) or PTC_WRITES.get(code) or "?"
        super().__init__(f"the sensor refused PTC command 0x{code:02x} ({name}): return code {status} ({PTC_ERRORS.get(status, 'unknown')})")
        self.code, self.status = code, status


class Ptc:
    """One PTC session: one TCP connection, any number of commands. Reads only; see Writer."""

    def __init__(self, run: Run, ip: str) -> None:
        run.need_stopped("a PTC session")
        self.run, self.ip = run, ip
        run.sessions.opening("ptc")
        try:
            self.sock = socket.create_connection((ip, PORTS["ptc"]), timeout=max(run.args.timeout, 0.2))
        except OSError as exc:
            run.sessions.not_opened("ptc")
            raise NoAnswer(f"no PTC answer at {ip}:{PORTS['ptc']} ({exc.strerror or exc})") from None

    def __enter__(self) -> "Ptc":
        return self

    def __exit__(self, *_exc) -> None:
        self.close()

    def close(self) -> None:
        """End the session and see it end: the sensor's side is awaited (its close, for up to a
        second) before this returns, so the next session is never opened beside this one."""
        if self.sock is not None:
            try:
                self.sock.shutdown(socket.SHUT_WR)
                self.sock.settimeout(1.0)
                while self.sock.recv(4096):
                    pass
            except OSError:
                pass
            try:
                self.sock.close()
            finally:
                self.sock = None
                self.run.sessions.closed("ptc")

    def _exchange(self, code: int, payload: bytes) -> bytes:
        if code in PTC_NEVER or code not in PTC_READS and code not in PTC_WRITES:
            raise ValueError(f"PTC command 0x{code:02x} ({PTC_NEVER.get(code, 'unknown')}) is never sent by this tool")
        try:
            self.sock.sendall(struct.pack(">BBBBI", 0x47, 0x74, code, 0, len(payload)) + payload)
            head = self._receive(8)
            first, second, command, status, length = struct.unpack(">BBBBI", head)
            if (first, second) != (0x47, 0x74) or command != code or length > 1 << 20:
                raise NoAnswer(f"what answers at {self.ip}:{PORTS['ptc']} does not speak PTC")
            body = self._receive(length)
        except OSError as exc:
            raise NoAnswer(f"the PTC session with {self.ip} ended during command 0x{code:02x} ({exc.strerror or exc})") from None
        if status != 0:
            raise PtcRefused(code, status)
        return body

    def _receive(self, count: int) -> bytes:
        data = b""
        while len(data) < count:
            piece = self.sock.recv(count - len(data))
            if not piece:
                raise OSError("connection closed")
            data += piece
        return data

    def read(self, code: int, payload: bytes = b"") -> bytes:
        if code not in PTC_READS:
            raise ValueError(f"PTC command 0x{code:02x} is not a read")
        return self._exchange(code, payload)


def _no_proxy_opener():
    class NoRedirect(urllib.request.HTTPRedirectHandler):
        def redirect_request(self, *_args, **_kwargs):  # never follow a sensor to another host
            return None
    return urllib.request.build_opener(urllib.request.ProxyHandler({}), NoRedirect)


def _http(run: Run, ip: str, query: str):
    """One request to pandar.cgi: the Body of a successful answer, or None when there is no such
    interface at `ip` (no web server, another one, an error answer)."""
    url = f"http://{ip}:{PORTS['http']}/pandar.cgi?{query}"
    run.sessions.opening("http")
    try:
        with _no_proxy_opener().open(url, timeout=max(run.args.timeout, 0.2)) as reply:
            answer = json.loads(reply.read(256 * 1024).decode("utf-8", "replace"))
    except (OSError, ValueError, urllib.error.URLError):
        return None
    finally:
        run.sessions.closed("http")
    if not isinstance(answer, dict) or not isinstance(answer.get("Head"), dict):
        return None
    if str(answer["Head"].get("ErrorCode")) != "0":
        return {"_error": str(answer["Head"].get("Message", ""))[:200]}
    body = answer.get("Body")
    return body if isinstance(body, dict) else {}


def http_get(run: Run, ip: str, thing: str):
    """Read `device_info` or `lidar_config`. This function cannot write: the query is fixed."""
    if thing not in ("device_info", "lidar_config"):
        raise ValueError(thing)
    body = _http(run, ip, f"action=get&object={thing}")
    return None if body is None or "_error" in body else body


class Writer:
    """Everything that changes a sensor. Made only in a run that writes with the driver stopped."""

    def __init__(self, run: Run) -> None:
        if not run.args.writes or not run.args.driver_stopped:
            raise Stop("refused", "a write needs --driver-stopped and no --dry-run; nothing was sent")
        self.run = run

    def ptc(self, session: Ptc, code: int, payload: bytes) -> bytes:
        if code not in PTC_WRITES:
            raise ValueError(f"PTC command 0x{code:02x} is not a write this tool makes")
        self.run.out["_wrote"] = True
        return session._exchange(code, payload)

    def http(self, ip: str, key: str, value: str) -> None:
        """`action=set&object=lidar&key=...`: clock_source and ptp_configuration, the two keys with a source."""
        if key not in ("clock_source", "ptp_configuration"):
            raise ValueError(f"no source for the HTTP key {key}")
        self.run.out["_wrote"] = True
        body = _http(self.run, ip, f"action=set&object=lidar&key={key}&value={value}")
        if body is None:
            raise Stop("unverified", f"the sensor at {ip} did not answer the HTTP write of {key}")
        if "_error" in body:
            raise Stop("failed", f"the sensor refused the HTTP write of {key}: {body['_error']}")


# ---- Hesai: reading ---------------------------------------------------------------------------------


def _text(raw: bytes):
    text = raw.split(b"\0", 1)[0].decode("ascii", "replace").strip()
    return text if text and text.isprintable() else None


def _address(raw: bytes) -> str:
    return ".".join(str(byte) for byte in raw)


def http_identity(info: dict) -> dict:
    """device_info (Hesai's developer guide): SN, Mac, Model, "SW Ver"."""
    identity = blank_identity(VENDOR)
    for key, name in (("model", "Model"), ("serial", "SN"), ("mac", "Mac"), ("firmware", "SW Ver")):
        value = info.get(name)
        identity[key] = str(value).strip()[:64] if isinstance(value, (str, int)) and str(value).strip() else None
    if identity["mac"]:
        identity["mac"] = identity["mac"].lower()
    return identity


def ptc_identity(session: Ptc) -> dict:
    """The inventory (0x07). nebula's layouts: the serial number in the first 18 bytes; then the
    date of manufacture (16), the MAC address (6) and the software version (16). The AT128P's
    answer is longer (216 bytes) and has the part number (32) between serial number and date.
    The MAC address is given only where a date stands before it, so a layout that does not fit
    gives none rather than a wrong one."""
    raw = session.read(0x07)
    identity = blank_identity(VENDOR)
    identity["serial"] = _text(raw[:18])
    for start in ((50, 18) if len(raw) >= 216 else (18,)):
        date = _text(raw[start:start + 16]) if len(raw) >= start + 38 else None
        if date and any(ch.isdigit() for ch in date) and all(ch.isdigit() or ch in "-/. :" for ch in date):
            identity["mac"] = ":".join(f"{byte:02x}" for byte in raw[start + 16:start + 22])
            identity["firmware"] = _text(raw[start + 22:start + 38])
            break
    return identity


def _as_address(value):
    try:
        return str(ipaddress.IPv4Address(value.strip())) if isinstance(value, str) else None
    except ValueError:
        return None


def _as_int(value, low: int, high: int):
    if isinstance(value, str) and value.strip().lstrip("-").isdigit():
        value = int(value.strip())
    return value if isinstance(value, int) and not isinstance(value, bool) and low <= value <= high else None


def http_state(state: dict, config: dict, ip: str) -> None:
    """lidar_config into `state`.

    ClockSource, PTPStatus, PTPProfile and PTPConfig: the fleet's PTP script. DestIp and DestPort:
    seen in a public diagnostic script for the Pandar40P (github.com/evilgenius79/pandar-40p,
    scripts/diagnostics/lidar_config.py); GpsPort by analogy, read only if it is there. The inner
    keys of PTPConfig are taken to be the ones nebula writes (Profile, Domain, Network, the three
    intervals, tsn_switch). No real XT32's answer is in this repository: `http_keys` and `ptp.raw`
    show what the sensor gave, and a key that is not there leaves its setting None, which is then
    read over PTC or said to be unreadable. The netmask and the gateway are not in lidar_config."""
    state["http_keys"] = sorted(str(key)[:40] for key in config)[:48]
    dest, ptp = state["dest"], state["ptp"]
    state["net"]["ip"] = ip  # it answers there
    dest["ip"] = _as_address(config.get("DestIp"))
    dest["data_port"] = _as_int(config.get("DestPort"), 1, 65535)
    gps = _as_int(config.get("GpsPort"), 1, 65535)
    if gps is not None:
        dest["gps_port"] = gps
    source = _as_int(config.get("ClockSource"), 0, 255)
    ptp["clock_source"] = None if source is None else CLOCK.get(source, f"unknown({source})")
    status = config.get("PTPStatus")
    ptp["status"] = status.strip()[:40] if isinstance(status, str) and status.strip() else None
    detail = config.get("PTPConfig")
    if isinstance(detail, str):
        try:
            detail = json.loads(detail)
        except ValueError:
            pass
    if isinstance(detail, dict):
        ptp["raw"] = {str(key)[:40]: value for key, value in list(detail.items())[:12] if isinstance(value, (str, int))}
        detail = {str(key).replace("_", "").lower(): value for key, value in detail.items()}
    else:
        ptp["raw"] = detail[:200] if isinstance(detail, str) else detail if isinstance(detail, int) else None
        detail = {}
    named = config.get("PTPProfile")
    profile = _as_int(detail.get("profile"), 0, 255)
    if profile is None:
        profile = _as_int(named, 0, 255)
    if profile is not None:
        ptp["profile"] = PROFILE.get(profile, f"unknown({profile})")
    elif isinstance(named, str) and named.strip():
        ptp["profile"] = "1588v2" if "1588" in named else named.strip()[:40]
    network = _as_int(detail.get("network"), 0, 255)
    ptp["transport"] = None if network is None else NETWORK.get(network, f"unknown({network})")
    ptp["domain"] = _as_int(detail.get("domain"), 0, 255)
    intervals = [_as_int(detail.get(name), -128, 127) for name in ("logannounceinterval", "logsyncinterval", "logmindelayreqinterval")]
    ptp["_intervals"] = tuple(intervals) if None not in intervals else None
    ptp["_tsn_switch"] = _as_int(detail.get("tsnswitch"), 0, 255)


def ptc_config(session: Ptc) -> dict:
    """The configuration (0x08), by nebula's HesaiConfigBase: four addresses of 4 bytes (own,
    netmask, gateway, destination), the data and GPS ports, ..., the clock source at byte 29, the
    VLAN flag and id at bytes 35 to 37."""
    raw = session.read(0x08)
    if len(raw) < 30:
        raise NoAnswer(f"the sensor's configuration answer has {len(raw)} bytes, fewer than the 30 of its known layout")
    data_port, gps_port = struct.unpack(">HH", raw[16:20])
    return {"ip": _address(raw[0:4]), "netmask": _address(raw[4:8]), "gateway": _address(raw[8:12]), "dest_ip": _address(raw[12:16]),
            "data_port": data_port, "gps_port": gps_port, "clock_source": raw[29],
            "vlan": raw[35:38] if len(raw) >= 38 else b"\0\0\0"}


def ptc_net_dest(state: dict, config: dict, only_missing: bool = False) -> None:
    values = {("net", "ip"): config["ip"], ("net", "netmask"): config["netmask"], ("net", "gateway"): config["gateway"],
              ("dest", "ip"): config["dest_ip"], ("dest", "data_port"): config["data_port"], ("dest", "gps_port"): config["gps_port"]}
    for (section, key), value in values.items():
        if not only_missing or state[section].get(key) is None:
            state[section][key] = value


def ptc_ptp(run: Run, session: Ptc, state: dict, config: dict) -> None:
    """The clock source (0x08), the PTP configuration (0x26: status, profile, domain, network, and
    for 1588v2 three intervals and on newer sensors a reserved byte) and the port state (0x06 type 1)."""
    ptp = state["ptp"]
    ptp["clock_source"] = CLOCK.get(config["clock_source"], f"unknown({config['clock_source']})")
    ptp["_intervals"], ptp["_reserved"] = None, None
    try:
        raw = session.read(0x26)
    except PtcRefused as exc:
        run.warn(f"the PTP configuration was not read: {exc}")
        raw = b""
    if len(raw) >= 4:
        ptp["profile"] = PROFILE.get(raw[1], f"unknown({raw[1]})")
        ptp["domain"], ptp["transport"] = raw[2], NETWORK.get(raw[3], f"unknown({raw[3]})")
        if raw[1] == 0 and len(raw) >= 7:
            ptp["_intervals"], ptp["_reserved"] = struct.unpack(">bbb", raw[4:7]), len(raw) >= 8
    try:
        raw = session.read(0x06, b"\x01")
        if len(raw) >= 12:
            state_number = struct.unpack(">i", raw[8:12])[0]
            ptp["status"] = PORT_STATE.get(state_number, f"unknown({state_number})")
    except PtcRefused:
        pass  # the status is extra


def public(state):
    """`state` without the tool's own notes (keys that start with '_')."""
    if state is None:
        return None
    return {key: ({k: v for k, v in value.items() if not k.startswith("_")} if isinstance(value, dict) else value)
            for key, value in state.items()}


def http_info(run: Run, ip: str):
    """device_info when the sensor has the HTTP interface and the run may use it, else None."""
    if run.args.channel == "ptc":
        return None
    info = http_get(run, ip, "device_info")
    if info is None and run.args.channel == "http":
        raise NoAnswer(f"no Hesai HTTP answer at {ip}:{PORTS['http']} (pandar.cgi)")
    return info


def http_read(run: Run, ip: str, state: dict) -> None:
    config = http_get(run, ip, "lidar_config")
    if config is None:
        raise NoAnswer(f"the sensor at {ip} answered device_info and not lidar_config")
    http_state(state, config, ip)
    state["channel"] = "http"


def ptc_session(run: Run, ip: str, what: str) -> Ptc:
    if run.args.channel == "http":
        raise Stop("refused", f"{what} needs a PTC session, and --channel http forbids one; nothing was sent")
    return Ptc(run, ip)


def read_all(run: Run, ip: str):
    """Identity and every setting, with as little as possible: HTTP where it answers, then one PTC
    session for what HTTP did not give, and that only when the driver is stopped."""
    state, info = blank_state(), http_info(run, ip)
    identity = http_identity(info) if info is not None else None
    if info is not None:
        http_read(run, ip, state)
    missing = [f"{section}.{key}" for section, keys in (("net", ("netmask", "gateway")), ("dest", ("ip", "data_port")))
               for key in keys if state[section][key] is None]
    if info is None or (run.args.driver_stopped and run.args.channel != "http"):
        with ptc_session(run, ip, "reading this sensor") as session:
            if identity is None:
                identity = ptc_identity(session)
            config = ptc_config(session)
            ptc_net_dest(state, config, only_missing=info is not None)
            if info is None:
                ptc_ptp(run, session, state, config)
        state["channel"] = "ptc" if info is None else "http+ptc"
    elif missing:
        run.warn(f"not read, because it needs a PTC session and the driver is not stopped: {', '.join(missing)}")
    return identity, state


def probe(run: Run) -> None:
    ip = run.args.ip
    info = http_info(run, ip)
    if info is not None:
        identity, channel = http_identity(info), "HTTP"
    else:
        with ptc_session(run, ip, "this sensor") as session:
            identity, channel = ptc_identity(session), "PTC"
    run.out["identity"] = identity
    raise Stop("reported", f"a Hesai sensor answers at {ip} over {channel} (serial number {identity['serial'] or 'not given'})")


def get_net(run: Run) -> None:
    identity, state = read_all(run, run.args.ip)
    run.out["identity"], run.out["after"] = identity, public(state)
    raise Stop("reported", f"the settings of the Hesai sensor at {run.args.ip}, read over {state['channel']}")


# ---- Hesai: the set commands ---------------------------------------------------------------------


def settle(run: Run, step: str, wanted: dict, state: dict) -> list:
    """Note what is wanted and what differs. Ends the run when nothing differs, or on a dry run."""
    out = run.out
    out["before"], out["wanted"] = public(state), {step: wanted}
    out["changed"] = differing(step, wanted, state[step])
    if not out["changed"]:
        out["after"], out["verified"], out["answers_at"] = public(state), True, run.args.ip
        raise Stop("unchanged", f"{step} is already as asked; nothing was sent")
    if run.args.dry_run:
        raise Stop("changes", f"would change {', '.join(out['changed'])}")
    return out["changed"]


def finish(run: Run, step: str, wanted: dict, state, good: bool, at) -> None:
    out = run.out
    out["after"], out["verified"], out["answers_at"] = public(state), good, at
    if good:
        raise Stop("set", f"{', '.join(out['changed'])} set and read back")
    still = differing(step, wanted, state[step]) if state else out["changed"]
    raise Stop("unverified", f"the change was sent, and the sensor did not answer with the new values within "
               f"{run.args.verify_timeout:g} s (still different: {', '.join(still)})")


def set_dest(run: Run) -> None:
    args, ip = run.args, run.args.ip
    wanted = {"ip": args.dest_ip, "data_port": args.data_port}
    state, info = blank_state(), http_info(run, ip)
    if info is not None:
        run.out["identity"] = http_identity(info)
        http_read(run, ip, state)
        shown = None not in (state["dest"]["ip"], state["dest"]["data_port"])
        if shown and (args.dry_run or not differing("dest", wanted, state["dest"])):
            settle(run, "dest", wanted, state)  # ends the run: already right, or a dry run
    # No HTTP key that sets the destination is in any source: PTC 0x20, in one session with its reads.
    with ptc_session(run, ip, "the destination") as session:
        if info is None:
            run.out["identity"] = ptc_identity(session)
        config = ptc_config(session)
        ptc_net_dest(state, config)
        state["channel"] = "ptc" if info is None else "http+ptc"
        settle(run, "dest", wanted, state)
        # SetDesIpandPort (ptc_client.cc): address, data port, GPS port. The GPS port is kept.
        payload = ipaddress.IPv4Address(wanted["ip"]).packed + struct.pack(">HH", wanted["data_port"], config["gps_port"])
        try:
            Writer(run).ptc(session, 0x20, payload)
        except PtcRefused as exc:
            raise Stop("failed", str(exc)) from None

        def read():
            ptc_net_dest(state, ptc_config(session))
            return state
        after, good = await_state(time.monotonic() + args.verify_timeout, read, lambda s: not differing("dest", wanted, s["dest"]))
    finish(run, "dest", wanted, after, good, ip)


def set_ptp(run: Run) -> None:
    args, ip = run.args, run.args.ip
    wanted = {"clock_source": args.clock_source, "profile": args.profile, "transport": args.transport, "domain": args.domain}
    if args.delay_mechanism != "E2E":
        run.cannot("ptp.delay_mechanism", "no source has a command for a Hesai's delay mechanism; nothing about it was sent")
    info = http_info(run, ip)
    if info is not None:
        run.out["identity"] = http_identity(info)
        return set_ptp_http(run, wanted)
    state = blank_state()
    with ptc_session(run, ip, "the PTP configuration of a sensor without the HTTP interface") as session:
        run.out["identity"] = ptc_identity(session)
        ptc_ptp(run, session, state, ptc_config(session))
        state["channel"] = "ptc"
        hidden = [key for key in ("profile", "transport", "domain") if state["ptp"][key] is None]
        if hidden:  # 0x26 was refused: what cannot be read is not written
            for key in ("transport", "domain"):
                run.cannot(f"ptp.{key}", "this sensor does not answer PTC 0x26 with its PTP configuration; it was not written")
            wanted = {"clock_source": wanted["clock_source"]}
        changed = settle(run, "ptp", wanted, state)
        writer = Writer(run)
        try:
            if "ptp.clock_source" in changed:
                writer.ptc(session, 0x1F, b"\x01")  # nebula set_clock_source, g_hesai_lidar_ptp_clock_source
            if set(changed) - {"ptp.clock_source"}:
                # nebula set_ptp_config: profile 0 (1588v2), domain, network (0 UDP/IP, 1 L2), then the
                # three intervals, and one reserved byte on the newer sensors. The intervals are kept.
                announce, sync, delay = state["ptp"]["_intervals"] or INTERVALS
                reserved = state["ptp"]["_reserved"]
                if reserved is None:
                    reserved = args.model.startswith(RESERVED_BYTE_MODELS)
                payload = struct.pack(">BBBbbb", 0, wanted["domain"], 1 if wanted["transport"] == "L2" else 0, announce, sync, delay)
                writer.ptc(session, 0x24, payload + (b"\0" if reserved else b""))
        except PtcRefused as exc:
            raise Stop("failed", str(exc)) from None

        def read():
            ptc_ptp(run, session, state, ptc_config(session))
            return state
        after, good = await_state(time.monotonic() + args.verify_timeout, read, lambda s: not differing("ptp", wanted, s["ptp"]))
    finish(run, "ptp", wanted, after, good, ip)


def set_ptp_http(run: Run, wanted: dict) -> None:
    """The clock source and the PTP configuration of a sensor with the HTTP interface: HTTP only,
    never a PTC session."""
    args, ip = run.args, run.args.ip
    state = blank_state()
    http_read(run, ip, state)
    # The PTP configuration is written as a whole. A part the sensor does not show cannot be
    # compared or read back, so then none of it is written, and that is said.
    if None in (state["ptp"]["profile"], state["ptp"]["transport"], state["ptp"]["domain"]):
        for key in ("transport", "domain"):
            run.cannot(f"ptp.{key}", "this sensor's lidar_config does not show its PTP configuration in a shape this tool knows; it was not written")
        wanted = {"clock_source": wanted["clock_source"]}
    changed = settle(run, "ptp", wanted, state)
    writer = Writer(run)
    if "ptp.clock_source" in changed:
        writer.http(ip, "clock_source", "1")  # the fleet's PTP script: 0 GPS, 1 PTP
    if set(changed) - {"ptp.clock_source"}:
        # nebula set_ptp_config_sync_http names the keys; the fleet's script says how the value is
        # written: quotes as %22, no raw spaces, braces, colons and commas as they are.
        announce, sync, delay = state["ptp"]["_intervals"] or INTERVALS
        value = {"Profile": 0, "Domain": wanted["domain"], "Network": 1 if wanted["transport"] == "L2" else 0,
                 "LogAnnounceInterval": announce, "LogSyncInterval": sync, "LogMinDelayReqInterval": delay,
                 "tsn_switch": state["ptp"]["_tsn_switch"] or 0}
        writer.http(ip, "ptp_configuration", urllib.parse.quote(json.dumps(value, separators=(",", ":")), safe="{}:,"))

    def read():
        try:
            http_read(run, ip, state)
        except NoAnswer:
            return None
        return state
    after, good = await_state(time.monotonic() + args.verify_timeout, read, lambda s: not differing("ptp", wanted, s["ptp"]))
    finish(run, "ptp", wanted, after, good, ip)


def set_net(run: Run) -> None:
    """The address, netmask and gateway: PTC 0x21. No HTTP key for them is in any source, so this is
    the one PTC session of an HTTP sensor too."""
    args, ip = run.args, run.args.ip
    wanted = {"ip": args.new_ip, "netmask": args.netmask, "gateway": args.gateway}
    state, info = blank_state(), http_info(run, ip)
    if info is not None:
        run.out["identity"] = http_identity(info)
    session = ptc_session(run, ip, "the address")
    try:
        if info is None:
            run.out["identity"] = ptc_identity(session)
        config = ptc_config(session)
        ptc_net_dest(state, config)
        state["channel"] = "ptc" if info is None else "http+ptc"
        if differing("net", wanted, state["net"]):
            check_new_address(args.new_ip, args.netmask, args.gateway)
        changed = settle(run, "net", wanted, state)
        writer = Writer(run)
        # SetNet (ptc_client.cc): address, netmask, gateway, VLAN flag, VLAN id. The VLAN is kept.
        payload = b"".join(ipaddress.IPv4Address(wanted[key]).packed for key in ("ip", "netmask", "gateway")) + config["vlan"]
        try:
            writer.ptc(session, 0x21, payload)
        except PtcRefused as exc:
            raise Stop("failed", str(exc)) from None
        deadline = time.monotonic() + args.verify_timeout
        if "net.ip" not in changed:
            def read():
                ptc_net_dest(state, ptc_config(session))
                return state
            after, good = await_state(deadline, read, lambda s: not differing("net", wanted, s["net"]))
            finish(run, "net", wanted, after, good, ip)
        moved(run, session, writer, wanted, info is not None, deadline)
    finally:
        session.close()


def moved(run: Run, session: Ptc, writer: Writer, wanted: dict, http: bool, deadline: float) -> None:
    """After 0x21 with a new address: wait for the sensor there, and restart it (0x10) only when it
    has not taken the address by itself and still answers at the old one.

    A sensor with the HTTP interface is awaited over HTTP while this run's one PTC session stays
    open for that restart; its netmask and gateway are then the ones that session read back, if it
    did. Any other sensor is awaited over PTC, so the session is closed first: never two at once."""
    args, out, old, new = run.args, run.out, run.args.ip, wanted["ip"]
    serial = (out["identity"] or {}).get("serial")
    echo = None
    if http:
        try:
            echo = ptc_config(session)
        except (NoAnswer, PtcRefused):
            session.close()
        if echo is None or echo["ip"] != new:
            echo = None
            run.warn("the netmask and the gateway were sent and not read back by this run")
    else:
        session.close()

    def at_new():
        """The state read at the new address, or None while nothing answers there."""
        found = blank_state()
        if http:
            there = http_get(run, new, "device_info")
            if there is None:
                return None
            identity = http_identity(there)
            try:
                http_read(run, new, found)
            except NoAnswer:
                return None
            if echo is not None:
                ptc_net_dest(found, echo, only_missing=True)
            found["channel"] = "http+ptc"
        else:
            try:
                with Ptc(run, new) as fresh:
                    identity = ptc_identity(fresh)
                    ptc_net_dest(found, ptc_config(fresh))
            except NoAnswer:
                return None
            found["channel"] = "ptc"
        if serial and identity["serial"] and identity["serial"] != serial:
            raise Stop("failed", f"another sensor answers at {new} (serial number {identity['serial']}, not {serial}); "
                       f"the address was sent to the sensor at {old}")
        return found

    def good(found) -> bool:
        return all(found["net"][key] in (value, None if http else value) for key, value in wanted.items())

    def at_old():
        """An open PTC session to the old address when the sensor still answers there, else None."""
        again = session if http else None
        try:
            if again is None:
                again = Ptc(run, old)
            if again.sock is not None:
                again.read(0x08)
                return again
        except (NoAnswer, PtcRefused):
            if again is not None:
                again.close()
        return None

    after, done = await_state(min(deadline, time.monotonic() + LIVE_GRACE), at_new, good)
    if not done:
        still = at_old()
        try:
            if still is not None and args.no_reboot:
                out["reboot"] = "needed"
                out["after"], out["verified"], out["answers_at"] = public(after), False, old
                raise Stop("unverified", f"the new address was sent; the sensor still answers at {old} and takes it at its next restart (--no-reboot)")
            if still is not None:
                try:
                    writer.ptc(still, 0x10, b"")  # RebootLidar (ptc_client.cc)
                    out["reboot"] = "sent"
                except (NoAnswer, PtcRefused) as exc:
                    out["reboot"] = "needed"
                    run.warn(f"the restart was not taken: {exc}")
        finally:
            if still is not None:
                still.close()
            session.close()
        after, done = await_state(deadline, at_new, good)
    session.close()
    if done:
        finish(run, "net", wanted, after, True, new)
    here = new if after is not None else None
    if here is None:
        still = at_old() if not http else None
        if still is not None:
            still.close()
        if still is not None or (http and http_get(run, old, "device_info") is not None):
            here = old
    out["after"], out["verified"], out["answers_at"] = public(after), False, here
    raise Stop("unverified", f"the new address was sent, and the sensor did not answer at {new} with it within {args.verify_timeout:g} s"
               + (f"; it answers at {here}" if here else "; it answers at neither address"))


def _extra(parser: argparse.ArgumentParser, _command: str) -> None:
    # auto: HTTP first, PTC for the rest. http: never a PTC session. ptc: never HTTP (a bench check).
    parser.add_argument("--channel", choices=("auto", "http", "ptc"), default="auto")


HANDLERS = {"probe": probe, "get-net": get_net, "set-net": set_net, "set-dest": set_dest, "set-ptp": set_ptp}


def main(argv=None, stdout=None) -> int:
    return run_tool(argv, VENDOR, Sessions(("ptc",), ("http",)), HANDLERS, _extra, stdout)


if __name__ == "__main__":
    sys.exit(main())
