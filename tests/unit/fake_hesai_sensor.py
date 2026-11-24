"""A fake Hesai sensor for the sensor-tool tests: a PTC server and, for the XT32 class, the HTTP
interface (`pandar.cgi`). Loopback only; nothing here is a real sensor.

It listens on every address in `addresses` and answers only on the one it has now, so a test can
move it. It records every PTC command code (`codes`), every HTTP query (`queries`), how many PTC
sessions were opened (`ptc_sessions`) and how many were open at once (`max_open`).

Faults: `refuse` (code -> PTC return code), `live=False` (a new address is taken only at a
restart), `deaf` (codes it acknowledges and does not take), `vanish` (after this code it answers
nowhere), `no_ptp_config` (0x26 is refused), `http_dest=False` / `http_ptp=False` (lidar_config
without the destination / without PTPConfig), `http_clock=False` (without ClockSource).
"""

from __future__ import annotations

import http.server
import json
import socket
import struct
import threading
import urllib.parse


def _free_port(address: str) -> int:
    with socket.socket() as probe:
        probe.bind((address, 0))
        return probe.getsockname()[1]


class FakeHesai:
    def __init__(self, addresses=("127.0.50.1", "127.0.50.2"), http=False, layout="base", extended_ptp=False, **faults) -> None:
        self.addresses, self.has_http, self.layout, self.extended_ptp = addresses, http, layout, extended_ptp
        self.ip, self.netmask, self.gateway = addresses[0], "255.0.0.0", "127.0.0.254"
        self.dest_ip, self.data_port, self.gps_port = "127.0.0.9", 2368, 10110
        self.clock_source, self.profile, self.domain, self.network = 0, 0, 0, 0
        self.intervals, self.vlan = (1, 1, 0), b"\x01\x00\x07"
        self.serial, self.mac = "XT0000000000001", bytes.fromhex("ec9f0da12801")
        self.pending_ip = None
        self.alive = True
        self.refuse = dict(faults.pop("refuse", {}))
        self.live, self.deaf, self.vanish = faults.pop("live", True), tuple(faults.pop("deaf", ())), faults.pop("vanish", None)
        self.no_ptp_config = faults.pop("no_ptp_config", False)
        self.http_dest, self.http_ptp = faults.pop("http_dest", True), faults.pop("http_ptp", True)
        self.http_clock = faults.pop("http_clock", True)
        assert not faults, faults
        self.codes, self.payloads, self.queries = [], {}, []
        self.ptc_sessions = self.open = self.max_open = 0
        self._lock = threading.Lock()
        self._closing = False
        self.ptc_port, self.http_port = _free_port(addresses[0]), _free_port(addresses[0])
        self._listeners, self._servers = [], []

    # --- life ---------------------------------------------------------------------------------------

    def __enter__(self) -> "FakeHesai":
        for address in self.addresses:
            listener = socket.socket()
            listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            listener.bind((address, self.ptc_port))
            listener.listen(8)
            self._listeners.append(listener)
            threading.Thread(target=self._accept, args=(listener,), daemon=True).start()
            if self.has_http:
                server = http.server.ThreadingHTTPServer((address, self.http_port), self._handler())
                server.daemon_threads = True
                self._servers.append(server)
                threading.Thread(target=server.serve_forever, kwargs={"poll_interval": 0.05}, daemon=True).start()
        return self

    def __exit__(self, *_exc) -> None:
        self._closing = True
        for listener in self._listeners:
            listener.close()
        for server in self._servers:
            server.shutdown()
            server.server_close()

    # --- PTC ----------------------------------------------------------------------------------------

    def _accept(self, listener: socket.socket) -> None:
        while not self._closing:
            try:
                connection, _peer = listener.accept()
            except OSError:
                return
            threading.Thread(target=self._session, args=(connection,), daemon=True).start()

    def _session(self, connection: socket.socket) -> None:
        with connection:
            if not self.alive or connection.getsockname()[0] != self.ip:
                return  # nothing answers at this address
            with self._lock:
                self.ptc_sessions += 1
                self.open += 1
                self.max_open = max(self.max_open, self.open)
            try:
                while True:
                    head = self._receive(connection, 8)
                    if head is None:
                        return
                    first, second, code, _status, length = struct.unpack(">BBBBI", head)
                    payload = self._receive(connection, length) if length else b""
                    if (first, second) != (0x47, 0x74) or payload is None:
                        return
                    self.codes.append(code)
                    self.payloads.setdefault(code, []).append(payload)
                    status, body = self._command(code, payload)
                    connection.sendall(struct.pack(">BBBBI", 0x47, 0x74, code, status, len(body)) + body)
                    if not self.alive or connection.getsockname()[0] != self.ip:
                        return  # it moved or went away: the session ends
            finally:
                with self._lock:
                    self.open -= 1

    @staticmethod
    def _receive(connection: socket.socket, count: int):
        data = b""
        while len(data) < count:
            try:
                piece = connection.recv(count - len(data))
            except OSError:
                return None
            if not piece:
                return None
            data += piece
        return data

    def _command(self, code: int, payload: bytes):
        if code in self.refuse:
            return self.refuse[code], b""
        if code == self.vanish:
            self.alive = False
            return 0, b""
        if code in self.deaf:
            return 0, b""
        if code == 0x07:
            return 0, self._inventory()
        if code == 0x08:
            return 0, self._config()
        if code == 0x26:
            if self.no_ptp_config:
                return 5, b""
            body = struct.pack(">BBBB", 1, self.profile, self.domain, self.network)
            if self.profile == 0:
                body += struct.pack(">bbb", *self.intervals) + (b"\0" if self.extended_ptp else b"")
            return 0, body
        if code == 0x06 and payload == b"\x01":
            return 0, struct.pack(">qii", -42, 9 if self.clock_source == 1 else 4, 1000)
        if code == 0x20 and len(payload) == 8:
            self.dest_ip = socket.inet_ntoa(payload[:4])
            self.data_port, self.gps_port = struct.unpack(">HH", payload[4:])
            return 0, b""
        if code == 0x21 and len(payload) == 15:
            new = socket.inet_ntoa(payload[:4])
            self.netmask, self.gateway, self.vlan = socket.inet_ntoa(payload[4:8]), socket.inet_ntoa(payload[8:12]), payload[12:]
            if self.live:
                self.ip = new
            elif new != self.ip:
                self.pending_ip = new
            return 0, b""
        if code == 0x10:
            if self.pending_ip:
                self.ip, self.pending_ip = self.pending_ip, None
            return 0, b""
        if code == 0x1F and len(payload) == 1:
            self.clock_source = payload[0]
            return 0, b""
        if code == 0x24 and len(payload) == (7 if self.extended_ptp else 6) and payload[0] == 0:
            self.profile, self.domain, self.network = payload[0], payload[1], payload[2]
            self.intervals = struct.unpack(">bbb", payload[3:6])
            return 0, b""
        return (1 if code in (0x20, 0x21, 0x1F, 0x24) else 5), b""

    def _inventory(self) -> bytes:
        def field(text: str, size: int) -> bytes:
            return text.encode().ljust(size, b"\0")
        rest = self.mac + field("2.1.7", 16) + field("1.0.0", 16) + field("2.0.11", 16) + field("2.1.4", 16)
        if self.layout == "at128p":  # the part number stands between serial number and date
            return (field(self.serial, 18) + field("AT128P-PN", 32) + field("2025-01-07", 16) + rest).ljust(216, b"\0")
        return (field(self.serial, 18) + field("2023-06-08", 16) + rest).ljust(120, b"\0")

    def _config(self) -> bytes:
        body = socket.inet_aton(self.pending_ip or self.ip) + socket.inet_aton(self.netmask) + socket.inet_aton(self.gateway)
        body += socket.inet_aton(self.dest_ip) + struct.pack(">HHHBHHH", self.data_port, self.gps_port, 600, 0, 0, 0, 36000)
        body += bytes([self.clock_source, 1, 0, 1, 0, 1]) + self.vlan
        return body.ljust(40, b"\0")

    # --- HTTP ---------------------------------------------------------------------------------------

    def _handler(self):
        fake = self

        class Handler(http.server.BaseHTTPRequestHandler):
            def log_message(self, *_args) -> None:
                pass

            def do_GET(self) -> None:  # noqa: N802 - the base class's name
                if not fake.alive or self.server.server_address[0] != fake.ip:
                    self.close_connection = True
                    self.connection.close()
                    return
                path, _, query = self.path.partition("?")
                fake.queries.append(query)
                answer = fake._http(query) if path == "/pandar.cgi" else None
                if answer is None:
                    answer = {"Head": {"ErrorCode": "1", "Message": "not support yet"}}
                raw = json.dumps(answer).encode()
                self.send_response(200)
                self.send_header("Content-Type", "application/json")
                self.send_header("Content-Length", str(len(raw)))
                self.end_headers()
                self.wfile.write(raw)

        return Handler

    def _http(self, query: str):
        ok = {"ErrorCode": "0", "Message": "Success"}
        if query == "action=get&object=device_info":
            return {"Head": ok, "Body": {"SN": self.serial, "Mac": "EC:9F:0D:A1:28:01", "Model": "XT32M2X", "SW Ver": "2.1.7", "FW Ver": "2.0.11"}}
        if query == "action=get&object=lidar_config":
            body = {"SpinSpeed": "600", "PTPStatus": "Free Run", "PTPProfile": str(self.profile)}
            if self.http_clock:
                body["ClockSource"] = str(self.clock_source)
            if self.http_ptp:
                body["PTPConfig"] = {"Profile": self.profile, "Domain": self.domain, "Network": self.network,
                                     "LogAnnounceInterval": self.intervals[0], "LogSyncInterval": self.intervals[1],
                                     "LogMinDelayReqInterval": self.intervals[2], "tsn_switch": 0}
            if self.http_dest:
                body.update(DestIp=self.dest_ip, DestPort=str(self.data_port), GpsPort=str(self.gps_port))
            return {"Head": ok, "Body": body}
        prefix = "action=set&object=lidar&key="
        if query.startswith(prefix):
            key, _, value = query[len(prefix):].partition("&value=")
            if "http:" + key in self.deaf:
                return {"Head": ok}
            if key == "clock_source" and value in ("0", "1"):
                self.clock_source = int(value)
                return {"Head": ok}
            if key == "ptp_configuration":
                wanted = json.loads(urllib.parse.unquote(value))
                self.profile, self.domain, self.network = wanted["Profile"], wanted["Domain"], wanted["Network"]
                self.intervals = (wanted["LogAnnounceInterval"], wanted["LogSyncInterval"], wanted["LogMinDelayReqInterval"])
                return {"Head": ok}
        return None
