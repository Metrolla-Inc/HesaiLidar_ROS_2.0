#!/usr/bin/env python3
"""A synthetic Hesai PandarXT-32 for the smoke test: UDP data packets and a PTC port that answers.

Data: deterministic UDP protocol 6.1 packets (the layout of the SDK's
libhesai/UdpProtocol/udp_protocol_v6_1.h: pre-header 6 B, header 6 B, 8 blocks of azimuth 2 B +
32 x (distance 2 B, reflectivity 1 B, reserved 1 B), tail 28 B = 1080 B), 600 rpm single return,
250 packets per revolution, one packet every 400 us. Channel c at azimuth a returns
(500 + 40 c + (a // 100) % 50) x 4 mm, so every point is 2.000-7.156 m from the sensor.
Generator first written for backlog task C13 (synthetic XT32 comparison); not checked against a
real sensor.

PTC: a TCP server on the PTC port that accepts the driver's connection and answers every command
with a PTC 1.0 header carrying a non-zero return code, so the SDK's "get correction from the lidar"
fails and it loads the calibration files named in its configuration, as it does when a real
sensor refuses. With use_ptc_connected the v2.0.10 SDK waits for this connection without a
timeout before it decodes anything.

Usage: fake_xt32.py --sensor-ip IP --data-port 2368 --ptc-port 9347 --host DEST [--seconds N]
"""

from __future__ import annotations

import argparse
import socket
import struct
import threading
import time

LASERS = 32
BLOCKS = 8
AZ_STEP = 18  # 0.18 deg per block at 600 rpm (0.01 deg units)
PKT_US = 400
DIST_UNIT_MM = 4
MIN_RANGE_M = 500 * DIST_UNIT_MM / 1000
MAX_RANGE_M = (500 + 40 * (LASERS - 1) + 49) * DIST_UNIT_MM / 1000


def packet(seq: int, azimuth0: int, utc: list[int], usec: int) -> bytes:
    pre = struct.pack("<HBBBB", 0xFFEE, 6, 1, 0, 0)
    hdr = struct.pack("<BBBBBB", LASERS, BLOCKS, 0, DIST_UNIT_MM, 1, 0x01)
    body = b""
    for b in range(BLOCKS):
        az = (azimuth0 + b * AZ_STEP) % 36000
        body += struct.pack("<H", az)
        for ch in range(LASERS):
            dist = 500 + ch * 40 + (az // 100) % 50
            refl = (ch * 7 + az // 1000) % 256
            body += struct.pack("<HBB", dist, refl, 0)
    tail = struct.pack("<HBHBHB", 0, 0, 0, 0, 0, 0)
    tail += struct.pack("<BBH", 0, 0x37, 600)
    tail += bytes(utc)
    tail += struct.pack("<IBI", usec, 0x42, seq)
    out = pre + hdr + body + tail
    assert len(out) == 1080, len(out)
    return out


def stream():
    seq = 0
    az = 36000 - 5 * AZ_STEP * BLOCKS
    usec_total = 0
    while True:
        sec = 5 + usec_total // 1_000_000
        utc = [26, 1, 2, 3, (4 + sec // 60) % 60, sec % 60]  # 2026-01-02 (the SDK adds 100 to a year below 70)
        yield packet(seq & 0xFFFFFFFF, az, utc, usec_total % 1_000_000)
        seq += 1
        az = (az + AZ_STEP * BLOCKS) % 36000
        usec_total += PKT_US


def ptc_server(ip: str, port: int, stop: threading.Event) -> None:
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind((ip, port))
    srv.listen(4)
    srv.settimeout(0.5)
    while not stop.is_set():
        try:
            conn, _ = srv.accept()
        except socket.timeout:
            continue
        threading.Thread(target=ptc_session, args=(conn, stop), daemon=True).start()


def ptc_session(conn: socket.socket, stop: threading.Event) -> None:
    conn.settimeout(0.5)
    buf = b""
    while not stop.is_set():
        try:
            data = conn.recv(4096)
        except socket.timeout:
            continue
        except OSError:
            break
        if not data:
            break
        buf += data
        # PTC 1.0 (the version libhesai/Lidar/lidar.cc asks for): 0x47 0x74, cmd, return code,
        # payload length (4 B, big endian), payload.
        while len(buf) >= 8:
            start = buf.find(b"\x47\x74")
            if start < 0:
                buf = b""
                break
            buf = buf[start:]
            if len(buf) < 8:
                break
            cmd = buf[2]
            length = struct.unpack(">I", buf[4:8])[0]
            if len(buf) < 8 + length:
                break
            buf = buf[8 + length:]
            reply = bytes([0x47, 0x74, cmd, 0x01]) + struct.pack(">I", 0)
            try:
                conn.sendall(reply)
            except OSError:
                return
    conn.close()


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--sensor-ip", required=True)
    ap.add_argument("--data-port", type=int, default=2368)
    ap.add_argument("--ptc-port", type=int, default=9347)
    ap.add_argument("--host", required=True, help="where to send the data packets")
    ap.add_argument("--seconds", type=float, default=30.0)
    a = ap.parse_args()
    stop = threading.Event()
    threading.Thread(target=ptc_server, args=(a.sensor_ip, a.ptc_port, stop), daemon=True).start()
    out = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    out.bind((a.sensor_ip, 0))
    n = 0
    t0 = time.monotonic()
    for p in stream():
        out.sendto(p, (a.host, a.data_port))
        n += 1
        target = t0 + n * PKT_US / 1e6
        now = time.monotonic()
        if now - t0 > a.seconds:
            break
        if target > now:
            time.sleep(target - now)
    stop.set()
    print(f"fake_xt32: sent {n} packets from {a.sensor_ip}")


if __name__ == "__main__":
    main()
