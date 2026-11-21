"""Unit tier: the synthetic PandarXT-32 of the smoke test against the SDK's protocol header.

docker/smoke/fake_xt32.py builds UDP protocol 6.1 packets; this pins its sizes to the field
lists of libhesai/UdpProtocol/udp_protocol_v6_1.h (read as text) and its range band to the one
check_cloud.py requires. The decoding itself is exercised in the smoke tier with the real driver.
"""

from __future__ import annotations

import re
import struct
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "docker" / "smoke"))
import fake_xt32  # noqa: E402

PROTOCOL = REPO / "src/driver/HesaiLidar_SDK_2.0/libhesai/UdpProtocol/udp_protocol_v6_1.h"
HEADER = REPO / "src/driver/HesaiLidar_SDK_2.0/libhesai/UdpProtocol/udp_protocol_header.h"
SIZES = {"uint8_t": 1, "uint16_t": 2, "uint32_t": 4}


def struct_size(text: str, name: str) -> int:
    """Packed size of a struct of the SDK headers: its data members (static constants skipped)."""
    body = re.search(r"struct " + name + r" \{(.*?)\n\};", text, re.S).group(1)
    total = 0
    for typ, _, count in re.findall(r"^\s*(uint8_t|uint16_t|uint32_t|ReservedInfo[123])\s+\w+(\[(\d+)\])?;",
                                    body, re.M):
        size = struct_size(text, typ) if typ.startswith("ReservedInfo") else SIZES[typ]
        total += size * (int(count) if count else 1)
    return total


def test_packet_size_and_markers(needs_sdk):
    text = HEADER.read_text() + PROTOCOL.read_text()
    block = struct_size(text, "HsLidarXTV1BodyAzimuth") + 32 * struct_size(text, "HsLidarXTV1BodyChannelData")
    total = (struct_size(text, "HS_LIDAR_PRE_HEADER") + struct_size(text, "HsLidarXTV1Header")
             + 8 * block + struct_size(text, "HsLidarXTV1Tail"))
    assert struct_size(text, "HsLidarXTV1Tail") == 28
    p = fake_xt32.packet(0, 0, [26, 1, 2, 3, 4, 5], 0)
    assert len(p) == total == 1080
    assert struct.unpack_from("<H", p, 0)[0] == 0xFFEE and p[2:4] == bytes([6, 1])
    assert p[6] == 32 and p[7] == 8  # lasers, blocks


def test_ranges_match_the_checker():
    checker = (REPO / "docker" / "smoke" / "check_cloud.py").read_text()
    assert f"MIN_RANGE_M = {fake_xt32.MIN_RANGE_M:.3f}" in checker
    assert f"MAX_RANGE_M = {fake_xt32.MAX_RANGE_M:.3f}" in checker
    stream = fake_xt32.stream()
    dists = set()
    for _ in range(260):
        p = next(stream)
        for b in range(8):
            base = 12 + b * (2 + 32 * 4) + 2
            for ch in range(32):
                dists.add(struct.unpack_from("<H", p, base + ch * 4)[0] * fake_xt32.DIST_UNIT_MM / 1000)
    assert min(dists) == fake_xt32.MIN_RANGE_M and max(dists) == fake_xt32.MAX_RANGE_M


def test_a_revolution_is_250_packets_and_deterministic():
    a, b = fake_xt32.stream(), fake_xt32.stream()
    first = [next(a) for _ in range(500)]
    assert first == [next(b) for _ in range(500)]
    azimuths = [struct.unpack_from("<H", p, 12)[0] for p in first]
    wraps = [i for i in range(1, 500) if azimuths[i] < azimuths[i - 1]]
    assert wraps[1] - wraps[0] == 250
