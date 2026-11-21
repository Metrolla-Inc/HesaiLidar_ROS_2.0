"""Unit tier: the PointCloud2 layout the Hesai driver published on a real sensor.

tests/fixtures/jt16-r2-lidar_points.cdr holds two clouds cut from a real JT16 recording
(fixtures/jt16-r2-lidar_points.json says where from). No recording of raw Hesai packets exists
(that bag has decoded clouds and IMU only), so packets in the smoke test are synthetic; this test
pins that the layout the smoke checker requires from the synthetic run (docker/smoke/check_cloud.py)
is the layout a real sensor produced through this driver: fields, types, offsets, point size,
frame id. Pure Python: a small ROS 2 CDR reader for sensor_msgs/msg/PointCloud2.
"""

from __future__ import annotations

import hashlib
import json
import math
import struct
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
FIXTURE = REPO / "tests" / "fixtures" / "jt16-r2-lidar_points.cdr"
META = json.loads(FIXTURE.with_suffix(".json").read_text())
FLOAT32, FLOAT64, UINT16 = 7, 8, 4  # sensor_msgs/msg/PointField datatypes


class Cdr:
    def __init__(self, data: bytes):
        assert data[:2] == b"\x00\x01", "not little-endian plain CDR"
        self.d, self.p = data[4:], 0

    def align(self, n: int) -> None:
        self.p += (-self.p) % n

    def u8(self) -> int:
        v = self.d[self.p]
        self.p += 1
        return v

    def u32(self) -> int:
        self.align(4)
        v = struct.unpack_from("<I", self.d, self.p)[0]
        self.p += 4
        return v

    def i32(self) -> int:
        self.align(4)
        v = struct.unpack_from("<i", self.d, self.p)[0]
        self.p += 4
        return v

    def string(self) -> str:
        n = self.u32()
        v = self.d[self.p:self.p + n - 1].decode()
        self.p += n
        return v

    def octets(self) -> bytes:
        n = self.u32()
        v = self.d[self.p:self.p + n]
        self.p += n
        return v


def point_cloud2(data: bytes) -> dict:
    c = Cdr(data)
    msg = {"stamp": (c.i32(), c.u32()), "frame_id": c.string(), "height": c.u32(), "width": c.u32()}
    fields = []
    for _ in range(c.u32()):
        fields.append((c.string(), c.u32(), c.u8(), c.u32()))
    msg["fields"] = fields
    msg["is_bigendian"] = bool(c.u8())
    msg["point_step"], msg["row_step"] = c.u32(), c.u32()
    msg["data"] = c.octets()
    msg["is_dense"] = bool(c.u8())
    return msg


def clouds() -> list[dict]:
    raw = FIXTURE.read_bytes()
    out = []
    for m in META["messages"]:
        chunk = raw[m["offset"]:m["offset"] + m["bytes"]]
        assert hashlib.sha256(chunk).hexdigest() == m["sha256"]
        out.append(point_cloud2(chunk))
    return out


def test_fixture_is_small_and_intact():
    assert FIXTURE.stat().st_size < 5 * 1024 * 1024
    assert len(clouds()) == 2


def test_layout_is_what_the_smoke_checker_requires():
    checker = (REPO / "docker" / "smoke" / "check_cloud.py").read_text()
    expected_fields = ["x", "y", "z", "intensity", "ring", "timestamp"]
    assert f"FIELDS = {expected_fields!r}".replace("'", '"') in checker
    for m in clouds():
        assert [f[0] for f in m["fields"]] == expected_fields
        assert [(f[1], f[2], f[3]) for f in m["fields"]] == [
            (0, FLOAT32, 1), (4, FLOAT32, 1), (8, FLOAT32, 1), (12, FLOAT32, 1), (16, UINT16, 1),
            (18, FLOAT64, 1)]
        assert m["point_step"] == 26 and not m["is_bigendian"]
        assert m["frame_id"] == "hesai_lidar"  # the per-model configs' default, = DEFAULT_FRAME_ID
        assert m["row_step"] == m["width"] * m["point_step"] == len(m["data"])


def test_points_are_a_real_16_ring_scan():
    for m in clouds():
        n = m["width"] * m["height"]
        assert n > 5000
        rings, finite, stamps = set(), 0, []
        for i in range(n):
            x, y, z, _ = struct.unpack_from("<ffff", m["data"], i * 26)
            rings.add(struct.unpack_from("<H", m["data"], i * 26 + 16)[0])
            stamps.append(struct.unpack_from("<d", m["data"], i * 26 + 18)[0])
            if all(math.isfinite(v) for v in (x, y, z)):
                finite += 1
        assert rings <= set(range(16)) and len(rings) == 16
        assert finite == n
        # The cloud's stamp is its first point's time (v2.0.10 ROS layer), within a millisecond.
        sec, nsec = m["stamp"]
        assert abs(sec + nsec * 1e-9 - min(stamps)) < 1e-3
