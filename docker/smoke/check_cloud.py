#!/usr/bin/env python3
"""Smoke checker: subscribe to a PointCloud2 topic and require clouds decoded from fake_xt32.py.

Requires N clouds on TOPIC with header.frame_id FRAME, the fields of the driver's XYZI point type
(x y z intensity ring timestamp), at least MIN_POINTS points each, and every point between the
synthetic sensor's minimum and maximum range (2.000-7.156 m, small tolerance for the angle and
firetime corrections, which move azimuth and elevation, not range). Prints a JSON summary and
exits 0 on success, 1 otherwise.

Usage: check_cloud.py TOPIC FRAME [--clouds 3] [--min-points 60000] [--timeout 60]
"""

from __future__ import annotations

import argparse
import json
import math
import struct
import sys
import time

import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2

MIN_RANGE_M = 2.000
MAX_RANGE_M = 7.156
TOLERANCE_M = 0.01
FIELDS = ["x", "y", "z", "intensity", "ring", "timestamp"]


def summarise(m: PointCloud2) -> dict:
    data = bytes(m.data)
    off = {f.name: f.offset for f in m.fields}
    n = m.width * m.height
    lo, hi, bad = math.inf, 0.0, 0
    rings = set()
    for i in range(n):
        base = i * m.point_step
        x, y, z = struct.unpack_from("<fff", data, base + off["x"])
        r = math.sqrt(x * x + y * y + z * z)
        lo, hi = min(lo, r), max(hi, r)
        if not MIN_RANGE_M - TOLERANCE_M <= r <= MAX_RANGE_M + TOLERANCE_M:
            bad += 1
        rings.add(struct.unpack_from("<H", data, base + off["ring"])[0])
    return {
        "stamp": f"{m.header.stamp.sec}.{m.header.stamp.nanosec:09d}",
        "frame_id": m.header.frame_id,
        "points": n,
        "fields": [f.name for f in m.fields],
        "point_step": m.point_step,
        "range_min": round(lo, 4),
        "range_max": round(hi, 4),
        "out_of_range": bad,
        "rings": sorted(rings),
    }


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("topic")
    ap.add_argument("frame")
    ap.add_argument("--clouds", type=int, default=3)
    ap.add_argument("--min-points", type=int, default=60000)
    ap.add_argument("--timeout", type=float, default=60.0)
    a = ap.parse_args()
    got: list[PointCloud2] = []
    rclpy.init()
    node = rclpy.create_node("hesai_smoke_check")
    node.create_subscription(PointCloud2, a.topic, got.append,
                             QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))
    t0 = time.monotonic()
    while len(got) < a.clouds + 1 and time.monotonic() - t0 < a.timeout:
        rclpy.spin_once(node, timeout_sec=0.2)
    rclpy.shutdown()
    # The first cloud can be a partial revolution (the driver starts mid-stream): skip it.
    clouds = [summarise(m) for m in got[1:a.clouds + 1]]
    problems = []
    if len(clouds) < a.clouds:
        problems.append(f"{len(got)} clouds on {a.topic} in {a.timeout:g} s, wanted {a.clouds + 1}")
    for c in clouds:
        if c["frame_id"] != a.frame:
            problems.append(f"frame_id {c['frame_id']!r}, wanted {a.frame!r}")
        if c["fields"] != FIELDS:
            problems.append(f"fields {c['fields']}, wanted {FIELDS}")
        if c["points"] < a.min_points:
            problems.append(f"{c['points']} points, wanted at least {a.min_points}")
        if c["out_of_range"]:
            problems.append(f"{c['out_of_range']} points outside {MIN_RANGE_M}-{MAX_RANGE_M} m")
        if c["rings"] != list(range(32)):
            problems.append(f"rings {c['rings'][:5]}..., wanted 0-31")
    print(json.dumps({"topic": a.topic, "clouds": clouds, "problems": problems}, indent=1))
    return 1 if problems else 0


if __name__ == "__main__":
    sys.exit(main())
