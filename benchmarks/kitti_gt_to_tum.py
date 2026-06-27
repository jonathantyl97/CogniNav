#!/usr/bin/env python3
"""Convert KITTI odometry pose file to TUM trajectory format for evo."""

from __future__ import annotations

import argparse
from pathlib import Path


def main() -> int:
    parser = argparse.ArgumentParser(description="KITTI poses to TUM")
    parser.add_argument("poses_file", type=Path, help="KITTI poses/<seq>.txt")
    parser.add_argument("times_file", type=Path, help="KITTI sequences/<seq>/times.txt")
    parser.add_argument("output", type=Path, help="Output TUM file")
    args = parser.parse_args()

    if not args.poses_file.is_file():
        raise SystemExit(f"Missing poses: {args.poses_file}")
    if not args.times_file.is_file():
        raise SystemExit(f"Missing times: {args.times_file}")

    times = [float(x) for x in args.times_file.read_text().split()]
    pose_rows = [line.split() for line in args.poses_file.read_text().splitlines() if line.strip()]

    if len(times) != len(pose_rows):
        raise SystemExit(f"times ({len(times)}) and poses ({len(pose_rows)}) length mismatch")

    lines: list[str] = []
    for t_sec, row in zip(times, pose_rows):
        if len(row) != 12:
            continue
        vals = list(map(float, row))
        # KITTI pose line: r11 r12 r13 tx r21 r22 r23 ty r31 r32 r33 tz
        r11, r12, r13, tx = vals[0], vals[1], vals[2], vals[3]
        r21, r22, r23, ty = vals[4], vals[5], vals[6], vals[7]
        r31, r32, r33, tz = vals[8], vals[9], vals[10], vals[11]
        # Rotation matrix to quaternion (xyzw)
        trace = r11 + r22 + r33
        if trace > 0.0:
            s = 0.5 / ((trace + 1.0) ** 0.5)
            qw = 0.25 / s
            qx = (r32 - r23) * s
            qy = (r13 - r31) * s
            qz = (r21 - r12) * s
        elif r11 > r22 and r11 > r33:
            s = 2.0 * ((1.0 + r11 - r22 - r33) ** 0.5)
            qw = (r32 - r23) / s
            qx = 0.25 * s
            qy = (r12 + r21) / s
            qz = (r13 + r31) / s
        elif r22 > r33:
            s = 2.0 * ((1.0 + r22 - r11 - r33) ** 0.5)
            qw = (r13 - r31) / s
            qx = (r12 + r21) / s
            qy = 0.25 * s
            qz = (r23 + r32) / s
        else:
            s = 2.0 * ((1.0 + r33 - r11 - r22) ** 0.5)
            qw = (r21 - r12) / s
            qx = (r13 + r31) / s
            qy = (r23 + r32) / s
            qz = 0.25 * s
        lines.append(f"{t_sec:.9f} {tx} {ty} {tz} {qx} {qy} {qz} {qw}")

    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text("\n".join(lines) + "\n")
    print(f"Wrote {len(lines)} poses to {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
