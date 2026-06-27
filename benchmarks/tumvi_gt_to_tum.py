#!/usr/bin/env python3
"""Convert TUM-VI ground-truth file to TUM trajectory format for evo."""

from __future__ import annotations

import argparse
from pathlib import Path


def main() -> int:
    parser = argparse.ArgumentParser(description="TUM-VI GT to TUM")
    parser.add_argument("groundtruth", type=Path, help="TUM-VI groundtruth text file")
    parser.add_argument("output", type=Path, help="Output TUM file")
    args = parser.parse_args()

    if not args.groundtruth.is_file():
        raise SystemExit(f"Missing ground truth: {args.groundtruth}")

    lines: list[str] = []
    with args.groundtruth.open() as f:
        for raw in f:
            line = raw.strip()
            if not line or line.startswith("#"):
                continue
            parts = line.split()
            if len(parts) < 8:
                continue
            ts = float(parts[0])
            px, py, pz = map(float, parts[1:4])
            qx, qy, qz, qw = map(float, parts[4:8])
            lines.append(f"{ts:.9f} {px} {py} {pz} {qx} {qy} {qz} {qw}")

    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text("\n".join(lines) + "\n")
    print(f"Wrote {len(lines)} poses to {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
