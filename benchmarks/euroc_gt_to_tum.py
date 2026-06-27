#!/usr/bin/env python3
"""Convert EuRoC ground-truth CSV to TUM trajectory format for evo."""

from __future__ import annotations

import argparse
import csv
from pathlib import Path


def main() -> int:
    parser = argparse.ArgumentParser(description="EuRoC GT to TUM")
    parser.add_argument("mav0_dir", type=Path, help="Path to mav0 folder")
    parser.add_argument("output", type=Path, help="Output TUM file")
    args = parser.parse_args()

    gt_file = args.mav0_dir / "state_groundtruth_estimate0" / "data.csv"
    if not gt_file.is_file():
        raise SystemExit(f"Missing ground truth: {gt_file}")

    lines: list[str] = []
    with gt_file.open() as f:
        reader = csv.reader(f)
        for row in reader:
            if not row or row[0].startswith("#"):
                continue
            ts = float(row[0]) / 1e9
            px, py, pz = map(float, row[1:4])
            qw, qx, qy, qz = map(float, row[4:8])
            lines.append(f"{ts:.9f} {px} {py} {pz} {qx} {qy} {qz} {qw}")

    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text("\n".join(lines) + "\n")
    print(f"Wrote {len(lines)} poses to {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
