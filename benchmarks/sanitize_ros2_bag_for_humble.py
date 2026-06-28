#!/usr/bin/env python3
"""Make rosbags/Jazzy metadata.yaml readable by ROS 2 Humble rosbag2."""

from __future__ import annotations

import argparse
import re
import shutil
import sys
from pathlib import Path


def sanitize_metadata(text: str) -> str:
    text = re.sub(
        r"\n      type_description_hash:.*?(?=\n  - |\n  [a-z_]+:)",
        "",
        text,
        flags=re.DOTALL,
    )
    text = re.sub(r"^  custom_data:.*\n", "", text, flags=re.MULTILINE)
    text = re.sub(r"^  ros_distro:.*\n", "", text, flags=re.MULTILINE)
    text = text.replace("offered_qos_profiles: []", 'offered_qos_profiles: ""')
    return text


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag_dir", type=Path, help="ROS 2 bag directory")
    parser.add_argument(
        "--out",
        type=Path,
        help="Write a Humble-compatible copy here (default: sanitize in place)",
    )
    args = parser.parse_args()

    src = args.bag_dir.resolve()
    meta = src / "metadata.yaml"
    if not meta.is_file():
        print(f"ERROR: missing {meta}", file=sys.stderr)
        return 1

    sanitized = sanitize_metadata(meta.read_text())

    if args.out:
        out = args.out.resolve()
        if out == src:
            meta.write_text(sanitized)
            return 0
        if out.exists():
            shutil.rmtree(out)
        shutil.copytree(src, out, dirs_exist_ok=True)
        (out / "metadata.yaml").write_text(sanitized)
        return 0

    meta.write_text(sanitized)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
