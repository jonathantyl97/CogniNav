#!/usr/bin/env python3
"""Apply TorWIC calibrations.txt to CogniNav ORB + warehouse YAML configs."""

from __future__ import annotations

import argparse
import re
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
CONFIG_DIR = ROOT / "ros2_ws" / "src" / "cogninav_bringup" / "config"
DEFAULT_CALIB = CONFIG_DIR / "torwic_calibrations.txt"
ORB_YAML = CONFIG_DIR / "orb_torwic_azure.yaml"
WAREHOUSE_YAML = CONFIG_DIR / "warehouse_torwic.yaml"


def parse_calibrations(path: Path) -> dict[str, float | list[float]]:
    text = path.read_text()
    vals: dict[str, float | list[float]] = {}
    for match in re.finditer(r"^([A-Za-z0-9_.]+)\s*[:=]\s*([^\n#]+)", text, re.MULTILINE):
        key, raw = match.group(1), match.group(2).strip()
        if raw.startswith("["):
            vals[key] = [float(x) for x in re.findall(r"[-+eE0-9.]+", raw)]
        elif raw.startswith('"'):
            vals[key] = raw.strip('"')
        else:
            try:
                vals[key] = float(raw)
            except ValueError:
                vals[key] = raw
    return vals


def quat_to_matrix(qx: float, qy: float, qz: float, qw: float) -> np.ndarray:
    x, y, z, w = qx, qy, qz, qw
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


def pose_matrix(px: float, py: float, pz: float, qx: float, qy: float, qz: float, qw: float) -> np.ndarray:
    mat = np.eye(4, dtype=np.float64)
    mat[:3, :3] = quat_to_matrix(qx, qy, qz, qw)
    mat[:3, 3] = [px, py, pz]
    return mat


def flatten_row_major(mat: np.ndarray) -> list[float]:
    return [float(mat[r, c]) for r in range(4) for c in range(4)]


def format_matrix_block(mat: np.ndarray, indent: str = "  ") -> str:
    flat = flatten_row_major(mat)
    lines = [
        f"{indent}rows: 4",
        f"{indent}cols: 4",
        f"{indent}dt: f",
        f"{indent}data: [{flat[0]:.8g}, {flat[1]:.8g}, {flat[2]:.8g}, {flat[3]:.8g},",
        f"{indent}       {flat[4]:.8g}, {flat[5]:.8g}, {flat[6]:.8g}, {flat[7]:.8g},",
        f"{indent}       {flat[8]:.8g}, {flat[9]:.8g}, {flat[10]:.8g}, {flat[11]:.8g},",
        f"{indent}       {flat[12]:.8g}, {flat[13]:.8g}, {flat[14]:.8g}, {flat[15]:.8g}]",
    ]
    return "\n".join(lines)


def build_orb_yaml(vals: dict[str, float | list[float]], calib_name: str) -> str:
    d1 = vals["Camera1.D"]  # type: ignore[index]
    d2 = vals["Camera2.D"]  # type: ignore[index]

    t_os_c1 = pose_matrix(
        vals["T_cam1_os.px"], vals["T_cam1_os.py"], vals["T_cam1_os.pz"],
        vals["T_cam1_os.qx"], vals["T_cam1_os.qy"], vals["T_cam1_os.qz"], vals["T_cam1_os.qw"],
    )
    t_os_c2 = pose_matrix(
        vals["T_cam2_os.px"], vals["T_cam2_os.py"], vals["T_cam2_os.pz"],
        vals["T_cam2_os.qx"], vals["T_cam2_os.qy"], vals["T_cam2_os.qz"], vals["T_cam2_os.qw"],
    )
    t_c1_c2 = np.linalg.inv(t_os_c1) @ t_os_c2
    t_imu_c1 = pose_matrix(
        vals["T_imu1_cam1.px"], vals["T_imu1_cam1.py"], vals["T_imu1_cam1.pz"],
        vals["T_imu1_cam1.qx"], vals["T_imu1_cam1.qy"], vals["T_imu1_cam1.qz"], vals["T_imu1_cam1.qw"],
    )

    stereo_block = format_matrix_block(t_c1_c2)
    imu_block = format_matrix_block(t_imu_c1)

    return f"""%YAML:1.0

# TorWIC Azure Kinect RGB @ 1280x720 — generated from {calib_name}
# Regenerate: python3 benchmarks/tools/apply_torwic_calibrations.py

File.version: "1.0"

Camera.type: "PinHole"

Camera1.fx: {vals['Camera1.fx']:.12g}
Camera1.fy: {vals['Camera1.fy']:.12g}
Camera1.cx: {vals['Camera1.cx']:.12g}
Camera1.cy: {vals['Camera1.cy']:.12g}
Camera1.k1: {d1[0]:.12g}
Camera1.k2: {d1[1]:.12g}
Camera1.p1: {d1[2]:.12g}
Camera1.p2: {d1[3]:.12g}
Camera1.k3: {d1[4]:.12g}

Camera2.fx: {vals['Camera2.fx']:.12g}
Camera2.fy: {vals['Camera2.fy']:.12g}
Camera2.cx: {vals['Camera2.cx']:.12g}
Camera2.cy: {vals['Camera2.cy']:.12g}
Camera2.k1: {d2[0]:.12g}
Camera2.k2: {d2[1]:.12g}
Camera2.p1: {d2[2]:.12g}
Camera2.p2: {d2[3]:.12g}
Camera2.k3: {d2[4]:.12g}

Camera.width: {int(vals['Camera1.width'])}
Camera.height: {int(vals['Camera1.height'])}
Camera.fps: 15
Camera.RGB: 1

Stereo.ThDepth: 40.0
Stereo.T_c1_c2: !!opencv-matrix
{stereo_block}

IMU.T_b_c1: !!opencv-matrix
{imu_block}

IMU.NoiseGyro: 1.0e-03
IMU.NoiseAcc: 1.0e-02
IMU.GyroWalk: 1.0e-05
IMU.AccWalk: 1.0e-03
IMU.Frequency: 200.0

ORBextractor.nFeatures: 1500
ORBextractor.scaleFactor: 1.2
ORBextractor.nLevels: 8
ORBextractor.iniThFAST: 20
ORBextractor.minThFAST: 7

Viewer.KeyFrameSize: 0.05
Viewer.KeyFrameLineWidth: 1.0
Viewer.GraphLineWidth: 0.9
Viewer.PointSize: 2.0
Viewer.CameraSize: 0.08
Viewer.CameraLineWidth: 3.0
Viewer.ViewpointX: 0.0
Viewer.ViewpointY: -0.7
Viewer.ViewpointZ: -3.5
Viewer.ViewpointF: 500.0
"""


def patch_warehouse_yaml(vals: dict[str, float | list[float]], baseline_m: float) -> None:
    text = WAREHOUSE_YAML.read_text()
    replacements = {
        'slam_mode: "stereo_inertial"': 'slam_mode: "stereo"',
        "fx: 610.0": f"fx: {vals['Camera1.fx']:.12g}",
        "fy: 610.0": f"fy: {vals['Camera1.fy']:.12g}",
        "cx: 640.0": f"cx: {vals['Camera1.cx']:.12g}",
        "cy: 360.0": f"cy: {vals['Camera1.cy']:.12g}",
        "baseline_m: 0.0757": f"baseline_m: {baseline_m:.12g}",
    }
    for old, new in replacements.items():
        if old not in text:
            raise SystemExit(f"warehouse_torwic.yaml missing expected token: {old}")
        text = text.replace(old, new)
    if "do_rectify: true" in text:
        text = text.replace("do_rectify: true", "do_rectify: false")
    comment_old = (
        "    # Stereo-only is more stable on TorWIC replay (approximate IMU extrinsics)."
    )
    comment_new = (
        "    # Stereo + TorWIC calibrations.txt (orb_torwic_azure.yaml)."
    )
    text = text.replace(comment_old, comment_new)
    WAREHOUSE_YAML.write_text(text)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--calib",
        type=Path,
        default=DEFAULT_CALIB,
        help=f"TorWIC calibrations.txt (default: {DEFAULT_CALIB})",
    )
    parser.add_argument("--check", action="store_true", help="Verify configs match calibrations")
    args = parser.parse_args()

    if not args.calib.is_file():
        print(f"Missing {args.calib} — see README.md (Datasets, TorWIC)", file=sys.stderr)
        return 1

    vals = parse_calibrations(args.calib)
    t_os_c1 = pose_matrix(
        vals["T_cam1_os.px"], vals["T_cam1_os.py"], vals["T_cam1_os.pz"],
        vals["T_cam1_os.qx"], vals["T_cam1_os.qy"], vals["T_cam1_os.qz"], vals["T_cam1_os.qw"],
    )
    t_os_c2 = pose_matrix(
        vals["T_cam2_os.px"], vals["T_cam2_os.py"], vals["T_cam2_os.pz"],
        vals["T_cam2_os.qx"], vals["T_cam2_os.qy"], vals["T_cam2_os.qz"], vals["T_cam2_os.qw"],
    )
    t_c1_c2 = np.linalg.inv(t_os_c1) @ t_os_c2
    baseline_m = float(t_c1_c2[0, 3])

    expected_orb = build_orb_yaml(vals, args.calib.name)
    if args.check:
        ok = ORB_YAML.read_text() == expected_orb
        print("orb_torwic_azure.yaml:", "OK" if ok else "OUT OF DATE")
        return 0 if ok else 1

    ORB_YAML.write_text(expected_orb)
    patch_warehouse_yaml(vals, baseline_m)
    print(f"Updated {ORB_YAML.name} and {WAREHOUSE_YAML.name}")
    print(
        f"  left fx/fy/cx/cy = {vals['Camera1.fx']:.3f} / {vals['Camera1.fy']:.3f} / "
        f"{vals['Camera1.cx']:.3f} / {vals['Camera1.cy']:.3f}"
    )
    print(f"  stereo baseline (cam1 x): {baseline_m:.4f} m")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
