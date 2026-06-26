#!/usr/bin/env python3
"""ATE/RPE evaluation harness (evo). Trajectory path wired in Phase 1."""

from __future__ import annotations

import argparse
import json
import sys
from datetime import datetime, timezone
from pathlib import Path


def main() -> int:
    parser = argparse.ArgumentParser(description="CogniNav open-dataset eval")
    parser.add_argument("--dataset", required=True)
    parser.add_argument("--seq", required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--traj", type=Path, help="Estimated trajectory (TUM format)")
    parser.add_argument("--gt", type=Path, help="Ground truth (TUM format)")
    parser.add_argument("--phase", type=int, default=0)
    parser.add_argument("--git-sha", default="unknown")
    parser.add_argument("--docker-image", default="unknown")
    parser.add_argument("--smoke-status", default="")
    parser.add_argument("--smoke-note", default="")
    args = parser.parse_args()

    result = {
        "timestamp_utc": datetime.now(timezone.utc).isoformat(),
        "dataset": args.dataset,
        "sequence": args.seq,
        "phase": args.phase,
        "git_sha": args.git_sha,
        "docker_image": args.docker_image,
        "status": "pending",
    }

    if args.smoke_status:
        result["status"] = args.smoke_status
        result["note"] = args.smoke_note or "Phase 0 smoke: ORB-SLAM3 native EuRoC run + workspace build."
    else:
        result["note"] = "Wire cogninav_vslam trajectory export in Phase 1."

    if args.traj and args.gt:
        try:
            from evo.core import metrics, sync
            from evo.tools import file_interface

            traj_est = file_interface.read_tum_trajectory_file(str(args.traj))
            traj_ref = file_interface.read_tum_trajectory_file(str(args.gt))
            traj_est, traj_ref = sync.associate_trajectories(traj_est, traj_ref)
            ape = metrics.APE(metrics.PoseRelation.translation_part)
            ape.process_data((traj_ref, traj_est))
            stats = ape.get_all_statistics()
            result.update(
                {
                    "status": "ok",
                    "ate_rmse_m": float(stats["rmse"]),
                    "ate_mean_m": float(stats["mean"]),
                    "ate_std_m": float(stats["std"]),
                }
            )
        except Exception as exc:  # noqa: BLE001
            result["status"] = "error"
            result["error"] = str(exc)
    elif not args.smoke_status:
        result["status"] = "skeleton"

    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps(result, indent=2))
    return 0 if result["status"] != "error" else 1


if __name__ == "__main__":
    sys.exit(main())
