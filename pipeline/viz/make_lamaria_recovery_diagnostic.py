#!/usr/bin/env python3
"""Separate within-frame relatch/optimizer moves from actual output-pose steps."""
import argparse
import csv
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run", type=Path, required=True)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--start", type=float, default=548.4)
    parser.add_argument("--end", type=float, default=555.4)
    args = parser.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)
    local, relatch = [], []
    for line in (args.run / "run.log").open():
        for prefix, destination in [("[LM_DIAG] ", local), ("[RELATCH_DIAG] ", relatch)]:
            if line.startswith(prefix):
                row = json.loads(line[len(prefix):])
                if args.start <= row["time"] <= args.end:
                    destination.append(row)
    source = list(csv.DictReader(next(args.run.glob("online_*.csv")).open()))
    online = []
    for previous, row in zip(source, source[1:]):
        if not args.start <= float(row["input_t_s"]) <= args.end:
            continue
        a = np.array([float(row[k]) for k in ("tx", "ty", "tz")])
        b = np.array([float(previous[k]) for k in ("tx", "ty", "tz")])
        online.append({"time": float(row["input_t_s"]), "position": a.tolist(),
                       "actual_output_step_m": float(np.linalg.norm(a-b)),
                       "dt_s": float(row["input_t_s"])-float(previous["input_t_s"]),
                       "state": int(row["state"]), "coasting": int(row["coasting"]),
                       "inliers": int(row["inliers"]), "map": int(row["map_id"]),
                       "world_version": int(row["world_version"])})
    explanation = (
        "LM_DIAG's opt_camera_displacement_m measures optimization from the pose at "
        "TrackLocalMap entry, which can already be a visual relatch hypothesis. It is "
        "not an output trajectory step. In this run, the 554.9922 s relatch moves "
        "14.441724 m, its inertial optimization moves 14.441795 m, but the output "
        "step is only 0.222910 m. At 549.7422 s the visual relatch moves 2.851122 m; "
        "the inertial optimization returns nearly the same distance and fails with "
        "four inliers. The rollback snapshot was captured after relatch, so it "
        "restores that visual hypothesis and causes a real 2.722269 m output step. "
        "The successful update at 549.8422 s returns to the inertial anchor with a "
        "2.975665 m output step. The snapshot boundary must include relatch, not "
        "only TrackLocalMap. No estimator change was made during this replay."
    )
    document = {"run": str(args.run.resolve()), "window": [args.start, args.end],
                "explanation": explanation, "online": online,
                "local_map": local, "relatch": relatch,
                "missing_telemetry": ["full pose vectors before/after relatch and optimizer",
                                      "last-keyframe ID and preintegration start/end/dT",
                                      "IMU residual vector, covariance spectrum and chi-square"]}
    (args.out / "recovery_diagnostic.json").write_text(json.dumps(document, indent=2)+"\n")
    fig, axes = plt.subplots(3, 1, figsize=(13, 10), sharex=True, constrained_layout=True)
    axes[0].plot([r["time"] for r in relatch], [r["camera_displacement_m"] for r in relatch], "o", label="Visual relatch move")
    axes[0].plot([r["time"] for r in local], [r["opt_camera_displacement_m"] for r in local], ".-", label="Optimizer move from entry pose")
    axes[0].set_ylabel("Within-frame move (m)")
    axes[0].set_title("Within-frame corrections are not output trajectory jumps")
    axes[0].legend()
    axes[1].plot([r["time"] for r in online], [r["actual_output_step_m"] for r in online], ".-", color="tab:purple", label="Actual consecutive output body-pose step")
    axes[1].set_ylabel("Output step (m)")
    axes[1].legend()
    axes[2].plot([r["time"] for r in online], [r["inliers"] for r in online], ".-", color="tab:green", label="Inliers")
    rejected = [r for r in local if not r["success"]]
    axes[2].scatter([r["time"] for r in rejected], [r["inliers"] for r in rejected], color="tab:red", label="Rejected local-map update", zorder=5)
    axes[2].set_ylabel("Inliers")
    axes[2].set_xlabel("Native sensor timestamp (s)")
    axes[2].legend()
    for ax in axes:
        ax.grid(alpha=.25)
    fig.savefig(args.out / "recovery_diagnostic.png", dpi=160)
    plt.close(fig)
    print(json.dumps({"samples": len(online), "output": str(args.out.resolve())}))


if __name__ == "__main__":
    main()
