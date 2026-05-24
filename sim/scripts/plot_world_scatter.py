#!/usr/bin/env python3
"""
World-frame scatter + error-vs-range visualisation for a single eval run.

Reads a CSV produced by evaluate_pallet_pose.py (extended schema with
absolute world positions) and produces one figure showing:
  - Top-down XY scatter (world frame): ground-truth pallet front face (star),
    detected face positions coloured by sim time, robot trajectory line.
  - 4 subplots of dx, dy, dz, yaw_err vs robot->pallet range.

Usage:
  python3 plot_world_scatter.py path/to/eval.csv [--out plot.png]
"""
import argparse
import csv
import os
import sys

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec


REQUIRED = ["stamp", "range_m",
            "dx_m", "dy_m", "dz_m", "yaw_err_deg",
            "det_x_w", "det_y_w", "det_z_w",
            "gt_x_w", "gt_y_w", "gt_z_w",
            "robot_x_w", "robot_y_w", "robot_yaw_w"]


def load_csv(path):
    with open(path, newline="") as f:
        reader = csv.DictReader(f)
        if reader.fieldnames is None:
            print("CSV has no header.", file=sys.stderr)
            sys.exit(1)
        missing = [c for c in REQUIRED if c not in reader.fieldnames]
        if missing:
            print("CSV missing columns:", missing, file=sys.stderr)
            print("Re-run the evaluator after upgrading it to the extended schema.",
                  file=sys.stderr)
            sys.exit(1)
        cols = {c: [] for c in REQUIRED}
        for row in reader:
            for c in REQUIRED:
                cols[c].append(float(row[c]))
    if not cols["stamp"]:
        print("CSV is empty.", file=sys.stderr)
        sys.exit(1)
    return {c: np.asarray(v, dtype=float) for c, v in cols.items()}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("csv", help="CSV file written by evaluate_pallet_pose.py")
    ap.add_argument("--out", default=None,
                    help="PNG output path (default: alongside the CSV)")
    args = ap.parse_args()

    d = load_csv(args.csv)
    out_path = args.out or (os.path.splitext(args.csv)[0] + "_scatter.png")
    t = d["stamp"] - d["stamp"][0]
    rng = d["range_m"]

    fig = plt.figure(figsize=(13, 6.5))
    gs = GridSpec(4, 2, figure=fig, width_ratios=[1.5, 1.0],
                  hspace=0.45, wspace=0.28)

    # ---- World top-down scatter (left, spans all rows) ----
    ax_w = fig.add_subplot(gs[:, 0])
    ax_w.plot(d["robot_x_w"], d["robot_y_w"], "-", color="0.55",
              alpha=0.85, lw=1.6, label="robot trajectory")
    sc = ax_w.scatter(d["det_x_w"], d["det_y_w"], c=t, cmap="viridis",
                      s=26, alpha=0.9, edgecolor="none", label="detected face")
    gtx = float(d["gt_x_w"].mean())
    gty = float(d["gt_y_w"].mean())
    gt_std = float(np.hypot(d["gt_x_w"].std(), d["gt_y_w"].std()))
    ax_w.scatter([gtx], [gty], marker="*", s=300, color="red",
                 edgecolor="black", linewidth=0.8, zorder=5,
                 label=f"gt face (sigma={gt_std*1000:.1f} mm)")
    ax_w.set_aspect("equal", adjustable="datalim")
    ax_w.set_xlabel("world X [m]")
    ax_w.set_ylabel("world Y [m]")
    ax_w.set_title("Detected front face vs ground truth (top-down)")
    ax_w.grid(alpha=0.3)
    ax_w.legend(loc="best", fontsize=9)
    cb = fig.colorbar(sc, ax=ax_w, shrink=0.75, pad=0.02)
    cb.set_label("time since start [s]")

    # ---- Error vs range (right column) ----
    panels = [
        ("dx_m",        "dx [m]",        "tab:blue"),
        ("dy_m",        "dy [m]",        "tab:green"),
        ("dz_m",        "dz [m]",        "tab:purple"),
        ("yaw_err_deg", "yaw err [deg]", "tab:orange"),
    ]
    last = None
    for i, (col, ylab, color) in enumerate(panels):
        ax = fig.add_subplot(gs[i, 1], sharex=last)
        ax.scatter(rng, d[col], c=color, s=14, alpha=0.85, edgecolor="none")
        ax.axhline(0.0, ls="--", color="k", lw=0.6, alpha=0.5)
        ax.set_ylabel(ylab)
        ax.grid(alpha=0.3)
        if i < len(panels) - 1:
            plt.setp(ax.get_xticklabels(), visible=False)
        else:
            ax.set_xlabel("robot -> pallet range [m]")
        last = ax

    n = len(d["stamp"])
    rmse_xy = float(np.sqrt((d["dx_m"]**2 + d["dy_m"]**2).mean()))
    rmse_yaw = float(np.sqrt((d["yaw_err_deg"]**2).mean()))
    fig.suptitle(f"{os.path.basename(args.csv)}    n={n}    "
                 f"xy_rmse={rmse_xy*100:.2f} cm    yaw_rmse={rmse_yaw:.2f} deg",
                 y=0.995)
    fig.savefig(out_path, dpi=130, bbox_inches="tight")
    plt.close(fig)
    print(f"wrote {out_path}")


if __name__ == "__main__":
    main()
