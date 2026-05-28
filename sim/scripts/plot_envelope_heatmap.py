#!/usr/bin/env python3
"""
Two-panel scatter "heatmap" of detector accuracy over the position sweep.

Reads a CSV produced by sweep_position_grid.py and renders one figure with:
  - left  panel: mean |yaw error| in degrees per cell
  - right panel: mean xy euclidean error in metres per cell

Each spawn cell is drawn as a coloured circle at its true
(distance, lateral) position so trapezoidal / non-rectangular sweep grids
render honestly (the previous pcolormesh assumed a rectangular grid).
Cells where the detector failed (n == 0 or NaN value) are drawn as open
grey circles with an "x". Colour-bar limits are auto-set from the
finite cells.

Usage:
  python3 plot_envelope_heatmap.py path/to/sweep.csv [--out plot.png]
"""
import argparse
import csv
import os
import sys

import numpy as np
import matplotlib.pyplot as plt


REQUIRED = ["distance_m", "lateral_m", "n", "xy_err_m", "abs_yaw_err_deg"]


def load_csv(path):
    with open(path, newline="") as f:
        reader = csv.DictReader(f)
        if reader.fieldnames is None:
            print("CSV has no header.", file=sys.stderr)
            sys.exit(1)
        missing = [c for c in REQUIRED if c not in reader.fieldnames]
        if missing:
            print("CSV missing columns:", missing, file=sys.stderr)
            sys.exit(1)
        rows = []
        for r in reader:
            rows.append({k: (float(v) if v not in ("", "nan") else float("nan"))
                         for k, v in r.items()})
    if not rows:
        print("CSV is empty.", file=sys.stderr)
        sys.exit(1)
    return rows


def plot_panel(ax, rows, value_key, title, cbar_label, cmap_name="viridis"):
    xs = np.array([r["distance_m"] for r in rows])
    ys = np.array([r["lateral_m"]  for r in rows])
    vs = np.array([r[value_key]    for r in rows], dtype=float)
    ns = np.array([r["n"]          for r in rows], dtype=float)

    valid = (ns > 0) & np.isfinite(vs)
    failed = ~valid

    # Colour limits from valid cells only
    if valid.any():
        vmin = 0.0
        vmax = float(np.max(vs[valid]))
        if vmax <= vmin:
            vmax = vmin + 1e-6
    else:
        vmin, vmax = 0.0, 1.0

    # Failed cells: open grey circle with an "×" inside
    if failed.any():
        ax.scatter(xs[failed], ys[failed], s=520, facecolors="none",
                   edgecolors="0.55", linewidths=1.3, zorder=2)
        for x, y in zip(xs[failed], ys[failed]):
            ax.text(x, y, "×", ha="center", va="center",
                    fontsize=11, color="0.4", zorder=3)

    # Successful cells: filled coloured circles
    sc = None
    if valid.any():
        sc = ax.scatter(xs[valid], ys[valid], c=vs[valid], cmap=cmap_name,
                        s=520, edgecolors="black", linewidths=0.6,
                        vmin=vmin, vmax=vmax, zorder=4)
        mid = 0.5 * (vmin + vmax)
        for x, y, v in zip(xs[valid], ys[valid], vs[valid]):
            colour = "white" if v > mid else "black"
            ax.text(x, y, f"{v:.2f}", ha="center", va="center",
                    fontsize=7, color=colour, zorder=5)

    # Pallet marker: a small brown square at (0, 0) on the distance axis
    # i.e. the pallet centre. Distance = 0 means "at the pallet".
    ax.scatter([0.0], [0.0], marker="s", s=180, color="saddlebrown",
               edgecolor="black", linewidths=1.0, zorder=10)
    ax.text(0.0, 0.0, "pallet\ncentre", ha="center", va="top",
            fontsize=7, color="0.2",
            transform=ax.transData,
            position=(0.0, -0.06))
    # Pallet face line: at distance ≈ 0.586 m from the centre
    ax.axvline(0.586, ls="--", color="0.6", lw=0.8, alpha=0.7)
    ax.text(0.586, ax.get_ylim()[1] if ax.get_ylim()[1] else 1.0,
            "front face", rotation=90, ha="right", va="top",
            fontsize=7, color="0.4")

    # Axes / cosmetics
    pad_x = 0.15
    pad_y = 0.15
    xmin = min(float(np.min(xs)) - pad_x, -0.1)
    xmax = float(np.max(xs)) + pad_x
    ymin = float(np.min(ys)) - pad_y
    ymax = float(np.max(ys)) + pad_y
    ax.set_xlim(xmin, xmax)
    ax.set_ylim(ymin, ymax)
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlabel("distance to pallet centre [m]")
    ax.set_ylabel("robot lateral (world Y) [m]")
    ax.set_title(title)
    ax.grid(alpha=0.3)
    ax.axhline(0.0, color="0.85", lw=0.8, zorder=0)

    if sc is not None:
        cb = plt.colorbar(sc, ax=ax, shrink=0.85, pad=0.02)
        cb.set_label(cbar_label)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("csv", help="CSV from sweep_position_grid.py")
    ap.add_argument("--out", default=None,
                    help="PNG output (default: alongside CSV)")
    args = ap.parse_args()

    rows = load_csv(args.csv)
    out = args.out or os.path.splitext(args.csv)[0] + "_envelope.png"

    fig, axes = plt.subplots(1, 2, figsize=(14, 6.5))
    plot_panel(axes[0], rows, "abs_yaw_err_deg",
               "Yaw error envelope", "mean |yaw error| [deg]")
    plot_panel(axes[1], rows, "xy_err_m",
               "XY error envelope", "mean xy error [m]")

    n_cells = len(rows)
    n_ok    = sum(1 for r in rows
                  if r["n"] > 0 and np.isfinite(r["xy_err_m"]))
    fig.suptitle(f"{os.path.basename(args.csv)}    "
                 f"cells with detections: {n_ok} / {n_cells}",
                 y=1.02)
    fig.tight_layout()
    fig.savefig(out, dpi=130, bbox_inches="tight")
    plt.close(fig)
    print(f"wrote {out}")


if __name__ == "__main__":
    main()
