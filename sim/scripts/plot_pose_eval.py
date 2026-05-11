#!/usr/bin/env python3
"""
Plot pallet-pose evaluation CSVs across a noise sweep.

Expects files named like  eval_s<base>_c<coeff>.csv  (output of the
evaluate_pallet_pose node).  Produces:
  - per-CSV time series of dxy and yaw error
  - sweep summary heatmaps (xy RMSE, yaw RMSE, fail rate) over (base_stddev, range_coeff)

Usage:
  python3 plot_pose_eval.py '/tmp/eval_*.csv' --out /tmp/eval_plots
"""
import argparse
import glob
import os
import re
import sys

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt


PARAM_RE = re.compile(r"eval_s(?P<base>[0-9.]+)_c(?P<coeff>[0-9.]+)\.csv$")


def parse_params(path):
    m = PARAM_RE.search(os.path.basename(path))
    if not m:
        return None, None
    return float(m["base"]), float(m["coeff"])


def summarize(df):
    if df.empty:
        return None
    dxy = np.hypot(df["dx_m"], df["dy_m"])
    yaw = df["yaw_err_deg"].abs()
    return {
        "n":           len(df),
        "xy_rmse":     float(np.sqrt((dxy ** 2).mean())),
        "yaw_rmse":    float(np.sqrt((df["yaw_err_deg"] ** 2).mean())),
        "fail_rate":   float((yaw > 5.0).mean()),
    }


def plot_timeseries(df, label, out_path):
    fig, axes = plt.subplots(2, 1, figsize=(8, 4.5), sharex=True)
    t = df["stamp"] - df["stamp"].iloc[0]
    axes[0].plot(t, np.hypot(df["dx_m"], df["dy_m"]), ".-", lw=0.8, ms=3)
    axes[0].set_ylabel("|dxy| [m]")
    axes[0].grid(alpha=0.3)
    axes[1].plot(t, df["yaw_err_deg"], ".-", lw=0.8, ms=3, color="tab:orange")
    axes[1].axhline(5,  ls="--", color="r", lw=0.7)
    axes[1].axhline(-5, ls="--", color="r", lw=0.7)
    axes[1].set_ylabel("yaw err [deg]")
    axes[1].set_xlabel("time [s]")
    axes[1].grid(alpha=0.3)
    fig.suptitle(label)
    fig.tight_layout()
    fig.savefig(out_path, dpi=120)
    plt.close(fig)


def plot_heatmap(table, value_col, title, out_path, cmap="viridis"):
    pivot = table.pivot(index="coeff", columns="base", values=value_col)
    fig, ax = plt.subplots(figsize=(6, 4))
    im = ax.imshow(pivot.values, origin="lower", aspect="auto", cmap=cmap)
    ax.set_xticks(range(len(pivot.columns)))
    ax.set_xticklabels([f"{v:g}" for v in pivot.columns])
    ax.set_yticks(range(len(pivot.index)))
    ax.set_yticklabels([f"{v:g}" for v in pivot.index])
    ax.set_xlabel("base_stddev [m]")
    ax.set_ylabel("range_coeff [/m]")
    ax.set_title(title)
    for (j, i), v in np.ndenumerate(pivot.values):
        ax.text(i, j, f"{v:.3g}", ha="center", va="center",
                color="white" if v > pivot.values.mean() else "black", fontsize=8)
    fig.colorbar(im, ax=ax)
    fig.tight_layout()
    fig.savefig(out_path, dpi=120)
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("glob", help="glob for eval CSVs, e.g. '/tmp/eval_*.csv'")
    ap.add_argument("--out", default="./eval_plots", help="output dir")
    args = ap.parse_args()

    paths = sorted(glob.glob(args.glob))
    if not paths:
        print("no files match", args.glob, file=sys.stderr)
        sys.exit(1)

    os.makedirs(args.out, exist_ok=True)
    rows = []
    for p in paths:
        base, coeff = parse_params(p)
        df = pd.read_csv(p)
        s = summarize(df)
        if s is None:
            print(f"skip {p}: empty")
            continue
        label = f"base={base}  coeff={coeff}  n={s['n']}"
        plot_timeseries(df, label, os.path.join(args.out, os.path.basename(p) + ".png"))
        rows.append({"base": base, "coeff": coeff, **s})
        print(f"{label:40s}  xy_rmse={s['xy_rmse']:.4f}m  yaw_rmse={s['yaw_rmse']:.2f}deg  fail={s['fail_rate']*100:.1f}%")

    if not rows:
        return
    table = pd.DataFrame(rows)
    table.to_csv(os.path.join(args.out, "summary.csv"), index=False)

    if table["base"].nunique() > 1 and table["coeff"].nunique() > 1:
        plot_heatmap(table, "xy_rmse",   "xy RMSE [m]",            os.path.join(args.out, "heatmap_xy.png"))
        plot_heatmap(table, "yaw_rmse",  "yaw RMSE [deg]",         os.path.join(args.out, "heatmap_yaw.png"), cmap="magma")
        plot_heatmap(table, "fail_rate", "|yaw|>5deg fraction",    os.path.join(args.out, "heatmap_fail.png"), cmap="inferno")
    print(f"\nwrote summary + plots to {args.out}/")


if __name__ == "__main__":
    main()
