#!/usr/bin/env python3
"""
Generate the "published pose convention" figure for Chapter 5.

Shows a wireframe pallet in 3/4 perspective together with two coordinate
frames so the reader can see at a glance how the conventions differ:

  - Pallet object frame at the front-face centre (published pose):
        x (red)   = forward / outward face normal (toward camera)
        y (green) = lateral / along pallet face width
        z (blue)  = up / vertical
  - Camera optical frame at the camera body (ROS REP-103/104):
        x (red)   = right in image
        y (green) = down in image
        z (blue)  = forward along the optical axis (into the scene)
  - The matched template band (top deck + 3 stringers) drawn on the front face
    to remind the reader what the detector actually locks onto.

Usage:
  python3 plot_convention_figure.py [--out /tmp/convention.pdf] [--show]
"""
import argparse
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection


# Pallet dimensions matching the template in primitive_detectors.yaml
PALLET_W   = 0.80          # width along y (lateral)
DECK_H     = 0.045         # top-deck height
STRINGER_H = 0.10          # stringer height
STRINGER_W = 0.10          # stringer width (each of the 3)
PALLET_L   = 1.22          # long axis depth; pallet body extends to -x
CAM_DIST   = 1.8           # camera placed at (CAM_DIST, 0, 0)
TEMPLATE_H = DECK_H + STRINGER_H


def draw_box_wireframe(ax, x0, x1, y0, y1, z0, z1,
                       color="saddlebrown", lw=1.2, alpha=1.0):
    c = np.array([
        [x0, y0, z0], [x1, y0, z0], [x1, y1, z0], [x0, y1, z0],
        [x0, y0, z1], [x1, y0, z1], [x1, y1, z1], [x0, y1, z1],
    ])
    edges = [(0, 1), (1, 2), (2, 3), (3, 0),
             (4, 5), (5, 6), (6, 7), (7, 4),
             (0, 4), (1, 5), (2, 6), (3, 7)]
    for i, j in edges:
        ax.plot(*zip(c[i], c[j]), color=color, lw=lw, alpha=alpha)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default="/tmp/convention.pdf",
                    help="output figure (extension determines format)")
    ap.add_argument("--show", action="store_true",
                    help="also open an interactive viewer")
    args = ap.parse_args()

    fig = plt.figure(figsize=(11, 7.0))
    ax = fig.add_subplot(111, projection="3d")

    # Origin at the front-face centre of the matched template band.
    z_band_lo = -TEMPLATE_H / 2
    z_band_hi =  TEMPLATE_H / 2
    z_deck_lo = z_band_hi - DECK_H

    # Top deck (full width, full depth)
    draw_box_wireframe(ax, -PALLET_L, 0,
                       -PALLET_W / 2, PALLET_W / 2,
                       z_deck_lo, z_band_hi)

    # Three stringers (left / middle / right)
    s_centers_y = [-PALLET_W / 2 + STRINGER_W / 2,
                   0.0,
                   PALLET_W / 2 - STRINGER_W / 2]
    for sy in s_centers_y:
        draw_box_wireframe(ax, -PALLET_L, 0,
                           sy - STRINGER_W / 2, sy + STRINGER_W / 2,
                           z_band_lo, z_deck_lo)

    # Faint shade on the front face so the viewer's eye anchors there
    front_face = np.array([
        [0, -PALLET_W / 2, z_band_lo],
        [0,  PALLET_W / 2, z_band_lo],
        [0,  PALLET_W / 2, z_band_hi],
        [0, -PALLET_W / 2, z_band_hi],
    ])
    ax.add_collection3d(Poly3DCollection([front_face], alpha=0.08,
                                         facecolor="saddlebrown",
                                         edgecolor="none"))

    # Axis arrows at the front-face centre.
    # z is a bit shorter than x/y so the "z (up)" label sits inside the axes
    # rather than running into the figure title.
    L  = 0.55
    Lz = 0.38
    ax.quiver(0, 0, 0, L, 0, 0, color="red",
              lw=2.5, arrow_length_ratio=0.12, zorder=5)
    ax.quiver(0, 0, 0, 0, L, 0, color="green",
              lw=2.5, arrow_length_ratio=0.12, zorder=5)
    ax.quiver(0, 0, 0, 0, 0, Lz, color="blue",
              lw=2.5, arrow_length_ratio=0.17, zorder=5)

    lbl = dict(fontsize=11, weight="bold")
    # Labels sit at the same z as their arrow tip so x/y read at arrow level.
    ax.text(L * 1.08, 0, 0.0, "x  (forward,\n    toward camera)",
            color="red", **lbl)
    ax.text(0.0, L * 1.08, 0.0, "y  (lateral)", color="green", **lbl)
    ax.text(0.04, 0.0, Lz * 1.05, "z  (up)", color="blue", **lbl)

    # Origin marker
    ax.scatter([0], [0], [0], color="black", s=30, zorder=6)
    ax.text(-0.05, -0.05, -0.18, "pallet  (front-face centre)",
            fontsize=9, color="black")

    # Camera icon at +x on the optical axis
    cam = 0.10
    draw_box_wireframe(ax,
                       CAM_DIST - cam / 2, CAM_DIST + cam / 2,
                       -cam / 2, cam / 2, -cam / 2, cam / 2,
                       color="black", lw=1.4)
    ax.plot([CAM_DIST - cam / 2, CAM_DIST - cam / 2 - 0.08],
            [0, 0], [0, 0], color="black", lw=1.8)  # lens stub
    ax.text(CAM_DIST + 0.05, 0, 0.13, "camera", fontsize=10)

    # Dotted optical axis between the pallet centre and the camera lens
    ax.plot([0, CAM_DIST - cam / 2 - 0.08], [0, 0], [0, 0],
            color="0.4", ls=":", lw=0.9, alpha=0.8)

    # --- Camera optical frame (ROS REP-103/104): x right, y down, z forward ---
    # Camera looks back toward the pallet (i.e. in the -x_world direction).
    cam_o = np.array([CAM_DIST, 0.0, 0.0])
    Lc = 0.22
    # z_optical = -x_world (forward into scene)
    ax.quiver(cam_o[0], cam_o[1], cam_o[2], -Lc, 0, 0, color="blue",
              lw=2.0, arrow_length_ratio=0.20, zorder=5)
    # x_optical = -y_world (right when looking down -x_world)
    ax.quiver(cam_o[0], cam_o[1], cam_o[2], 0, -Lc, 0, color="red",
              lw=2.0, arrow_length_ratio=0.20, zorder=5)
    # y_optical = -z_world (down in the image)
    ax.quiver(cam_o[0], cam_o[1], cam_o[2], 0, 0, -Lc, color="green",
              lw=2.0, arrow_length_ratio=0.20, zorder=5)

    clbl = dict(fontsize=9.5, weight="bold")
    ax.text(cam_o[0] - Lc * 1.15, 0.04, 0.02, "z (fwd)",
            color="blue", **clbl)
    ax.text(cam_o[0] + 0.02, -Lc * 1.20, 0.02, "x (right)",
            color="red", **clbl)
    ax.text(cam_o[0] + 0.04, 0.02, -Lc * 1.15, "y (down)",
            color="green", **clbl)

    # Cosmetics
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_zlabel("z [m]")
    ax.set_box_aspect([3, 1.5, 1.1])
    ax.view_init(elev=22, azim=-62)
    ax.set_title("Coordinate-frame conventions used by the pallet detector",
                 fontsize=12, pad=6)
    ax.grid(True, alpha=0.3)
    for pane in (ax.xaxis.pane, ax.yaxis.pane, ax.zaxis.pane):
        pane.set_alpha(0.0)

    plt.tight_layout()
    fig.savefig(args.out, dpi=180, bbox_inches="tight")
    print(f"wrote {args.out}")
    if args.show:
        plt.show()


if __name__ == "__main__":
    main()
