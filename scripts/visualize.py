#!/usr/bin/env python3
"""Render the planner output: a static summary PNG and an animated GIF showing
the vehicle tracking the Hybrid A* path.

Usage:
    python3 visualize.py <out_dir> [--no-gif]

Reads <out_dir>/{meta.json, obstacles.csv, path.csv, traj.csv} produced by the
C++ demo and writes <out_dir>/{summary.png, demo.gif}.
"""
import csv
import json
import math
import sys
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
import numpy as np
import imageio.v2 as imageio


def load_csv(path):
    with open(path) as f:
        return list(csv.DictReader(f))


def car_corners(x, y, theta, length, width):
    # Rear axle at (x, y); shift body centre forward a bit for a natural look.
    cx = x + math.cos(theta) * (length / 2 - 1.0)
    cy = y + math.sin(theta) * (length / 2 - 1.0)
    c, s = math.cos(theta), math.sin(theta)
    hl, hw = length / 2, width / 2
    pts = [(-hl, -hw), (hl, -hw), (hl, hw), (-hl, hw)]
    return [(cx + px * c - py * s, cy + px * s + py * c) for px, py in pts]


def draw_static(ax, meta, obstacles, path, traj):
    W, H = meta["map"]["width"], meta["map"]["height"]
    ax.set_xlim(0, W)
    ax.set_ylim(0, H)
    ax.set_aspect("equal")
    ax.set_facecolor("#0e1117")

    for o in obstacles:
        x0, y0 = float(o["x0"]), float(o["y0"])
        x1, y1 = float(o["x1"]), float(o["y1"])
        ax.add_patch(Rectangle((min(x0, x1), min(y0, y1)), abs(x1 - x0),
                               abs(y1 - y0), color="#3a4152"))

    if path:
        px = [float(r["x"]) for r in path]
        py = [float(r["y"]) for r in path]
        ax.plot(px, py, "-", color="#f2c744", lw=2.2, label="Hybrid A* plan")
    if traj:
        tx = [float(r["x"]) for r in traj]
        ty = [float(r["y"]) for r in traj]
        ax.plot(tx, ty, "-", color="#2ea3f2", lw=1.6, alpha=0.9,
                label="tracked trajectory")

    sx, sy, sth = meta["start"]
    gx, gy, gth = meta["goal"]
    ax.plot(sx, sy, "o", color="#3ddc84", ms=9, label="start")
    ax.plot(gx, gy, "*", color="#ff6b6b", ms=16, label="goal")
    ax.legend(loc="upper right", facecolor="#1b1f2a", labelcolor="white",
              framealpha=0.9)
    ax.tick_params(colors="#8b93a7")
    for spine in ax.spines.values():
        spine.set_color("#2a2f3a")


def main():
    if len(sys.argv) < 2:
        print("usage: visualize.py <out_dir> [--no-gif]")
        sys.exit(1)
    out = sys.argv[1]
    make_gif = "--no-gif" not in sys.argv

    meta = json.load(open(os.path.join(out, "meta.json")))
    obstacles = load_csv(os.path.join(out, "obstacles.csv"))
    path = load_csv(os.path.join(out, "path.csv"))
    traj_path = os.path.join(out, "traj.csv")
    traj = load_csv(traj_path) if os.path.exists(traj_path) else []

    veh = meta["vehicle"]
    scen = meta["scenario"]
    P = meta["planning"]
    T = meta["tracking"]

    # ---- static summary ----
    fig, ax = plt.subplots(figsize=(11, 7.5))
    fig.patch.set_facecolor("#0e1117")
    draw_static(ax, meta, obstacles, path, traj)
    title = (f"Hybrid A* + Pure Pursuit  —  scenario: {scen}\n"
             f"plan {P['plan_time_ms']:.0f} ms, {P['nodes_expanded']} nodes, "
             f"path {P['path_length_m']:.1f} m  |  "
             f"tracking RMS {T['cte_rms_m']*100:.1f} cm, "
             f"max {T['cte_max_m']*100:.1f} cm")
    ax.set_title(title, color="white", fontsize=12)
    fig.tight_layout()
    png = os.path.join(out, "summary.png")
    fig.savefig(png, dpi=130, facecolor=fig.get_facecolor())
    plt.close(fig)
    print("wrote", png)

    if not make_gif or not traj:
        return

    # ---- animation ----
    step = max(1, len(traj) // 90)
    frames = []
    L, Wd = veh["length"], veh["width"]
    for i in range(0, len(traj), step):
        fig, ax = plt.subplots(figsize=(9, 6))
        fig.patch.set_facecolor("#0e1117")
        draw_static(ax, meta, obstacles, path, traj[: i + 1])
        r = traj[i]
        x, y, th = float(r["x"]), float(r["y"]), float(r["theta"])
        corners = car_corners(x, y, th, L, Wd)
        poly = plt.Polygon(corners, closed=True, color="#2ea3f2", alpha=0.85)
        ax.add_patch(poly)
        # look-ahead target
        ax.plot(float(r["target_x"]), float(r["target_y"]), "x",
                color="#ff6b6b", ms=9, mew=2)
        ax.set_title(f"{scen}  t={float(r['t']):.1f}s  "
                     f"v={float(r['v']):.1f} m/s  "
                     f"cte={float(r['cte'])*100:+.0f} cm",
                     color="white", fontsize=11)
        fig.tight_layout()
        fig.canvas.draw()
        buf = np.asarray(fig.canvas.buffer_rgba())
        frames.append(buf[..., :3].copy())
        plt.close(fig)

    gif = os.path.join(out, "demo.gif")
    imageio.mimsave(gif, frames, duration=0.08, loop=0)
    print("wrote", gif, f"({len(frames)} frames)")


if __name__ == "__main__":
    main()
