#!/usr/bin/env python3
"""
Visualize a runtest-format map (N, D, R, B, K, …, G, M) and robot_trajectory.txt
(time, x, y, charge per line, as written by runtest.cpp). Path and waypoints are
colored by battery (green = high, red = low) using B from the map as the scale max.

Usage:
  python visualize_runtest_map.py debug_maps/10x10_simple_explore.txt
  python visualize_runtest_map.py map.txt path/to/robot_trajectory.txt
  python visualize_runtest_map.py map.txt --save out.png --no-show
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.collections import LineCollection
from matplotlib.colors import Normalize


def _read_nonempty_lines(path: Path) -> list[str]:
    raw = path.read_text(encoding="utf-8", errors="replace").splitlines()
    return [ln.strip() for ln in raw if ln.strip()]


def parse_runtest_map(path: Path) -> dict:
    lines = _read_nonempty_lines(path)
    i = 0

    def take() -> str:
        nonlocal i
        if i >= len(lines):
            raise ValueError(f"unexpected EOF in {path}")
        s = lines[i]
        i += 1
        return s

    if take() != "N":
        raise ValueError("expected N")
    xs, ys = map(int, take().split(","))
    if take() != "D":
        raise ValueError("expected D")
    detection = int(take())
    if take() != "R":
        raise ValueError("expected R")
    rx, ry = map(int, take().split(","))
    if take() != "B":
        raise ValueError("expected B")
    bmax, b0 = map(int, take().split(","))
    if take() != "K":
        raise ValueError("expected K")
    nk = int(take())
    chargers: list[tuple[int, int]] = []
    for _ in range(nk):
        cx, cy = map(int, take().split(","))
        chargers.append((cx, cy))
    if take() != "G":
        raise ValueError("expected G")
    gx, gy = map(int, take().split(","))
    if take() != "M":
        raise ValueError("expected M")

    grid_rows: list[list[int]] = []
    for _ in range(xs):
        parts = [int(x) for x in take().split(",")]
        if len(parts) != ys:
            raise ValueError(f"expected {ys} y-values per M line, got {len(parts)}")
        grid_rows.append(parts)

    # grid_rows[x_idx][y_idx] for 0-based indices; file x = x_idx+1, y = y_idx+1
    occ = np.array(grid_rows, dtype=float).T  # shape (ys, xs) -> imshow row = y

    return {
        "x_size": xs,
        "y_size": ys,
        "detection": detection,
        "robot": (rx, ry),
        "charge_max": bmax,
        "charge_start": b0,
        "chargers": chargers,
        "goal": (gx, gy),
        "occ": occ,
        "_path": str(path),
    }


def validate_trajectory_vs_map(m: dict, tr: dict[str, np.ndarray], xs: int, ys: int) -> int:
    """Return count of samples outside bounds or on obstacle (file value != 0). Warn to stderr."""
    occ = m["occ"]
    bad = 0
    for k in range(len(tr["x"])):
        x, y = int(tr["x"][k]), int(tr["y"][k])
        if x < 1 or x > xs or y < 1 or y > ys:
            bad += 1
            continue
        if occ[y - 1, x - 1] != 0:
            bad += 1
    if bad:
        print(
            f"Warning: {bad}/{len(tr['x'])} trajectory samples are out of bounds or on a map obstacle "
            f"(file cell != 0). The trajectory may be from a different map than {m.get('_path', 'this file')}.",
            file=sys.stderr,
        )
    return bad


def parse_trajectory(path: Path) -> dict[str, np.ndarray]:
    times: list[int] = []
    xs: list[int] = []
    ys: list[int] = []
    ch: list[int] = []
    with path.open(encoding="utf-8", errors="replace") as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            parts = line.split(",")
            if len(parts) < 4:
                continue
            t, x, y, c = map(int, parts[:4])
            times.append(t)
            xs.append(x)
            ys.append(y)
            ch.append(c)
    if not times:
        raise ValueError(f"no trajectory rows in {path}")
    return {
        "t": np.array(times, dtype=int),
        "x": np.array(xs, dtype=int),
        "y": np.array(ys, dtype=int),
        "charge": np.array(ch, dtype=int),
    }


def main() -> int:
    ap = argparse.ArgumentParser(description="Plot runtest map + robot trajectory.")
    ap.add_argument("map", type=Path, help="Map file (runtest format)")
    ap.add_argument(
        "trajectory",
        type=Path,
        nargs="?",
        default=Path("robot_trajectory.txt"),
        help="Trajectory CSV (default: ./robot_trajectory.txt)",
    )
    ap.add_argument("--save", type=Path, default=None, help="Save figure to PNG (no extension needed)")
    ap.add_argument("--no-show", action="store_true", help="Do not open interactive window")
    ap.add_argument("--dpi", type=int, default=150)
    ap.add_argument(
        "--no-traj-check",
        action="store_true",
        help="Do not warn when trajectory points fall on obstacles or out of bounds",
    )
    args = ap.parse_args()

    if not args.map.is_file():
        print(f"map not found: {args.map}", file=sys.stderr)
        return 1
    if not args.trajectory.is_file():
        print(f"trajectory not found: {args.trajectory}", file=sys.stderr)
        return 1

    m = parse_runtest_map(args.map)
    tr = parse_trajectory(args.trajectory)
    xs, ys = m["x_size"], m["y_size"]

    if not args.no_traj_check:
        validate_trajectory_vs_map(m, tr, xs, ys)

    fig, ax = plt.subplots(figsize=(max(6, xs / 8), max(6, ys / 8)), dpi=args.dpi)
    # imshow: first dim = row = y, second = col = x. extent places pixel [0,0] lower-left at (0.5,0.5)
    # so cell (x,y) = occ[y-1,x-1] is centered at (x,y) for 1-based runtest coords.
    ax.imshow(
        m["occ"],
        origin="lower",
        extent=(0.5, xs + 0.5, 0.5, ys + 0.5),
        aspect="equal",
        interpolation="nearest",
        cmap="gray_r",
        vmin=0,
        vmax=1,
        zorder=0,
    )

    cxs = [c[0] for c in m["chargers"]]
    cys = [c[1] for c in m["chargers"]]
    if cxs:
        ax.scatter(cxs, cys, c="crimson", s=120, marker="*", zorder=3, label="chargers", edgecolors="k", linewidths=0.5)

    gx, gy = m["goal"]
    ax.scatter([gx], [gy], c="limegreen", s=200, marker="s", zorder=4, label="goal", edgecolors="k", linewidths=0.5)

    rx, ry = m["robot"]
    ax.scatter([rx], [ry], c="cyan", s=120, zorder=4, label="start", edgecolors="k", linewidths=0.5)

    # Path colored by battery (segment color = mean charge at its endpoints)
    cmax = max(int(m["charge_max"]), 1)
    norm = Normalize(vmin=0, vmax=cmax)
    cmap_charge = plt.get_cmap("RdYlGn")
    x = tr["x"].astype(float)
    y = tr["y"].astype(float)
    ch = tr["charge"].astype(float)
    points = np.column_stack([x, y])
    sm_for_cbar = None
    if len(points) > 1:
        segments = np.stack([points[:-1], points[1:]], axis=1)
        c_seg = 0.5 * (ch[:-1] + ch[1:])
        lc = LineCollection(
            segments,
            cmap=cmap_charge,
            norm=norm,
            array=c_seg,
            linewidths=2.4,
            alpha=0.92,
            zorder=2,
            capstyle="round",
        )
        ax.add_collection(lc)
        sm_for_cbar = lc
    sc = ax.scatter(
        x,
        y,
        c=ch,
        cmap=cmap_charge,
        norm=norm,
        s=38,
        zorder=3,
        alpha=0.9,
        edgecolors="k",
        linewidths=0.4,
        label="trajectory (color = battery)",
    )
    if sm_for_cbar is None:
        sm_for_cbar = sc
    fig.colorbar(sm_for_cbar, ax=ax, fraction=0.046, pad=0.04, label="battery")

    ax.set_xlim(0.5, xs + 0.5)
    ax.set_ylim(0.5, ys + 0.5)
    ax.set_aspect("equal")
    ax.set_xlabel("x (runtest 1-based)")
    ax.set_ylabel("y (runtest 1-based)")
    ax.set_title(f"{args.map.name}  +  {args.trajectory.name}")
    ax.grid(True, which="major", color="w", alpha=0.15, linestyle="-")
    ax.legend(loc="upper right", fontsize=8)

    fig.tight_layout()
    if args.save:
        out = args.save
        if out.suffix.lower() not in {".png", ".pdf", ".svg"}:
            out = out.with_suffix(".png")
        fig.savefig(out, dpi=args.dpi, bbox_inches="tight")
        print(f"wrote {out}")
    if not args.no_show:
        plt.show()
    else:
        plt.close(fig)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
