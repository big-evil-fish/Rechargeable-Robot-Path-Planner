#!/usr/bin/env python3
"""
Emit a runtest-format map: perfect maze (DFS carve) on odd sizes, many chargers,
1-based R/G/K matching runtest.cpp.

File grid (M section): one line per x (1..width); on each line, y=1..height
comma-separated; 0 = free, nonzero = wall (matches existing debug_maps).

Usage:
  python gen_large_maze_map.py -o debug_maps/out.txt --width 129 --height 129 --chargers 28 --seed 1
  python gen_large_maze_map.py --preset huge   --chargers 48 -o debug_maps/maze.txt --seed 2
  python gen_large_maze_map.py --preset stress --chargers 60 -o debug_maps/big.txt --seed 0

Presets: medium 65, large 101, huge 151, stress 201 (all odd sizes after trim).

Each output map includes a ring of wall cells one thick on all four sides around the
carved maze (playable area is (N-2)x(N-2)); start/goal/chargers lie in the interior.
"""

from __future__ import annotations

import argparse
import random
import sys
from collections import deque


def carve_maze_dfs(width: int, height: int, rng: random.Random) -> list[list[int]]:
    """1 = wall/obstacle in file, 0 = free. Recursive backtracker on odd-step grid."""
    if width % 2 == 0:
        width -= 1
    if height % 2 == 0:
        height -= 1
    if width < 5 or height < 5:
        raise ValueError("width/height must be >= 5 (odd recommended)")

    g = [[1] * height for _ in range(width)]

    def neighbors(cx: int, cy: int) -> list[tuple[int, int, int, int]]:
        opts = []
        for dx, dy in ((0, -2), (0, 2), (-2, 0), (2, 0)):
            nx, ny = cx + dx, cy + dy
            if 0 <= nx < width and 0 <= ny < height and g[nx][ny] == 1:
                opts.append((nx, ny, cx + dx // 2, cy + dy // 2))
        rng.shuffle(opts)
        return opts

    stack = [(0, 0)]
    g[0][0] = 0
    while stack:
        cx, cy = stack[-1]
        opts = neighbors(cx, cy)
        if not opts:
            stack.pop()
            continue
        nx, ny, wx, wy = opts[0]
        g[wx][wy] = 0
        g[nx][ny] = 0
        stack.append((nx, ny))

    # Opposite corner free for far-goal placement
    g[width - 1][height - 1] = 0
    return g


def bfs_dist(
    g: list[list[int]], sx: int, sy: int
) -> tuple[list[list[int | None]], list[tuple[int, int]]]:
    """8-connected free cells (value 0). Returns dist grid and list of (x,y) free cells."""
    w, h = len(g), len(g[0])
    dist: list[list[int | None]] = [[None] * h for _ in range(w)]
    q: deque[tuple[int, int]] = deque()
    if g[sx][sy] != 0:
        return dist, []
    dist[sx][sy] = 0
    q.append((sx, sy))
    cells: list[tuple[int, int]] = []
    while q:
        x, y = q.popleft()
        cells.append((x, y))
        d = dist[x][y]
        assert d is not None
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                if dx == 0 and dy == 0:
                    continue
                nx, ny = x + dx, y + dy
                if 0 <= nx < w and 0 <= ny < h and g[nx][ny] == 0 and dist[nx][ny] is None:
                    dist[nx][ny] = d + 1
                    q.append((nx, ny))
    return dist, cells


def pick_goal_far(
    dist: list[list[int | None]], cells: list[tuple[int, int]]
) -> tuple[int, int]:
    best = cells[0]
    bd = -1
    for x, y in cells:
        d = dist[x][y]
        if d is not None and d > bd:
            bd = d
            best = (x, y)
    return best


def place_chargers(
    g: list[list[int]],
    free: list[tuple[int, int]],
    avoid: set[tuple[int, int]],
    k: int,
    rng: random.Random,
    min_sep: int,
) -> list[tuple[int, int]]:
    """Greedy spread: random order, accept if Chebyshev >= min_sep to chosen."""
    cand = [(x, y) for x, y in free if (x, y) not in avoid]
    rng.shuffle(cand)
    out: list[tuple[int, int]] = []

    def ok(cx: int, cy: int) -> bool:
        for px, py in out:
            if max(abs(cx - px), abs(cy - py)) < min_sep:
                return False
        return True

    for x, y in cand:
        if len(out) >= k:
            break
        if ok(x, y):
            out.append((x, y))
    # If greedy spread under-filled, fill remaining without sep constraint
    for x, y in cand:
        if len(out) >= k:
            break
        if (x, y) not in out and (x, y) not in avoid:
            out.append((x, y))
    return out[:k]


def pad_with_wall_ring(inner: list[list[int]]) -> list[list[int]]:
    """Surround inner maze (0 free, 1 wall) with a single layer of walls on all sides."""
    wi, hi = len(inner), len(inner[0])
    wo, ho = wi + 2, hi + 2
    g = [[1] * ho for _ in range(wo)]
    for xi in range(wi):
        for yi in range(hi):
            g[xi + 1][yi + 1] = inner[xi][yi]
    return g


def write_map(
    path: str,
    g: list[list[int]],
    rx: int,
    ry: int,
    gx: int,
    gy: int,
    chargers: list[tuple[int, int]],
    detection: int,
    charge_max: int,
    charge_start: int,
) -> None:
    w, h = len(g), len(g[0])
    lines = [
        "N",
        f"{w},{h}",
        "D",
        str(detection),
        "R",
        f"{rx},{ry}",
        "B",
        f"{charge_max},{charge_start}",
        "K",
        str(len(chargers)),
    ]
    for cx, cy in chargers:
        lines.append(f"{cx},{cy}")
    lines.append("G")
    lines.append(f"{gx},{gy}")
    lines.append("M")
    for xi in range(w):
        row = ",".join(str(g[xi][yi]) for yi in range(h))
        lines.append(row)

    text = "\n".join(lines) + "\n"
    with open(path, "w", encoding="ascii", newline="\n") as f:
        f.write(text)


def main() -> int:
    p = argparse.ArgumentParser(description="Generate large maze maps for runtest.")
    p.add_argument("-o", "--output", required=True, help="Output .txt path")
    p.add_argument("--width", type=int, default=101, help="Outer width (even values shrink by 1)")
    p.add_argument("--height", type=int, default=101, help="Outer height")
    p.add_argument("--chargers", type=int, default=24, help="Number of charger pairs K")
    p.add_argument("--seed", type=int, default=0)
    p.add_argument("--detection", type=int, default=3, help="Detection radius D (Chebyshev)")
    p.add_argument("--charge-max", type=int, default=200)
    p.add_argument("--charge-start", type=int, default=160)
    p.add_argument(
        "--min-charger-sep",
        type=int,
        default=6,
        help="Try to space chargers at least this Chebyshev apart before filling",
    )
    p.add_argument(
        "--preset",
        choices=("none", "medium", "large", "huge", "stress"),
        default="none",
        help="Overrides width/height: medium 65, large 101, huge 151, stress 201",
    )
    args = p.parse_args()

    if args.preset == "medium":
        args.width, args.height = 65, 65
    elif args.preset == "large":
        args.width, args.height = 101, 101
    elif args.preset == "huge":
        args.width, args.height = 151, 151
    elif args.preset == "stress":
        args.width, args.height = 201, 201

    rng = random.Random(args.seed)
    # Carve slightly smaller grid, then embed in (width x height) with a 1-cell wall border.
    inner_w = max(5, args.width - 2)
    inner_h = max(5, args.height - 2)
    if inner_w % 2 == 0:
        inner_w -= 1
    if inner_h % 2 == 0:
        inner_h -= 1
    inner = carve_maze_dfs(inner_w, inner_h, rng)
    g = pad_with_wall_ring(inner)
    w, h = len(g), len(g[0])

    # Inner carve start (0,0) maps to padded 0-based (1,1) → runtest (2,2)
    sx, sy = 1, 1
    if g[sx][sy] != 0:
        print("maze start (inside padding) not free", file=sys.stderr)
        return 1
    dist, cells = bfs_dist(g, sx, sy)
    if not cells:
        print("no free cells", file=sys.stderr)
        return 1
    gx, gy = pick_goal_far(dist, cells)

    avoid = {(sx, sy), (gx, gy)}
    chargers = place_chargers(
        g, cells, avoid, args.chargers, rng, min_sep=args.min_charger_sep
    )
    if len(chargers) < args.chargers:
        print(
            f"warning: only placed {len(chargers)} chargers (free cells or spacing)",
            file=sys.stderr,
        )

    write_map(
        args.output,
        g,
        sx + 1,
        sy + 1,
        gx + 1,
        gy + 1,
        [(cx + 1, cy + 1) for cx, cy in chargers],
        args.detection,
        args.charge_max,
        args.charge_start,
    )
    print(
        f"wrote {args.output}  ({w}x{h} with 1-cell wall ring, {len(chargers)} chargers, "
        f"goal dist ~ {dist[gx][gy]})"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
