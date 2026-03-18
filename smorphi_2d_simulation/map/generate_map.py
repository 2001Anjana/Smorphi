#!/usr/bin/env python3
"""Generate a 3x3 m test map (PGM) for Smorphi auto-transform simulation.

Resolution: 0.05 m/pixel  →  60 x 60 pixels
Coordinate system: origin at bottom-left (0,0).

The map contains:
  - Outer walls (border)
  - A corridor with a ~25 cm gap (passable only in I-shape)
  - A corridor with a ~40 cm gap (passable in O-shape)
  - A blocked passage with ~14 cm gap (impassable)
  - Open areas for free navigation

PGM convention:  254 = free,  0 = occupied,  205 = unknown
"""

import struct, sys

W, H = 60, 60  # pixels
RES = 0.05      # m per pixel

# Start with everything free
grid = [[254] * W for _ in range(H)]


def set_occupied(r, c):
    """Mark pixel (row, col) as occupied.  Row 0 = top of PGM = y_max."""
    if 0 <= r < H and 0 <= c < W:
        grid[r][c] = 0


def fill_rect(r0, c0, r1, c1):
    """Fill rectangle [r0..r1) x [c0..c1) with occupied cells."""
    for r in range(r0, r1):
        for c in range(c0, c1):
            set_occupied(r, c)


# ---------- Outer walls (2-pixel thick border) ----------
for i in range(W):
    for t in range(2):
        set_occupied(t, i)       # top wall
        set_occupied(H-1-t, i)   # bottom wall
        set_occupied(i, t)       # left wall
        set_occupied(i, W-1-t)   # right wall

# ---------- Obstacle layout ----------
# We design obstacles creating corridors along the y-axis (vertical in map).
# Robot travels roughly left-to-right (in world: increasing x).
#
# PGM rows: row 0 = top = y=3.0m,  row 59 = bottom = y=0.0m
# PGM cols: col 0 = left = x=0.0m, col 59 = right = x=3.0m
#
# Helper: world (x,y) → pixel (col, row)
#   col = int(x / RES)
#   row = H - 1 - int(y / RES)

def world_to_pixel(x_m, y_m):
    c = int(x_m / RES)
    r = H - 1 - int(y_m / RES)
    return r, c

# ---- Vertical wall 1: at x=1.0m, from y=0.7m to y=1.25m (bottom portion) ----
# Creates a gap at the bottom between this wall and the bottom border.
# Gap ~ 25 cm  (y from 0.10m to 0.35m in world → rows 57 to 52 → gap = 5 px = 0.25m)
# Wall from y=0.45m to y=1.60m
for y_px in range(28, 49):  # rows 28..48 → y from ~0.55m to ~1.60m
    for x_px in range(18, 22):  # cols 18..21 → x from 0.90m to 1.10m  (4 px = 0.20m thick wall)
        set_occupied(y_px, x_px)

# Bottom extension of wall 1 to create 25cm gap
# Wall continues from y=0.10m to y=0.30m → leaves gap from y=0.30m to y=0.55m = 5 px = 0.25m
for y_px in range(54, 58):  # rows 54..57 → y from ~0.10m to ~0.30m
    for x_px in range(18, 22):
        set_occupied(y_px, x_px)

# ---- Vertical wall 2: at x=2.0m, from y=1.4m to y=2.8m (top portion) ----
# Creates a gap at the top between this wall and the top border.
# Gap ~ 40cm (y from 2.8m to top wall ~2.9m → but we make it wider)
# Actually, let's design this as a gap in the middle of the wall.
# Wall top portion: y=1.8m to y=2.9m
for y_px in range(2, 24):  # rows 2..23 → y from ~1.80m to ~2.90m
    for x_px in range(38, 42):  # cols 38..41 → x from 1.90m to 2.10m
        set_occupied(y_px, x_px)

# Wall bottom portion: y=0.10m to y=1.0m → gap from y=1.0m to y=1.40m = 8 px = 0.40m
for y_px in range(40, 58):  # rows 40..57 → y from ~0.10m to ~1.0m
    for x_px in range(38, 42):
        set_occupied(y_px, x_px)

# Gap between wall 2 portions: rows 24..39 → y from 1.0m to 1.80m = 16 px = 0.80m
# That's too wide. Let me adjust: bottom portion up to y=1.40m → rows 32..57
for y_px in range(32, 58):
    for x_px in range(38, 42):
        set_occupied(y_px, x_px)

# Now gap is rows 24..31 → y from 1.40m to 1.80m = 8 px = 0.40m ✓

# ---- Small block creating a 14cm (impassable) gap ----
# Near bottom-right area. A small block at x=2.5m, creating a gap of ~14cm
# with another block or wall.
# Block at x=2.5m, y=0.3m to y=0.7m
for y_px in range(46, 54):  # rows 46..53 → y from 0.30m to 0.70m
    for x_px in range(48, 52):  # cols 48..51 → x from 2.40m to 2.60m
        set_occupied(y_px, x_px)

# Another block at x=2.5m, y=0.87m to y=1.4m → gap from y=0.70 to y=0.87 = ~3.4 px ≈ 0.17m
# Actually let's make the gap = 0.14m = ~2.8 px → 3 px gap
# Block from y=0.85m to y=1.4m
for y_px in range(32, 43):  # rows 32..42 → y from ~0.85m to ~1.40m
    for x_px in range(48, 52):
        set_occupied(y_px, x_px)

# Gap between these two blocks: rows 43..45 → y from 0.70m to 0.85m = 3 px = 0.15m ✓ (impassable)


# ---------- Write PGM ----------
pgm_path = "/home/anjana/Documents/PR2/Simulation_03_17/smorphi_2d_simulation/map/sim_map.pgm"

with open(pgm_path, 'w') as f:
    f.write("P2\n")
    f.write(f"# Smorphi 2D simulation test map\n")
    f.write(f"{W} {H}\n")
    f.write("254\n")
    for r in range(H):
        row_str = " ".join(str(grid[r][c]) for c in range(W))
        f.write(row_str + "\n")

print(f"Map written to {pgm_path}")
print(f"  Size: {W}x{H} pixels,  {W*RES:.1f}x{H*RES:.1f} m")
print(f"  25 cm gap (I-shape only) at x≈1.0m")
print(f"  40 cm gap (O-shape ok)   at x≈2.0m")
print(f"  15 cm gap (impassable)   at x≈2.5m")
