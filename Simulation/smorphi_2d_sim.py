#!/usr/bin/env python3
"""
Smorphi 2-D Simulation  –  Auto Shape Transform & A* Path Planning
====================================================================
A standalone Pygame simulation that mirrors the real Smorphi robot's
shape-transformation decision logic and provides A* path-planning with
footprint-aware collision checking.

Controls
--------
Left-click free space : set navigation goal
Right-click           : toggle obstacle cell
1 / 2                 : manual shape -> O / I
A                     : toggle auto-transform
R                     : reset robot
Space                 : pause / resume
L                     : toggle LiDAR ray visualisation
G                     : toggle grid lines
ESC                   : quit
"""

import sys, math, heapq, time, json, os
from typing import List, Tuple, Optional, Set

import pygame

# ──────────────────────────── constants ────────────────────────────
WIN_W, WIN_H = 1100, 750          # window size  (map + HUD)
MAP_W, MAP_H = 800, 750           # map area in pixels
HUD_X        = MAP_W              # HUD starts here
CELL         = 15                 # grid cell size (pixels)
COLS, ROWS   = MAP_W // CELL, MAP_H // CELL   # 53×50

FPS = 60

# Colours
COL_BG       = (18,  18,  24)
COL_GRID     = (35,  35,  50)
COL_WALL     = (70,  75,  90)
COL_WALL_HL  = (90, 100, 130)
COL_FREE     = (28,  28,  38)
COL_PATH     = (100, 220, 255)
COL_PATH_DIM = (33, 73, 85)
COL_GOAL     = (255, 80,  120)
COL_LIDAR    = (0,   255, 130, 50)       # semi-transparent
COL_LIDAR_HIT= (0,   255, 100, 200)
COL_O        = (80,  200, 255)           # O-shape colour
COL_I        = (255, 190, 60)            # I-shape colour
COL_HUD_BG   = (25,  25,  35)
COL_HUD_TXT  = (210, 215, 225)
COL_HUD_VAL  = (120, 230, 180)
COL_HUD_WARN = (255, 100, 100)
COL_HUD_OK   = (100, 255, 160)
COL_HUD_HEAD = (255, 210, 100)

# ── Robot dimensions ──
# O-shape: 5×5 cells (square, wider)
# I-shape: 3×5 cells (narrow width, same length in heading dir)
# "width" here is the side-to-side dimension that matters for gaps
M_PER_CELL = 0.05                        # each cell ~= 5 cm

# Auto-transform thresholds (matching shape_auto_node.py)
TH_I              = 0.35                 # O->I: gap_front < 0.35 m
TH_O              = 0.40                 # I->O: gap_front > 0.40 m
REQUIRED_RADIUS   = 0.40                 # sector clearance needed
BACK_NARROW_THRESH = 0.35                # back gap must exceed to go I->O
STABLE_SAMPLES    = 5                    # consecutive ticks needed
COOLDOWN_S        = 2.0                  # s between transforms

# LiDAR sim
LIDAR_RANGE   = 4.0                      # metres
LIDAR_RAYS    = 180                      # number of rays (full 360)

# Path-following
PURSUIT_LOOKAHEAD = 3                    # cells

# Transform animation
TRANSFORM_DURATION = 0.55               # seconds for pivot animation

# Per-module colours (M1..M4)
MODULE_COLOURS = [
    ( 80, 200, 255),   # M1 – cyan-blue
    ( 60, 230, 160),   # M2 – teal
    (255, 190,  60),   # M3 – amber
    (220,  80, 160),   # M4 – magenta
]


# ──────────────────────────── footprints ──────────────────────────
def footprint_O() -> Set[Tuple[int,int]]:
    """O-shape: 2x2 cells centred on (0,0)."""
    return {
        (0,0), (0,1),
        (1,0), (1,1)
    }
def footprint_I() -> Set[Tuple[int,int]]:
    """I-shape: 1x4 cells centred on (0,0).
    Compact form that can fit through 4-cell-wide corridors."""
    return {
        (0,0), (0,1), (0,2), (0,3)
    }


# ──────────────────────────── map builder ─────────────────────────
def build_default_map() -> List[List[int]]:
    """Three rooms connected by narrow corridors.

    Corridor width = 4 cells.
      O-shape (5×5, offsets -2..+2) needs 5 → CANNOT fit
      I-shape (3×3, offsets -1..+1) needs 3 → CAN fit
    """
    grid = [[0]*COLS for _ in range(ROWS)]

    def rect(r1, c1, r2, c2):
        for r in range(max(0,r1), min(ROWS,r2+1)):
            for c in range(max(0,c1), min(COLS,c2+1)):
                grid[r][c] = 1

    def hwall(r, c1, c2):
        rect(r, c1, r, c2)

    def vwall(c, r1, r2):
        rect(r1, c, r2, c)

    # ── outer boundary ──
    hwall(0, 0, COLS-1)
    hwall(ROWS-1, 0, COLS-1)
    vwall(0, 0, ROWS-1)
    vwall(COLS-1, 0, ROWS-1)

    # ═══ ROOM A (top-left) rows 1-22, cols 1-19 ═══
    # right wall with door opening at rows 11-14
    vwall(20, 1, 10)
    vwall(20, 15, ROWS-2)
    # bottom wall
    hwall(22, 1, 19)

    # ═══ HORIZONTAL CORRIDOR rows 10-15, cols 20-36 ═══
    # 4 free rows: 11, 12, 13, 14  (walls at row 10 and row 15)
    hwall(10, 20, 36)      # top wall
    # bottom wall — gap at cols 24-27 for vertical corridor
    hwall(15, 20, 23)      # bottom left segment
    hwall(15, 28, 36)      # bottom right segment

    # ═══ ROOM B (top-right) rows 1-22, cols 37-COLS-2 ═══
    # left wall with door at rows 11-14
    vwall(37, 1, 10)
    vwall(37, 15, ROWS-2)
    # bottom wall
    hwall(22, 37, COLS-2)

    # ═══ VERTICAL CORRIDOR cols 23-28, rows 16-38 ═══
    # Starts right below horizontal corridor gap (row 16)
    # 4 free cols: 24, 25, 26, 27  (walls at col 23 and col 28)
    vwall(23, 16, 38)      # left wall
    vwall(28, 16, 38)      # right wall

    # ═══ ROOM C (bottom) rows 38-ROWS-2, cols 1-COLS-2 ═══
    # top wall with opening at cols 24-27
    hwall(38, 1, 23)
    hwall(38, 28, COLS-2)

    # ── obstacles in rooms ──
    # Room A
    rect(5, 8, 6, 9)
    rect(16, 5, 17, 6)
    rect(4, 14, 5, 15)

    # Room B
    rect(5, 42, 6, 43)
    rect(14, 45, 15, 47)
    rect(17, 40, 18, 41)

    # Room C
    rect(42, 10, 43, 11)
    rect(44, 30, 45, 31)
    rect(41, 45, 42, 46)
    rect(46, 20, 47, 22)

    return grid


# ──────────────────────────── map loader ──────────────────────────
MAP_FILE = "map.json"     # default JSON produced by map_editor.py

def load_map_from_file(path: str) -> Optional[List[List[int]]]:
    """Load a grid map saved by map_editor.py.
    Returns the grid list-of-lists, or None on failure."""
    if not os.path.exists(path):
        print(f"[Sim] Map file not found: {path}")
        return None
    try:
        with open(path) as f:
            data = json.load(f)
        if len(data) == ROWS and len(data[0]) == COLS:
            print(f"[Sim] Loaded map: {path}")
            return data
        # Resize / crop to fit current grid dimensions
        grid = [[0]*COLS for _ in range(ROWS)]
        for r in range(min(len(data), ROWS)):
            for c in range(min(len(data[r]), COLS)):
                grid[r][c] = data[r][c]
        print(f"[Sim] Loaded map (resized to {ROWS}x{COLS}): {path}")
        return grid
    except Exception as e:
        print(f"[Sim] Failed to load map '{path}': {e}")
        return None


# ──────────────────────────── A* planner ──────────────────────────
def astar(grid, start, goal, footprint_cells) -> Optional[List[Tuple[int,int]]]:
    """A* on grid. start/goal are (row, col).
    footprint_cells: set of (dr, dc) offsets that must all be free."""
    sr, sc = start
    gr, gc = goal

    if not _passable(grid, sr, sc, footprint_cells):
        return None
    if not _passable(grid, gr, gc, footprint_cells):
        return None

    open_set: list = []
    heapq.heappush(open_set, (0.0, sr, sc))
    came_from = {}
    g_score = {(sr, sc): 0.0}

    def h(r, c):
        return math.hypot(r - gr, c - gc)

    visited: Set[Tuple[int,int]] = set()

    while open_set:
        _, cr, cc = heapq.heappop(open_set)
        if (cr, cc) in visited:
            continue
        visited.add((cr, cc))
        if (cr, cc) == (gr, gc):
            path = []
            n = (gr, gc)
            while n in came_from:
                path.append(n)
                n = came_from[n]
            path.append((sr, sc))
            path.reverse()
            return path

        for dr, dc in [(-1,0),(1,0),(0,-1),(0,1),(-1,-1),(-1,1),(1,-1),(1,1)]:
            nr, nc = cr+dr, cc+dc
            if not _passable(grid, nr, nc, footprint_cells):
                continue
            cost = 1.414 if (dr != 0 and dc != 0) else 1.0
            ng = g_score[(cr,cc)] + cost
            if ng < g_score.get((nr,nc), float('inf')):
                g_score[(nr,nc)] = ng
                came_from[(nr,nc)] = (cr,cc)
                heapq.heappush(open_set, (ng + h(nr,nc), nr, nc))

    return None


def _passable(grid, r, c, footprint):
    for dr, dc in footprint:
        rr, cc = r+dr, c+dc
        if rr < 0 or rr >= ROWS or cc < 0 or cc >= COLS:
            return False
        if grid[rr][cc] == 1:
            return False
    return True


# ──────────────────────────── LiDAR sim ───────────────────────────
def cast_ray(grid, ox, oy, angle, max_range_cells):
    """Cast a single ray from (ox,oy) in world-pixel coords.
    Returns distance in cells and hit point (px coords)."""
    dx = math.cos(angle)
    dy = math.sin(angle)
    step = 0.4   # sub-cell steps for accuracy
    dist = 0.0
    while dist < max_range_cells:
        dist += step
        px = ox + dx * dist * CELL
        py = oy + dy * dist * CELL
        c = int(px) // CELL
        r = int(py) // CELL
        if r < 0 or r >= ROWS or c < 0 or c >= COLS:
            return dist, (px, py)
        if grid[r][c] == 1:
            return dist, (px, py)
    return max_range_cells, (ox + dx*max_range_cells*CELL, oy + dy*max_range_cells*CELL)


def simulate_lidar(grid, cx, cy, heading, num_rays, max_range_m):
    """Full 360deg LiDAR from pixel centre (cx,cy).
    Returns list of (angle_rad, distance_m, hit_px, hit_py)."""
    max_range_cells = max_range_m / M_PER_CELL
    results = []
    for i in range(num_rays):
        angle = heading + 2*math.pi * i / num_rays
        dist_cells, (hx,hy) = cast_ray(grid, cx, cy, angle, max_range_cells)
        dist_m = dist_cells * M_PER_CELL
        results.append((angle, dist_m, hx, hy))
    return results


def compute_gap_metrics(lidar_results, heading):
    """From lidar results compute gap_width_front, gap_width_back,
    directional distances (F/L/B/R), sector_min (left->back)."""
    half = math.radians(20)   # angular window for gap slice
    dir_half = math.radians(10)

    def rel(a):
        """Relative angle to heading, normalised to [-pi, pi]."""
        d = (a - heading + math.pi) % (2*math.pi) - math.pi
        return d

    front_pts_left = []
    front_pts_right = []
    back_pts_left = []
    back_pts_right = []
    dirs = {'front': [], 'left': [], 'back': [], 'right': []}
    sector_vals = []

    for angle, dist_m, _, _ in lidar_results:
        ra = rel(angle)

        # Front gap: points near heading (ra ~ 0)
        if abs(ra) < half:
            lateral = dist_m * math.sin(ra)
            if lateral > 0.01:
                front_pts_left.append(lateral)
            elif lateral < -0.01:
                front_pts_right.append(lateral)

        # Back gap: points near heading+180 (|ra| ~ pi)
        ba = abs(abs(ra) - math.pi)
        if ba < half:
            lateral = dist_m * math.sin(ra)
            if lateral > 0.01:
                back_pts_left.append(abs(lateral))
            elif lateral < -0.01:
                back_pts_right.append(abs(lateral))

        # Directional distances
        if abs(ra) <= dir_half:
            dirs['front'].append(dist_m)
        if abs(ra - math.pi/2) <= dir_half:
            dirs['left'].append(dist_m)
        # back: ra near +/-pi
        if abs(abs(ra) - math.pi) <= dir_half:
            dirs['back'].append(dist_m)
        if abs(ra + math.pi/2) <= dir_half:
            dirs['right'].append(dist_m)

        # Sector: left(90) -> back(180), relative angle in [pi/4, pi]
        if math.pi/4 <= ra <= math.pi:
            sector_vals.append(dist_m)

    # Gap width = left_min + |right_max| (distance between closest walls)
    gap_front = None
    if front_pts_left and front_pts_right:
        gap_front = min(front_pts_left) + min(abs(v) for v in front_pts_right)

    gap_back = None
    if back_pts_left and back_pts_right:
        gap_back = min(back_pts_left) + min(back_pts_right)

    dir_dists = {}
    for k in dirs:
        dir_dists[k] = min(dirs[k]) if dirs[k] else float('inf')

    sector_min = min(sector_vals) if sector_vals else float('inf')

    return gap_front, gap_back, dir_dists, sector_min


# ──────────────────────────── main class ──────────────────────────
class SmorphiSim:
    def __init__(self, map_path: Optional[str] = None):
        pygame.init()
        self.screen = pygame.display.set_mode((WIN_W, WIN_H))
        pygame.display.set_caption("Smorphi 2D Sim  |  Auto Transform & Path Planning")
        self.clock  = pygame.time.Clock()
        self.font_sm = pygame.font.SysFont("Consolas", 12)
        self.font_md = pygame.font.SysFont("Consolas", 14, bold=True)
        self.font_lg = pygame.font.SysFont("Consolas", 16, bold=True)

        # ── load map ──
        self.map_path = map_path   # remember for M-key hot-reload
        loaded = load_map_from_file(map_path) if map_path else None
        self.grid = loaded if loaded is not None else build_default_map()
        self.using_custom_map = (loaded is not None)

        # Robot state
        self.robot_r = 10.0
        self.robot_c = 10.0
        self.heading  = 0.0         # radians (0 = right)
        self.shape    = 0           # 0 = O, 1 = I
        self.footprints = [footprint_O(), footprint_I()]

        # Auto-transform state
        self.auto_enabled = True
        self.count_below  = 0
        self.count_above  = 0
        self.last_transform_time = 0.0

        # Sensor values
        self.gap_front   = None
        self.gap_back    = None
        self.dir_dists   = {}
        self.sector_min  = float('inf')

        # LiDAR
        self.lidar_results = []
        self.show_lidar    = True
        self.show_grid     = False

        # Path planning
        self.goal       = None
        self.path: Optional[List[Tuple[int,int]]] = None
        self.path_idx   = 0
        self.following  = False
        self.no_path_msg = ""       # message when no path found

        # Transform animation state
        self.transforming      = False
        self.transform_dir     = 1       # +1 = O→I,  -1 = I→O
        self.transform_progress = 0.0   # 0.0 … 1.0
        self.transform_start_t  = 0.0

        # Misc
        self.paused     = False
        self.running    = True
        self.status_msg = "Ready. Left-click to set a goal."

    # ────────── footprint ──────────
    def current_footprint(self):
        return self.footprints[self.shape]

    def _footprint_center(self):
        """Return (center_dr, center_dc) – the centre of the current
        footprint in cell offsets from the robot origin."""
        fp = self.current_footprint()
        cr = sum(dr for dr, dc in fp) / len(fp)
        cc = sum(dc for dr, dc in fp) / len(fp)
        return cr, cc

    # ────────── drawing ──────────
    def draw_map(self):
        for r in range(ROWS):
            for c in range(COLS):
                x, y = c * CELL, r * CELL
                if self.grid[r][c] == 1:
                    pygame.draw.rect(self.screen, COL_WALL, (x, y, CELL, CELL))
                    pygame.draw.rect(self.screen, COL_WALL_HL, (x, y, CELL, CELL), 1)
                else:
                    pygame.draw.rect(self.screen, COL_FREE, (x, y, CELL, CELL))
        if self.show_grid:
            for r in range(ROWS+1):
                pygame.draw.line(self.screen, COL_GRID, (0, r*CELL), (MAP_W, r*CELL))
            for c in range(COLS+1):
                pygame.draw.line(self.screen, COL_GRID, (c*CELL, 0), (c*CELL, MAP_H))

    def draw_path(self):
        if not self.path:
            return
        # Draw path line
        if len(self.path) > 1:
            pts = [(c*CELL+CELL//2, r*CELL+CELL//2) for r,c in self.path]
            for i in range(len(pts)-1):
                col = COL_PATH if i >= self.path_idx else COL_PATH_DIM
                pygame.draw.line(self.screen, col, pts[i], pts[i+1], 2)
        # Draw waypoint dots
        for i, (r, c) in enumerate(self.path):
            x = c * CELL + CELL//2
            y = r * CELL + CELL//2
            if i < self.path_idx:
                col = COL_PATH_DIM
            elif i == self.path_idx:
                col = (255, 255, 255)
            else:
                col = COL_PATH
            pygame.draw.circle(self.screen, col, (x, y), 2)

    def draw_goal(self):
        if self.goal is None:
            return
        gr, gc = self.goal
        x = gc * CELL + CELL//2
        y = gr * CELL + CELL//2
        t = pygame.time.get_ticks() / 400.0
        radius = int(8 + 3 * math.sin(t * 3))
        pygame.draw.circle(self.screen, COL_GOAL, (x, y), radius, 2)
        pygame.draw.circle(self.screen, COL_GOAL, (x, y), 3)

    def draw_lidar(self):
        if not self.show_lidar or not self.lidar_results:
            return
        cx = self.robot_c * CELL + CELL//2
        cy = self.robot_r * CELL + CELL//2
        lidar_surf = pygame.Surface((MAP_W, MAP_H), pygame.SRCALPHA)
        for angle, dist_m, hx, hy in self.lidar_results:
            hx_c = max(0, min(MAP_W-1, hx))
            hy_c = max(0, min(MAP_H-1, hy))
            pygame.draw.line(lidar_surf, COL_LIDAR,
                             (int(cx), int(cy)), (int(hx_c), int(hy_c)), 1)
            if dist_m < LIDAR_RANGE - 0.05:
                pygame.draw.circle(lidar_surf, COL_LIDAR_HIT,
                                   (int(hx_c), int(hy_c)), 2)
        self.screen.blit(lidar_surf, (0, 0))

    # ────────── module geometry helpers ──────────
    def _module_centres_O(self):
        """Pixel offsets (dx, dy) of each module centre from robot origin
        for the O-shape (2×2 grid).
          [M1][M2]
          [M3][M4]
        Origin = top-left corner of M1."""
        h = CELL / 2
        return [
            (h,       h      ),   # M1
            (h + CELL, h      ),   # M2
            (h,       h + CELL),   # M3
            (h + CELL, h + CELL),  # M4
        ]

    def _module_centres_I(self):
        """Pixel offsets (dx, dy) of each module centre for I-shape (1×4).
          [M1][M2][M3][M4]"""
        h = CELL / 2
        return [
            (h,           h),   # M1
            (h +   CELL,  h),   # M2
            (h + 2*CELL,  h),   # M3
            (h + 3*CELL,  h),   # M4
        ]

    def _module_centres_animated(self, t):
        """Interpolate module centres for 0≤t≤1.

        O→I  (t = transform_progress when dir=+1):
          M1, M2 – fixed (they are cells 1,2 in both shapes).
          M3, M4 – pivot −90° around the bottom-right corner of M2.

        Pivot point (px,py) in local offset coords:
          bottom-right corner of M2  =  top-right corner of M4 in O-shape
          = (2·CELL, CELL)  from robot origin (top-left of M1).
        """
        O = self._module_centres_O()
        I = self._module_centres_I()

        # pivot = bottom-right corner of M2 in local coords
        px = 2 * CELL   # 2 cells right of origin
        py = CELL        # 1 cell below origin (bottom of top row)

        # angle swept:  O→I moves M3,M4 from "below" to "right"
        # In screen coords (y down), rotating -90° around that pivot
        # takes (0,CELL) below M1/M2 row to the right of M2.
        angle = -math.pi / 2 * t  # 0 → -90°
        cos_a = math.cos(angle)
        sin_a = math.sin(angle)

        centres = []
        for idx in range(4):
            if idx < 2:
                # M1, M2: simple linear blend (they barely move)
                ox, oy = O[idx]
                ix, iy = I[idx]
                centres.append((ox + (ix-ox)*t, oy + (iy-oy)*t))
            else:
                # M3, M4: rotate around pivot
                ox, oy = O[idx]
                # offset from pivot
                dx = ox - px
                dy = oy - py
                # rotate
                rx = px + dx*cos_a - dy*sin_a
                ry = py + dx*sin_a + dy*cos_a
                centres.append((rx, ry))
        return centres

    def draw_robot(self):
        # Robot origin in pixels = top-left of M1
        ox = self.robot_c * CELL
        oy = self.robot_r * CELL

        # Choose module centre positions
        if self.transforming:
            t = self.transform_progress
            if self.transform_dir == 1:   # O → I
                centres = self._module_centres_animated(t)
            else:                          # I → O  (reverse)
                centres = self._module_centres_animated(1.0 - t)
        elif self.shape == 0:
            centres = self._module_centres_O()
        else:
            centres = self._module_centres_I()

        # Draw each module as a filled square centred on its position
        for i, (dx, dy) in enumerate(centres):
            mx = int(ox + dx - CELL/2)
            my = int(oy + dy - CELL/2)
            col = MODULE_COLOURS[i % len(MODULE_COLOURS)]
            pygame.draw.rect(self.screen, col, (mx, my, CELL, CELL))
            pygame.draw.rect(self.screen, (255,255,255), (mx, my, CELL, CELL), 1)
            label = self.font_sm.render(str(i+1), True, (255,255,255))
            self.screen.blit(label, (mx+3, my+3))

        # ── Forward-direction arrow from Block 1 (M1) ──
        # Arrow starts at centre of M1 and points in heading direction
        m1_dx, m1_dy = centres[0]
        m1_cx = ox + m1_dx          # M1 centre x (pixels)
        m1_cy = oy + m1_dy          # M1 centre y (pixels)

        arrow_len = CELL * 1.8      # arrow extends well beyond M1
        tip_x = m1_cx + arrow_len * math.cos(self.heading)
        tip_y = m1_cy + arrow_len * math.sin(self.heading)

        # Arrow shaft
        arrow_col = (200, 255, 60)  # bright yellow-green
        pygame.draw.line(self.screen, arrow_col,
                         (int(m1_cx), int(m1_cy)),
                         (int(tip_x), int(tip_y)), 2)

        # Triangular arrowhead
        head_size = 5.0
        angle_l = self.heading + math.pi + math.pi / 5   # left barb
        angle_r = self.heading + math.pi - math.pi / 5   # right barb
        p1 = (tip_x, tip_y)
        p2 = (tip_x + head_size * math.cos(angle_l),
              tip_y + head_size * math.sin(angle_l))
        p3 = (tip_x + head_size * math.cos(angle_r),
              tip_y + head_size * math.sin(angle_r))
        pygame.draw.polygon(self.screen, arrow_col,
                            [(int(p1[0]),int(p1[1])),
                             (int(p2[0]),int(p2[1])),
                             (int(p3[0]),int(p3[1]))])

        # Small dot at M1 centre (arrow origin)
        pygame.draw.circle(self.screen, arrow_col, (int(m1_cx), int(m1_cy)), 2)

        # Shape label above M1
        if self.transforming:
            lbl = "O>I" if self.transform_dir == 1 else "I>O"
            lbl_col = (255, 255, 80)
        else:
            lbl = "O" if self.shape == 0 else "I"
            lbl_col = (255, 255, 255)
        label = self.font_md.render(lbl, True, lbl_col)
        self.screen.blit(label, (int(m1_cx)-4, int(m1_cy)-20))

    def draw_hud(self):
        pygame.draw.rect(self.screen, COL_HUD_BG, (HUD_X, 0, WIN_W-HUD_X, WIN_H))
        pygame.draw.line(self.screen, (60,60,80), (HUD_X, 0), (HUD_X, WIN_H), 2)

        x0 = HUD_X + 10
        y = 8

        def head(text):
            nonlocal y
            s = self.font_lg.render(text, True, COL_HUD_HEAD)
            self.screen.blit(s, (x0, y)); y += 22

        def row(label, value, vc=COL_HUD_VAL):
            nonlocal y
            s1 = self.font_sm.render(label, True, COL_HUD_TXT)
            s2 = self.font_sm.render(str(value), True, vc)
            self.screen.blit(s1, (x0, y))
            self.screen.blit(s2, (x0 + 120, y))
            y += 16

        def sep():
            nonlocal y
            pygame.draw.line(self.screen, (50,50,70), (x0, y+2), (WIN_W-8, y+2))
            y += 8

        head("SMORPHI 2D SIM")
        sep()

        head("Robot")
        sn = "O-shape (2x2)" if self.shape == 0 else "I-shape (1x4)"
        if self.transforming:
            sn = "Transforming O>I" if self.transform_dir==1 else "Transforming I>O"
            sc = (255, 255, 80)
        else:
            sc = COL_O if self.shape == 0 else COL_I
        row("Shape:", sn, sc)
        row("Pos:", f"({self.robot_c:.1f}, {self.robot_r:.1f})")
        row("Heading:", f"{math.degrees(self.heading):.0f} deg")
        sep()

        head("LiDAR / Gaps")
        gf = f"{self.gap_front:.3f}m" if self.gap_front is not None else "N/A"
        gb = f"{self.gap_back:.3f}m" if self.gap_back is not None else "N/A"
        gfc = COL_HUD_WARN if (self.gap_front is not None and self.gap_front < TH_I) else COL_HUD_VAL
        row("Gap Front:", gf, gfc)
        row("Gap Back:", gb)
        for d in ['front','left','back','right']:
            v = self.dir_dists.get(d, float('inf'))
            row(f"  {d.title()}:", f"{v:.2f}m" if v < 50 else "inf")
        smc = COL_HUD_OK if self.sector_min >= REQUIRED_RADIUS else COL_HUD_WARN
        smv = f"{self.sector_min:.3f}m" if self.sector_min < 50 else "inf"
        row("Sector Min:", smv, smc)
        sep()

        head("Auto Transform")
        ac = COL_HUD_OK if self.auto_enabled else COL_HUD_WARN
        row("Enabled:", "ON" if self.auto_enabled else "OFF", ac)
        row("Cnt O->I:", f"{self.count_below}/{STABLE_SAMPLES}")
        row("Cnt I->O:", f"{self.count_above}/{STABLE_SAMPLES}")
        elapsed = time.time() - self.last_transform_time
        cdl = max(0, COOLDOWN_S - elapsed)
        cdc = COL_HUD_WARN if cdl > 0 else COL_HUD_OK
        row("Cooldown:", f"{cdl:.1f}s", cdc)
        sep()

        head("Path Planning")
        if self.goal:
            row("Goal:", f"({self.goal[1]}, {self.goal[0]})")
        else:
            row("Goal:", "None")
        if self.path:
            row("Path:", f"{len(self.path)} cells")
            row("Progress:", f"{self.path_idx}/{len(self.path)}")
            st = "Following" if self.following else "Reached"
        elif self.no_path_msg:
            st = self.no_path_msg
        else:
            st = "Idle"
        stc = COL_HUD_OK if self.following else COL_HUD_TXT
        row("Status:", st, stc)
        sep()

        head("Controls")
        for txt in [
            "LClick  Set goal",
            "RClick  Toggle wall",
            "1 / 2   Shape O / I",
            "A       Auto toggle",
            "L       LiDAR rays",
            "G       Grid lines",
            "R       Reset",
            "Space   Pause",
            "Esc     Quit",
        ]:
            s = self.font_sm.render(txt, True, (130,130,155))
            self.screen.blit(s, (x0, y)); y += 14

        # Thresholds reference
        y += 6
        sep()
        head("Thresholds")
        row("O->I gap<", f"{TH_I}m")
        row("I->O gap>", f"{TH_O}m")
        row("Sector >=", f"{REQUIRED_RADIUS}m")
        row("Back >=", f"{BACK_NARROW_THRESH}m")
        row("Samples:", str(STABLE_SAMPLES))
        row("Cooldown:", f"{COOLDOWN_S}s")

    def draw_status_bar(self):
        """Status text at bottom of map area."""
        bar_h = 20
        pygame.draw.rect(self.screen, (15,15,22), (0, MAP_H - bar_h, MAP_W, bar_h))
        s = self.font_sm.render(self.status_msg, True, (160, 170, 180))
        self.screen.blit(s, (8, MAP_H - bar_h + 3))

    # ────────── update logic ──────────
    def update_lidar(self):
        cx = self.robot_c * CELL + CELL//2
        cy = self.robot_r * CELL + CELL//2
        self.lidar_results = simulate_lidar(self.grid, cx, cy, self.heading,
                                            LIDAR_RAYS, LIDAR_RANGE)
        self.gap_front, self.gap_back, self.dir_dists, self.sector_min = \
            compute_gap_metrics(self.lidar_results, self.heading)

    def update_auto_transform(self):
        if not self.auto_enabled:
            return
        if self.gap_front is None:
            return

        gap = self.gap_front
        sector_ok = (self.sector_min >= REQUIRED_RADIUS)

        # O -> I
        if gap < TH_I and sector_ok:
            self.count_below += 1
            self.count_above = 0
        # I -> O
        elif gap > TH_O:
            back_ok = (self.gap_back is not None and self.gap_back > 0 and
                       self.gap_back >= BACK_NARROW_THRESH)
            if back_ok and sector_ok:
                self.count_above += 1
                self.count_below = 0
            else:
                self.count_above = 0
                self.count_below = 0
        else:
            self.count_below = 0
            self.count_above = 0

        now = time.time()
        can_send = (now - self.last_transform_time) >= COOLDOWN_S

        if self.count_below >= STABLE_SAMPLES and can_send:
            if self.shape != 1:
                self.set_shape(1)
                self.status_msg = "AUTO: O -> I (narrow gap detected)"
            self.count_below = 0

        if self.count_above >= STABLE_SAMPLES and can_send:
            if self.shape != 0:
                self.set_shape(0)
                self.status_msg = "AUTO: I -> O (wide area ahead + behind)"
            self.count_above = 0

    def set_shape(self, s):
        if s == self.shape and not self.transforming:
            return
        # Start pivot animation
        self.transform_dir = 1 if s == 1 else -1   # +1 O→I, -1 I→O
        self.transform_progress = 0.0
        self.transform_start_t = time.time()
        self.transforming = True
        self.shape = s   # logical shape changes immediately (for planner)
        self.last_transform_time = time.time()
        self.count_below = 0
        self.count_above = 0
        # Re-plan if following a path (footprint changed)
        if self.path and self.goal:
            self.plan_path()

    def update_transform_anim(self):
        """Advance the pivot animation each frame."""
        if not self.transforming:
            return
        elapsed = time.time() - self.transform_start_t
        self.transform_progress = min(1.0, elapsed / TRANSFORM_DURATION)
        if self.transform_progress >= 1.0:
            self.transforming = False

    def plan_path(self):
        if self.goal is None:
            return
        sr = int(round(self.robot_r))
        sc = int(round(self.robot_c))
        gr, gc = self.goal
        fp = self.current_footprint()
        self.path = astar(self.grid, (sr, sc), (gr, gc), fp)
        self.path_idx = 0
        if self.path:
            self.following = True
            self.no_path_msg = ""
            shape_name = "O" if self.shape == 0 else "I"
            self.status_msg = f"Path found ({len(self.path)} steps, {shape_name}-shape)"
        else:
            self.following = False
            shape_name = "O" if self.shape == 0 else "I"
            self.no_path_msg = f"No path ({shape_name})"
            self.status_msg = f"No path found with {shape_name}-shape! Try other shape or remove walls."

    def update_path_follow(self):
        if not self.following or not self.path:
            return
        if self.path_idx >= len(self.path):
            self.following = False
            self.status_msg = "Goal reached!"
            return

        # ── Pure-pursuit with M1-leads-toward-goal logic ──
        # The robot origin is M1's grid position.  We want M1
        # (at offset 0,0) to always be the module closest to the
        # goal.  To achieve this we compute heading from M1 to
        # the final goal, and steer M1 directly along the A* path.
        target_idx = min(self.path_idx + PURSUIT_LOOKAHEAD, len(self.path)-1)
        tr, tc = self.path[target_idx]

        # Direction vector from M1 (robot origin) to the path target
        dr = tr - self.robot_r
        dc = tc - self.robot_c
        dist = math.hypot(dr, dc)

        if dist < 0.5:
            self.path_idx = target_idx + 1
            if self.path_idx >= len(self.path):
                self.following = False
                self.status_msg = "Goal reached!"
            return

        # Heading always points from M1 toward the FINAL goal
        # so the arrow on M1 always faces the destination.
        if self.goal is not None:
            goal_dr = self.goal[0] - self.robot_r
            goal_dc = self.goal[1] - self.robot_c
            if math.hypot(goal_dr, goal_dc) > 0.01:
                self.heading = math.atan2(goal_dr, goal_dc)
        else:
            self.heading = math.atan2(dr, dc)

        speed = 0.08
        self.robot_r += dr / dist * speed
        self.robot_c += dc / dist * speed

        # Advance path index when close to current waypoint
        if self.path_idx < len(self.path):
            wr, wc = self.path[self.path_idx]
            if abs(self.robot_r - wr) < 0.6 and abs(self.robot_c - wc) < 0.6:
                self.path_idx += 1

    # ────────── events ──────────
    def handle_events(self):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.running = False
            elif event.type == pygame.KEYDOWN:
                self.on_key(event.key)
            elif event.type == pygame.MOUSEBUTTONDOWN:
                self.on_click(event)

    def on_key(self, key):
        if key == pygame.K_ESCAPE:
            self.running = False
        elif key == pygame.K_SPACE:
            self.paused = not self.paused
            self.status_msg = "PAUSED" if self.paused else "Resumed"
        elif key == pygame.K_1:
            self.set_shape(0)
            self.status_msg = "Manual: set O-shape"
        elif key == pygame.K_2:
            self.set_shape(1)
            self.status_msg = "Manual: set I-shape"
        elif key == pygame.K_a:
            self.auto_enabled = not self.auto_enabled
            self.status_msg = f"Auto-transform {'ON' if self.auto_enabled else 'OFF'}"
        elif key == pygame.K_r:
            self.robot_r, self.robot_c = 10.0, 10.0
            self.heading = 0.0
            self.shape = 0
            self.path = None
            self.goal = None
            self.following = False
            self.status_msg = "Robot reset"
        elif key == pygame.K_l:
            self.show_lidar = not self.show_lidar
        elif key == pygame.K_g:
            self.show_grid = not self.show_grid
        elif key == pygame.K_m:
            # Hot-reload map from file
            path = self.map_path or MAP_FILE
            loaded = load_map_from_file(path)
            if loaded is not None:
                self.grid = loaded
                self.using_custom_map = True
                self.path = None
                self.goal = None
                self.following = False
                self.status_msg = f"Map reloaded from {path}"
            else:
                self.status_msg = f"Could not load {path}"

    def on_click(self, event):
        mx, my = event.pos
        if mx >= MAP_W:
            return
        c = mx // CELL
        r = my // CELL
        if not (0 <= r < ROWS and 0 <= c < COLS):
            return

        if event.button == 1:   # left = set goal
            if self.grid[r][c] == 0:
                self.goal = (r, c)
                self.plan_path()
        elif event.button == 3: # right = toggle wall
            self.grid[r][c] = 1 - self.grid[r][c]
            if self.path and self.goal:
                self.plan_path()

    # ────────── main loop ──────────
    def run(self):
        while self.running:
            self.clock.tick(FPS)
            self.handle_events()

            if not self.paused:
                self.update_lidar()
                self.update_auto_transform()
                self.update_transform_anim()
                self.update_path_follow()

            self.screen.fill(COL_BG)
            self.draw_map()
            self.draw_path()
            self.draw_goal()
            self.draw_lidar()
            self.draw_robot()
            self.draw_status_bar()
            self.draw_hud()

            if self.paused:
                ps = self.font_lg.render("PAUSED", True, (255,255,100))
                self.screen.blit(ps, (MAP_W//2 - 40, 10))

            pygame.display.flip()

        pygame.quit()


# ──────────────────────────── entry point ─────────────────────────
if __name__ == "__main__":
    # Usage:
    #   python smorphi_2d_sim.py              -> uses built-in map
    #   python smorphi_2d_sim.py map.json     -> loads map from map_editor
    #   python smorphi_2d_sim.py path/to/any.json
    map_arg = sys.argv[1] if len(sys.argv) > 1 else None
    # Auto-detect map.json in the same folder even without CLI arg
    if map_arg is None and os.path.exists(MAP_FILE):
        map_arg = MAP_FILE
        print(f"[Sim] Auto-detected {MAP_FILE} – loading it. "
              "Pass a different path as argv[1] to override, "
              "or delete map.json to use the built-in map.")
    sim = SmorphiSim(map_path=map_arg)
    sim.run()
