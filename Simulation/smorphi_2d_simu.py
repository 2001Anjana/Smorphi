#!/usr/bin/env python3
"""
Smorphi 2-D Simulation – Animated Pivot & Modular Logic
=======================================================
Updated: 2x2 (O) and 1x4 (I) shapes with pivot animation.
"""

import sys, math, heapq, time
from typing import List, Tuple, Optional, Set
import pygame

# ———————————————————————————— constants ————————————————————————————
WIN_W, WIN_H = 1100, 750
MAP_W, MAP_H = 800, 750
HUD_X        = MAP_W
CELL         = 25  # Larger cells for better visibility of 2x2/1x4
COLS, ROWS   = MAP_W // CELL, MAP_H // CELL
FPS = 60

# Colors
COL_BG       = (18, 18, 24)
COL_GRID     = (30, 30, 45)
COL_WALL     = (60, 65, 80)
COL_FREE     = (25, 25, 35)
COL_PATH     = (100, 220, 255)
COL_GOAL     = (255, 80, 120)

# Module Colors (M1-M4)
MOD_COLORS = [
    (230, 60, 60),   # M1: Red
    (60, 230, 60),   # M2: Green
    (60, 100, 230),  # M3: Blue
    (230, 230, 60)   # M4: Yellow
]

# Auto-transform thresholds
TH_I, TH_O = 0.25, 0.35
STABLE_SAMPLES = 5
TRANSFORM_SPEED = 0.05  # Increment per frame (1.0 = instant)

# ———————————————————————————— footprints ——————————————————————————
def get_o_offsets(): return [(0,0), (0,1), (1,1), (1,0)]
def get_i_offsets(): return [(0,0), (0,1), (0,2), (0,3)]

# ———————————————————————————— A* planner ——————————————————————————
def astar(grid, start, goal, footprint) -> Optional[List[Tuple[int,int]]]:
    sr, sc = int(round(start[0])), int(round(start[1]))
    gr, gc = goal
    if not _passable(grid, sr, sc, footprint) or not _passable(grid, gr, gc, footprint):
        return None

    open_set = [(0.0, sr, sc)]
    came_from = {}
    g_score = {(sr, sc): 0.0}
    
    while open_set:
        _, cr, cc = heapq.heappop(open_set)
        if (cr, cc) == (gr, gc):
            path = []
            curr = (gr, gc)
            while curr in came_from:
                path.append(curr)
                curr = came_from[curr]
            path.append((sr, sc))
            return path[::-1]

        for dr, dc in [(-1,0),(1,0),(0,-1),(0,1),(-1,-1),(-1,1),(1,-1),(1,1)]:
            nr, nc = cr+dr, cc+dc
            if not _passable(grid, nr, nc, footprint): continue
            
            weight = 1.414 if dr != 0 and dc != 0 else 1.0
            tentative_g = g_score[(cr, cc)] + weight
            if tentative_g < g_score.get((nr, nc), float('inf')):
                came_from[(nr, nc)] = (cr, cc)
                g_score[(nr, nc)] = tentative_g
                h = math.hypot(nr - gr, nc - gc)
                heapq.heappush(open_set, (tentative_g + h, nr, nc))
    return None

def _passable(grid, r, c, footprint):
    for dr, dc in footprint:
        rr, cc = r + dr, c + dc
        if not (0 <= rr < ROWS and 0 <= cc < COLS) or grid[rr][cc] == 1:
            return False
    return True

# ———————————————————————————— main class ——————————————————————————
class SmorphiSim:
    def __init__(self):
        pygame.init()
        self.screen = pygame.display.set_mode((WIN_W, WIN_H))
        pygame.display.set_caption("Smorphi 2D - Pivot Animation")
        self.clock = pygame.time.Clock()
        self.font = pygame.font.SysFont("Consolas", 14, bold=True)
        
        self.grid = [[0]*COLS for _ in range(ROWS)]
        # Boundary
        for i in range(COLS): self.grid[0][i] = self.grid[ROWS-1][i] = 1
        for i in range(ROWS): self.grid[i][0] = self.grid[i][COLS-1] = 1

        self.robot_r, self.robot_c = 10.0, 10.0
        self.heading = 0.0
        self.shape = 0  # 0: O, 1: I
        self.transform_progress = 0.0  # 0.0 (O) to 1.0 (I)
        
        self.goal = None
        self.path = None
        self.path_idx = 0
        self.following = False
        self.auto_enabled = True
        self.running = True

    def get_module_positions(self):
        """Calculates module positions with pivot math."""
        # M1 and M2 are the base
        m1 = (0, 0)
        m2 = (0, 1)
        
        # Pivot point is the joint between M2 and M3
        # In O: M3 is (1,1), M4 is (1,0)
        # In I: M3 is (0,2), M4 is (0,3)
        # Angle: 90 deg (O) to 0 deg (I)
        angle = math.radians(90 * (1.0 - self.transform_progress))
        
        # Pivot around (0, 1)
        # M3 relative to pivot in I-shape is (0, 1). Rotate that by 'angle'
        m3_r = math.sin(angle)
        m3_c = 1 + math.cos(angle)
        
        # M4 relative to M3 in I-shape is (0, 1). 
        m4_r = m3_r + math.sin(angle)
        m4_c = m3_c + math.cos(angle)
        
        return [m1, m2, (m3_r, m3_c), (m4_r, m4_c)]

    def draw_robot(self):
        mod_offsets = self.get_module_positions()
        
        # Calculate rotation matrix for robot heading
        cos_h = math.cos(self.heading)
        sin_h = math.sin(self.heading)

        for i, (dr, dc) in enumerate(mod_offsets):
            # Rotate module offset by robot heading
            rr = dr * cos_h - dc * sin_h
            rc = dr * sin_h + dc * cos_h
            
            # World pixels
            px = (self.robot_c + rc) * CELL
            py = (self.robot_r + rr) * CELL
            
            rect = pygame.Rect(px, py, CELL, CELL)
            pygame.draw.rect(self.screen, MOD_COLORS[i], rect)
            pygame.draw.rect(self.screen, (255,255,255), rect, 1)
            
            # Module ID
            txt = self.font.render(str(i+1), True, (255,255,255))
            self.screen.blit(txt, (px + 8, py + 5))

    def update(self):
        # Handle animation
        target_progress = 1.0 if self.shape == 1 else 0.0
        if self.transform_progress < target_progress:
            self.transform_progress = min(target_progress, self.transform_progress + TRANSFORM_SPEED)
        elif self.transform_progress > target_progress:
            self.transform_progress = max(target_progress, self.transform_progress - TRANSFORM_SPEED)

        if self.following and self.path:
            tr, tc = self.path[self.path_idx]
            dr, dc = tr - self.robot_r, tc - self.robot_c
            dist = math.hypot(dr, dc)
            
            if dist < 0.2:
                self.path_idx += 1
                if self.path_idx >= len(self.path):
                    self.following = False
            else:
                self.heading = math.atan2(dr, dc)
                self.robot_r += (dr/dist) * 0.1
                self.robot_c += (dc/dist) * 0.1

    def run(self):
        while self.running:
            self.clock.tick(FPS)
            for event in pygame.event.get():
                if event.type == pygame.QUIT: self.running = False
                if event.type == pygame.MOUSEBUTTONDOWN:
                    mx, my = event.pos
                    if mx < MAP_W:
                        c, r = mx // CELL, my // CELL
                        if event.button == 1: # Left: Goal
                            self.goal = (r, c)
                            fp = get_i_offsets() if self.shape == 1 else get_o_offsets()
                            self.path = astar(self.grid, (self.robot_r, self.robot_c), self.goal, fp)
                            self.path_idx = 0
                            self.following = True if self.path else False
                        if event.button == 3: # Right: Wall
                            self.grid[r][c] = 1 - self.grid[r][c]
                if event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_1: self.shape = 0
                    if event.key == pygame.K_2: self.shape = 1

            self.update()
            self.screen.fill(COL_BG)
            
            # Draw Map
            for r in range(ROWS):
                for c in range(COLS):
                    if self.grid[r][c] == 1:
                        pygame.draw.rect(self.screen, COL_WALL, (c*CELL, r*CELL, CELL, CELL))
                    pygame.draw.rect(self.screen, COL_GRID, (c*CELL, r*CELL, CELL, CELL), 1)

            if self.path:
                pts = [(c*CELL+CELL//2, r*CELL+CELL//2) for r,c in self.path]
                if len(pts) > 1: pygame.draw.lines(self.screen, COL_PATH, False, pts, 2)

            self.draw_robot()
            
            # HUD
            pygame.draw.rect(self.screen, (25, 25, 30), (HUD_X, 0, WIN_W-HUD_X, WIN_H))
            info = [
                f"Shape: {'I (1x4)' if self.shape==1 else 'O (2x2)'}",
                f"Progress: {self.transform_progress:.2f}",
                "",
                "Controls:",
                "L-Click: Set Goal",
                "R-Click: Toggle Wall",
                "1: O-Shape",
                "2: I-Shape"
            ]
            for i, txt in enumerate(info):
                s = self.font.render(txt, True, (200, 200, 200))
                self.screen.blit(s, (HUD_X + 20, 50 + i*25))

            pygame.display.flip()
        pygame.quit()

if __name__ == "__main__":
    SmorphiSim().run()