#!/usr/bin/env python3
"""
Smorphi Map Editor
==================

Controls
--------
Left Click   : Add wall
Right Click  : Remove wall
S            : Save map to JSON
L            : Load map from JSON
C            : Clear map
G            : Toggle grid
ESC          : Exit
"""

import pygame
import json
import os

MAP_FILE = "map.json"

# ----- Map settings -----
CELL = 15
MAP_W, MAP_H = 800, 750
COLS = MAP_W // CELL
ROWS = MAP_H // CELL

# Colors
COL_BG = (18,18,24)
COL_FREE = (30,30,40)
COL_WALL = (70,80,100)
COL_GRID = (50,50,70)

class MapEditor:

    def __init__(self):

        pygame.init()

        self.screen = pygame.display.set_mode((MAP_W, MAP_H))
        pygame.display.set_caption("Smorphi Map Editor")

        self.clock = pygame.time.Clock()
        self.font = pygame.font.SysFont("Consolas", 13)

        # Grid map
        self.grid = [[0]*COLS for _ in range(ROWS)]
        self.status_msg = ""       # shown at bottom of screen
        self.status_timer = 0

        self.running = True
        self.show_grid = True

    # -------------------------------------
    # Save map to JSON
    # -------------------------------------
    def save_map(self):
        with open(MAP_FILE, "w") as f:
            json.dump(self.grid, f)
        self.status_msg = f"Saved → {MAP_FILE}"
        self.status_timer = 180
        print(f"Map saved as {MAP_FILE}")

    # -------------------------------------
    # Load map from JSON
    # -------------------------------------
    def load_map(self):
        if not os.path.exists(MAP_FILE):
            self.status_msg = f"File not found: {MAP_FILE}"
            self.status_timer = 180
            print(f"No {MAP_FILE} found.")
            return
        try:
            with open(MAP_FILE) as f:
                data = json.load(f)
            # Validate dimensions
            if len(data) == ROWS and len(data[0]) == COLS:
                self.grid = data
                self.status_msg = f"Loaded ← {MAP_FILE}"
            else:
                # Resize / crop to fit
                self.grid = [[0]*COLS for _ in range(ROWS)]
                for r in range(min(len(data), ROWS)):
                    for c in range(min(len(data[r]), COLS)):
                        self.grid[r][c] = data[r][c]
                self.status_msg = f"Loaded (resized) ← {MAP_FILE}"
            self.status_timer = 180
            print(self.status_msg)
        except Exception as e:
            self.status_msg = f"Load error: {e}"
            self.status_timer = 180
            print(self.status_msg)

    # -------------------------------------
    # Clear map
    # -------------------------------------
    def clear_map(self):
        self.grid = [[0]*COLS for _ in range(ROWS)]
        self.status_msg = "Map cleared"
        self.status_timer = 120
        print("Map cleared")

    # -------------------------------------
    # Draw map
    # -------------------------------------
    def draw_map(self):

        for r in range(ROWS):
            for c in range(COLS):

                x = c * CELL
                y = r * CELL

                if self.grid[r][c] == 1:
                    pygame.draw.rect(self.screen, COL_WALL, (x,y,CELL,CELL))
                else:
                    pygame.draw.rect(self.screen, COL_FREE, (x,y,CELL,CELL))

        if self.show_grid:
            for r in range(ROWS):
                pygame.draw.line(self.screen, COL_GRID, (0,r*CELL),(MAP_W,r*CELL))
            for c in range(COLS):
                pygame.draw.line(self.screen, COL_GRID, (c*CELL,0),(c*CELL,MAP_H))

        # Controls overlay (top-left)
        controls = ["LClick=wall  RClick=erase  S=save  L=load  C=clear  G=grid  Esc=quit"]
        for i, txt in enumerate(controls):
            s = self.font.render(txt, True, (160,170,180))
            self.screen.blit(s, (6, 4 + i*16))

        # Status message (bottom)
        if self.status_timer > 0:
            s = self.font.render(self.status_msg, True, (120, 230, 140))
            self.screen.blit(s, (6, MAP_H - 20))
            self.status_timer -= 1

    # -------------------------------------
    # Mouse drawing
    # -------------------------------------
    def handle_mouse(self):

        mx, my = pygame.mouse.get_pos()

        c = mx // CELL
        r = my // CELL

        if not (0 <= r < ROWS and 0 <= c < COLS):
            return

        buttons = pygame.mouse.get_pressed()

        # Left click → add wall
        if buttons[0]:
            self.grid[r][c] = 1

        # Right click → remove wall
        if buttons[2]:
            self.grid[r][c] = 0

    # -------------------------------------
    # Events
    # -------------------------------------
    def handle_events(self):

        for event in pygame.event.get():

            if event.type == pygame.QUIT:
                self.running = False

            if event.type == pygame.KEYDOWN:

                if event.key == pygame.K_ESCAPE:
                    self.running = False

                if event.key == pygame.K_s:
                    self.save_map()

                if event.key == pygame.K_l:
                    self.load_map()

                if event.key == pygame.K_c:
                    self.clear_map()

                if event.key == pygame.K_g:
                    self.show_grid = not self.show_grid

    # -------------------------------------
    # Main loop
    # -------------------------------------
    def run(self):

        while self.running:

            self.clock.tick(60)

            self.handle_events()
            self.handle_mouse()

            self.screen.fill(COL_BG)

            self.draw_map()

            pygame.display.flip()

        pygame.quit()


# ----- run editor -----

if __name__ == "__main__":

    editor = MapEditor()
    editor.run()