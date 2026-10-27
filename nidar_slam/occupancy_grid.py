"""Log-odds occupancy grid."""
import math
import numpy as np


class OccupancyGrid:
    def __init__(self, width_m=12.0, height_m=12.0, resolution_m=0.1):
        self.resolution = float(resolution_m)
        self.width = int(round(width_m / self.resolution))
        self.height = int(round(height_m / self.resolution))
        self.log_odds = np.zeros((self.height, self.width), dtype=np.float32)
        self.origin_x = self.width // 2
        self.origin_y = self.height // 2
        self.l_occ = 0.85
        self.l_free = -0.40
        self.l_min = -4.0
        self.l_max = 4.0

    def world_to_cell(self, x, y):
        col = int(round(self.origin_x + x / self.resolution))
        row = int(round(self.origin_y - y / self.resolution))
        if 0 <= col < self.width and 0 <= row < self.height:
            return row, col
        return None

    def update_ray(self, x0, y0, x1, y1, hit=True):
        start = self.world_to_cell(x0, y0)
        end = self.world_to_cell(x1, y1)
        if start is None or end is None:
            return
        r0, c0 = start
        r1, c1 = end
        cells = self._bresenham(c0, r0, c1, r1)
        for i, (col, row) in enumerate(cells):
            if not (0 <= row < self.height and 0 <= col < self.width):
                continue
            is_endpoint = i == len(cells) - 1
            delta = self.l_occ if (hit and is_endpoint) else self.l_free
            self.log_odds[row, col] = float(np.clip(self.log_odds[row, col] + delta,
                                                    self.l_min, self.l_max))

    @staticmethod
    def _bresenham(x0, y0, x1, y1):
        points = []
        dx, dy = abs(x1 - x0), abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy
        x, y = x0, y0
        while True:
            points.append((x, y))
            if x == x1 and y == y1:
                break
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x += sx
            if e2 < dx:
                err += dx
                y += sy
        return points

    def image(self, pose=None):
        # Unknown = grey, free = white, occupied = black.
        img = np.full(self.log_odds.shape, 127, dtype=np.uint8)
        img[self.log_odds < -0.25] = 245
        img[self.log_odds > 0.65] = 15
        if pose is not None:
            cell = self.world_to_cell(pose[0], pose[1])
            if cell:
                import cv2
                row, col = cell
                cv2.circle(img, (col, row), 4, 80, -1)
                angle = pose[2]
                end = (int(col + 18 * math.cos(angle)), int(row - 18 * math.sin(angle)))
                cv2.line(img, (col, row), end, 80, 2)
        return img
