"""Approximate camera optical-flow motion signal; not flight-grade odometry."""
import math
import cv2
import numpy as np


class OpticalFlowMotion:
    def __init__(self, hfov_deg=66.0, camera_height_m=1.0, max_step_m=0.15):
        self.hfov_deg = float(hfov_deg)
        self.camera_height_m = float(camera_height_m)
        self.max_step_m = float(max_step_m)
        self.prev_gray = None
        self.prev_points = None

    def update(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        if self.prev_gray is None:
            self.prev_gray = gray
            self.prev_points = cv2.goodFeaturesToTrack(gray, maxCorners=250, qualityLevel=0.01,
                                                       minDistance=8, blockSize=7)
            return 0.0, 0.0, 0.0, False

        if self.prev_points is None or len(self.prev_points) < 12:
            self.prev_gray = gray
            self.prev_points = cv2.goodFeaturesToTrack(gray, maxCorners=250, qualityLevel=0.01,
                                                       minDistance=8, blockSize=7)
            return 0.0, 0.0, 0.0, False

        next_points, status, _ = cv2.calcOpticalFlowPyrLK(
            self.prev_gray, gray, self.prev_points, None,
            winSize=(21, 21), maxLevel=3,
            criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01))
        if next_points is None or status is None:
            self.prev_gray = gray
            self.prev_points = None
            return 0.0, 0.0, 0.0, False

        good_old = self.prev_points[status.flatten() == 1].reshape(-1, 2)
        good_new = next_points[status.flatten() == 1].reshape(-1, 2)
        self.prev_gray = gray
        self.prev_points = good_new.reshape(-1, 1, 2) if len(good_new) else None
        if len(good_old) < 8:
            return 0.0, 0.0, 0.0, False

        # Translation estimate from median feature displacement. This only approximates
        # ground-plane motion when camera points downward, altitude is known, and yaw is small.
        median_flow = np.median(good_new - good_old, axis=0)
        height, width = gray.shape[:2]
        focal_px = width / (2.0 * math.tan(math.radians(self.hfov_deg) / 2.0))
        metres_per_pixel = self.camera_height_m / max(focal_px, 1.0)
        # Image moves opposite to camera ground translation. Positive body x is forward;
        # image vertical mapping depends on camera orientation and is configured as a rough default.
        dx = float(np.clip(-median_flow[1] * metres_per_pixel, -self.max_step_m, self.max_step_m))
        dy = float(np.clip(-median_flow[0] * metres_per_pixel, -self.max_step_m, self.max_step_m))
        valid = np.isfinite(dx) and np.isfinite(dy)
        return dx, dy, 0.0, bool(valid)
