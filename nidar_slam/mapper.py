"""Pose integrator and ultrasonic occupancy-grid updates."""
import math
import time
import cv2
from .occupancy_grid import OccupancyGrid


class MappingEngine:
    def __init__(self, cfg):
        self.cfg = cfg
        self.grid = OccupancyGrid(
            width_m=float(cfg["map_width_m"]),
            height_m=float(cfg["map_height_m"]),
            resolution_m=float(cfg["map_resolution_m"]),
        )
        self.pose = [0.0, 0.0, 0.0]
        self.sensors = cfg.get("sensors", {})
        self.max_range = float(cfg.get("max_range_m", 3.0))
        self.min_range = float(cfg.get("min_range_m", 0.04))
        self.last_update = time.monotonic()

    def integrate_motion(self, dx_body, dy_body, dtheta):
        # dx/dy are local body-frame increments in metres.
        theta_mid = self.pose[2] + dtheta / 2.0
        self.pose[0] += math.cos(theta_mid) * dx_body - math.sin(theta_mid) * dy_body
        self.pose[1] += math.sin(theta_mid) * dx_body + math.cos(theta_mid) * dy_body
        self.pose[2] = (self.pose[2] + dtheta + math.pi) % (2.0 * math.pi) - math.pi

    def update_ranges(self, sample):
        if not sample:
            return
        received = sample.get("received_monotonic", time.monotonic())
        if time.monotonic() - received > float(self.cfg.get("sensor_stale_s", 0.5)):
            return
        x, y, theta = self.pose
        for name, spec in self.sensors.items():
            distance = sample.get(name)
            if distance is None or not (self.min_range <= distance <= self.max_range):
                continue
            sx = float(spec["x_m"])
            sy = float(spec["y_m"])
            sensor_angle = math.radians(float(spec["angle_deg"]))
            # Sensor origin transformed from body frame into map frame.
            ox = x + math.cos(theta) * sx - math.sin(theta) * sy
            oy = y + math.sin(theta) * sx + math.cos(theta) * sy
            ray_angle = theta + sensor_angle
            ex = ox + distance * math.cos(ray_angle)
            ey = oy + distance * math.sin(ray_angle)
            self.grid.update_ray(ox, oy, ex, ey, hit=True)

    def render(self):
        img = self.grid.image(self.pose)
        img = cv2.resize(img, None, fx=2, fy=2, interpolation=cv2.INTER_NEAREST)
        x, y, theta = self.pose
        text = f"pose x={x:.2f}m y={y:.2f}m yaw={math.degrees(theta):.1f}deg | prototype only"
        cv2.putText(img, text, (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.48, 90, 1, cv2.LINE_AA)
        cv2.putText(img, "Unknown=grey  Free=white  Occupied=black", (10, 45),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.42, 90, 1, cv2.LINE_AA)
        return img
