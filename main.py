"""NIDAR 2D mapping prototype. Does not control or arm a drone."""
import argparse
import time
import cv2
import yaml

from nidar_slam.serial_reader import UltrasonicSerialReader
from nidar_slam.optical_flow import OpticalFlowMotion
from nidar_slam.mapper import MappingEngine


def open_camera(cfg):
    if cfg.get("camera_backend", "opencv").lower() == "picamera2":
        try:
            from picamera2 import Picamera2
        except ImportError as exc:
            raise RuntimeError("picamera2 is not installed. Use camera_backend: opencv or install Picamera2.") from exc
        cam = Picamera2()
        cam.configure(cam.create_video_configuration(
            main={"size": (int(cfg["camera_width"]), int(cfg["camera_height"])), "format": "RGB888"}))
        cam.start()
        return ("picamera2", cam)
    cap = cv2.VideoCapture(int(cfg.get("camera_index", 0)))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, int(cfg["camera_width"]))
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, int(cfg["camera_height"]))
    if not cap.isOpened():
        raise RuntimeError("Could not open camera. Check camera_index/camera_backend and camera permissions.")
    return ("opencv", cap)


def read_frame(camera):
    backend, obj = camera
    if backend == "picamera2":
        frame = obj.capture_array()
        return cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
    ok, frame = obj.read()
    return frame if ok else None


def close_camera(camera):
    if camera is None:
        return
    backend, obj = camera
    if backend == "picamera2":
        obj.stop()
    else:
        obj.release()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", default="config.yaml")
    parser.add_argument("--no-serial", action="store_true", help="Run camera/map display without ESP32 serial")
    args = parser.parse_args()

    with open(args.config, "r", encoding="utf-8") as f:
        cfg = yaml.safe_load(f)

    engine = MappingEngine(cfg)
    flow = OpticalFlowMotion(
        hfov_deg=float(cfg["camera_hfov_deg"]),
        camera_height_m=float(cfg["assumed_camera_height_m"]),
        max_step_m=float(cfg["max_step_m"]),
    )
    camera = None
    serial_reader = None
    try:
        camera = open_camera(cfg)
        if not args.no_serial:
            serial_reader = UltrasonicSerialReader(
                cfg["serial_port"], int(cfg["baud_rate"]),
                timeout_s=float(cfg.get("serial_timeout_s", 0.1)),
                stale_s=float(cfg.get("sensor_stale_s", 0.5)),
            )
            serial_reader.start()

        print("NIDAR mapping prototype started. Press q in the map window to exit.")
        print("This program does not command flight motors.")
        while True:
            frame = read_frame(camera)
            if frame is None:
                print("Camera frame unavailable; stopping.")
                break

            dx, dy, dtheta, motion_valid = flow.update(frame)
            # This is only a rough optical-flow estimate; do not use as flight-grade odometry.
            engine.integrate_motion(dx, dy, dtheta if motion_valid else 0.0)

            ranges = serial_reader.latest() if serial_reader else {}
            engine.update_ranges(ranges)
            display = engine.render()
            cv2.imshow("NIDAR 2D occupancy grid (prototype)", display)
            cv2.imshow("Camera", cv2.resize(frame, (640, 480)))
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q") or key == 27:
                break
            time.sleep(0.005)
    except KeyboardInterrupt:
        pass
    finally:
        if serial_reader:
            serial_reader.stop()
        close_camera(camera)
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
