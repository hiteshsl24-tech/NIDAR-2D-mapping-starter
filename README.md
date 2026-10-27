# NIDAR AirMouse — 2D Mapping Starter (VS Code)

This repository is a **prototype/starter**, not flight-ready autonomous SLAM. It reads HC-SR04 measurements from an ESP32 over USB serial, reads a camera stream, estimates a rough planar motion signal from optical flow, and updates a 2D occupancy grid.

## Important limitations
- This is not a validated full SLAM implementation and must not be used to command a flying drone without extensive simulation and safety testing.
- Monocular camera motion has scale ambiguity. The included optical-flow estimate is approximate and depends on camera calibration, camera height, level flight, and low yaw/roll/pitch.
- Ultrasonic sensors have wide beams and may return unreliable readings on angled, soft, narrow, or irregular objects.
- This starter does **not** perform loop closure, robust visual-inertial optimization, survivor detection, or autonomous flight control.
- Use SITL / bench tests first. Keep the Pixhawk's independent stabilization and failsafes enabled.

## Hardware assumed
- Raspberry Pi 5 running Raspberry Pi OS / Ubuntu, or a Linux laptop for testing
- Pi Camera Module 3 (CSI) or USB webcam
- ESP32 DevKit with four HC-SR04 sensors
- USB serial connection from ESP32 to Pi
- Pixhawk remains responsible for stabilization; this project does not arm or control motors.

## Project layout
```text
NIDAR_2D_SLAM_VSCode/
  main.py                 # Runs camera + serial reader + occupancy grid display
  config.yaml              # Sensor geometry and runtime settings
  requirements.txt
  nidar_slam/
    __init__.py
    config.py
    serial_reader.py
    optical_flow.py
    occupancy_grid.py
    mapper.py
  firmware/
    esp32_ultrasonic.ino   # Arduino IDE firmware for ESP32
  tests/
    test_grid.py
```

## Install in VS Code
1. Extract this ZIP.
2. Open the `NIDAR_2D_SLAM_VSCode` folder in VS Code.
3. Create and activate a virtual environment:
   ```bash
   python3 -m venv .venv
   source .venv/bin/activate
   ```
   On Windows PowerShell: `.venv\\Scripts\\Activate.ps1`
4. Install dependencies:
   ```bash
   python -m pip install --upgrade pip
   pip install -r requirements.txt
   ```
5. Edit `config.yaml` and set `serial_port`.
   - Linux: commonly `/dev/ttyUSB0` or `/dev/ttyACM0`
   - macOS: commonly `/dev/cu.usbserial-*` or `/dev/cu.usbmodem*`
   - Windows: e.g. `COM4`
6. Run:
   ```bash
   python main.py
   ```
7. Press `q` in the map window to quit.

## Camera setup
- For a USB webcam, set `camera_index: 0`.
- For Raspberry Pi Camera Module 3, install the Raspberry Pi camera stack and use `camera_backend: picamera2`. Install `picamera2` using your OS package manager where possible.
- For quick laptop testing, use `camera_backend: opencv`.

## ESP32 serial protocol
The ESP32 sends one JSON object per line, for example:
```json
{"seq":12,"front_mm":540,"left_mm":820,"right_mm":760,"rear_mm":610,"valid":{"front":true,"left":true,"right":true,"rear":true}}
```
Distances are in millimetres. A value is valid only when the sensor reports a plausible distance. The Python reader ignores malformed lines and stale/invalid readings.

## Sensor placement
Edit `config.yaml`. Coordinates are metres in the drone body frame:
- +x forward
- +y left
- positive angle counter-clockwise from +x
- front sensor faces 0 degrees
- left sensor faces +90 degrees
- rear sensor faces 180 degrees
- right sensor faces -90 degrees

Map coordinates use metres, with the starting pose at (0, 0, 0). This simple mapper assumes the drone stays approximately level and that motion estimate is already in metres. **Do not assume its map is metrically accurate until calibrated against a surveyed test area.**

## HC-SR04 electrical safety
- HC-SR04 uses 5 V; ESP32 GPIO is 3.3 V only.
- Put a voltage divider on every Echo output. Example: 10 kΩ from Echo to GPIO junction, 18 kΩ from junction to GND.
- Share ground between ESP32 and sensor power supply.
- Trigger sensors sequentially to reduce acoustic cross-talk.
- Confirm the ESP32 board's GPIO pinout before wiring.

## Upload to GitHub
From the project folder:
```bash
git init
git add .
git commit -m "Add NIDAR 2D mapping starter"
git branch -M main
git remote add origin https://github.com/YOUR-USERNAME/YOUR-REPOSITORY.git
git push -u origin main
```
Create an empty GitHub repository first and replace the URL with your repository URL.

## Safety checklist before any flight
- Test serial sensors with motors off, then test for interference with motors running while restrained.
- Validate map scale and pose against measured ground truth.
- Test all sensor failure cases and stale-data handling.
- Never let this prototype arm or command motors.
- Use a netted test area, propellers removed for bench tests, a spotter, and independent Pixhawk failsafes.
# NIDAR-2D-mapping-starter
