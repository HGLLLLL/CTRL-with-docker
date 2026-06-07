# CTRL — Autonomous Lane-Following Car

A Dockerized **ROS Noetic** catkin workspace for an autonomous lane-following car (the "CTRL" / golfbot-style platform). The heart of the project is the **`lane_follower`** package: cameras → lane/turn detection → fuzzy controller → velocity command. Everything else (Docker image, camera driver, lidar driver, Arduino firmware) exists to support it.

---

## Hardware split

The system runs across two computers:

| Unit | Role |
| --- | --- |
| **Raspberry Pi 4 (4 GB), in Docker** | Cameras → lane/turn detection → fuzzy controller → publishes a velocity command (`arduino_vel`). |
| **Arduino Mega (rosserial over USB)** | Subscribes to `arduino_vel`, runs per-wheel encoder PID, drives the motors, and publishes ultrasonic distance back. |

**Workflow:** develop on a PC, then deploy to the car's Raspberry Pi 4 to run. The full self-driving stack runs inside the Docker container **on the Pi**, where the cameras and the Arduino Mega are physically attached. On the dev PC you can build and run the vision-only launch; the full pipeline with serial/motors only makes sense on the Pi.

> An interactive node-architecture diagram for the Shih tuning variant lives in
> [`lane_detect_bringup_Shih_architecture.html`](lane_detect_bringup_Shih_architecture.html) — open it in a browser.

---

## The control pipeline

```
camera1 (/dev/video0) ──► /camera/image_raw  ──► lane_detect_v2.py ──► lane_detect  (offset, angle)
camera2 (/dev/video2) ──► /camera2/image_raw ──► turn_detect.py    ──► turn_detect  (turn_direction, pixel_size, offset)
                                                          │                │
                                                          └──────┬─────────┘
                                                                 ▼
                                              lane_controller_fuzzy.py ──► arduino_vel (geometry_msgs/Twist)
                                                                 ▼
                                   rosserial_python serial_node.py (/dev/ttyUSB0 @ 115200)
                                                                 ▼
              ┌──────────────────  Arduino Mega (agent_ref/CTRL_rosserial_tuned)  ──────────────────┐
              │  arduino_vel → per-wheel target RPM → encoder PID @20Hz → motors                    │
              │  HC-SR04 @10Hz → publishes ultrasonic (std_msgs/Float32, cm; -1.0 = out of range)   │
              └────────────────────────────────────────────────────────────────────────────────────┘
```

**Key contracts**

- The drive topic is **`arduino_vel`** (`geometry_msgs/Twist`), *not* `/cmd_vel`. Both the Python controller and the Mega firmware must agree.
- Only `linear.x` and `angular.z` are used. The Mega maps them to wheel RPM:
  `target_rpm_L = linear.x·LINEAR_SCALE − angular.z·ANGULAR_SCALE`,
  `target_rpm_R = linear.x·LINEAR_SCALE + angular.z·ANGULAR_SCALE`
  (`LINEAR_SCALE=289.4`, `ANGULAR_SCALE=23.8` — retune per wheel diameter / track width).
- **Serial baud is 115200**, matched in the launch file and the firmware.
- The Mega also publishes `ultrasonic` (`std_msgs/Float32`, cm, `-1.0` = out of range) — available for obstacle logic.
- Custom messages: `LaneData.msg` (`offset`, `angle`) and `TurnDetect.msg` (`turn_direction`, `pixel_size`, `offset`) under `catkin_ws/src/lane_follower/msg/`.

---

## Prerequisites

Make sure you have Docker installed. If not, download it [here](https://www.docker.com/products/docker-desktop).

For the full self-driving run you also need the hardware attached to the **Pi**: two USB cameras (`/dev/video0`, `/dev/video2`) and the Arduino Mega (`/dev/ttyUSB0`).

---

## Quick start

```bash
# Clone the repository
git clone https://github.com/HGLLLLL/CTRL-with-docker.git
cd CTRL-with-docker

# Build the image (ros-noetic-zsh:latest) and run the container (interactive zsh).
# catkin_ws/ is bind-mounted into /root/catkin_ws.
# The container runs --privileged --net=host so it can reach /dev/* hardware.
make
```

Open another terminal into the running container:

```bash
make attach
```

Other `make` targets:

| Command | Effect |
| --- | --- |
| `make`        | Build image + run container (interactive, `--rm`). |
| `make attach` | Open another zsh in the running container. |
| `make stop`   | Stop and remove the container. |
| `make clean`  | Stop + remove container, and remove the image. |
| `make rviz`   | Run `rviz` inside the container. |

> The `run` target does **not** forward X11 by default. To use rviz/gazebo GUIs, uncomment the `DISPLAY` / `.X11-unix` lines in the `Makefile` and run `xhost +local:root` on the host.

---

## Building the workspace

`scripts/entrypoint.sh` builds the workspace automatically on first container start. Re-run after editing code:

```bash
cd /root/catkin_ws
catkin_make
source devel/setup.zsh
```

Build a single package:

```bash
catkin_make --pkg <package_name>   # camera | lane_follower | arduino_mega_ctrl | rplidar_ros
```

> After changing a `.msg` file you must `catkin_make` and re-`source devel/setup.zsh`, or the Python message imports fall back to stubs.

---

## Launching the self-driving system

Run this **on the Pi** (where the USB devices live). One launch file starts everything — the rosserial bridge to the Mega, both cameras, lane detection, turn detection, and the fuzzy controller:

```bash
roslaunch lane_follower lane_detect_bringup.launch
```

Useful args: `use_two_cameras:=true|false`, `cam1_fps` / `cam1_width` / `cam1_height`, `cam2_*`.

Launch variants:

| Launch file | Use |
| --- | --- |
| `lane_detect_bringup.launch`      | **Main entry point** — full stack including the rosserial bridge to the Mega. |
| `lane_detect_bringup_Shih.launch` | Tuning variant with more aggressive turn/speed params. |
| `lane_detect_v2.launch`           | Vision-only — omits the rosserial node (no Mega/serial needed). Good for the dev PC. |

Controller and detector behavior is tuned almost entirely through **launch-file params** — prefer adjusting a `<param>` over hardcoding values in the Python nodes.

---

## `lane_follower` — the brain

Python nodes under `catkin_ws/src/lane_follower/scripts/`:

- **`lane_detect_v2.py`** (`lane_detect_node`) — lane offset/angle from camera1. A multi-stage vision pipeline: preprocess (gray → blur → threshold → morphology → ROI) → contour extraction + fragment merge → per-side lane tracking → offset/yaw measurement at an anchor row → Kalman/EMA smoothing → optional debug overlay. Publishes `LaneData(offset, angle)` on `lane_detect`. Can subscribe to an image topic *or* open a camera directly via `cv2.VideoCapture`.
- **`turn_detect.py`** (`turn_detect_node`) — intersection/turn-sign detection from camera2. Finds triangular contours, classifies `left`/`right` by apex position, and only fires after `CONFIRM_FRAMES` consecutive stable detections. Publishes `TurnDetect` on `turn_detect`.
- **`lane_controller_fuzzy.py`** (`lane_controller_fuzzy`) — fuses both into `arduino_vel`. A dependency-free 5×5 Sugeno fuzzy controller maps (offset, angle) → angular velocity, wrapped in a priority state machine: active hard-turn > scheduled follow-up turn > scanning-for-lost-sign > approaching/aligning to a sign > normal fuzzy follow. Sends spaced-out stop commands on shutdown so the car halts reliably on Ctrl-C.

---

## Repository layout

| Path | What it is |
| --- | --- |
| `catkin_ws/src/lane_follower/` | **The core** — vision, turn detection, fuzzy controller, launch files, custom msgs. |
| `catkin_ws/src/sensors/camera/` | `camera` package — opens `/dev/videoN` via OpenCV V4L2 (MJPG), publishes `image_raw`. |
| `catkin_ws/src/sensors/rplidar_ros/` | Vendored Slamtec RPLIDAR driver (not wired into the lane pipeline). |
| `catkin_ws/src/arduino_mega_ctrl/` | rosserial bringup variants (note: some launches are stale — prefer `lane_follower`'s). |
| `agent_ref/` | **Reference copies** of the Arduino Mega firmware. Flashed externally via the Arduino IDE — *not* part of the catkin build. |
| `Dockerfile`, `Makefile`, `scripts/`, `dotfiles/` | The Docker/runtime layer (base `ros:noetic`, zsh shell, udev rules). |

---

## Arduino Mega firmware (`agent_ref/`)

- **`CTRL_rosserial_tuned/CTRL_rosserial_tuned.ino`** — the live firmware. rosserial node @115200: subscribes `arduino_vel`, converts Twist → per-wheel target RPM, runs a per-wheel PID loop @20 Hz with quadrature encoder feedback (CPR 330), drives two motors through a TB6612-style driver, and publishes `ultrasonic` from an HC-SR04 @10 Hz. `target_rpm = 0 ⇒ PWM 0` so a stop command actually stops the wheels.
- **`ultrasonic_serial_test/ultrasonic_serial_test.ino`** — standalone HC-SR04 bring-up sketch, not the flight firmware.

---

## Notes & conventions

- **Device paths:** udev rules provide `/dev/arduino` / `/dev/camera` / `/dev/rplidar`, but the active launch files reference raw paths (`/dev/ttyUSB0`, `/dev/video0`, `/dev/video2`). Match the existing raw-path style when editing.
- MJPG + modest resolution is used deliberately to limit USB bandwidth on the Pi.
- Comments and commit messages are bilingual (Traditional Chinese + English) — preserve the existing language when editing nearby lines.
- There is no test runner, linter, or CI configured in this repo.
