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
                                   rosserial_python serial_node.py (/dev/arduino @ 115200)
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

For the full self-driving run you also need the hardware attached to the **Pi**: two USB cameras (`/dev/video0`, `/dev/video2`) and the Arduino Mega. The Mega is referenced through the stable symlink **`/dev/arduino`** rather than a raw `/dev/ttyUSB*` path — see [Stable USB device names](#stable-usb-device-names-udev) below.

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

## Stable USB device names (udev)

USB enumeration order is not guaranteed — the Arduino Mega can come up as `/dev/ttyUSB0` one boot and `/dev/ttyUSB1` the next. To avoid that, the launch files no longer hardcode raw `/dev/ttyUSB*` paths; the device is now declared once at the top of each launch file as an `arg` (default **`/dev/arduino`**) and threaded into the rosserial / camera nodes:

```xml
<!-- ===== USB Port 宣告(要改裝置就改這裡) ===== -->
<arg name="arduino_port" default="/dev/arduino" />
<arg name="cam1_device"  default="/dev/video0" />
<arg name="cam2_device"  default="/dev/video2" />
```

The `/dev/arduino` symlink is created by a udev rule that matches the Mega's CH340 USB-serial chip by `idVendor:idProduct` (`1a86:7523`), so it points at the Mega no matter which USB port it's plugged into. Install the rules **once on the Pi host** (outside the container):

```bash
bash catkin_ws/src/arduino_mega_ctrl/scripts/create_udev_rules.sh
```

Because the container runs `--privileged --net=host` (shared host `/dev`), the symlink created on the host automatically shows up inside the container — no change to the Docker startup is needed. To point a launch at a different device, override the arg:

```bash
roslaunch lane_follower lane_detect_bringup.launch arduino_port:=/dev/ttyUSB0
```

---

## Fixing rosserial sync errors

If the Mega connects but rosserial reports *"Lost sync with device"* or *"Unable to sync with device"*, the Arduino's bundled `ros_lib` is out of step with the host's ROS Noetic message definitions. A pre-generated Noetic `ros_lib` is checked in as **`catkin_ws/ros_lib_noetic.tar.gz`** — unpack it into the Arduino IDE's `libraries/` folder to replace the stale copy. Step-by-step instructions (with the root cause explained) are in **`catkin_ws/rosserial_fix_guide.html`** — open it in a browser.

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

- **Device paths:** the active launch files declare devices as `arg`s at the top (`arduino_port` → `/dev/arduino`, `cam1_device` / `cam2_device` → `/dev/video0` / `/dev/video2`) and thread them into the nodes. Override with `roslaunch ... arduino_port:=/dev/ttyUSB0`. Install the `/dev/arduino` udev symlink on the Pi host with `create_udev_rules.sh` (see [Stable USB device names](#stable-usb-device-names-udev)).
- MJPG + modest resolution is used deliberately to limit USB bandwidth on the Pi.
- Comments and commit messages are bilingual (Traditional Chinese + English) — preserve the existing language when editing nearby lines.
- There is no test runner, linter, or CI configured in this repo.
