# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Overview

A Dockerized ROS Noetic catkin workspace for an autonomous lane-following car ("CTRL" / golfbot-style platform). **The center of gravity of this repo is the `lane_follower` package under `catkin_ws/src/` — that is where almost all active development happens.** Everything else (Docker image, camera driver, lidar driver, firmware) exists to support it.

**Workflow: develop on a PC, then deploy to the car's Raspberry Pi 4 (4 GB) to run.** Code is edited on a dev PC and carried over to the Pi; the actual self-driving stack runs inside the Docker container **on the Pi**, where the cameras and the Arduino Mega are physically attached (that's why the `Makefile` uses `--privileged --net=host` and raw `/dev/*` paths — it must run on the machine with the hardware). On the PC you mostly edit code and can run the vision-only launch; the full pipeline with serial/motors only makes sense on the Pi. The Pi does the vision and control; it talks over USB serial to an **Arduino Mega**, which is the motor-control MCU sitting between the Pi and the motors — it closes the speed loop on the wheels and reports sensor data back. So the division of labour is:

- **Raspberry Pi 4 (in Docker):** cameras → lane/turn detection → fuzzy controller → publishes a velocity command.
- **Arduino Mega (rosserial over USB):** subscribes to that velocity command, runs per-wheel encoder PID, drives the motors, and publishes ultrasonic distance back.

The bulk of what the system does is visible in the `lane_detect`/`lane_follower` nodes; read those first.

## Common commands

Build/run is orchestrated by the top-level `Makefile`. The self-driving target is run **on the Pi** (where the USB devices are attached); on the dev PC the same commands work for building and for the vision-only launch:

- `make` — build image `ros-noetic-zsh:latest` and run container `ros-noetic-zsh` (interactive zsh, `--rm`, `--privileged`, `--net=host`, `--ulimit nofile=1024:524288`). `catkin_ws/` is bind-mounted into `/root/catkin_ws`.
- `make attach` — open another zsh in the running container.
- `make stop` / `make clean` — stop+remove container / also remove image.
- `make rviz` — run `rviz` inside the container.

Inside the container (`scripts/entrypoint.sh` does this automatically on first start; re-run after editing code):

```bash
cd /root/catkin_ws
catkin_make
source devel/setup.zsh
```

Build a single package: `catkin_make --pkg <package_name>` (package names: `camera`, `lane_follower`, `arduino_mega_ctrl`, `rplidar_ros`).

**Launch the full self-driving system:**

```bash
roslaunch lane_follower lane_detect_bringup.launch
```

This one launch file starts everything: the rosserial bridge to the Mega, both cameras, lane detection, turn detection, and the fuzzy controller. Useful args: `use_two_cameras:=true|false`, `cam1_fps`/`cam1_width`/`cam1_height`, `cam2_*`.

There is no test runner, linter, or CI configured in this repo.

## The control pipeline (the big picture)

This is the core of the system and spans several packages. Data flows topic-to-topic:

```
camera1 (/dev/video0) ──► /camera/image_raw  ──► lane_detect_v2.py ──► lane_detect  (LaneData: offset, angle)
camera2 (/dev/video2) ──► /camera2/image_raw ──► turn_detect.py    ──► turn_detect  (TurnDetect: turn_direction, pixel_size, offset)
                                                          │                │
                                                          └──────┬─────────┘
                                                                 ▼
                                              lane_controller_fuzzy.py ──► arduino_vel (geometry_msgs/Twist)
                                                                 ▼
                                   rosserial_python serial_node.py (/dev/ttyUSB0 @ 115200)
                                                                 ▼
              ┌──────────────────────  Arduino Mega (agent_ref/CTRL_rosserial_tuned)  ──────────────────────┐
              │  subscribes arduino_vel → per-wheel target RPM → encoder PID @20Hz → motors                 │
              │  reads HC-SR04 @10Hz → publishes ultrasonic (std_msgs/Float32, cm; -1.0 = out of range)     │
              └────────────────────────────────────────────────────────────────────────────────────────────┘
```

Key contracts to keep consistent when editing:

- **The drive topic is `arduino_vel` (`geometry_msgs/Twist`), not `/cmd_vel`.** `lane_controller_fuzzy.py` publishes it and the Mega firmware subscribes to it. Renaming requires changing both the Python node and `agent_ref/CTRL_rosserial_tuned/CTRL_rosserial_tuned.ino`.
- **Only `linear.x` and `angular.z` are used.** The Mega maps them to wheel RPM as `target_rpm_L = linear.x*LINEAR_SCALE − angular.z*ANGULAR_SCALE`, `target_rpm_R = linear.x*LINEAR_SCALE + angular.z*ANGULAR_SCALE` (`LINEAR_SCALE=289.4`, `ANGULAR_SCALE=23.8` — retune per wheel diameter/track width).
- **Serial baud is 115200** in the active launch (`lane_detect_bringup.launch`) and in the firmware (`nh.getHardware()->setBaud(115200)`). They must match.
- The Mega also **publishes** `ultrasonic` (`std_msgs/Float32`, cm, `-1.0` = timeout/out of range). Nothing in the lane pipeline consumes it yet — it's available for obstacle logic.
- Custom messages live in `lane_follower/msg/`: `LaneData.msg` (`offset`, `angle`) and `TurnDetect.msg` (`turn_direction`, `pixel_size`, `offset`). After changing a `.msg` you must `catkin_make` and re-`source devel/setup.zsh` or the Python imports fall back to stubs.
- `lane_controller_fuzzy.py` registers `rospy.on_shutdown(self.shutdown_hook)` to send spaced-out stop commands (20× with `time.sleep(0.05)`) so the car halts reliably on Ctrl-C (commit 6f2473e). Preserve this behavior when touching the controller.

## `lane_follower` — the brain (read this package first)

Python nodes in `scripts/`, all tuned via launch params rather than code:

- **`lane_detect_v2.py`** (node `lane_detect_node`) — lane offset/angle from camera1. A self-contained monolith holding a multi-stage vision pipeline: `preprocess` (gray→blur→Otsu/adaptive threshold→morphology→ROI) → `find_lane_contours` (contour extraction + floating-fragment merge + ROI clipping) → `LaneTracker` (per-side left/right tracking with short-term memory) → `measure_at_anchor` (offset & yaw at an anchor row near the image bottom) → `LaneSmoother` (Kalman or EMA backend, with a bounded predict-when-lost mode) → optional `render` debug overlay. Publishes `LaneData(offset, angle)` on `lane_detect`. Can either subscribe to an image topic **or**, if `~image_topic` is a number / `/dev/videoN`, open that camera directly with `cv2.VideoCapture`.
- **`turn_detect.py`** (node `turn_detect_node`) — intersection/turn-sign detection from camera2. Thresholds the frame, finds triangular contours, classifies `left`/`right` by apex position relative to the bounding-box center, and only fires after `CONFIRM_FRAMES` consecutive stable detections. Publishes `TurnDetect(turn_direction, pixel_size, offset)` on `turn_detect`.
- **`lane_controller_fuzzy.py`** (node `lane_controller_fuzzy`) — fuses both into `arduino_vel`. A dependency-free 5×5 Sugeno fuzzy controller maps (offset, angle) → angular velocity for normal lane following, wrapped in a priority state machine: active hard-turn > scheduled follow-up turn > scanning-for-lost-sign > approaching/aligning to a sign > normal fuzzy follow. Hard turns are triggered when a `turn_detect` sign exceeds a pixel-size threshold; after the *first* hard turn it arms a scheduled same-direction follow-up turn `scheduled_turn_delay` seconds later.

Launch files (`launch/`):

- **`lane_detect_bringup.launch`** — the main entry point; full stack including the rosserial bridge to the Mega.
- **`lane_detect_bringup_Shih.launch`** — a tuning variant with more aggressive turn/speed params (higher `base_speed`/`max_angular`, lower `sign_detect_pixel_threshold`).
- **`lane_detect_v2.launch`** — vision-only; omits the rosserial node (no Mega/serial needed).

The controller exposes extensive fuzzy/turn params (hard-turn thresholds/durations, scheduled follow-up turn, cooldown, sign-alignment). **Tune via launch params, not code.**

## The Arduino Mega firmware (`agent_ref/`)

Reference copies of the firmware that runs on the Mega — the on-MCU counterpart to the rosserial bridge. **Flashed externally via the Arduino IDE; NOT part of the catkin build**, and per project convention `agent_ref/` is reference context, not a build target.

- **`CTRL_rosserial_tuned/CTRL_rosserial_tuned.ino`** — the live firmware. rosserial node at baud 115200: subscribes `arduino_vel`, converts Twist → per-wheel target RPM, runs a per-wheel PID loop at 20 Hz (50 ms) using quadrature encoder feedback (CPR 330, low-pass filtered), drives the two motors through a TB6612-style driver (`PWMA/AIN1/AIN2/STBY/PWMB/BIN1/BIN2`). Separately schedules an HC-SR04 ultrasonic read at 10 Hz and publishes `ultrasonic`. Holds `target_rpm = 0 ⇒ PWM 0` (and integral reset) so a stop command from the Pi actually stops the wheels.
- **`ultrasonic_serial_test/ultrasonic_serial_test.ino`** — a standalone HC-SR04 bring-up/test sketch, not the flight firmware.

## Other ROS packages (`catkin_ws/src/`)

- **`sensors/camera`** (package name `camera`) — `src/camera.py`. Opens `/dev/videoN` via OpenCV V4L2 with MJPG and configurable resolution/FPS (params `~camera_id`, `~camera_name`, `~width`, `~height`, `~frame_rate`). Publishes `/<camera_name>/image_raw` (sensor_msgs/Image) and `/golfbot/<camera_name>_web` (base64 String for a web UI). Instantiated twice (camera1/camera2) in the bringup launches.
- **`sensors/rplidar_ros`** — vendored Slamtec RPLIDAR driver (full SDK under `sdk/`). Launch files for S2 and merged-lidar setups. Not wired into the lane-following pipeline; present for navigation experiments.
- **`arduino_mega_ctrl`** — rosserial bringup variants (`arduino_bringup.launch`, `lane_following.launch`) plus `scripts/move_straight_5s.py` test. **Note: these launch files are stale** — `lane_following.launch` references `lane_detect.py`/`lane_controller.py` which no longer exist, and they use baud 57600. Prefer `lane_follower/lane_detect_bringup.launch`.
- **`rosserial`** — empty placeholder directory. The actual rosserial packages come from apt (`ros-noetic-rosserial*`), so launches call the installed `rosserial_python serial_node.py` directly.

## Docker layer (`Dockerfile`, `Makefile`, `scripts/`, `dotfiles/`, `cachefile/`)

- Base: `ros:noetic`. Installs ROS packages (tf2, cv_bridge, image_transport, rosserial + rosserial-arduino + rosserial-python, rplidar-ros, web_video_server, robot_state_publisher, …), Python deps via pip, and a zsh/oh-my-zsh/powerlevel10k shell.
- `scripts/entrypoint.sh` (container start): starts sshd, sources ROS, `catkin_make`s the workspace, then installs udev rules from `scripts/*.rules` for stable USB symlinks (`/dev/rplidar`, `/dev/arduino`, `/dev/camera`). **The entrypoint also `cp`s `plate.rules` and `realsensecamera.rules`, which are NOT present in `scripts/`** — those copies warn but the script continues.
- The `Makefile` `run` target intentionally does **not** forward X11 (the `DISPLAY` / `.X11-unix` lines are commented out). To use rviz/gazebo GUIs, uncomment those lines and run `xhost +local:root` on the host.

## Conventions

- **Device paths:** udev rules exist (`/dev/arduino`, `/dev/camera`, `/dev/rplidar`), but the **active launch files reference raw paths** (`/dev/ttyUSB0`, `/dev/video0`, `/dev/video2`) directly. When editing launches, match the existing raw-path style unless deliberately migrating to symlinks (and verify the symlink resolves for the connected hardware).
- Camera nodes follow a `~camera_id`/`~width`/`~height`/`~frame_rate` private-param pattern; keep it consistent across the dual-camera config. MJPG + modest resolution is used deliberately to limit USB bandwidth on the Pi (see camera commit history).
- Comments and commit messages are bilingual (Traditional Chinese + English); preserve the existing language when editing nearby lines.
- Controller/detector behavior is tuned almost entirely through launch-file params. Prefer adding/adjusting a `<param>` over hardcoding values in the Python nodes.
