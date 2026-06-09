# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Overview

A Dockerized ROS Noetic catkin workspace for a robot platform ("DLV" / golfbot-style vehicle). The host repo provides the Docker image and configuration; all ROS development happens inside the container against the bind-mounted `catkin_ws/`.

## Common commands

All build/run is orchestrated by the top-level `Makefile`:

- `make` — build image `ros-noetic-zsh:latest` and run container `ros-noetic-zsh` (interactive zsh, `--rm`, `--privileged`, `--net=host`). `catkin_ws/` is bind-mounted into `/root/catkin_ws`.
- `make attach` — open another zsh in the running container.
- `make stop` / `make clean` — stop+remove container / also remove image.
- `make rviz` — run `rviz` inside the container.

Inside the container (the entrypoint at `scripts/entrypoint.sh` does this automatically on first start, but re-run after editing):

```bash
cd /root/catkin_ws
catkin_make
source devel/setup.zsh
```

Build a single package: `catkin_make --pkg <package_name>`.

Launch the main system: `roslaunch dlv_bringup bringup.launch` (args: `teleop:=true|false`, `method:=web|gui|joystick|keyboard`, `sensor:=true|false`).

There is no test runner, linter, or CI configured in this repo.

## Architecture

### Docker layer (`Dockerfile`, `Makefile`, `scripts/`, `dotfiles/`, `cachefile/`)

- Base: `ros:noetic`. Installs ROS packages (tf2, cv_bridge, image_transport, rosserial, rplidar-ros, web_video_server, robot_state_publisher, …), Python deps (`pyserial`, `tornado`, `filterpy`, `qrcode_terminal`), and a zsh/oh-my-zsh/powerlevel10k shell environment.
- `scripts/entrypoint.sh` runs on container start: starts sshd, sources ROS, `catkin_make`s the workspace, then installs udev rules from `scripts/*.rules` so USB devices appear as stable symlinks: `/dev/rplidar`, `/dev/plate`, `/dev/arduino`, `/dev/camera`, `/dev/realsensecamera`. Note: the entrypoint copies `plate.rules` and `realsensecamera.rules` which are not present in `scripts/` — those `cp` calls will warn but the script continues.
- The `Makefile` `run` target intentionally does **not** forward X11; lines for `DISPLAY` / `.X11-unix` are commented out. To use rviz/gazebo GUIs you must uncomment those and run `xhost +local:root` on the host.

### ROS workspace (`catkin_ws/src/`)

Top-level orchestrator is **`dlv_bringup`** — it only contains launch files and depends (via `<include>`) on packages that are **not in this repo**: `dlv_plate_ctrl` and `dlv_teleop`. Those must be cloned alongside for `bringup.launch` to actually run end-to-end.

Packages present in this repo:

- **`dlv_bringup`** — launch-only umbrella. `bringup.launch` wires up plate control, camera, and teleop. Other launch files cover line following, mecanum odometry, multitask, rosbag recording, and tests.
- **`sensors/camera`** — `camera.py` node. Opens `/dev/videoN` via OpenCV V4L2 backend with MJPG/configurable resolution+FPS (params `camera_id`, `camera_name`, `width`, `height`, `frame_rate`). Publishes `/<camera_name>/image_raw` (sensor_msgs/Image) and `/golfbot/<camera_name>_web` (base64 String for web UI). Used in dual-camera configurations.
- **`sensors/rplidar_ros`** — vendored Slamtec RPLIDAR driver (full SDK under `sdk/`). Launch files for S2 and merged-lidar setups.
- **`lane_follower`** — Python lane-detection + control nodes (`lane_detect.py`, `lane_detect_v2.py`, `lane_controller.py`, `lane_controller_fuzzy.py`, `turn_detect.py`). Multiple launch variants exist for different detector/controller pairings.
- **`arduino_mega_ctrl`** — rosserial bridge launches (`arduino_bringup.launch`, `lane_following.launch`) plus a `move_straight_5s.py` test script. Expects `/dev/arduino`.
- **`simple_twist_publisher`** — small C++ utility node.
- **`nav_scripts`** — 雷射 + 編碼器導航腳本集 (Python)：
  - `lidar_filter_node.py`：訂閱 `/scan`，擷取正前方 `±~front_angle_range` 度扇形、過濾 `0/inf`，發布最短距離到 `/lidar_output` (std_msgs/Float32)。處理 RPLidar 陣列頭尾相接。
  - `odometry_node.py`：訂閱 `/encoders` (geometry_msgs/Point，x=左輪累積 ticks，y=右輪累積 ticks)，差動驅動運動學積分後發布 `/odometry` (nav_msgs/Odometry) 並廣播 `odom -> base_link` TF。必要參數 `~wheel_radius` / `~wheel_base` / `~ticks_per_rev` 必須由 launch 指定，未指定會 `logfatal` 結束。
  - `lidar_odom_nav_node.py`：半寫死狀態機（直行→右轉 90°→直行→左轉 90°→直行→停車），距離門檻來自 `/lidar_output`，轉彎角度用 `/odometry` 累積差量判斷（已處理 yaw wrap-around）。發布 `/cmd_vel`。
  - `pure_odom_nav_node.py`：純里程計盲走，動作以 `DEFAULT_PLAN` (list of dict, 支援 `forward / backward / turn / wait / stop`) 描述，方便擴充路徑點。
  - 整合 launch：`nav_bringup.launch`，用 `nav_mode:=lidar_odom | pure_odom | none` 切換要跑哪個導航腳本（`lidar_filter` 與 `odometry` 一律啟動）。`cmd_vel_topic` 預設 remap 成 `/arduino_vel` 對接 rosserial。

### Conventions

- The recent commit history shows active iteration on camera configuration (frame rate, resolution, dual-camera IDs, MJPG compression to reduce USB bandwidth). When touching camera launch/params, keep the `~camera_id`, `~width`, `~height`, `~frame_rate` private-param pattern consistent across dual cameras.
- Comments and commit messages in this repo are bilingual (Traditional Chinese + English); preserve existing language when editing nearby lines.
- USB devices should be referenced by their udev symlink (e.g. `/dev/camera`), not by raw `/dev/ttyUSB*` or `/dev/video*`, except where a numeric index is required by V4L2 (handled in `camera.py` by parsing `/dev/videoN`).
