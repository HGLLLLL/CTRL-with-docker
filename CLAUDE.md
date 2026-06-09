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

Launch the main mission (走線 → 硬轉觸發 → 光達避障交棒)：
`roslaunch main_control mission_bringup.launch`
（常用覆寫：`hard_turn_trigger_count:=3 handoff_stop_duration:=1.0 cmd_vel_topic:=/arduino_vel`）

Sub-system launches when iterating on a single layer:
- `roslaunch lane_follower lane_detect_bringup.launch` — 走線 + 轉彎偵測 + fuzzy 控制（不含 lidar / odom）。
- `roslaunch nav_scripts nav_bringup.launch nav_mode:=lidar_odom` — 光達避障獨立跑，啟動時 `wait_for_phase=false`，直接從 `S_FWD_1` 起跑。
- `roslaunch arduino_mega_ctrl arduino_bringup.launch` — 只開 rosserial bridge。

There is no test runner, linter, or CI configured in this repo.

## Architecture

### Docker layer (`Dockerfile`, `Makefile`, `scripts/`, `dotfiles/`, `cachefile/`)

- Base: `ros:noetic`. Installs ROS packages (tf2, cv_bridge, image_transport, rosserial, rplidar-ros, web_video_server, robot_state_publisher, …), Python deps (`pyserial`, `tornado`, `filterpy`, `qrcode_terminal`), and a zsh/oh-my-zsh/powerlevel10k shell environment.
- `scripts/entrypoint.sh` runs on container start: starts sshd, sources ROS, `catkin_make`s the workspace, then installs udev rules from `scripts/*.rules` so USB devices appear as stable symlinks: `/dev/rplidar`, `/dev/plate`, `/dev/arduino`, `/dev/camera`, `/dev/realsensecamera`. Note: the entrypoint copies `plate.rules` and `realsensecamera.rules` which are not present in `scripts/` — those `cp` calls will warn but the script continues.
- The `Makefile` `run` target intentionally does **not** forward X11; lines for `DISPLAY` / `.X11-unix` are commented out. To use rviz/gazebo GUIs you must uncomment those and run `xhost +local:root` on the host.

### ROS workspace (`catkin_ws/src/`)

Top-level orchestrator is now **`main_control`** — pure launch package. `mission_bringup.launch` 把走線階段（`lane_follower`）跟光達避障階段（`nav_scripts`）串成一個任務，靠 latched topic `/mission/phase` 做兩階段交棒（見下方 *Mission phase handoff*）。

Packages present in this repo:

- **`main_control`** — launch-only。`mission_bringup.launch` 同時啟動 rosserial、雙鏡頭、rplidar、`lidar_filter` / `odometry` / `lane_detect_v2` / `turn_detect` / `lane_controller_fuzzy` / `lidar_odom_nav_node`，並把 `lane_controller_fuzzy` 的 `cmd_vel` remap 到 `cmd_vel_topic`（預設 `/arduino_vel`）。`exec_depend` 列出所有實際提供節點的 package。
- **`sensors/camera`** — `camera.py` 節點。OpenCV V4L2 後端 + MJPG，private params `camera_id` / `camera_name` / `width` / `height` / `frame_rate`。發布 `/<camera_name>/image_raw` (sensor_msgs/Image) 與 `/golfbot/<camera_name>_web` (base64 String for web UI)。`mission_bringup` 跑雙鏡頭：`camera`（前視走線，`/dev/video0`）+ `camera2`（轉彎標誌偵測，`/dev/video2`）。
- **`sensors/rplidar_ros`** — vendored Slamtec RPLIDAR driver (full SDK under `sdk/`)。`mission_bringup` 用 `rplidar_a1.launch`；另有 `merged_lidar.launch` / `view_rplidar_a1.launch`。
- **`lane_follower`** — Python 走線堆疊：
  - 節點：`lane_detect_v2.py`（單一 detector，發 `lane_detect` topic `LaneData.msg`）、`turn_detect.py`（轉彎箭頭 / 標誌偵測，發 `turn_detect` topic `TurnDetect.msg`，二值化已參數化：`~threshold_method` / `~threshold_value` / `~invert_binary`）、`lane_controller_fuzzy.py`（fuzzy 控制 + 多段硬轉 + 任務交棒）。
  - Msg：`LaneData.msg`、`TurnDetect.msg`（自定義訊息，必須先 build 才能 import）。
  - Launch 變體：`lane_detect_bringup.launch`（正式）、`lane_detect_bringup_Shih.launch`（個人調參）、`lane_detect_view.launch`（含 GUI debug）。
- **`arduino_mega_ctrl`** — rosserial bridge launches (`arduino_bringup.launch`, `lane_following.launch`) plus `move_straight_5s.py` 測試腳本。底層也是 `rosserial_python serial_node.py`，但 `mission_bringup` 自己起 serial_node 不走這支 launch。Arduino 串口在 `mission_bringup` 寫死 `/dev/ttyUSB0`（不是 `/dev/arduino` udev symlink）。
- **`nav_scripts`** — 雷射 + 編碼器導航腳本集 (Python)：
  - `lidar_filter_node.py`：訂閱 `/scan`，擷取正前方 `±~front_angle_range` 度扇形（`~front_angle_offset_deg` 可旋轉中心角，預設 180°）、過濾 `0/inf` 及超過 `~max_valid_range`，發布最短距離到 `/lidar_output` (std_msgs/Float32)。處理 RPLidar 陣列頭尾相接。
  - `odometry_node.py`：訂閱 `/encoders` (geometry_msgs/Point，x=左輪累積 ticks，y=右輪累積 ticks)，差動驅動運動學積分後發布 `/odometry` (nav_msgs/Odometry)，可選廣播 `odom -> base_link` TF (`~publish_tf`)。必要參數 `~wheel_radius` / `~wheel_base` / `~ticks_per_rev` 必須由 launch 指定，未指定會 `logfatal` 結束。
  - `lidar_odom_nav_node.py`：狀態機 `S_IDLE → S_FWD_1 → 右轉 90° → S_FWD_2 → 左轉 90° → S_FWD_3 → 停車`。距離門檻 `dist_th_*` 看 `/lidar_output`，轉彎角度用 `/odometry` 累積差量（含 yaw wrap-around）。`~wait_for_phase=true` 時啟動停在 `S_IDLE`，收到 `/mission/phase == ~trigger_phase`（預設 `"lidar_avoid"`）才進 `S_FWD_1`。發布 `/cmd_vel`。
  - `pure_odom_nav_node.py`：純里程計盲走，動作以 `DEFAULT_PLAN` (list of dict, 支援 `forward / backward / turn / wait / stop`) 描述。
  - Launch：`nav_bringup.launch`（`nav_mode:=lidar_odom | pure_odom | none`；單獨跑時 `wait_for_phase=false`）、`odom.launch`、`lidar_test.launch`。
- **`rosserial/`** — 空的頂層資料夾，當作 vendor placeholder（要從上游 clone `ros-drivers/rosserial` 進來；`rosserial_python` 是 apt 裝的，這個資料夾留給 source build 時用）。

### Mission phase handoff（兩階段交棒）

`main_control/mission_bringup.launch` 用 latched topic `/mission/phase` (std_msgs/String) 串走線 → 光達避障，避免兩個控制器同時搶 `cmd_vel`：

1. 開機：`lane_controller_fuzzy` 在 init 時 publish 一次 `"lane"`（latched）。`lidar_odom_nav_node` `wait_for_phase=true`，看到非 `"lidar_avoid"` 就停在 `S_IDLE`，**不發任何 `cmd_vel`**。
2. 走線階段：fuzzy 控制器全權控制底盤，含多段硬轉 (`hard_turn_*_1` / `hard_turn_*_2` / `scheduled_turn_*`)，每次 vision 觸發的硬轉完成後 `hard_turn_count++`。
3. 觸發交棒：`hard_turn_count == ~hard_turn_trigger_count` 時，先停車 `~handoff_stop_duration` 秒，然後 publish `/mission/phase = "lidar_avoid"`。
4. 之後 `lane_controller_fuzzy` 自靜音不再 publish `cmd_vel`；`lidar_odom_nav_node` 收到 phase 後從 `S_FWD_1` 起跑接管。

調參入口都在 `mission_bringup.launch` 的 `<arg>` 區塊。獨立跑 `nav_bringup.launch` 時 `wait_for_phase=false`，所以直接從 `S_FWD_1` 起跑，方便單獨測試光達狀態機。

### Conventions

- Camera launch/params 保持 `~camera_id` / `~camera_name` / `~width` / `~height` / `~frame_rate` private-param 模式，雙鏡頭兩邊一致。MJPG 是為了壓 USB 頻寬的必要選擇，不要關掉。
- Comments and commit messages in this repo are bilingual (Traditional Chinese + English); preserve existing language when editing nearby lines.
- USB 裝置「應該」用 udev symlink（`/dev/camera`、`/dev/arduino`、`/dev/rplidar`），但目前 `mission_bringup.launch` 暫時寫死 `/dev/ttyUSB0` 跟 `/dev/video0` / `/dev/video2`。若實機插拔順序會變動，建議改回 symlink；改的時候要同時確認 `scripts/entrypoint.sh` 能成功 `cp` 對應 `.rules`（`scripts/` 目前只有 `arduino.rules` / `camera.rules` / `rplidar.rules`，`plate.rules` 跟 `realsensecamera.rules` 不存在，那兩行 `cp` 會 warn 但不中止）。
- 對控制器的修改要同時看「兩階段交棒」會不會被影響：`lane_controller_fuzzy.py` 是 `/mission/phase = "lane"` 的 publisher、`lidar_odom_nav_node.py` 是 subscriber。任何會讓 `hard_turn_count` 不再正確遞增、或讓 `lane_controller` 提前靜音的改動，都會卡住整個任務。
- 對 `lane_follower/msg/*.msg` 的修改要重新 `catkin_make`，不然 import `from lane_follower.msg import ...` 會失敗（`turn_detect.py` 已有 try/except fallback，但只是為了避免 import 階段炸掉，不會真的 publish 出去）。
