#!/bin/zsh

# Start the SSH daemon in the background
/usr/sbin/sshd

# Compile the project
source /opt/ros/noetic/setup.zsh
catkin_make
source /root/catkin_ws/devel/setup.zsh
cd /root/catkin_ws

# 把預先打包的 rosdebug skill 放進 Claude 設定目錄 (每次開機刷新，idempotent)。
# CLAUDE_CONFIG_DIR 由 Makefile 掛載到主機，所以 skill 與登入憑證都會落在持久目錄。
CLAUDE_DIR="${CLAUDE_CONFIG_DIR:-/root/.claude}"
if [ -d /opt/claude-skills/rosdebug ]; then
  mkdir -p "$CLAUDE_DIR/skills"
  cp -r /opt/claude-skills/rosdebug "$CLAUDE_DIR/skills/"
  echo "Loaded rosdebug skill into $CLAUDE_DIR/skills/"
  echo " "
fi

# Setup USB connection
echo "Remap the serial port(ttyUSBX, ttyACMX) to custom name"
echo " "

echo "Rplidar usb connection as /dev/rplidar"
echo "Plate usb connection as /dev/plate"
echo "Arduino usb connection as /dev/arduino"
echo "Camera usb connection as /dev/camera"
echo "Realsense camera usb connection as /dev/realsensecamera"
echo " "

echo "Check these using the command : ls -l /dev|grep ttyUSB"
echo "Check the detail of the connection, using the command: udevadm info --attribute-walk /dev/ttyUSBX"
echo "(replace the /dev/ttyUSBX with your target device)"
echo " "

echo "Start copy rule files in scripts, to /etc/udev/rules.d/"
cp /root/scripts/rplidar.rules /etc/udev/rules.d
cp /root/scripts/plate.rules /etc/udev/rules.d
cp /root/scripts/arduino.rules /etc/udev/rules.d
cp /root/scripts/camera.rules /etc/udev/rules.d
cp /root/scripts/realsensecamera.rules /etc/udev/rules.d
echo " "

echo "Restarting udev"
service udev restart
udevadm control --reload-rules
udevadm trigger
echo " "

echo "Finish usb port setup"
echo " "

exec "$@"