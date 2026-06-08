#!/bin/bash
# 在 RPi 主機上(容器外)執行一次,幫 Arduino 與 RPLIDAR 建立固定的 /dev symlink。
# 之後不論插哪個 USB 孔、重開機,都會固定出現:
#   Arduino Mega → /dev/arduino           (launch 檔的 port 用這個)
#   RPLIDAR      → /dev/rplidar_front / /dev/rplidar_rear (依序號比對)
#
# 因為 Docker 用 --privileged --net=host 共用主機 /dev,主機建立的 symlink
# 會自動出現在容器內,所以 Docker 啟動指令不用改。
#
# 用法:  bash catkin_ws/src/arduino_mega_ctrl/scripts/create_udev_rules.sh
set -e

# repo 根目錄(本腳本在 catkin_ws/src/arduino_mega_ctrl/scripts/,往上 4 層)。
# 規則檔以 repo 根目錄的 scripts/ 為單一來源(與 Docker image 內的同一份)。
REPO_ROOT="$(cd "$(dirname "$0")/../../../.." && pwd)"
SRC_DIR="$REPO_ROOT/scripts"

install_rule() {
  local src="$1" dst="$2"
  if [ ! -f "$src" ]; then
    echo "  [skip] 找不到 $src"
    return
  fi
  echo "  $src -> $dst"
  sudo cp "$src" "$dst"
}

echo "安裝 udev 規則到 /etc/udev/rules.d/ ..."
install_rule "$SRC_DIR/arduino.rules" /etc/udev/rules.d/99-arduino.rules
install_rule "$SRC_DIR/rplidar.rules" /etc/udev/rules.d/99-rplidar.rules

echo "重新載入 udev 並觸發..."
sudo udevadm control --reload
sudo udevadm trigger --subsystem-match=tty

echo
echo "完成。目前的 symlink:"
ls -l /dev/arduino /dev/rplidar_front /dev/rplidar_rear 2>/dev/null \
  || echo "  (若沒看到,把對應裝置重新插一次再執行 ls -l /dev/arduino /dev/rplidar_*)"
