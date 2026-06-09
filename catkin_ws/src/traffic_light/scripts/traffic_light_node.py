#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
traffic_light_node — 讀取 Grove Vision AI V2 (USB-C / serial) 的偵測結果。

模組自帶鏡頭與晶片做推論,本節點只負責:
  1. 開啟序列埠讀資料
  2. 把讀到的「原始字串」與「解析後的數值」印到畫面 (loginfo)
  3. 順便發布 /traffic_light (std_msgs/String)

第一次使用先看 loginfo 印出的原始 JSON,確認欄位後再決定怎麼對應 label。

用法:
  rosrun traffic_light traffic_light_node.py
  rosrun traffic_light traffic_light_node.py _port:=/dev/ttyACM0 _baud:=921600
"""

import json
import rospy
from std_msgs.msg import String

try:
    import serial
except ImportError:
    serial = None


class TrafficLightNode:
    def __init__(self):
        rospy.init_node('traffic_light')

        self.port = rospy.get_param('~port', '/dev/ttyACM0')
        self.baud = rospy.get_param('~baud', 921600)      # SSCMA 韌體常見預設
        self.print_raw = rospy.get_param('~print_raw', True)
        self.min_score = rospy.get_param('~min_score', 0)  # 先設 0,所有偵測都印出來
        # 模組開埠後會 reboot 並停在待命,不會自己推論。開啟後主動送 AT+INVOKE
        # 讓它進入連續推論,topic 才會有資料。
        self.auto_invoke = rospy.get_param('~auto_invoke', True)
        self.boot_wait = rospy.get_param('~boot_wait', 2.0)  # 等模組開機完成再下指令(秒)
        # label 對應表:box 裡的 target_id → 名稱。先以你訓練模型的類別順序為準。
        self.labels = rospy.get_param('~labels', ['red', 'green', 'yellow'])

        self.pub = rospy.Publisher('/traffic_light', String, queue_size=1)

        if serial is None:
            rospy.logfatal("找不到 pyserial,請先安裝:  pip3 install pyserial")
            raise SystemExit(1)

    def open_serial(self):
        """開啟序列埠,失敗就重試,直到成功或節點關閉。"""
        while not rospy.is_shutdown():
            try:
                ser = serial.Serial(self.port, self.baud, timeout=1)
                rospy.loginfo("已連上 Grove Vision AI V2: %s @ %d", self.port, self.baud)
                self.start_inference(ser)
                return ser
            except serial.SerialException as e:
                rospy.logwarn_throttle(5.0, "開啟 %s 失敗 (%s),5 秒後重試...", self.port, e)
                rospy.sleep(1.0)
        return None

    def start_inference(self, ser):
        """開埠後模組會 reboot 並待命;送 AT+INVOKE 讓它連續推論。
        參數 -1,0,1:N=-1 無限次、不過濾、第三個 1 = 不回傳影像
        (實機驗證可把每幀回傳從 ~3KB 縮到 ~0.4KB,大幅降低序列埠負載)。"""
        if not self.auto_invoke:
            return
        rospy.sleep(self.boot_wait)        # 等 bootloader 跑完,否則指令會被忽略
        ser.write(b'AT+INVOKE=-1,0,1\r\n')
        rospy.loginfo("已送出 AT+INVOKE=-1,0,1,模組開始連續推論(不含影像)")

    def spin(self):
        ser = self.open_serial()
        buf = b''
        while not rospy.is_shutdown():
            try:
                chunk = ser.read(ser.in_waiting or 1)
            except serial.SerialException as e:
                rospy.logwarn("序列埠中斷 (%s),嘗試重新連線...", e)
                ser = self.open_serial()
                buf = b''
                continue

            if not chunk:
                continue
            buf += chunk
            while b'\n' in buf:
                line, buf = buf.split(b'\n', 1)
                self.handle_line(line)

    def handle_line(self, raw):
        text = raw.decode('utf-8', 'ignore').strip()
        if not text:
            return
        if self.print_raw:
            rospy.loginfo("RAW: %s", text)

        # 嘗試解析 JSON;模組偶爾會夾雜非 JSON 的 AT 回應,解析失敗就略過。
        try:
            msg = json.loads(text)
        except ValueError:
            return

        # SSCMA 偵測結果通常在 data.boxes,格式約為 [x, y, w, h, score, target_id]。
        # 不同韌體欄位可能不同,所以這裡寫得寬鬆一點。
        data = msg.get('data', {}) if isinstance(msg, dict) else {}
        boxes = data.get('boxes', []) if isinstance(data, dict) else []

        best = None
        for b in boxes:
            try:
                x, y, w, h, score, target = b[0], b[1], b[2], b[3], b[4], int(b[5])
            except (IndexError, ValueError, TypeError):
                continue
            label = self.labels[target] if 0 <= target < len(self.labels) else str(target)
            rospy.loginfo("  偵測: %-7s score=%-3s box=(%s,%s,%s,%s)",
                          label, score, x, y, w, h)
            if score >= self.min_score and (best is None or score > best[0]):
                best = (score, label)

        state = best[1] if best else 'none'
        self.pub.publish(state)


if __name__ == '__main__':
    try:
        TrafficLightNode().spin()
    except rospy.ROSInterruptException:
        pass
