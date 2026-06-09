#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
任務一：光達資料預處理節點

訂閱 /scan (sensor_msgs/LaserScan)，
擷取正前方 ±front_angle_range 度範圍內的點雲，
過濾 0.0 與 inf 後取最短距離，
發布到 /lidar_output (std_msgs/Float32)，單位：公尺。

備註：RPLidar 的 LaserScan 通常 angle_min 接近 -pi、angle_max 接近 +pi，
       0 rad 為車體正前方；但「正前方」這段索引在陣列裡可能會橫跨頭尾，
       因此計算索引時要做 modulo wrap 處理。
"""

import math

import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32


class LidarFilterNode(object):
    def __init__(self):
        # ---- 參數 ----
        # 正前方 ±front_angle_range 度（總共 2X 度的扇形）
        self.front_angle_range_deg = rospy.get_param('~front_angle_range', 15.0)
        self.front_angle_range_rad = math.radians(self.front_angle_range_deg)

        # 最大有效距離（超過視為雜訊，可在 launch 調整）
        self.max_valid_range = rospy.get_param('~max_valid_range', 12.0)

        # 找不到有效點時要回傳的值（預設用 inf 代表「前方無障礙」）
        self.no_obstacle_value = float(
            rospy.get_param('~no_obstacle_value', float('inf'))
        )

        # ---- ROS I/O ----
        self.pub = rospy.Publisher('/lidar_output', Float32, queue_size=10)
        self.sub = rospy.Subscriber('/scan', LaserScan, self.scan_cb, queue_size=1)

        rospy.loginfo(
            "[lidar_filter] front_angle_range=±%.1f deg, max_valid_range=%.2f m",
            self.front_angle_range_deg, self.max_valid_range,
        )

    # ------------------------------------------------------------------
    def scan_cb(self, msg):
        """處理單筆 LaserScan，輸出前方最短距離。"""
        n = len(msg.ranges)
        if n == 0 or msg.angle_increment == 0.0:
            return

        # 把「正前方 0 rad」轉換成 ranges 陣列中的索引（可能是負的，後面 mod 處理）
        center_idx = int(round((0.0 - msg.angle_min) / msg.angle_increment))
        half_span = int(math.ceil(self.front_angle_range_rad / abs(msg.angle_increment)))

        # 收集索引：用 modulo 處理頭尾相接（例如 center_idx 接近 0 或接近 n-1 時）
        min_dist = float('inf')
        for offset in range(-half_span, half_span + 1):
            idx = (center_idx + offset) % n
            r = msg.ranges[idx]

            # 過濾無效值：0.0、NaN、inf、負值、超過最大有效距離
            if r is None:
                continue
            if math.isnan(r) or math.isinf(r):
                continue
            if r <= 0.0:
                continue
            if r > self.max_valid_range:
                continue

            if r < min_dist:
                min_dist = r

        out = Float32()
        out.data = min_dist if math.isfinite(min_dist) else self.no_obstacle_value
        self.pub.publish(out)


def main():
    rospy.init_node('lidar_filter_node')
    LidarFilterNode()
    rospy.spin()


if __name__ == '__main__':
    main()
