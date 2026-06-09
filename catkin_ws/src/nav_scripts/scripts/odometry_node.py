#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
任務二：編碼器里程計計算節點

訂閱 /encoders (geometry_msgs/Point)：
    x = 左輪累積 ticks (int)
    y = 右輪累積 ticks (int)

利用差分計算左右輪距離 -> 中心點平移與旋轉 ->
積分得到全域座標 (x, y, theta)。

發布：
    /odometry (nav_msgs/Odometry)
    TF: odom -> base_link

待測參數（先空著，請在 launch 內填入）：
    ~wheel_radius   : 輪子半徑（公尺）
    ~wheel_base     : 左右輪距（公尺）
    ~ticks_per_rev  : 輪子轉一圈的 ticks 數
"""

import math

import rospy
import tf2_ros
from geometry_msgs.msg import Point, Quaternion, TransformStamped
from nav_msgs.msg import Odometry


def yaw_to_quaternion(yaw):
    """只考慮繞 z 軸的 yaw -> 四元數。"""
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


class OdometryNode(object):
    def __init__(self):
        # ---- 參數（待測，先空著由 launch 帶入）----
        self.wheel_radius = rospy.get_param('~wheel_radius', None)
        self.wheel_base = rospy.get_param('~wheel_base', None)
        self.ticks_per_rev = rospy.get_param('~ticks_per_rev', None)

        if (self.wheel_radius is None
                or self.wheel_base is None
                or self.ticks_per_rev is None):
            rospy.logfatal(
                "[odometry] 缺少必要參數: wheel_radius / wheel_base / ticks_per_rev"
            )
            raise rospy.ROSInitException("missing odometry params")

        # frame_id 設定
        self.odom_frame = rospy.get_param('~odom_frame', 'odom')
        self.base_frame = rospy.get_param('~base_frame', 'base_link')

        # 是否要廣播 TF（除錯時可關掉，避免與其他 odom 衝突）
        self.publish_tf = bool(rospy.get_param('~publish_tf', True))

        # 一個 tick 對應的線位移（公尺）
        self.meters_per_tick = (2.0 * math.pi * self.wheel_radius) / float(self.ticks_per_rev)

        # ---- 狀態 ----
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # 上一次的 ticks 與時間；第一筆只記錄不積分
        self.prev_left_ticks = None
        self.prev_right_ticks = None
        self.prev_time = None

        # ---- ROS I/O ----
        self.pub = rospy.Publisher('/odometry', Odometry, queue_size=10)
        self.tf_br = tf2_ros.TransformBroadcaster() if self.publish_tf else None
        self.sub = rospy.Subscriber('/encoders', Point, self.encoders_cb, queue_size=20)

        rospy.loginfo(
            "[odometry] wheel_radius=%.4f m, wheel_base=%.4f m, ticks_per_rev=%.1f, m/tick=%.6f",
            self.wheel_radius, self.wheel_base, self.ticks_per_rev, self.meters_per_tick,
        )

    # ------------------------------------------------------------------
    def encoders_cb(self, msg):
        """處理單筆編碼器訊息並更新里程計。"""
        now = rospy.Time.now()
        # 來源宣告為 int，但 Point 是 float64，這裡用 int() 強制截斷以保險
        left_ticks = int(msg.x)
        right_ticks = int(msg.y)

        # 第一筆：只記錄基準
        if self.prev_left_ticks is None:
            self.prev_left_ticks = left_ticks
            self.prev_right_ticks = right_ticks
            self.prev_time = now
            return

        # ticks 差分 -> 各輪位移
        d_left_ticks = left_ticks - self.prev_left_ticks
        d_right_ticks = right_ticks - self.prev_right_ticks
        d_left = d_left_ticks * self.meters_per_tick
        d_right = d_right_ticks * self.meters_per_tick

        # 差動驅動運動學
        d_center = (d_left + d_right) * 0.5
        d_theta = (d_right - d_left) / self.wheel_base

        # 用中點法積分（在小角度時等效於直接用 theta + d_theta/2）
        mid_theta = self.theta + d_theta * 0.5
        self.x += d_center * math.cos(mid_theta)
        self.y += d_center * math.sin(mid_theta)
        self.theta += d_theta
        # 把 theta 收斂到 (-pi, pi]，避免越積越大
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

        # 速度估計（用本次時間差）
        dt = (now - self.prev_time).to_sec()
        if dt > 1e-6:
            v = d_center / dt
            w = d_theta / dt
        else:
            v = 0.0
            w = 0.0

        # 更新上一筆
        self.prev_left_ticks = left_ticks
        self.prev_right_ticks = right_ticks
        self.prev_time = now

        # 組 Odometry 訊息
        quat = yaw_to_quaternion(self.theta)
        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = quat
        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = w
        self.pub.publish(odom)

        # 廣播 TF odom -> base_link
        if self.tf_br is not None:
            tf_msg = TransformStamped()
            tf_msg.header.stamp = now
            tf_msg.header.frame_id = self.odom_frame
            tf_msg.child_frame_id = self.base_frame
            tf_msg.transform.translation.x = self.x
            tf_msg.transform.translation.y = self.y
            tf_msg.transform.translation.z = 0.0
            tf_msg.transform.rotation = quat
            self.tf_br.sendTransform(tf_msg)


def main():
    rospy.init_node('odometry_node')
    OdometryNode()
    rospy.spin()


if __name__ == '__main__':
    main()
