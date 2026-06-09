#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
任務四：純里程計盲走腳本

訂閱：/odometry  (nav_msgs/Odometry)
發布：/cmd_vel   (geometry_msgs/Twist)

設計目標：以「路徑點陣列」描述動作序列，方便未來新增。
範例任務：
    前進 0.50 m -> 右轉 90° -> 前進 0.30 m -> 停車

每個動作 (Step) 是一個 dict：
    {'type': 'forward',  'distance': 0.5}    # 前進指定公尺
    {'type': 'backward', 'distance': 0.3}    # 後退指定公尺
    {'type': 'turn',     'angle_deg': -90}   # 右轉 -90，左轉 +90
    {'type': 'wait',     'duration': 1.0}    # 原地停 1 秒
    {'type': 'stop'}                         # 結束任務

如要擴充，只需在 DEFAULT_PLAN 加新項目即可。
"""

import math

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


# ---- 預設路徑（依任務描述）----
DEFAULT_PLAN = [
    {'type': 'forward', 'distance': 0.30},
    {'type': 'wait',    'duration': 0.3},      # 等慣性穩定，避免量測誤差累積
    {'type': 'turn',    'angle_deg': -90.0},   # 右轉
    {'type': 'wait',    'duration': 0.3},
    {'type': 'forward', 'distance': 0.30},
    {'type': 'wait',    'duration': 0.3},
    {'type': 'turn',    'angle_deg': -90.0},
    {'type': 'wait',    'duration': 0.3},
    {'type': 'forward', 'distance': 0.30},
    {'type': 'wait',    'duration': 0.3},
    {'type': 'turn',    'angle_deg': -90.0},
    {'type': 'wait',    'duration': 0.3},
    {'type': 'forward', 'distance': 0.30},
    {'type': 'wait',    'duration': 0.3},
    {'type': 'turn',    'angle_deg': -90.0},
    {'type': 'wait',    'duration': 0.3},
    {'type': 'stop'},
]


def quat_to_yaw(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def angle_diff(a, b):
    d = a - b
    return math.atan2(math.sin(d), math.cos(d))


class PureOdomNavNode(object):
    def __init__(self):
        self.v_linear = float(rospy.get_param('~v_linear', 0.30))
        self.v_angular = float(rospy.get_param('~v_angular', 1.0))

        # 容差
        self.dist_tol = float(rospy.get_param('~dist_tol', 0.02))             # 公尺
        self.turn_tol = math.radians(float(rospy.get_param('~turn_tol_deg', 2.0)))

        self.rate_hz = float(rospy.get_param('~rate_hz', 20.0))

        # 路徑：先用預設，使用者可改 DEFAULT_PLAN 或之後改成讀 yaml
        self.plan = DEFAULT_PLAN

        # ---- 狀態 ----
        self.have_odom = False
        self.cur_x = 0.0
        self.cur_y = 0.0
        self.cur_yaw = 0.0
        self.prev_x = 0.0
        self.prev_y = 0.0
        self.prev_yaw = 0.0

        # 累積位移 / 累積角度（在執行一個 step 期間有效）
        self.dist_accum = 0.0
        self.yaw_accum = 0.0

        # ---- ROS I/O ----
        self.pub_cmd = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/odometry', Odometry, self.odom_cb, queue_size=10)

        rospy.on_shutdown(self.on_shutdown)

    # ------------------------------------------------------------------
    def odom_cb(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        yaw = quat_to_yaw(msg.pose.pose.orientation)

        if self.have_odom:
            # 用差量累積，避免直接相減 yaw 造成 wrap-around 錯誤
            dx = x - self.prev_x
            dy = y - self.prev_y
            self.dist_accum += math.hypot(dx, dy)
            self.yaw_accum += angle_diff(yaw, self.prev_yaw)

        self.prev_x, self.prev_y, self.prev_yaw = x, y, yaw
        self.cur_x, self.cur_y, self.cur_yaw = x, y, yaw
        self.have_odom = True

    # ------------------------------------------------------------------
    def _reset_accum(self):
        self.dist_accum = 0.0
        self.yaw_accum = 0.0

    def _make_twist(self, vx=0.0, wz=0.0):
        t = Twist()
        t.linear.x = vx
        t.angular.z = wz
        return t

    def _publish(self, twist):
        self.pub_cmd.publish(twist)

    # ---- 單一動作執行 ----
    def do_forward(self, distance):
        rospy.loginfo("[pure_odom] forward %.3f m", distance)
        self._reset_accum()
        target = abs(distance)
        sign = 1.0 if distance >= 0 else -1.0
        rate = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown():
            if self.dist_accum >= (target - self.dist_tol):
                break
            self._publish(self._make_twist(vx=sign * self.v_linear))
            rate.sleep()
        self._publish(self._make_twist())  # 動作結束發 0

    def do_backward(self, distance):
        # backward 就是 forward 反向，方便閱讀獨立列出
        self.do_forward(-abs(distance))

    def do_turn(self, angle_deg):
        rospy.loginfo("[pure_odom] turn %.1f deg", angle_deg)
        self._reset_accum()
        target = math.radians(abs(angle_deg))
        sign = 1.0 if angle_deg >= 0 else -1.0  # 左轉 = +，右轉 = -
        rate = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown():
            if abs(self.yaw_accum) >= (target - self.turn_tol):
                break
            self._publish(self._make_twist(wz=sign * self.v_angular))
            rate.sleep()
        self._publish(self._make_twist())

    def do_wait(self, duration):
        rospy.loginfo("[pure_odom] wait %.2f s", duration)
        self._publish(self._make_twist())
        rospy.sleep(duration)

    # ------------------------------------------------------------------
    def run(self):
        # 等到第一筆 odom 進來才開始
        rospy.loginfo("[pure_odom] 等待 /odometry ...")
        wait_rate = rospy.Rate(20.0)
        while not rospy.is_shutdown() and not self.have_odom:
            wait_rate.sleep()
        if rospy.is_shutdown():
            return
        rospy.loginfo("[pure_odom] /odometry 已收到，開始執行 %d 個步驟", len(self.plan))

        for i, step in enumerate(self.plan):
            if rospy.is_shutdown():
                break
            t = step.get('type', 'stop')
            rospy.loginfo("[pure_odom] step %d/%d: %s", i + 1, len(self.plan), step)

            if t == 'forward':
                self.do_forward(step.get('distance', 0.0))
            elif t == 'backward':
                self.do_backward(step.get('distance', 0.0))
            elif t == 'turn':
                self.do_turn(step.get('angle_deg', 0.0))
            elif t == 'wait':
                self.do_wait(step.get('duration', 0.0))
            elif t == 'stop':
                self._publish(self._make_twist())
                rospy.loginfo("[pure_odom] 任務完成，停車。")
                break
            else:
                rospy.logwarn("[pure_odom] 未知動作: %s，跳過。", t)

        # 收尾
        self._publish(self._make_twist())

    def on_shutdown(self):
        try:
            stop = self._make_twist()
            for _ in range(3):
                self.pub_cmd.publish(stop)
                rospy.sleep(0.05)
        except Exception:
            pass


def main():
    rospy.init_node('pure_odom_nav_node')
    node = PureOdomNavNode()
    node.run()


if __name__ == '__main__':
    main()
