#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
任務三：光達 + 里程計複合避障狀態機

訂閱：
    /lidar_output (std_msgs/Float32)     -> 前方最短距離（公尺）
    /odometry      (nav_msgs/Odometry)   -> 取 yaw 判斷轉彎角度
發布：
    /cmd_vel       (geometry_msgs/Twist) -> 控制底盤

流程（半寫死）：
    State 1: 直行，直到 /lidar_output < dist_th_1
    State 2: 右轉 90 度（用 odom 角度變化判斷）
    State 3: 直行，直到 /lidar_output < dist_th_2
    State 4: 左轉 90 度
    State 5: 直行，直到 /lidar_output < dist_th_3
    State 6: 右轉 90 度
    State 7: 直行，直到 /lidar_output < dist_th_4
    State 8: 停車，結束

注意：角度差需處理 wrap-around（例如從 +179 度繞到 -179 度）。
"""

import math

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32


# ---- 狀態列舉 ----
S_FWD_1 = 1     # 直行 -> dist_th_1
S_TURN_R = 2    # 右轉 90 度
S_FWD_2 = 3     # 直行 -> dist_th_2
S_TURN_L = 4    # 左轉 90 度
S_FWD_3 = 5     # 直行 -> dist_th_3
S_STOP = 6      # 停車
S_BRAKE = 7     # 轉彎結束後的剎車過渡（publish 全 0 twist 等底盤慣性衰減）
S_TURN_R2 = 8   # 第二次右轉 90 度
S_FWD_4 = 9     # 直行 -> dist_th_4


def quat_to_yaw(q):
    """從 nav_msgs/Odometry 的 orientation 取出 yaw。"""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def angle_diff(a, b):
    """回傳 a - b，收斂到 (-pi, pi]。"""
    d = a - b
    return math.atan2(math.sin(d), math.cos(d))


class LidarOdomNavNode(object):
    def __init__(self):
        # ---- 速度參數 ----
        self.v_linear = float(rospy.get_param('~v_linear', 0.30))   # m/s
        self.v_angular = float(rospy.get_param('~v_angular', 1.0))  # rad/s

        # ---- 觸發距離（依任務描述為固定值，但仍開放 launch 覆寫）----
        self.dist_th_1 = float(rospy.get_param('~dist_th_1', 0.15))
        self.dist_th_2 = float(rospy.get_param('~dist_th_2', 0.10))
        self.dist_th_3 = float(rospy.get_param('~dist_th_3', 0.20))
        self.dist_th_4 = float(rospy.get_param('~dist_th_4', 0.20))

        # ---- 轉彎容差 ----
        # 旋轉角度目標 = 90 度。實際下指令停轉的角度為:
        #   turn_target - turn_overshoot - turn_tol
        # 其中 turn_overshoot 為「預估底盤從滿速到停下還會多轉的角度」（慣性補償），
        # 由實測校準；turn_tol 是額外安全餘量（寧可少轉一點，剎車階段也能再補測）。
        self.turn_target = math.radians(90.0)
        self.turn_overshoot = math.radians(
            float(rospy.get_param('~turn_overshoot_deg', 10.0))
        )
        self.turn_tol = math.radians(float(rospy.get_param('~turn_tol_deg', 2.0)))

        # ---- 剎車過渡 ----
        # 轉彎達標 -> 進入 S_BRAKE，publish 全 0 twist 持續 brake_duration 秒，
        # 等底盤的角速度真正衰減到 0 後，再切到下一個直行狀態。
        # 期間 turn_accum 仍在更新，因此 log 出來的最終 Δyaw 才是「真正」轉了多少。
        self.brake_duration = float(rospy.get_param('~brake_duration', 0.3))
        self.brake_end_time = None     # rospy.Time，剎車結束時刻
        self.brake_next_state = None   # 剎車完成後要切去的狀態

        # 控制迴圈頻率
        self.rate_hz = float(rospy.get_param('~rate_hz', 20.0))

        # ---- 狀態 ----
        self.state = S_FWD_1
        self.front_dist = float('inf')
        self.have_odom = False
        self.cur_yaw = 0.0
        # 進入轉彎狀態時要先 latch 起始 yaw，計算累積差量
        self.turn_start_yaw = None
        self.turn_accum = 0.0
        self.prev_yaw_for_accum = 0.0

        # ---- ROS I/O ----
        self.pub_cmd = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/lidar_output', Float32, self.lidar_cb, queue_size=10)
        rospy.Subscriber('/odometry', Odometry, self.odom_cb, queue_size=10)

        rospy.on_shutdown(self.on_shutdown)
        rospy.loginfo(
            "[nav_lidar_odom] v=%.2f m/s, w=%.2f rad/s, th=%.2f/%.2f/%.2f/%.2f m, "
            "overshoot=%.1f deg, brake=%.2f s",
            self.v_linear, self.v_angular,
            self.dist_th_1, self.dist_th_2, self.dist_th_3, self.dist_th_4,
            math.degrees(self.turn_overshoot), self.brake_duration,
        )

    # ------------------------------------------------------------------
    def lidar_cb(self, msg):
        self.front_dist = msg.data

    def odom_cb(self, msg):
        yaw = quat_to_yaw(msg.pose.pose.orientation)
        if self.have_odom:
            # 用差量累積，避免 wrap-around 把 90 度誤判成 -270 度
            d = angle_diff(yaw, self.prev_yaw_for_accum)
            self.turn_accum += d
        self.prev_yaw_for_accum = yaw
        self.cur_yaw = yaw
        self.have_odom = True

    # ------------------------------------------------------------------
    def _start_turn(self):
        """進入轉彎狀態時，把累積值歸零。"""
        self.turn_accum = 0.0
        self.turn_start_yaw = self.cur_yaw

    def _enter_brake(self, next_state):
        """轉彎達標 -> 進入剎車過渡。"""
        self.brake_next_state = next_state
        self.brake_end_time = rospy.Time.now() + rospy.Duration(self.brake_duration)
        self.state = S_BRAKE

    def _make_twist(self, vx=0.0, wz=0.0):
        t = Twist()
        t.linear.x = vx
        t.angular.z = wz
        return t

    def run(self):
        rate = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown():
            cmd = self._make_twist()

            if self.state == S_FWD_1:
                cmd = self._make_twist(vx=self.v_linear)
                if self.front_dist < self.dist_th_1:
                    rospy.loginfo("[nav] State1 done (d=%.3f) -> 右轉 90", self.front_dist)
                    self.state = S_TURN_R
                    self._start_turn()

            elif self.state == S_TURN_R:
                # 右轉 = 角速度為負（右手系：z 軸向上、逆時針為正）
                cmd = self._make_twist(wz=-self.v_angular)
                # 提早停轉以補償慣性 overshoot
                if abs(self.turn_accum) >= (
                    self.turn_target - self.turn_overshoot - self.turn_tol
                ):
                    rospy.loginfo("[nav] State2 cmd-stop (Δyaw=%.1f deg) -> brake",
                                  math.degrees(self.turn_accum))
                    self._enter_brake(next_state=S_FWD_2)

            elif self.state == S_FWD_2:
                cmd = self._make_twist(vx=self.v_linear)
                if self.front_dist < self.dist_th_2:
                    rospy.loginfo("[nav] State3 done (d=%.3f) -> 左轉 90", self.front_dist)
                    self.state = S_TURN_L
                    self._start_turn()

            elif self.state == S_TURN_L:
                cmd = self._make_twist(wz=+self.v_angular)
                if abs(self.turn_accum) >= (
                    self.turn_target - self.turn_overshoot - self.turn_tol
                ):
                    rospy.loginfo("[nav] State4 cmd-stop (Δyaw=%.1f deg) -> brake",
                                  math.degrees(self.turn_accum))
                    self._enter_brake(next_state=S_FWD_3)

            elif self.state == S_BRAKE:
                # 持續發 0 速度讓底盤慣性衰減；odom 仍會更新 turn_accum
                cmd = self._make_twist()
                if rospy.Time.now() >= self.brake_end_time:
                    rospy.loginfo("[nav] brake done (final Δyaw=%.1f deg) -> 直行",
                                  math.degrees(self.turn_accum))
                    self.state = self.brake_next_state

            elif self.state == S_FWD_3:
                cmd = self._make_twist(vx=self.v_linear)
                if self.front_dist < self.dist_th_3:
                    rospy.loginfo("[nav] State5 done (d=%.3f) -> 右轉 90 (#2)",
                                  self.front_dist)
                    self.state = S_TURN_R2
                    self._start_turn()

            elif self.state == S_TURN_R2:
                cmd = self._make_twist(wz=-self.v_angular)
                if abs(self.turn_accum) >= (
                    self.turn_target - self.turn_overshoot - self.turn_tol
                ):
                    rospy.loginfo("[nav] State6 cmd-stop (Δyaw=%.1f deg) -> brake",
                                  math.degrees(self.turn_accum))
                    self._enter_brake(next_state=S_FWD_4)

            elif self.state == S_FWD_4:
                cmd = self._make_twist(vx=self.v_linear)
                if self.front_dist < self.dist_th_4:
                    rospy.loginfo("[nav] State7 done (d=%.3f) -> 停車", self.front_dist)
                    self.state = S_STOP

            elif self.state == S_STOP:
                cmd = self._make_twist()
                self.pub_cmd.publish(cmd)
                rospy.loginfo("[nav] State8: 任務完成，停車。")
                break

            self.pub_cmd.publish(cmd)
            rate.sleep()

    # ------------------------------------------------------------------
    def on_shutdown(self):
        # 關機/Ctrl+C 時補送幾個 0 速度，確保底盤不會繼續跑
        try:
            stop = self._make_twist()
            for _ in range(3):
                self.pub_cmd.publish(stop)
                rospy.sleep(0.05)
        except Exception:
            pass


def main():
    rospy.init_node('lidar_odom_nav_node')
    node = LidarOdomNavNode()
    node.run()


if __name__ == '__main__':
    main()
