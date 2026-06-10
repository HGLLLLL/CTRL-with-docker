#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
import math
from dataclasses import dataclass
from typing import Optional, Tuple
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

# ---------------------------------------------------------
# 請確保您已在 ROS package 中建立名為 TurnDetect.msg 的自定義訊息，
# 內容包含:
#   string turn_direction
#   float32 pixel_size
# 並將 `your_ros_package_name` 替換為您的實際 package 名稱。
# ---------------------------------------------------------
try:
    from lane_follower.msg import TurnDetect
except ImportError:
    rospy.logwarn("尚未匯入 TurnDetect 訊息，請確認您的 package 名稱與 msg 設定。以下使用類似結構替代以防語法錯誤。")
    class TurnDetect:
        turn_direction = ""
        pixel_size = 0.0
        offset = 0.0

# ==========================================
# Configuration / Settings (從原 config.py 整合)
# ==========================================
BLUR_KERNEL_SIZE = (5, 5)
# 'fixed'    : 用 THRESHOLD_VALUE 當固定門檻，最好調，灰色不會被當黑色 (推薦)
# 'otsu'     : 自動找門檻，但當畫面同時有灰跟黑時，灰會被併進黑色那一群
# 'adaptive' : 區域自適應，光照不均時用
THRESHOLD_METHOD = 'fixed'
# 固定門檻值 (0~255)：值越「小」代表越嚴格，只有「更黑」的才會被當成黑色。
# 灰色被誤判成黑色 → 把這個值「調小」(例如 80 → 60 → 50)。
# 黑色三角形抓不到 / 太斷裂 → 把這個值「調大」(例如 60 → 80 → 100)。
THRESHOLD_VALUE = 60
INVERT_BINARY = True

MIN_AREA = 1000.0  
ASPECT_RATIO_MIN = 0.5
ASPECT_RATIO_MAX = 2.0
AREA_RATIO_MIN = 0.4
POLY_EPSILON_RATIO = 0.02

CONFIRM_FRAMES = 10
LOST_FRAMES = 5
SAME_OBJECT_DIST_RATIO = 0.2
SHOW_WINDOW = True

# ==========================================
# Data Structures
# ==========================================
@dataclass
class Triangle:
    contour: np.ndarray
    area: float
    bbox: Tuple[int, int, int, int] # x, y, w, h
    apex: Tuple[int, int]
    direction: str # "left" or "right"
    center: Tuple[int, int]

class State:
    DETECTED = "DETECTED"
    NOT_DETECTED = "NOT_DETECTED"

# ==========================================
# Algorithm Classes and Functions
# ==========================================
class Tracker:
    def __init__(self):
        self.state = State.NOT_DETECTED
        self.consecutive_detects = 0
        self.consecutive_lost = 0
        self.last_triangle: Optional[Triangle] = None

    def update(self, triangle: Optional[Triangle]) -> str:
        if triangle is not None:
            is_same = False
            if self.last_triangle:
                lx, ly = self.last_triangle.center
                cx, cy = triangle.center
                dist = math.hypot(cx - lx, cy - ly)
                max_dist = SAME_OBJECT_DIST_RATIO * self.last_triangle.bbox[2]
                if dist < max_dist and triangle.direction == self.last_triangle.direction:
                    is_same = True
            else:
                is_same = True # 第一次偵測

            if not is_same:
                # 若不是同一物件（位置突變或方向改變），重置計數
                self.consecutive_detects = 0

            self.consecutive_detects += 1
            self.consecutive_lost = 0
            self.last_triangle = triangle

            if self.consecutive_detects >= CONFIRM_FRAMES:
                self.state = State.DETECTED
        else:
            self.consecutive_lost += 1
            if self.consecutive_lost >= LOST_FRAMES:
                self.state = State.NOT_DETECTED
                self.consecutive_detects = 0
                self.last_triangle = None
        
        return self.state

def preprocess(frame: np.ndarray,
               method: str = THRESHOLD_METHOD,
               thresh_value: int = THRESHOLD_VALUE,
               invert: bool = INVERT_BINARY) -> np.ndarray:
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, BLUR_KERNEL_SIZE, 0)

    if method == 'otsu':
        _, binary = cv2.threshold(blur, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    elif method == 'adaptive':
        binary = cv2.adaptiveThreshold(
            blur, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 11, 2
        )
    else:  # 'fixed'
        _, binary = cv2.threshold(blur, thresh_value, 255, cv2.THRESH_BINARY)

    if invert:
        binary = cv2.bitwise_not(binary)

    return binary

def find_triangle(binary: np.ndarray) -> Optional[Triangle]:
    contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    best_triangle = None
    max_area = 0

    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area < MIN_AREA:
            continue
            
        x, y, w, h = cv2.boundingRect(cnt)
        if h == 0: continue
        aspect_ratio = float(w) / h
        if not (ASPECT_RATIO_MIN <= aspect_ratio <= ASPECT_RATIO_MAX):
            continue
            
        bbox_area = w * h
        if (area / bbox_area) < AREA_RATIO_MIN:
            continue

        peri = cv2.arcLength(cnt, True)
        approx = cv2.approxPolyDP(cnt, POLY_EPSILON_RATIO * peri, True)
        
        # 尋找三角形
        if len(approx) == 3:
            if area > max_area:
                max_area = area
                
                # 尋找 apex (與另外兩點連線中點距離最遠的點)
                pts = [pt[0] for pt in approx]
                max_dist = -1
                apex_idx = 0
                
                for i in range(3):
                    pt_test = pts[i]
                    pt_other1 = pts[(i+1)%3]
                    pt_other2 = pts[(i+2)%3]
                    
                    mid_x = (pt_other1[0] + pt_other2[0]) / 2.0
                    mid_y = (pt_other1[1] + pt_other2[1]) / 2.0
                    
                    dist = math.hypot(pt_test[0] - mid_x, pt_test[1] - mid_y)
                    if dist > max_dist:
                        max_dist = dist
                        apex_idx = i
                
                apex = pts[apex_idx]
                center_x = x + w / 2.0
                center_y = y + h / 2.0
                
                # 方向判定：由 apex.x 相對於 bbox 中心 x 決定
                direction = "right" if apex[0] > center_x else "left"
                
                best_triangle = Triangle(
                    contour=cnt,
                    area=area,
                    bbox=(x, y, w, h),
                    apex=tuple(apex),
                    direction=direction,
                    center=(int(center_x), int(center_y))
                )
                
    return best_triangle

def draw_overlay(frame: np.ndarray, triangle: Optional[Triangle], state: str, fps: float) -> np.ndarray:
    display = frame.copy()
    
    cv2.putText(display, f"FPS: {fps:.1f}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)
    
    color = (0, 255, 0) if state == State.DETECTED else (0, 0, 255)
    cv2.putText(display, f"State: {state}", (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 1, color, 2)
    
    if triangle:
        x, y, w, h = triangle.bbox
        cv2.rectangle(display, (x, y), (x+w, y+h), (255, 0, 0), 2)
        cv2.drawContours(display, [triangle.contour], -1, (0, 255, 255), 2)
        
        cv2.circle(display, triangle.apex, 5, (0, 0, 255), -1)
        
        offset = triangle.center[0] - frame.shape[1] / 2.0
        info = f"Area: {triangle.area:.0f} Dir: {triangle.direction}"
        cv2.putText(display, info, (x, y - 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
        cv2.putText(display, f"Offset: {offset:.1f}", (x, y - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
        
    return display

# ==========================================
# ROS Node
# ==========================================
class TurnDetectNode:
    def __init__(self):
        rospy.init_node('turn_detect_node', anonymous=True)
        self.bridge = CvBridge()
        
        # 讀取 ROS 參數配置
        self.image_topic = rospy.get_param('~image_topic', '/usb_cam/image_raw')
        self.debug_topic = rospy.get_param('~debug_topic', 'turn_detect/image_out')
        self.binary_topic = rospy.get_param('~binary_topic', 'turn_detect/binary')
        self.show_window = rospy.get_param('~show_window', SHOW_WINDOW)

        # 二值化參數 (可在 launch 即時調整，不用重 build)
        self.threshold_method = rospy.get_param('~threshold_method', THRESHOLD_METHOD)
        self.threshold_value = int(rospy.get_param('~threshold_value', THRESHOLD_VALUE))
        self.invert_binary = bool(rospy.get_param('~invert_binary', INVERT_BINARY))
        rospy.loginfo(
            f"Threshold: method={self.threshold_method}, "
            f"value={self.threshold_value}, invert={self.invert_binary}"
        )

        # 建立 Publisher 與 Subscriber
        self.pub = rospy.Publisher('turn_detect', TurnDetect, queue_size=10)
        self.debug_pub = rospy.Publisher(self.debug_topic, Image, queue_size=1)
        self.binary_pub = rospy.Publisher(self.binary_topic, Image, queue_size=1)
        self.sub = rospy.Subscriber(self.image_topic, Image, self.image_callback, queue_size=1, buff_size=2**24)
        
        self.tracker = Tracker()
        
        self.tick_freq = cv2.getTickFrequency()
        self.prev_tick = cv2.getTickCount()
        
        rospy.loginfo(f"Turn Detect Node Started.")
        rospy.loginfo(f"Subscribed to topic: {self.image_topic}")

    def image_callback(self, msg):
        try:
            # 將 ROS Image 轉為 OpenCV BGR 格式
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(f"CV Bridge Error: {e}")
            return
            
        curr_tick = cv2.getTickCount()
        fps = self.tick_freq / (curr_tick - self.prev_tick) if (curr_tick - self.prev_tick) > 0 else 0.0
        self.prev_tick = curr_tick
            
        binary = preprocess(frame,
                            method=self.threshold_method,
                            thresh_value=self.threshold_value,
                            invert=self.invert_binary)

        # 發布二值化結果（mono8），有訂閱者才送以省 CPU
        if self.binary_pub.get_num_connections() > 0:
            try:
                bin_msg = self.bridge.cv2_to_imgmsg(binary, "mono8")
                bin_msg.header = msg.header
                self.binary_pub.publish(bin_msg)
            except CvBridgeError as e:
                rospy.logerr(f"CV Bridge Error on binary publish: {e}")

        triangle = find_triangle(binary)
        state = self.tracker.update(triangle)
        
        # 如果狀態為 DETECTED，發布結果至 ROS topic
        if state == State.DETECTED and self.tracker.last_triangle:
            msg_out = TurnDetect()
            msg_out.turn_direction = self.tracker.last_triangle.direction
            msg_out.pixel_size = float(self.tracker.last_triangle.area)
            msg_out.offset = float(self.tracker.last_triangle.center[0] - frame.shape[1] / 2.0)
            self.pub.publish(msg_out)
        
        if self.show_window or self.debug_pub.get_num_connections() > 0:
            display = draw_overlay(frame, triangle, state, fps)
            
            if self.show_window:
                cv2.imshow("ROS Turn Detect", display)
                cv2.waitKey(1)
                
            if self.debug_pub.get_num_connections() > 0:
                try:
                    out_msg = self.bridge.cv2_to_imgmsg(display, "bgr8")
                    out_msg.header = msg.header
                    self.debug_pub.publish(out_msg)
                except CvBridgeError as e:
                    rospy.logerr(f"CV Bridge Error on publish: {e}")

    def run(self):
        rospy.spin()
        if self.show_window:
            cv2.destroyAllWindows()

if __name__ == "__main__":
    try:
        node = TurnDetectNode()
        node.run()
    except rospy.ROSInterruptException:
        pass