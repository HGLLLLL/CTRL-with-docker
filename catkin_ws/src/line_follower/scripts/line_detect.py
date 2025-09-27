#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import String
from geometry_msgs.msg import Point
from cv_bridge import CvBridge, CvBridgeError


class LineDetectorHybridThreshold:
    def __init__(self):
        rospy.init_node('line_detect', anonymous=True)
        rospy.loginfo("line_detect node started")


        self.bridge = CvBridge()
        camera_id = rospy.get_param("~camera_id", "/dev/camera")
        self.cap = cv2.VideoCapture(camera_id)
        if not self.cap.isOpened():
            rospy.logerr("cannot access camera %s" % camera_id)
            rospy.signal_shutdown("cannot access camera %s" % camera_id)
            return
        rospy.loginfo("launched camera %s" % camera_id)

        # NEW: 取得可自定義的過濾參數 (將只被 detect_line 使用)
        self.min_line_width = rospy.get_param('~min_line_width', 50)   # 最小線寬 (pixels)
        self.max_line_width = rospy.get_param('~max_line_width', 500)  # 最大線寬 (pixels)
        self.min_line_area = rospy.get_param('~min_line_area', 15333)   # 最小輪廓面積

        self.binary_threshold = rospy.get_param('~binary_threshold', 50)

        self.detection_pub = rospy.Publisher('/line_detect/detection_data', Point, queue_size=1)
        self.intersection_pub = rospy.Publisher('/line_detect/intersection_type', String, queue_size=1)
        self.debug_image_pub = rospy.Publisher('/line_detect/image_processed', Image, queue_size=1)


        rospy.Timer(rospy.Duration(1.0/30.0), self.process_frame)


    def _preprocess_image(self, roi):
        """
        Conbine global Otsu thresholding and adaptive thresholding to enhance line detection.
        """
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)

        _, thresh = cv2.threshold(gray, self.binary_threshold, 255, cv2.THRESH_BINARY_INV)

        return thresh

    def process_frame(self, event):
        ret, cv_image = self.cap.read()
        if not ret:
            rospy.logwarn("cannot read frame from camera")
            return


        display_image = cv_image.copy()
        height, width, _ = display_image.shape
        
        # MODIFIED: 定義 ROI 的起始與結束位置
        roi_start_row = height * 1 // 4
        roi_end_row = height * 8 // 9  # 排除最底部的 1/9

        # 確保起始點在結束點之前
        if roi_start_row >= roi_end_row:
            rospy.logwarn_throttle(5, "ROI definition invalid. Start row is after end row. Skipping frame.")
            return

        bottom_roi = display_image[roi_start_row:roi_end_row, :]
        top_roi = display_image[:roi_start_row, :]


        # 呼叫新的偵測函數
        pixel_deviation, angle_deviation, main_line_center_x = self.detect_line(bottom_roi)
        intersection_type = self.detect_intersection(top_roi, main_line_center_x)


        detection_data = Point(x=pixel_deviation, y=angle_deviation, z=0)
        self.detection_pub.publish(detection_data)
        self.intersection_pub.publish(String(data=intersection_type))


        if main_line_center_x is not None:
            # 畫線時需加上 ROI 的起始高度偏移，並在新的結束點停止
            cv2.line(display_image, (main_line_center_x, roi_start_row), (main_line_center_x, roi_end_row), (0, 0, 255), 1)


        info_text_line = "PixelDev: {:.1f}, AngleDev: {:.1f}".format(pixel_deviation, angle_deviation)
        info_text_fork = "Intersection: {}".format(intersection_type)
        cv2.putText(display_image, info_text_line, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        cv2.putText(display_image, info_text_fork, (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
        
        # 畫出 ROI 的上下邊界，方便除錯
        cv2.line(display_image, (0, roi_start_row), (width, roi_start_row), (255, 255, 0), 2) # 上邊界 (黃色)
        cv2.line(display_image, (0, roi_end_row), (width, roi_end_row), (255, 0, 255), 2)   # 下邊界 (洋紅色)


        cv2.imshow("Processed Frame", display_image)
        cv2.waitKey(1)


        try:
            self.debug_image_pub.publish(self.bridge.cv2_to_imgmsg(display_image, "bgr8"))
        except CvBridgeError as e:
            rospy.logerr(e)


    def detect_line(self, roi):
        """
        NEW & IMPROVED
        偵測線條，並根據面積和寬度過濾輪廓
        """
        thresh = self._preprocess_image(roi)
        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if not contours:
            return 0.0, 0.0, None

        valid_contours = []
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < self.min_line_area:
                continue
            rect = cv2.minAreaRect(cnt)
            w, h = rect[1]
            line_width = min(w, h)
            if not (self.min_line_width < line_width < self.max_line_width):
                continue
            valid_contours.append(cnt)

        if not valid_contours:
            return 0.0, 0.0, None

        largest_contour = max(valid_contours, key=cv2.contourArea)
        M = cv2.moments(largest_contour)
        pixel_deviation, angle_deviation, line_center_x = 0.0, 0.0, None
        if M["m00"] > 0:
            cx = int(M["m10"] / M["m00"])
            pixel_deviation = cx - (roi.shape[1] // 2)
            line_center_x = cx
            
            rect = cv2.minAreaRect(largest_contour)
            rect_width, rect_height = rect[1]
            angle = rect[2]
            
            if rect_width < rect_height:
                angle_deviation = angle + 90
            else:
                angle_deviation = angle
                
            cv2.drawContours(roi, [largest_contour], -1, (0, 255, 0), 2)

        return pixel_deviation, angle_deviation, line_center_x


    def detect_intersection(self, roi, main_line_center_x):
        if main_line_center_x is None:
            return "STRAIGHT"
        thresh = self._preprocess_image(roi)
        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return "STRAIGHT"
        largest_contour = max(contours, key=cv2.contourArea)
        if cv2.contourArea(largest_contour) < 45000:
             return "STRAIGHT"
        x, y, w, h = cv2.boundingRect(largest_contour)
        cv2.drawContours(roi, [largest_contour], -1, (255, 0, 0), 2)
        center_line_ref = main_line_center_x
        margin = 30
        is_left = (x + w - 200) < (center_line_ref - margin)
        is_right = x + 200 > (center_line_ref + margin)
        if not is_left and not is_right:
            return "T_JUNCTION"
        elif is_left and not is_right:
            return "LEFT_FORK"
        elif not is_left and is_right:
            return "RIGHT_FORK"
        else:
            return "STRAIGHT"


    def on_shutdown(self):
        rospy.loginfo("shutting 'line_detector_node'...")
        self.cap.release()
        rospy.loginfo("camera released")
        cv2.destroyAllWindows()
        rospy.loginfo("OpenCV windows closed")


if __name__ == '__main__':
    try:
        ld_node = LineDetectorHybridThreshold()
        rospy.on_shutdown(ld_node.on_shutdown)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass