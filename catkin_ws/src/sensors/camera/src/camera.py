#!/usr/bin/env python

import rospy
import cv2
from sensor_msgs.msg import Image
from std_msgs.msg import String
import base64
from cv_bridge import CvBridge

class Camera:
  def __init__(self):
    rospy.init_node('camera')
    
    self.camera_id_param = rospy.get_param('~camera_id', '/dev/video0')
    self.camera_name = rospy.get_param('~camera_name', 'camera')
    self.frame_rate = rospy.get_param('~frame_rate', 30)
    self.width = rospy.get_param('~width', 640)
    self.height = rospy.get_param('~height', 480)
    
    # Try to parse the camera_id as an integer backend for OpenCV, to avoid GStreamer errors
    if isinstance(self.camera_id_param, str) and self.camera_id_param.startswith('/dev/video'):
        self.camera_id = int(self.camera_id_param.replace('/dev/video', ''))
    else:
        try:
            self.camera_id = int(self.camera_id_param)
        except ValueError:
            self.camera_id = self.camera_id_param

    # Explicitly using V4L2 backend
    self.cap = cv2.VideoCapture(self.camera_id, cv2.CAP_V4L2)

    # 強制降低解析度並使用 MJPG 壓縮格式節省 USB 頻寬
    self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
    self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
    self.cap.set(cv2.CAP_PROP_FPS, self.frame_rate)

    if self.cap.isOpened():
      rospy.loginfo('Camera connected: %s (OpenCV ID: %s)', self.camera_id_param, self.camera_id)
    else :
      rospy.logwarn('Camera not connected: %s', self.camera_id_param)

    # Standard ROS image publisher
    self.image_pub = rospy.Publisher('/' + self.camera_name + '/image_raw', Image, queue_size=1)
    # Existing web publish
    self.web_pub = rospy.Publisher('/golfbot/' + self.camera_name + '_web', String, queue_size=1)
    
    self.bridge = CvBridge()
    self.rate = rospy.Rate(self.frame_rate)

  def talker(self):
    while not rospy.is_shutdown():
      ret, frame = self.cap.read()
      if not ret : 
        self.rate.sleep()
        continue
      
      # 1. Publish standard ROS Image
      try:
        img_msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
        img_msg.header.stamp = rospy.Time.now()
        self.image_pub.publish(img_msg)
      except Exception as e:
        rospy.logerr("CvBridge Error: %s", e)

      # 2. Encode the image for web
      _, buffer = cv2.imencode('.jpg', frame)
      image_as_str = base64.b64encode(buffer).decode('utf-8')

      # Publish the encoded image
      self.web_pub.publish(image_as_str)

      self.rate.sleep()

    self.cap.release()
    cv2.destroyAllWindows()
    
if __name__ == '__main__':
  camera = Camera()
  try:
    camera.talker()
  except rospy.ROSInterruptException:
    pass