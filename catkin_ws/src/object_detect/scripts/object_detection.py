#!/usr/bin/env python3
import cv2
import numpy as np
import pyrealsense2 as rs
from ultralytics import YOLO
import rospy
import os
import rospkg

# =============================================================================
# RealSenseCamera
# =============================================================================
class RealSenseCamera:
    def __init__(self):
        self.pipeline = rs.pipeline()
        self.started = False  # flag 紀錄 pipeline 是否成功啟動
        try:
            config = rs.config()
            config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
            config.enable_stream(rs.stream.color, 1920, 1080, rs.format.bgr8, 30)
            self.pipeline.start(config)
            self.align = rs.align(rs.stream.color)
            self.started = True
            print("RealSense Camera Initialized.")
        except Exception as e:
            rospy.logerr(f"❌ Failed to start RealSense pipeline: {e}")
            self.pipeline = None

    def get_aligned_images(self, retries=5):
        if not self.started:
            return None, None, None
        for i in range(retries):
            try:
                frames = self.pipeline.wait_for_frames(timeout_ms=5000)
                aligned_frames = self.align.process(frames)
                depth_frame = aligned_frames.get_depth_frame()
                color_frame = aligned_frames.get_color_frame()
                if depth_frame and color_frame:
                    depth_intrin = depth_frame.profile.as_video_stream_profile().get_intrinsics()
                    img_color = np.asanyarray(color_frame.get_data())
                    return depth_intrin, img_color, depth_frame
            except RuntimeError:
                rospy.logwarn(f"⚠️ Frame didn't arrive (retry {i+1}/{retries})")
        return None, None, None

    def stop(self):
        if self.started and self.pipeline is not None:
            try:
                self.pipeline.stop()
                print("RealSense Camera Stopped.")
            except RuntimeError:
                rospy.logwarn("⚠️ Tried to stop pipeline, but it was not running.")
        else:
            print("ℹ️ Camera was never started, skip stopping.")


# =============================================================================
# ObjectDetector
# =============================================================================
class ObjectDetector:
    def __init__(self, model_path, conf_thresh=0.6):
        self.model = YOLO(model_path, task='detect')
        self.conf_thresh = conf_thresh
        print(f"Object Detector Initialized with model: {model_path}")

    def get_median_depth(self, depth_frame, u, v, sample_size=5):
        half_size = sample_size // 2
        depth_values = []
        height, width = depth_frame.get_height(), depth_frame.get_width()
        for du in range(-half_size, half_size + 1):
            for dv in range(-half_size, half_size + 1):
                x, y = int(u) + du, int(v) + dv
                if 0 <= x < width and 0 <= y < height:
                    depth = depth_frame.get_distance(x, y)
                    if depth > 0:
                        depth_values.append(depth)
        return np.median(depth_values) if depth_values else None

    def estimate_object_radius(self, box_xyxy, surface_depth, depth_intrin):
        box_width_px = box_xyxy[2] - box_xyxy[0]
        box_height_px = box_xyxy[3] - box_xyxy[1]
        diameter_px = (box_width_px + box_height_px) / 2
        radius_px = diameter_px / 2
        fx = depth_intrin.fx
        return (radius_px * surface_depth) / fx

    def detect(self, img_color, aligned_depth_frame, depth_intrin):
        results = self.model.predict([img_color], conf=self.conf_thresh, verbose=False)
        detections = []
        im_out = img_color.copy()

        for result in results:
            if result.boxes is None:
                continue
            boxes_xyxy = result.boxes.xyxy.cpu().numpy()
            confs = result.boxes.conf.cpu().numpy()
            classes = result.boxes.cls.cpu().numpy().astype(int)
            names = result.names

            for i in range(len(boxes_xyxy)):
                box = boxes_xyxy[i]
                ux = (box[0] + box[2]) / 2
                uy = (box[1] + box[3]) / 2

                surface_dis = self.get_median_depth(aligned_depth_frame, ux, uy)
                if surface_dis is None:
                    continue
                object_radius = self.estimate_object_radius(box, surface_dis, depth_intrin)
                center_dis = surface_dis + object_radius
                center_camera_xyz = rs.rs2_deproject_pixel_to_point(depth_intrin, (ux, uy), center_dis)
                world_x = round(center_camera_xyz[0],2)
                world_y = round(center_camera_xyz[2],2)
                world_z = round(-center_camera_xyz[1],2)

                class_id = int(classes[i])
                class_name = names.get(class_id, str(class_id))
                conf = float(confs[i])

                detections.append({
                    'class_id': class_id,
                    'class_name': class_name,
                    'conf': round(conf, 3),
                    'xyz': (world_x, world_y, world_z)
                })

                # 繪製框和文字
                x1, y1, x2, y2 = box
                cv2.rectangle(im_out, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
                cv2.circle(im_out, (int(ux), int(uy)), 4, (0, 0, 255), -1)
                label = f"{class_name} ({conf:.2f})"
                label_xyz = f"XYZ: {world_x},{world_y},{world_z}"
                (w, h), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 2)
                cv2.rectangle(im_out, (int(x1), int(y1)-20), (int(x1)+w, int(y1)), (0,255,0), -1)
                cv2.putText(im_out, label, (int(x1), int(y1)-5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,0),1)
                cv2.putText(im_out, label_xyz, (int(x1), int(y2)+15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255),1)

        return im_out, detections

# =============================================================================
# App
# =============================================================================
class App:
    def __init__(self, model_path):
        self.camera = RealSenseCamera()
        self.detector = ObjectDetector(model_path)

    def run(self):
        try:
            while not rospy.is_shutdown():
                depth_intrin, img_color, aligned_depth_frame = self.camera.get_aligned_images()
                if depth_intrin is None:
                    continue
                im_out, detections = self.detector.detect(img_color, aligned_depth_frame, depth_intrin)
                cv2.imshow('Detection', im_out)

                if detections:
                    print("--- Detected Objects ---")
                    for det in detections:
                        print(f"[{det['class_name']}] conf={det['conf']} XYZ={det['xyz']}")
                key = cv2.waitKey(1)
                if key & 0xFF == ord('q') or key == 27:
                    print("Exiting...")
                    break
        except KeyboardInterrupt:
            print("Interrupted by user. Exiting...")
        finally:
            self.shutdown()

    def shutdown(self):
        self.camera.stop()
        cv2.destroyAllWindows()
        print("Shutdown complete.")

# =============================================================================
# Main
# =============================================================================
if __name__ == '__main__':
    try:
        rospy.init_node('object_detection_node', anonymous=True)
        rospack = rospkg.RosPack()
        package_path = rospack.get_path('object_detect')
        model_path = os.path.join(package_path, 'scripts', 'coffee_merge.pt')
        app = App(model_path)
        rospy.on_shutdown(app.shutdown)
        app.run()
    except rospy.ROSInterruptException:
        print("ROS node interrupted.")
    except Exception as e:
        rospy.logerr(f"Failed to start object detection: {e}")
