#!/usr/bin/env python3
import numpy as np
import pyrealsense2 as rs
from ultralytics import YOLO
import rospy
import rospkg, os, traceback
from object_detect.srv import DetectCoffeeSupply, DetectCoffeeSupplyResponse
from std_srvs.srv import Trigger, TriggerResponse # Import the Trigger service type

# =============================================================================
# Class: RealSenseCamera
# =============================================================================
class RealSenseCamera:
    def __init__(self):
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
        self.started = False
        rospy.loginfo("RealSenseCamera object created, but pipeline not started.")

    def start(self):
        if self.started:
            rospy.logwarn("[Camera] Pipeline is already running.")
            return True
        try:
            self.pipeline.start(self.config)
            self.started = True
            rospy.loginfo("RealSense Camera Pipeline Started.")
            return True
        except Exception as e:
            rospy.logerr(f"[Camera] Failed to start RealSense pipeline: {e}")
            self.started = False
            return False

    def stop(self):
        if not self.started:
            return # Not an error to stop an already stopped camera
        try:
            self.pipeline.stop()
            rospy.loginfo("RealSense Camera Pipeline Stopped.")
        except Exception as e:
            rospy.logwarn(f"[Camera] Stop pipeline encountered a warning: {e}")
        finally:
            self.started = False

    def get_color_image(self):
        if not self.started:
            rospy.logwarn("[Camera] get_color_image called but pipeline is not started.")
            return None
        frames = self.pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        if not color_frame:
            return None
        return np.asanyarray(color_frame.get_data())

# =============================================================================
# Class: ObjectDetector
# =============================================================================
class ObjectDetector:
    def __init__(self, model_path, conf_thresh=0.6):
        self.model = YOLO(model_path)
        self.conf_thresh = conf_thresh
        rospy.loginfo(f"Object Detector Initialized with model: {model_path}")

    def _safe_class_name(self, names, class_id):
        try:
            return names.get(class_id, str(class_id))
        except Exception:
            return str(class_id)

    def detect(self, img_color):
        try:
            results = self.model.predict([img_color], conf=self.conf_thresh, verbose=False)
        except Exception as e:
            rospy.logerr(f"[YOLO] Inference failed: {e}")
            return []

        detections = []
        for result in results:
            if not result.boxes or result.boxes.xyxy is None:
                continue
            boxes = result.boxes.xyxy.cpu().numpy()
            confs = result.boxes.conf.cpu().numpy()
            classes = result.boxes.cls.cpu().numpy().astype(int)
            names = result.names
            for i, box in enumerate(boxes):
                ux, uy = (box[0] + box[2]) / 2, (box[1] + box[3]) / 2
                detections.append({
                    'class_name': self._safe_class_name(names, int(classes[i])),
                    'xy': (round(ux, 1), round(uy, 1))
                })
        return detections

# =============================================================================
# Class: App
# =============================================================================
class App:
    def __init__(self, model_path, cup_color_model_path):
        rospy.loginfo("Initializing Coffee Supply Server ......")
        self.camera = RealSenseCamera() # Create camera object, but do not start it
        self.detector = ObjectDetector(model_path)
        self.cup_color_detector = ObjectDetector(cup_color_model_path)
        
        # Main service for detection
        self.coffee_supply_service = rospy.Service('CoffeeSupply', DetectCoffeeSupply, self.coffee_command)
        
        # NEW: Services to control the camera's lifecycle
        self.start_cam_service = rospy.Service('start_coffee_camera', Trigger, self.handle_start_camera)
        self.stop_cam_service = rospy.Service('stop_coffee_camera', Trigger, self.handle_stop_camera)
        
        self.positions = {}

    # NEW: Service handler to start the camera
    def handle_start_camera(self, req):
        rospy.loginfo("Request received to start camera...")
        success = self.camera.start()
        message = "Camera started successfully." if success else "Failed to start camera."
        return TriggerResponse(success=success, message=message)

    # NEW: Service handler to stop the camera
    def handle_stop_camera(self, req):
        rospy.loginfo("Request received to stop camera...")
        self.camera.stop()
        return TriggerResponse(success=True, message="Camera stopped.")
    
    # ... (get_relative_position, get_all_relative methods are unchanged) ...
    # def get_relative_position(self, from_name, to_name):
    #     from_pos = self.positions.get(from_name.lower())
    #     to_pos = self.positions.get(to_name.lower())
    #     if from_pos is None or to_pos is None: return None
    #     return tuple(round(to_pos[i] - from_pos[i], 3) for i in range(2))

    # def get_all_relative(self):
    #     tree_to_target = self.get_relative_position('tree', 'black') or self.get_relative_position('tree', 'white')
    #     home_to_target = self.get_relative_position('home', 'black') or self.get_relative_position('home', 'white')
    #     home_to_tree = self.get_relative_position('home', 'tree')
    #     return {'tree_to_target': tree_to_target, 'home_to_target': home_to_target, "home_to_tree": home_to_tree}

    def coffee_command(self, req):
        # MODIFIED: Check if camera is started before proceeding
        if not self.camera.started:
            rospy.logerr("Camera is not started. Please call 'start_coffee_camera' service first.")
            return DetectCoffeeSupplyResponse(success=False)

        self.positions = {}
        detections_all = []
        cup_detections_all = []
        table = 0

        try:
            # Combined multi-frame detection loop
            for _ in range(10):
                img_color = self.camera.get_color_image()
                if img_color is None:
                    rospy.sleep(0.002)
                    continue
                detections_all.extend(self.detector.detect(img_color))
                cup_detections_all.extend(self.cup_color_detector.detect(img_color))
                rospy.sleep(0.05)
            
            
            if not detections_all:
                rospy.loginfo("No menu detections in captured frames.")
                return DetectCoffeeSupplyResponse(success=False)

            object_groups = {}
            distance_threshold = 50
            for det in detections_all:
                name = det['class_name'].lower()
                xy = np.array(det['xy'])
                group_list = object_groups.setdefault(name, [])
                if not group_list:
                    group_list.append([xy])
                else:
                    found = False
                    for group in group_list:
                        if np.linalg.norm(np.median(group, axis=0) - xy) < distance_threshold:
                            group.append(xy)
                            found = True
                            break
                    if not found:
                        group_list.append([xy])

            target_name, target_xy = None, None
            for candidate in ['black', 'white']:
                if candidate in object_groups and object_groups[candidate]:
                    groups = object_groups[candidate]
                    if groups:
                        largest_group = max(groups, key=len)
                        if largest_group:
                            target_name = candidate
                            target_xy = np.median(np.array(largest_group), axis=0)
                            break
            
            if not target_name:
                rospy.loginfo("No target coffee (black/white) group found on menu.")
                return DetectCoffeeSupplyResponse(success=False)
            
            self.positions[target_name] = tuple(target_xy)

            for lm in ['tree', 'home']:
                if lm in object_groups and object_groups[lm]:
                    lg = max(object_groups[lm], key=len)
                    if lg:
                        self.positions[lm] = tuple(np.median(np.array(lg), axis=0))   

            tree_pos = self.positions.get("tree")
            home_pos = self.positions.get("home")
            target_pos = self.positions.get(target_name)
            threshold = 100

            if target_pos and tree_pos and home_pos:
                mid_x = (tree_pos[0] + home_pos[0]) / 2
                if abs(target_pos[0] - mid_x) < threshold:
                    # X 軸靠中間，分左右
                    table = 1 if abs(target_pos[1] - home_pos[1]) < abs(target_pos[1] - tree_pos[1]) else 2
                else:
                    table = 3 if abs(target_pos[1] - home_pos[1]) < abs(target_pos[1] - tree_pos[1]) else 4
            else:
                rospy.logwarn("Missing position info, cannot determine table.")
      
            cup_positions = {}
            cup_object_groups = {}
            for det in cup_detections_all:
                name = det['class_name'].lower()
                xy = np.array(det['xy'])
                group_list = cup_object_groups.setdefault(name, [])
                if not group_list: group_list.append([xy]); continue
                found = False
                for group in group_list:
                    if np.linalg.norm(np.median(group, axis=0) - xy) < distance_threshold:
                        group.append(xy); found = True; break
                if not found: group_list.append([xy])

            for cup_name in ['white', 'black']:
                if cup_name in cup_object_groups and cup_object_groups[cup_name]:
                    largest_group = max(cup_object_groups[cup_name], key=len)
                    if largest_group:
                        cup_xy = np.median(np.array(largest_group), axis=0)
                        rel_x = cup_xy[0] - self.positions.get(target_name, (0,0))[0]
                        if rel_x < 0:
                            cup_positions[cup_name] = 'left'
                        elif rel_x > 0:
                            cup_positions[cup_name] = 'right'
                        else:
                            cup_positions[cup_name] = 'unknow'

            
            cup_side = cup_positions.get(target_name, "")
            
            return DetectCoffeeSupplyResponse(
                success=(table != 0), target_name=target_name, table=table, cup_side=cup_side)

        except Exception as e:
            rospy.logerr(f"[App] coffee_command exception: {e}\n{traceback.format_exc()}")
            return DetectCoffeeSupplyResponse(success=False)
        # REMOVED: The 'finally' block that stops the camera is gone. Control is now external.

    def shutdown(self):
        # This is a safety net to ensure camera is stopped when the node is killed.
        if self.camera:
            self.camera.stop()
        rospy.loginfo("Coffee Supply Server Shutdown.")

# =============================================================================
# Main
# =============================================================================
if __name__ == '__main__':
    try:
        rospy.init_node('coffee_supply_server_node')
        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path('object_detect')
        model_path = os.path.join(pkg_path, 'scripts', 'coffee_supply.pt')
        cup_model_path = os.path.join(pkg_path, 'scripts', 'coffee.pt')
        server = App(model_path, cup_model_path)
        rospy.on_shutdown(server.shutdown)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"Failed to start Coffee Supply Server: {e}")