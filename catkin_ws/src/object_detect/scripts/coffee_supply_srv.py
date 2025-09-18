# #!/usr/bin/env python3
# import numpy as np
# import pyrealsense2 as rs
# from ultralytics import YOLO
# import rospy
# import rospkg, os, traceback
# from object_detect.srv import DetectCoffeeSupply, DetectCoffeeSupplyResponse

# # =============================================================================
# # Class: RealSenseCamera
# # =============================================================================
# class RealSenseCamera:
#     def __init__(self):
#         self.pipeline = rs.pipeline()
#         self.started = False
#         try:
#             config = rs.config()
#             config.enable_stream(rs.stream.color, 848, 480, rs.format.bgr8, 30)
#             self.pipeline.start(config)
#             self.started = True
#             rospy.loginfo("RealSense Camera Initialized (RGB only).")
#         except Exception as e:
#             rospy.logerr(f"[Camera] Failed to start RealSense pipeline: {e}")

#     def get_color_image(self):
#         if not self.started:
#             return None
#         frames = self.pipeline.wait_for_frames()
#         color_frame = frames.get_color_frame()
#         if not color_frame:
#             return None
#         return np.asanyarray(color_frame.get_data())

#     def stop(self):
#         if self.started:
#             try:
#                 self.pipeline.stop()
#                 rospy.loginfo("RealSense Camera Stopped.")
#             except Exception as e:
#                 rospy.logwarn(f"[Camera] Stop pipeline warning: {e}")
#             self.started = False


# # =============================================================================
# # Class: ObjectDetector
# # =============================================================================
# class ObjectDetector:
#     def __init__(self, model_path, conf_thresh=0.4, sample_size=5):
#         self.model = YOLO(model_path)
#         self.conf_thresh = conf_thresh
#         self.sample_size = sample_size
#         rospy.loginfo(f"Object Detector Initialized with model: {model_path}")

#     def get_median_depth(self, depth_frame, u, v):
#         half_size = self.sample_size // 2
#         depth_values = []
#         h, w = depth_frame.get_height(), depth_frame.get_width()
#         for du in range(-half_size, half_size + 1):
#             for dv in range(-half_size, half_size + 1):
#                 x, y = int(u) + du, int(v) + dv
#                 if 0 <= x < w and 0 <= y < h:
#                     d = depth_frame.get_distance(x, y)
#                     if d > 0:
#                         depth_values.append(d)
#         return float(np.median(depth_values)) if depth_values else None

#     def _safe_class_name(self, names, class_id):
#         """安全取得類別名稱，避免索引錯誤"""
#         try:
#             if isinstance(names, dict):
#                 return names.get(class_id, str(class_id))
#             if isinstance(names, (list, tuple)):
#                 return names[class_id] if 0 <= class_id < len(names) else str(class_id)
#         except Exception as e:
#             rospy.logwarn(f"[ClassName] Invalid mapping: names={names}, class_id={class_id}, err={e}")
#         return str(class_id)

#     def detect(self, img_color):
#         try:
#             results = self.model.predict([img_color], conf=self.conf_thresh, verbose=False)
#         except Exception as e:
#             rospy.logerr(f"[YOLO] Inference failed: {e}")
#             return []

#         detections = []
#         for result in results:
#             if not result.boxes or result.boxes.xyxy is None:
#                 continue
#             boxes = result.boxes.xyxy.cpu().numpy()
#             confs = result.boxes.conf.cpu().numpy()
#             classes = result.boxes.cls.cpu().numpy().astype(int)
#             names = result.names
#             rospy.logdebug(f"[YOLO] boxes={boxes.shape}, confs={len(confs)}, classes={len(classes)}, names={names}")
#             for i, box in enumerate(boxes):
#                 if i >= len(classes) or i >= len(confs):
#                     rospy.logwarn(f"[YOLO] Index mismatch: i={i}, classes={len(classes)}, confs={len(confs)}")
#                     continue
#                 ux, uy = (box[0] + box[2]) / 2, (box[1] + box[3]) / 2
#                 detections.append({
#                     'class_id': int(classes[i]),
#                     'class_name': self._safe_class_name(names, int(classes[i])),
#                     'conf': round(float(confs[i]),3),
#                     'xy': (round(ux,1), round(uy,1)),
#                     'xyz': (round(ux,1), round(uy,1), 0.0)  # z 設為 0
#                 })
#         return detections


# # =============================================================================
# # Class: App
# # =============================================================================
# class App:
#     def __init__(self, model_path, cup_color_model_path):
#         rospy.loginfo("Initializing Coffee Supply Server ......")
#         self.camera = RealSenseCamera()
#         self.detector = ObjectDetector(model_path)
#         self.cup_color_detector = ObjectDetector(cup_color_model_path)
#         self.coffee_supply_service = rospy.Service('CoffeeSupply', DetectCoffeeSupply, self.coffee_command)
#         self.positions = {}

#     def get_relative_position(self, from_name, to_name):
#         from_pos = self.positions.get(from_name.lower())
#         to_pos = self.positions.get(to_name.lower())
#         if from_pos is None or to_pos is None:
#             return None
#         if len(from_pos) < 2 or len(to_pos) < 2:
#             rospy.logwarn(f"Invalid positions: {from_pos}, {to_pos}")
#             return None
#         return tuple(round(to_pos[i] - from_pos[i], 3) for i in range(2))  


#     def get_all_relative(self):
#         tree_to_target = self.get_relative_position('tree', 'black') or self.get_relative_position('tree', 'white')
#         home_to_target = self.get_relative_position('home', 'black') or self.get_relative_position('home', 'white')
#         return {'tree_to_target': tree_to_target, 'home_to_target': home_to_target}

#     def coffee_command(self, req):
#         if not self.camera.started:
#             rospy.logwarn("Camera not initialized or failed to start.")
#             return DetectCoffeeSupplyResponse(success=False, target_name="", table=0, cup_side="")

#         self.positions = {}
#         detections_all = []
#         try:
#             # --- 多幀偵測 ---
#             for _ in range(10):
#                 img_color = self.camera.get_color_image()
#                 if img_color is None:
#                     rospy.sleep(0.002)
#                     continue
#                 detections_all.extend(self.detector.detect(img_color))
#                 rospy.sleep(0.05)

#             if not detections_all:
#                 rospy.loginfo("No detections in captured frames.")
#                 return DetectCoffeeSupplyResponse(success=False, target_name="", table=0, cup_side="")

#             # --- 分群 ---
#             object_groups = {}
#             distance_threshold = 50
#             for det in detections_all:
#                 name = det['class_name'].lower()
#                 xy = np.array(det['xy'])
#                 group_list = object_groups.setdefault(name, [])
#                 for group in group_list:
#                     if np.linalg.norm(np.median(group, axis=0) - xy) < distance_threshold:
#                         group.append(xy)
#                         break
#                 else:
#                     group_list.append([xy])

#             # --- 選出 target ---
#             target_name, target_xy = None, None
#             for candidate in ['black', 'white']:
#                 if candidate in object_groups and object_groups[candidate]:
#                     groups = object_groups[candidate]
#                     if groups:
#                         largest_group = max(groups, key=len)
#                         if largest_group:
#                             target_name = candidate
#                             target_xy = np.median(np.array(largest_group), axis=0)
#                             break

#             if not target_name:
#                 rospy.loginfo("No coffee (black/white) group found.")
#                 return DetectCoffeeSupplyResponse(success=False, target_name="", table=0, cup_side="")

#             self.positions[target_name] = tuple(target_xy)

#             # --- landmark ---
#             for lm in ['tree', 'home']:
#                 if lm in object_groups and object_groups[lm]:
#                     groups = object_groups[lm]
#                     lg = max(groups, key=len)
#                     if lg:
#                         self.positions[lm] = tuple(np.median(np.array(lg), axis=0))

#             # --- 桌號判斷 ---
#             rels = self.get_all_relative()
#             def dist(vec): return round(np.linalg.norm(vec),1) if vec is not None else None
#             tol = 20
#             def near(a,b): return a is not None and b is not None and abs(a-b)<tol

#             dist_tree = dist(rels['tree_to_target'])
#             dist_home = dist(rels['home_to_target'])
#             table_lookup = {(160,70):1, (75,155):2, (205,150):3, (155,200):4}
#             table = 0
#             for (dt, dh), t in table_lookup.items():
#                 if near(dist_tree, dt) and near(dist_home, dh):
#                     table = t
#                     break
#             success = table != 0

#             # --- 咖啡杯左右判斷 ---
#             cup_positions = {}
#             for _ in range(5):
#                 img_color = self.camera.get_color_image()
#                 if img_color is None:
#                     rospy.sleep(0.002)
#                     continue
#                 cup_dets = self.cup_color_detector.detect(img_color)
#                 for det in cup_dets:
#                     if 'xy' not in det or len(det['xy']) != 2:
#                         rospy.logwarn(f"[Cup] Invalid det.xy: {det}")
#                         continue
#                     name = det['class_name'].lower()
#                     rel_x = det['xy'][0] - self.positions.get(target_name,(0,0))[0]
#                     cup_positions[name] = 'left' if rel_x < 0 else 'right'
#                 rospy.sleep(0.03)

#             white_side = cup_positions.get('white', 'unknown')
#             black_side = cup_positions.get('black', 'unknown')
#             rospy.loginfo(f"White coffee is on the {white_side} side and black coffee is on the {black_side} side")

#             cup_side = cup_positions.get(target_name, "")

#             return DetectCoffeeSupplyResponse(
#                 success=success,
#                 target_name=target_name,
#                 table=table,
#                 cup_side=cup_side
#             )

#         except Exception as e:
#             rospy.logerr(f"[App] coffee_command exception: {e}")
#             rospy.logerr(traceback.format_exc())
#             return DetectCoffeeSupplyResponse(success=False, target_name="", table=0, cup_side="")
#         finally:
#             if self.camera:
#                 self.camera.stop()

#     def shutdown(self):
#         if self.camera:
#             self.camera.stop()
#         rospy.loginfo("Coffee Supply Server Shutdown.")


# # =============================================================================
# # Main
# # =============================================================================
# if __name__ == '__main__':
#     try:
#         rospy.init_node('coffee_supply_server_node')
#         rospack = rospkg.RosPack()
#         pkg_path = rospack.get_path('object_detect')
#         model_path = os.path.join(pkg_path, 'scripts', 'coffee_supply.pt')
#         cup_model_path = os.path.join(pkg_path, 'scripts', 'coffee.pt')
#         server = App(model_path, cup_model_path)
#         rospy.on_shutdown(server.shutdown)
#         rospy.spin()
#     except rospy.ROSInterruptException:
#         pass
#     except Exception as e:
#         rospy.logerr(f"Failed to start Coffee Supply Server: {e}")



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
    def __init__(self, model_path, conf_thresh=0.4):
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
    def get_relative_position(self, from_name, to_name):
        from_pos = self.positions.get(from_name.lower())
        to_pos = self.positions.get(to_name.lower())
        if from_pos is None or to_pos is None: return None
        return tuple(round(to_pos[i] - from_pos[i], 3) for i in range(2))

    def get_all_relative(self):
        tree_to_target = self.get_relative_position('tree', 'black') or self.get_relative_position('tree', 'white')
        home_to_target = self.get_relative_position('home', 'black') or self.get_relative_position('home', 'white')
        return {'tree_to_target': tree_to_target, 'home_to_target': home_to_target}

    def coffee_command(self, req):
        # MODIFIED: Check if camera is started before proceeding
        if not self.camera.started:
            rospy.logerr("Camera is not started. Please call 'start_coffee_camera' service first.")
            return DetectCoffeeSupplyResponse(success=False)

        self.positions = {}
        detections_all = []
        cup_detections_all = []

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

            rels = self.get_all_relative()
            dist = lambda vec: round(np.linalg.norm(vec), 1) if vec is not None else None
            near = lambda a, b, tol=20: a is not None and b is not None and abs(a - b) < tol
            dist_tree, dist_home = dist(rels['tree_to_target']), dist(rels['home_to_target'])
            
            table_lookup = {(160, 70): 1, (75, 155): 2, (205, 150): 3, (155, 200): 4}
            table = 0
            for (dt, dh), t in table_lookup.items():
                if near(dist_tree, dt) and near(dist_home, dh):
                    table = t
                    break
            
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
                        cup_positions[cup_name] = 'left' if rel_x < 0 else 'right'
            
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