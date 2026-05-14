"""
車道辨識程式 — v4 + offset (車輛定位量測)
基於 v4 (ix=30 為預設)，加入下列功能：

[v4 → v4_offset 新增]
1. 車輛 anchor 點：影像中央，y 在 ROI 上邊界 (car_x, car_y) = (w/2, h*0.45)
2. 量測算法：水平帶 (anchor_y ± band_half=15) 內，左右車道線 mask 的平均 x
   → left_x, right_x, lane_center_x
3. 回報三個量：
   - left_dist  = car_x - left_x   (車輛距左線多少 px，正=偏右)
   - right_dist = right_x - car_x  (車輛距右線多少 px，正=偏左)
   - offset     = car_x - lane_center_x (正=車偏右、負=車偏左)
4. 視覺化：量測線、anchor 點、左右量測點、車道中心、修正箭頭、文字疊印
5. 單邊只看到一條線時退化為「該邊距離 + 另一邊 ?」

- 輸入：src.mp4
- 輸出：output_v4_ix30_offset.mp4
"""

import argparse
import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from cv_bridge import CvBridge, CvBridgeError


# ---------------------------------------------------------------------------
# 共用幾何工具
# ---------------------------------------------------------------------------

def get_roi_mask(h, w):
    """建立 ROI 遮罩：畫面下半部全寬"""
    mask = np.zeros((h, w), dtype=np.uint8)
    roi_vertices = np.array([
        [
            (0, h),              # 左下
            (0, int(h * 0.45)),  # 左上
            (w, int(h * 0.45)),  # 右上
            (w, h),              # 右下
        ]
    ], dtype=np.int32)
    cv2.fillPoly(mask, roi_vertices, 255)
    return mask, roi_vertices


def compute_contour_mask(contour, h, w):
    """將輪廓轉成二值遮罩"""
    mask = np.zeros((h, w), dtype=np.uint8)
    cv2.drawContours(mask, [contour], -1, 255, cv2.FILLED)
    return mask


def compute_overlap(mask_a, mask_b):
    """計算兩個遮罩的重疊比例 (相對於較小遮罩面積)"""
    intersection = cv2.bitwise_and(mask_a, mask_b)
    inter_area = cv2.countNonZero(intersection)
    area_a = cv2.countNonZero(mask_a)
    area_b = cv2.countNonZero(mask_b)
    min_area = min(area_a, area_b)
    if min_area == 0:
        return 0.0
    return inter_area / min_area


def min_contour_distance(c1, c2):
    """近似計算兩條 contour 之間的最小點對距離 (取樣節省時間)"""
    step1 = max(1, len(c1) // 50)
    step2 = max(1, len(c2) // 50)
    pts1 = c1[::step1].reshape(-1, 2).astype(np.float32)
    pts2 = c2[::step2].reshape(-1, 2).astype(np.float32)

    min_dist = float('inf')
    for pt in pts1:
        dists = np.sqrt(np.sum((pts2 - pt) ** 2, axis=1))
        d = float(np.min(dists))
        if d < min_dist:
            min_dist = d
        if min_dist < 1:
            return min_dist
    return min_dist


def get_contour_angle(contour):
    """用 minAreaRect 取輪廓主軸角度 (0~180 度)"""
    if len(contour) < 5:
        return 90
    rect = cv2.minAreaRect(contour)
    _, (rect_w, rect_h), angle = rect
    if rect_w < rect_h:
        angle = angle + 90
    return angle % 180


def find_intersection(line1, line2):
    """
    計算兩條 cv2.fitLine 直線的交點，回傳 np.array([x, y])，平行則回傳 None
    """
    vx1, vy1, x1, y1 = line1.flatten()
    vx2, vy2, x2, y2 = line2.flatten()
    det = vx1 * vy2 - vy1 * vx2
    if abs(det) < 1e-6:
        return None
    t1 = ((x2 - x1) * vy2 - (y2 - y1) * vx2) / det
    return np.array([x1 + t1 * vx1, y1 + t1 * vy1])


# ---------------------------------------------------------------------------
# v4 新增：用 mask closing 合併輪廓 (取代 convexHull)
# ---------------------------------------------------------------------------

def merge_contours_via_mask(contours, h, w, close_size=7):
    """
    將多條輪廓「邏輯上」合併為單一輪廓：
    1. 將所有 contour 畫到同張 mask
    2. 做 morphological closing (dilate→erode) 把距離 < close_size 的縫補起來
    3. 重新 findContours，取最大塊作為合併結果

    與 convexHull 的差別：closing 只填補小縫隙，不會把兩條線之間的白底全包進來。
    """
    if not contours:
        return None
    if len(contours) == 1:
        return contours[0]

    mask = np.zeros((h, w), dtype=np.uint8)
    for c in contours:
        cv2.drawContours(mask, [c], -1, 255, cv2.FILLED)

    if close_size > 1:
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (close_size, close_size))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    found, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not found:
        return contours[0]
    return max(found, key=cv2.contourArea)


def clip_contour_to_roi(contour, roi_mask, h, w, min_area=200):
    """
    將單一 contour 用 ROI mask 裁切，回傳 ROI 內的子輪廓 list
    (一條 contour 被 ROI 切後可能變成 0、1、N 條)
    """
    cmask = compute_contour_mask(contour, h, w)
    cmask_in_roi = cv2.bitwise_and(cmask, roi_mask)
    sub_contours, _ = cv2.findContours(cmask_in_roi, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    return [sc for sc in sub_contours if cv2.contourArea(sc) >= min_area]


# ---------------------------------------------------------------------------
# v4 新增：浮動線三規則判定 (規則 A / B；移除規則 C)
# ---------------------------------------------------------------------------

def resolve_floating_v4(main_contours, floating_contours, h, w,
                        merge_dist=10, ix_thresh=80, ix_to_segment_max=200):
    """
    處理位於 ROI 之上的浮動線段，決定是否合併到主線、或丟棄。

    規則 A (實體連接):
        浮動線與某條主線的最小距離 < merge_dist → 合併到該主線
    規則 B (延伸交點靠近):
        浮動線與某條主線各自擬合直線後的交點，到「兩段最近端點」的距離
        都 < ix_thresh，且交點本身距離兩段不能過遠 (< ix_to_segment_max)
        → 合併到該主線
    都不通過 → 丟棄 (回傳到 dropped 供 debug)

    回傳:
        merged_main_contours : list
            原本的 main_contours，但其中部分已經與通過規則的浮動線合併
        dropped_floats : list
            被丟棄的浮動線 (debug 用，會在輸出畫淡黃色框)
    """
    # 用「群組」概念：每條主線一個 bucket，通過規則的浮動線放進對應 bucket
    buckets = [[c] for c in main_contours]
    dropped = []

    for float_c in floating_contours:
        bucket_idx = -1

        # ---- 規則 A：實體連接 ----
        best_dist = float('inf')
        best_a_idx = -1
        for i, main_c in enumerate(main_contours):
            d = min_contour_distance(float_c, main_c)
            if d < best_dist:
                best_dist = d
                best_a_idx = i
        if best_dist < merge_dist and best_a_idx != -1:
            bucket_idx = best_a_idx

        # ---- 規則 B：延伸交點靠近 ----
        if bucket_idx == -1 and len(float_c) >= 5:
            float_line = cv2.fitLine(float_c, cv2.DIST_L2, 0, 0.01, 0.01)
            best_total = float('inf')
            best_b_idx = -1
            for i, main_c in enumerate(main_contours):
                if len(main_c) < 5:
                    continue
                main_line = cv2.fitLine(main_c, cv2.DIST_L2, 0, 0.01, 0.01)
                ix_pt = find_intersection(float_line, main_line)
                if ix_pt is None:
                    continue

                pts_float = float_c.reshape(-1, 2).astype(np.float64)
                pts_main = main_c.reshape(-1, 2).astype(np.float64)
                d_float = float(np.min(np.sqrt(np.sum((pts_float - ix_pt) ** 2, axis=1))))
                d_main = float(np.min(np.sqrt(np.sum((pts_main - ix_pt) ** 2, axis=1))))

                # 兩段端點到交點都要近 (主要判據)
                if d_float < ix_thresh and d_main < ix_thresh:
                    # 額外硬上限：交點本身不能遠離兩段太多 (避免無窮遠交點誤判)
                    if d_float < ix_to_segment_max and d_main < ix_to_segment_max:
                        total = d_float + d_main
                        if total < best_total:
                            best_total = total
                            best_b_idx = i
            if best_b_idx != -1:
                bucket_idx = best_b_idx

        # ---- 沒通過任何規則 → 丟棄 ----
        if bucket_idx == -1:
            dropped.append(float_c)
        else:
            buckets[bucket_idx].append(float_c)

    # 把每個 bucket 用 mask closing 合併成單一 contour
    merged_main_contours = []
    for bucket in buckets:
        if len(bucket) == 1:
            merged_main_contours.append(bucket[0])
        else:
            merged = merge_contours_via_mask(bucket, h, w, close_size=11)
            if merged is not None:
                merged_main_contours.append(merged)

    return merged_main_contours, dropped


# ---------------------------------------------------------------------------
# Tracker (從 v3 沿用，僅微調：移除 prev_*_angle 對 rule C 的依賴)
# ---------------------------------------------------------------------------


# ---------------------------------------------------------------------------
# v4_offset 新增：車輛定位量測
# ---------------------------------------------------------------------------

def compute_lane_offset(left_contours, right_contours, h, w,
                        car_x=None, anchor_ratio=0.85, band_half=5, known_lane_width=None):
    """
    用水平掃描帶量測車輛 anchor 點到左右車道線的 pixel 距離。

    參數：
        left_contours, right_contours: tracker 分類後的左右車道輪廓 (list)
        car_x: 車輛 x 位置 (None → 預設 w//2)
        anchor_ratio: anchor 點的 y 座標比例 (0.85 = 從上算下到 85%)
        band_half: 取樣帶高度的一半 (像素)，總帶高 = 2*band_half+1

    回傳 dict：
        car_x, car_y           : anchor 點座標
        left_x, right_x        : 左右車道線在帶內的平均 x (None = 沒偵測到)
        lane_center_x          : (left_x + right_x) / 2 (None 若任一邊缺)
        left_dist, right_dist  : car_x - left_x、right_x - car_x
        offset                 : car_x - lane_center_x
        status                 : "ok" / "left_only" / "right_only" / "none"
    """
    if car_x is None:
        car_x = w // 2
    car_y = int(h * anchor_ratio)

    y0 = max(0, car_y - band_half)
    y1 = min(h, car_y + band_half + 1)

    def avg_x_in_band(contours):
        if not contours:
            return None
        mask = np.zeros((h, w), dtype=np.uint8)
        cv2.drawContours(mask, contours, -1, 255, cv2.FILLED)
        band = mask[y0:y1, :]
        ys, xs = np.where(band > 0)
        if len(xs) == 0:
            return None
        return float(np.mean(xs))

    left_x = avg_x_in_band(left_contours)
    right_x = avg_x_in_band(right_contours)

    info = {
        "car_x": int(car_x),
        "car_y": int(car_y),
        "left_x": left_x,
        "right_x": right_x,
        "lane_center_x": None,
        "left_dist": None,
        "right_dist": None,
        "offset": None,
        "status": "none",
        "lane_width": known_lane_width,
    }

    if left_x is not None and right_x is not None:
        if info["lane_width"] is None:
            info["lane_width"] = right_x - left_x
            
        info["lane_center_x"] = (left_x + right_x) / 2.0
        info["left_dist"] = car_x - left_x
        info["right_dist"] = right_x - car_x
        info["offset"] = car_x - info["lane_center_x"]
        info["status"] = "ok"
    elif left_x is not None:
        info["left_dist"] = car_x - left_x
        if info["lane_width"] is not None:
            info["lane_center_x"] = left_x + (info["lane_width"] / 2.0)
            info["offset"] = car_x - info["lane_center_x"]
            info["status"] = "left_only_estimated"
        else:
            info["status"] = "left_only"
    elif right_x is not None:
        info["right_dist"] = right_x - car_x
        if info["lane_width"] is not None:
            info["lane_center_x"] = right_x - (info["lane_width"] / 2.0)
            info["offset"] = car_x - info["lane_center_x"]
            info["status"] = "right_only_estimated"
        else:
            info["status"] = "right_only"

    return info


def draw_lane_offset(img, off):
    """把 compute_lane_offset 的結果疊印在影像上"""
    h, w = img.shape[:2]
    cx, cy = off["car_x"], off["car_y"]

    # 量測線 (淡綠水平線，貫穿全寬)
    cv2.line(img, (0, cy), (w, cy), (180, 220, 180), 1)

    # 兩個帶邊界 (更淡)
    band = 5
    cv2.line(img, (0, cy - band), (w, cy - band), (160, 200, 160), 1)
    cv2.line(img, (0, cy + band), (w, cy + band), (160, 200, 160), 1)

    # anchor 白色圓點
    cv2.circle(img, (cx, cy), 6, (255, 255, 255), -1)
    cv2.circle(img, (cx, cy), 6, (0, 0, 0), 1)

    # 左右量測點
    if off["left_x"] is not None:
        lx = int(off["left_x"])
        cv2.drawMarker(img, (lx, cy), (0, 0, 255), cv2.MARKER_CROSS, 14, 2)
    if off["right_x"] is not None:
        rx = int(off["right_x"])
        cv2.drawMarker(img, (rx, cy), (255, 0, 0), cv2.MARKER_CROSS, 14, 2)

    # 車道中心 + 修正箭頭
    if off["lane_center_x"] is not None:
        ccx = int(off["lane_center_x"])
        cv2.line(img, (ccx, cy - 10), (ccx, cy + 10), (0, 255, 255), 2)
        if abs(ccx - cx) > 2:
            cv2.arrowedLine(img, (cx, cy), (ccx, cy), (0, 255, 255), 2,
                            tipLength=0.25)

    # 文字疊印 (左下角)
    y_text = h - 60
    if off["status"] == "ok":
        msg1 = f"L={off['left_dist']:+.0f}  R={off['right_dist']:+.0f}  off={off['offset']:+.0f} (W={off['lane_width']:.0f})"
        color = (255, 255, 255)
    elif off["status"] == "left_only_estimated":
        msg1 = f"L={off['left_dist']:+.0f}  R=(est)  off={off['offset']:+.0f} (W={off['lane_width']:.0f})"
        color = (200, 255, 200)
    elif off["status"] == "right_only_estimated":
        msg1 = f"L=(est)  R={off['right_dist']:+.0f}  off={off['offset']:+.0f} (W={off['lane_width']:.0f})"
        color = (200, 255, 200)
    elif off["status"] == "left_only":
        msg1 = f"L={off['left_dist']:+.0f}  R=?  (right not seen)"
        color = (180, 180, 255)
    elif off["status"] == "right_only":
        msg1 = f"L=?  R={off['right_dist']:+.0f}  (left not seen)"
        color = (255, 180, 180)
    else:
        msg1 = "no lane"
        color = (128, 128, 128)
    cv2.rectangle(img, (5, y_text - 18), (5 + 350, y_text + 6), (0, 0, 0), -1)
    cv2.putText(img, msg1, (10, y_text),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 1, cv2.LINE_AA)
    return img, off


def touches_frame_border(contour, h, w, margin=5):
    """判斷輪廓是否碰到畫面左右邊界"""
    x, y, bw, bh = cv2.boundingRect(contour)
    touches_left = x <= margin
    touches_right = (x + bw) >= (w - margin)
    if touches_left and touches_right:
        return "both"
    if touches_left:
        return "left"
    if touches_right:
        return "right"
    return None


class LaneTracker:
    """
    車道線追蹤器 v4
    - 結構與 v3 相同：重疊度配對 + 邊緣排除 + 外側排除
    - tracker 本身不變，因為前處理已經把背景 / 凸包問題清掉了
    """

    def __init__(self):
        self.prev_left_mask = None
        self.prev_right_mask = None
        self.prev_left_cx = None
        self.prev_right_cx = None
        self.prev_left_angle = None
        self.prev_right_angle = None
        self.no_left_count = 0
        self.no_right_count = 0
        self.max_memory_frames = 15

    def classify_contours(self, valid_contours, center_x, h, w):
        if not valid_contours:
            self._update_no_detection()
            return [], []

        contour_info = []
        for contour in valid_contours:
            M = cv2.moments(contour)
            if M["m00"] == 0:
                continue
            cx = int(M["m10"] / M["m00"])
            mask = compute_contour_mask(contour, h, w)
            border_touch = touches_frame_border(contour, h, w)
            angle = get_contour_angle(contour)
            contour_info.append({
                "contour": contour,
                "cx": cx,
                "mask": mask,
                "border_touch": border_touch,
                "angle": angle,
                "area": cv2.contourArea(contour),
            })

        if not contour_info:
            self._update_no_detection()
            return [], []

        # ===== 第一步：追蹤重疊度比對 =====
        left_matched_idx = None
        right_matched_idx = None
        unmatched = list(range(len(contour_info)))
        overlap_threshold = 0.15

        if self.prev_left_mask is not None and self.no_left_count < self.max_memory_frames:
            best_overlap = 0
            best_idx = -1
            for i in unmatched:
                ov = compute_overlap(contour_info[i]["mask"], self.prev_left_mask)
                if ov > best_overlap:
                    best_overlap = ov
                    best_idx = i
            if best_overlap > overlap_threshold:
                left_matched_idx = best_idx
                unmatched.remove(best_idx)

        if self.prev_right_mask is not None and self.no_right_count < self.max_memory_frames:
            best_overlap = 0
            best_idx = -1
            for i in unmatched:
                ov = compute_overlap(contour_info[i]["mask"], self.prev_right_mask)
                if ov > best_overlap:
                    best_overlap = ov
                    best_idx = i
            if best_overlap > overlap_threshold:
                right_matched_idx = best_idx
                unmatched.remove(best_idx)

        # ===== 第二步：未配對的用位置 + 角度 + 地板排除判斷 =====
        for i in unmatched[:]:
            info = contour_info[i]
            cx = info["cx"]
            border = info["border_touch"]

            is_likely_floor = False
            if border == "left" and cx < center_x:
                if self.prev_left_mask is not None and self.no_left_count < self.max_memory_frames:
                    ov = compute_overlap(info["mask"], self.prev_left_mask)
                    if ov < 0.05:
                        is_likely_floor = True
                elif left_matched_idx is not None:
                    is_likely_floor = True

            if border == "right" and cx >= center_x:
                if self.prev_right_mask is not None and self.no_right_count < self.max_memory_frames:
                    ov = compute_overlap(info["mask"], self.prev_right_mask)
                    if ov < 0.05:
                        is_likely_floor = True
                elif right_matched_idx is not None:
                    is_likely_floor = True

            if is_likely_floor:
                continue

            if left_matched_idx is None and right_matched_idx is None:
                assigned = self._assign_single_line(info, center_x)
                if assigned == "left":
                    left_matched_idx = i
                else:
                    right_matched_idx = i
            elif left_matched_idx is None and cx < center_x:
                left_matched_idx = i
            elif right_matched_idx is None and cx >= center_x:
                right_matched_idx = i
            elif left_matched_idx is None or right_matched_idx is None:
                assigned = self._assign_by_angle_and_memory(info, center_x)
                if assigned == "left" and left_matched_idx is None:
                    left_matched_idx = i
                elif assigned == "right" and right_matched_idx is None:
                    right_matched_idx = i

        # ===== 第三步：外側排除 =====
        if left_matched_idx is not None and right_matched_idx is not None:
            left_cx = contour_info[left_matched_idx]["cx"]
            right_cx = contour_info[right_matched_idx]["cx"]
            if left_cx > right_cx:
                left_ov = 0
                right_ov = 0
                if self.prev_left_mask is not None:
                    left_ov = compute_overlap(
                        contour_info[left_matched_idx]["mask"], self.prev_left_mask
                    )
                if self.prev_right_mask is not None:
                    right_ov = compute_overlap(
                        contour_info[right_matched_idx]["mask"], self.prev_right_mask
                    )
                if left_ov > right_ov:
                    right_matched_idx = None
                else:
                    left_matched_idx = None

        left_contours = [contour_info[left_matched_idx]["contour"]] if left_matched_idx is not None else []
        right_contours = [contour_info[right_matched_idx]["contour"]] if right_matched_idx is not None else []

        self._update_memory(contour_info, left_matched_idx, right_matched_idx, h, w)
        return left_contours, right_contours

    def _assign_single_line(self, info, center_x):
        cx = info["cx"]
        angle = info["angle"]

        left_ov = 0
        right_ov = 0
        if self.prev_left_mask is not None and self.no_left_count < self.max_memory_frames:
            left_ov = compute_overlap(info["mask"], self.prev_left_mask)
        if self.prev_right_mask is not None and self.no_right_count < self.max_memory_frames:
            right_ov = compute_overlap(info["mask"], self.prev_right_mask)

        if left_ov > 0.1 or right_ov > 0.1:
            return "left" if left_ov > right_ov else "right"

        if self.prev_left_angle is not None and self.prev_right_angle is not None:
            left_angle_diff = min(
                abs(angle - self.prev_left_angle),
                180 - abs(angle - self.prev_left_angle)
            )
            right_angle_diff = min(
                abs(angle - self.prev_right_angle),
                180 - abs(angle - self.prev_right_angle)
            )
            if abs(left_angle_diff - right_angle_diff) > 15:
                return "left" if left_angle_diff < right_angle_diff else "right"

        if self.prev_left_cx is not None and self.prev_right_cx is not None:
            dist_to_left = abs(cx - self.prev_left_cx)
            dist_to_right = abs(cx - self.prev_right_cx)
            return "left" if dist_to_left < dist_to_right else "right"

        return "left" if cx < center_x else "right"

    def _assign_by_angle_and_memory(self, info, center_x):
        cx = info["cx"]
        angle = info["angle"]

        if self.prev_left_angle is not None and self.prev_right_angle is not None:
            left_angle_diff = min(
                abs(angle - self.prev_left_angle),
                180 - abs(angle - self.prev_left_angle)
            )
            right_angle_diff = min(
                abs(angle - self.prev_right_angle),
                180 - abs(angle - self.prev_right_angle)
            )
            return "left" if left_angle_diff < right_angle_diff else "right"

        left_ov = 0
        right_ov = 0
        if self.prev_left_mask is not None:
            left_ov = compute_overlap(info["mask"], self.prev_left_mask)
        if self.prev_right_mask is not None:
            right_ov = compute_overlap(info["mask"], self.prev_right_mask)

        if left_ov > 0.05 or right_ov > 0.05:
            return "left" if left_ov > right_ov else "right"

        return "left" if cx < center_x else "right"

    def _update_memory(self, contour_info, left_idx, right_idx, h, w):
        if left_idx is not None:
            self.prev_left_mask = contour_info[left_idx]["mask"]
            self.prev_left_cx = contour_info[left_idx]["cx"]
            self.prev_left_angle = contour_info[left_idx]["angle"]
            self.no_left_count = 0
        else:
            self.no_left_count += 1

        if right_idx is not None:
            self.prev_right_mask = contour_info[right_idx]["mask"]
            self.prev_right_cx = contour_info[right_idx]["cx"]
            self.prev_right_angle = contour_info[right_idx]["angle"]
            self.no_right_count = 0
        else:
            self.no_right_count += 1

    def _update_no_detection(self):
        self.no_left_count += 1
        self.no_right_count += 1


# ---------------------------------------------------------------------------
# 主流程
# ---------------------------------------------------------------------------

def process_frame(frame, roi_mask, roi_vertices, tracker, ix_thresh,
                  anchor_ratio=0.45, band_half=15, known_lane_width=None, max_area_thresh=20000):
    h, w = frame.shape[:2]

    # 1. 灰階
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # 2. 二值化 (Otsu) — 暫不套 ROI，保留全圖以便偵測 ROI 外的浮動線
    _, binary = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)

    # 3. 形態學
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
    binary = cv2.morphologyEx(binary, cv2.MORPH_OPEN, kernel, iterations=2)
    binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel, iterations=2)

    # 4. 全圖 findContours (在 ROI 套用之前，保留實體連接資訊)
    contours_full, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    min_area = 500
    valid_full = [c for c in contours_full if cv2.contourArea(c) > min_area]

    # 5. 分主線 vs 浮動線 (依靠 ROI 上邊界 y = h*0.45)
    roi_y_top = int(h * 0.45)
    main_contours = []
    floating_contours = []
    for c in valid_full:
        # 任何一點落在 ROI 內 → 主線；完全在 ROI 外 → 浮動線
        if np.any(c[:, 0, 1] >= roi_y_top):
            main_contours.append(c)
        else:
            floating_contours.append(c)

    # 6. 浮動線三規則判定 (在 ROI 套用之前完成)
    resolved_main, dropped_floats = resolve_floating_v4(
        main_contours, floating_contours, h, w,
        merge_dist=10,
        ix_thresh=ix_thresh,
        ix_to_segment_max=200,
    )

    # 7. 主線不再強制裁切到 ROI 內，除非面積過大
    # 加入邏輯：只要該主線(或合併後的線)有部分在 ROI 內，且黑色部分相連，就視為同一條線，不予以截斷。
    # 但若相連起來的面積大於 max_area_thresh (可能是場外大片地板)，則退回原本的模式：將其裁切到 ROI 內。
    valid_contours = []
    for c in resolved_main:
        if cv2.contourArea(c) > max_area_thresh:
            sub = clip_contour_to_roi(c, roi_mask, h, w, min_area=min_area)
            valid_contours.extend(sub)
        else:
            valid_contours.append(c)

    # 8. tracker 分類
    center_x = w // 2
    left_contours, right_contours = tracker.classify_contours(valid_contours, center_x, h, w)

    # 9. 繪製
    overlay = frame.copy()
    alpha = 0.4
    if left_contours:
        cv2.drawContours(overlay, left_contours, -1, (0, 0, 255), cv2.FILLED)
    if right_contours:
        cv2.drawContours(overlay, right_contours, -1, (255, 0, 0), cv2.FILLED)
    result = cv2.addWeighted(overlay, alpha, frame, 1 - alpha, 0)

    if left_contours:
        cv2.drawContours(result, left_contours, -1, (0, 0, 255), 2)
    if right_contours:
        cv2.drawContours(result, right_contours, -1, (255, 0, 0), 2)

    # debug：被丟棄的浮動線用淡黃色細框標出
    if dropped_floats:
        cv2.drawContours(result, dropped_floats, -1, (0, 255, 255), 1)

    cv2.polylines(result, roi_vertices, isClosed=True, color=(0, 200, 0), thickness=1)

    cv2.putText(result, "Left (Red)", (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
    cv2.putText(result, "Right (Blue)", (w - 200, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)
    cv2.putText(result, f"[v4 ix={ix_thresh} +offset]", (w // 2 - 80, h - 15),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

    # === v4_offset 新增：量測並疊印 ===
    off = compute_lane_offset(left_contours, right_contours, h, w,
                              anchor_ratio=anchor_ratio, band_half=band_half,
                              known_lane_width=known_lane_width)
    result, off_result = draw_lane_offset(result, off)

    return result, off_result["lane_width"], off_result["offset"]


class LaneDetectionNode:
    def __init__(self, args):
        self.args = args
        self.bridge = CvBridge()
        self.tracker = LaneTracker()
        self.known_lane_width = None
        self.roi_mask = None
        self.roi_vertices = None

        rospy.init_node('lane_detection_node', anonymous=True)
        
        # 讀取 video_port 參數，預設 "/dev/ttyUSB1"
        self.video_port = rospy.get_param('~video_port', '/dev/ttyUSB1')
        
        # Publisher for the lane offset and annotated image
        self.offset_pub = rospy.Publisher('lane_detect', Float32, queue_size=1)
        self.image_pub = rospy.Publisher('lane_detect/annotated_image', Image, queue_size=1)
        
        rospy.loginfo(f"Lane detection ROS node started. Capturing from {self.video_port}")
        
        # 初始化 VideoCapture
        # 如果 self.video_port 是全數字，就轉成整數；否則以字串當作裝置路徑
        port = int(self.video_port) if str(self.video_port).isdigit() else self.video_port
        self.cap = cv2.VideoCapture(port)
        
        if not self.cap.isOpened():
            rospy.logerr(f"Failed to open video port: {self.video_port}")

    def process_image(self, cv_image):
        h, w = cv_image.shape[:2]
        
        if self.roi_mask is None:
            self.roi_mask, self.roi_vertices = get_roi_mask(h, w)
            
        result, self.known_lane_width, offset = process_frame(
            cv_image, self.roi_mask, self.roi_vertices, self.tracker, self.args.ix_thresh,
            anchor_ratio=self.args.anchor_ratio,
            band_half=self.args.band_half,
            known_lane_width=self.known_lane_width,
            max_area_thresh=self.args.max_area_thresh
        )
        
        if offset is not None:
            self.offset_pub.publish(Float32(offset))
            
        try:
            ros_result_image = self.bridge.cv2_to_imgmsg(result, "bgr8")
            self.image_pub.publish(ros_result_image)
        except CvBridgeError as e:
            rospy.logerr(e)

    def run(self):
        rate = rospy.Rate(30) # 預設 30 Hz
        while not rospy.is_shutdown():
            if not self.cap.isOpened():
                rate.sleep()
                continue
            
            ret, frame = self.cap.read()
            if not ret:
                rospy.logwarn_throttle(1.0, f"Cannot read frame from {self.video_port}")
                rate.sleep()
                continue
            
            self.process_image(frame)
            rate.sleep()
            
        self.cap.release()


def main():
    parser = argparse.ArgumentParser(description="Lane detection HGL ROS1 Node")
    parser.add_argument("--intersection-thresh", type=float, default=30.0,
                        dest="ix_thresh",
                        help="規則 B 的延伸交點距離閾值 (此版本預設 30)")
    parser.add_argument("--anchor-ratio", type=float, default=0.45,
                        help="anchor 點 y 比例 (0~1，越大越靠下，預設 0.45 = ROI 上邊界)")
    parser.add_argument("--band-half", type=int, default=15,
                        help="量測帶半高 (像素，預設 15)")
    parser.add_argument("--max-area-thresh", type=int, default=20000,
                        help="輪廓最大面積閾值，若大於此值便退回 ROI 裁切模式 (預設 20000)")
    
    # We use parse_known_args in ROS in case ROS passes extra arguments like __name:=...
    args, unknown = parser.parse_known_args()

    node = LaneDetectionNode(args)
    try:
        node.run()
    except KeyboardInterrupt:
        print("Shutting down")
    finally:
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
