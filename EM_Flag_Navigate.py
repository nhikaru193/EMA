import cv2
import numpy as np
from picamera2 import Picamera2
from time import sleep

def classify_by_vertex_count(contour):
    epsilon = 0.02 * cv2.arcLength(contour, True)
    approx = cv2.approxPolyDP(contour, epsilon, True)
    vertices = len(approx)
    shape_name = "多角形"
    if vertices == 3:
        shape_name = "三角形"
    elif vertices == 4:
        shape_name = "長方形"
    elif vertices == 8:
        shape_name = "T字"
    elif vertices == 12:
        shape_name = "十字"
    elif vertices == 16:
        shape_name = "E字"
    return shape_name, approx

def detect_shapes_by_region(show_debug=False):
    # カメラ初期化
    picam2 = Picamera2()
    config = picam2.create_still_configuration(main={"size": (320, 240)})
    picam2.configure(config)
    picam2.start()
    sleep(2)
    img = picam2.capture_array()
    picam2.close()

    if img is None:
        raise ValueError("画像の取得に失敗しました")

    height, width = img.shape[:2]
    grid_cols = 5
    block_w = width // grid_cols
    block_names = {0: "すごく左", 1: "左", 2: "中央", 3: "右", 4: "すごく右"}
    blocks_shapes = {i: [] for i in range(grid_cols)}

    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    lower_black = np.array([0, 0, 0])
    upper_black = np.array([180, 50, 50])
    black_mask = cv2.inRange(hsv, lower_black, upper_black)
    kernel = np.ones((5,5), np.uint8)
    black_mask = cv2.morphologyEx(black_mask, cv2.MORPH_CLOSE, kernel)
    black_mask_blur = cv2.GaussianBlur(black_mask, (5,5), 0)

    min_black_area = 5000
    contours, _ = cv2.findContours(black_mask_blur, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    valid_regions = [c for c in contours if cv2.contourArea(c) >= min_black_area]

    for region in valid_regions:
        mask_roi = np.zeros((height, width), dtype=np.uint8)
        cv2.drawContours(mask_roi, [region], -1, 255, thickness=cv2.FILLED)
        x, y, w, h = cv2.boundingRect(region)
        roi_mask = mask_roi[y:y+h, x:x+w]
        roi_img = img[y:y+h, x:x+w]
        gray_roi = cv2.cvtColor(roi_img, cv2.COLOR_BGR2GRAY)
        masked_gray = cv2.bitwise_and(gray_roi, gray_roi, mask=roi_mask)
        _, binary_roi = cv2.threshold(masked_gray, 50, 255, cv2.THRESH_BINARY)
        roi_contours, _ = cv2.findContours(binary_roi, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for cnt in roi_contours:
            shape_name, approx = classify_by_vertex_count(cnt)
            if len(approx) < 3:
                continue
            M = cv2.moments(cnt)
            if M["m00"] == 0:
                continue
            cx = int(M["m10"] / M["m00"])
            global_cx = x + cx
            block_idx = min(global_cx // block_w, grid_cols - 1)
            blocks_shapes[block_idx].append(shape_name)

    # 結果まとめ
    result = {}
    for i in range(grid_cols):
        region_name = block_names[i]
        shapes = blocks_shapes[i]
        if not shapes:
            result[region_name] = {}
        else:
            counts = {}
            for s in shapes:
                counts[s] = counts.get(s, 0) + 1
            result[region_name] = counts

    if show_debug:
        for region, shapes in result.items():
            if not shapes:
                print(f"{region}: 図形なし")
            else:
                print(f"{region}: " + ", ".join([f"{k}: {v}" for k, v in shapes.items()]))

    return result

while true:
	
