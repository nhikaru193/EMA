import cv2
import numpy as np
import math
from picamera2 import Picamera2
from time import sleep
from typing import Tuple, List, Dict, Optional, Any

class Flag_B:
    """
    カメラ画像から指定された色の領域を探し、その中に描かれた
    反対色（黒または白）の特定の図形を検出・識別するクラス。
    """
    # --- 形状分類パラメータ (変更なし) ---
    SHAPE_APPROX_EPSILON = 0.03
    MIN_SHAPE_AREA = 100
    RECTANGLE_MIN_SOLIDITY = 0.95
    T_SHAPE_VERTEX_RANGE = (7, 9)
    T_SHAPE_MAX_SOLIDITY = 0.9
    CROSS_SHAPE_VERTEX_RANGE = (11, 13)
    CROSS_SHAPE_MAX_SOLIDITY = 0.75

    LOWER_OUTER_AREA_HSV = np.array([0, 0, 20]) # 120より大きくすれば、より白くなる
    UPPER_OUTER_AREA_HSV = np.array([255, 50, 255])
    
    # 領域内の「黒い図形」を抽出するための二値化しきい値
    INNER_SHAPE_BINARY_THRESHOLD = 45 # 80より小さくすると、より黒くなる
    MORPHOLOGY_KERNEL_SIZE = (5, 5)

    def __init__(self, width: int = 640, height: int = 480, min_outer_area: int = 1000, triangle_tolerance: float = 5.0):
        """
        コンストラクタ
        Args:
            min_outer_area (int): 検出対象とする外側領域の最小面積
            triangle_tolerance (float): 正三角形と見なす重心と垂心の距離の許容誤差(ピクセル)
        """
        self.width = width
        self.height = height
        self.min_outer_area = min_outer_area
        self.triangle_tolerance = triangle_tolerance

        self.last_image: Optional[np.ndarray] = None
        self.detected_areas: List[Dict[str, Any]] = []

        self.camera = Picamera2()
        config = self.camera.create_still_configuration(main={"size": (self.width, self.height)})
        self.camera.configure(config)
        self.camera.start()
        print("カメラを初期化しました。")
        sleep(2)

    def _calculate_distance(self, p1: Tuple[float, float], p2: Tuple[float, float]) -> float:
        """2点間のユークリッド距離を計算する"""
        return math.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)

    def _calculate_centroid(self, points: np.ndarray) -> np.ndarray:
        """図形の重心（頂点座標の算術平均）を計算する"""
        return np.mean(points, axis=0)

    def _calculate_orthocenter(self, points: np.ndarray) -> Optional[np.ndarray]:
        """三角形の垂心を計算する"""
        if len(points) != 3:
            return None
        (x1, y1), (x2, y2), (x3, y3) = points.astype(float)
        A = np.array([[x3 - x2, y3 - y2], [x1 - x3, y1 - y3]])
        B = np.array([x1 * (x3 - x2) + y1 * (y3 - y2), x2 * (x1 - x3) + y2 * (y1 - y3)])
        if np.linalg.det(A) == 0:
            return None
        try:
            return np.linalg.solve(A, B)
        except np.linalg.LinAlgError:
            return None

    def _classify_shape(self, contour: np.ndarray) -> Tuple[str, Optional[np.ndarray]]:
        """輪郭から頂点数や幾何学的特徴を用いて図形を判別する"""
        if cv2.contourArea(contour) < self.MIN_SHAPE_AREA:
            return "不明", None
        shape_name = "不明"
        epsilon = self.SHAPE_APPROX_EPSILON * cv2.arcLength(contour, True)
        approx = cv2.approxPolyDP(contour, epsilon, True)
        vertices = len(approx)
        hull = cv2.convexHull(contour)
        hull_area = cv2.contourArea(hull)
        solidity = cv2.contourArea(contour) / hull_area if hull_area > 0 else 0
        points = np.squeeze(approx)
        if vertices == 3:
            centroid = self._calculate_centroid(points)
            orthocenter = self._calculate_orthocenter(points)
            if centroid is not None and orthocenter is not None:
                distance = self._calculate_distance(centroid, orthocenter)
                if distance < self.triangle_tolerance:
                    shape_name = "正三角形"
                else:
                    shape_name = "三角形"
            else:
                shape_name = "三角形"
        elif vertices == 4 and solidity > self.RECTANGLE_MIN_SOLIDITY:
            shape_name = "長方形"
        elif self.T_SHAPE_VERTEX_RANGE[0] <= vertices <= self.T_SHAPE_VERTEX_RANGE[1] and solidity < self.T_SHAPE_MAX_SOLIDITY:
            shape_name = "T字"
        elif self.CROSS_SHAPE_VERTEX_RANGE[0] <= vertices <= self.CROSS_SHAPE_VERTEX_RANGE[1] and solidity < self.CROSS_SHAPE_MAX_SOLIDITY:
            shape_name = "十字"
        return shape_name, approx

    def detect(self) -> List[Dict[str, Any]]:
        """
        メインの検出処理。画像を取得し、白領域、黒図形、位置を検出する。
        """
        self.last_image = self.camera.capture_array()
        self.last_image = cv2.rotate(self.last_image, cv2.ROTATE_90_COUNTERCLOCKWISE)

        if self.last_image is None:
            print("エラー: 画像が取得できませんでした。")
            return []

        self.detected_areas = []
        img_bgr = cv2.cvtColor(self.last_image, cv2.COLOR_RGB2BGR)

        # --- 1. 白い領域を特定 ---
        hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)
        # 変更: 白色のHSV範囲でマスクを生成
        outer_area_mask = cv2.inRange(hsv, self.LOWER_OUTER_AREA_HSV, self.UPPER_OUTER_AREA_HSV)
        kernel = np.ones(self.MORPHOLOGY_KERNEL_SIZE, np.uint8)
        outer_area_mask = cv2.morphologyEx(outer_area_mask, cv2.MORPH_CLOSE, kernel)
        outer_area_contours, _ = cv2.findContours(outer_area_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        valid_outer_regions = [c for c in outer_area_contours if cv2.contourArea(c) > self.min_outer_area]

        # --- 2. 各白領域内で黒い図形を探す ---
        for region_contour in valid_outer_regions:
            x, y, w, h = cv2.boundingRect(region_contour)
            roi_bgr = img_bgr[y:y+h, x:x+w]
            gray_roi = cv2.cvtColor(roi_bgr, cv2.COLOR_BGR2GRAY)

            #閾値より"暗い"部分を白(255)にするため、THRESH_BINARY_INV を使用
            _, binary_roi_inv = cv2.threshold(
                gray_roi, self.INNER_SHAPE_BINARY_THRESHOLD, 255, cv2.THRESH_BINARY_INV
            )

            # --- 白領域の形にマスクして、領域外の黒いノイズを除去 ---
            mask = np.zeros(roi_bgr.shape[:2], dtype="uint8")
            shifted_contour = region_contour - (x, y)
            cv2.drawContours(mask, [shifted_contour], -1, 255, -1)
            masked_binary_roi = cv2.bitwise_and(binary_roi_inv, binary_roi_inv, mask=mask)

            # マスク処理された画像から黒図形の輪郭を探す
            contours_in_roi, _ = cv2.findContours(masked_binary_roi, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            shapes_in_area = [] # 変更: 変数名
            for cnt in contours_in_roi:
                shape_name, approx = self._classify_shape(cnt)
                if shape_name != "不明" and approx is not None:
                    approx_global = approx + (x, y)
                    M_shape = cv2.moments(approx_global)
                    if M_shape["m00"] > 0:
                        cx_shape = int(M_shape["m10"] / M_shape["m00"])
                        cy_shape = int(M_shape["m01"] / M_shape["m00"])
                        shapes_in_area.append({
                            "name": shape_name,
                            "contour": approx_global,
                            "center": (cx_shape, cy_shape)
                        })
            
            if shapes_in_area:
                M_area = cv2.moments(region_contour) # 変更: 変数名
                area_cx = int(M_area["m10"] / M_area["m00"]) if M_area["m00"] > 0 else 0

                location = "中央"
                if area_cx < self.width / 3:
                    location = "左"
                elif area_cx > self.width * 2 / 3:
                    location = "右"

                self.detected_areas.append({
                    "area_contour": region_contour, # 変更: キー名
                    "shapes": shapes_in_area,
                    "location": location
                })
        
        print(f"{len(self.detected_areas)}個の白領域を検出し、{sum(len(f['shapes']) for f in self.detected_areas)}個の黒い図形を見つけました。")
        return self.detected_areas

    def draw_results(self, image_to_draw: np.ndarray) -> np.ndarray:
        """検出結果を画像に描画する。"""
        if not self.detected_areas:
            return image_to_draw

        img_with_results = image_to_draw.copy()
        for area_info in self.detected_areas:
            # 白領域の輪郭を描画 (青色)
            cv2.drawContours(img_with_results, [area_info["area_contour"]], -1, (255, 0, 0), 3)
            
            for shape_info in area_info["shapes"]:
                # 黒図形の輪郭を描画 (緑色)
                cv2.drawContours(img_with_results, [shape_info["contour"]], -1, (0, 255, 0), 2)
                
                bx, by, _, _ = cv2.boundingRect(shape_info["contour"])
                label = f"{shape_info['name']} ({area_info['location']})"
                cv2.putText(img_with_results, label, (bx, by - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        return img_with_results
        
    def close(self):
        """カメラリソースを解放する。"""
        self.camera.close()
        print("カメラを解放しました。")


if __name__ == '__main__':
    # 白い紙の上の黒い図形などを検出する場合
    detector = Flag_B(triangle_tolerance=5.0)

    try:
        detected_data = detector.detect()

        if detected_data:
            print("\n--- 検出結果詳細 ---")
            for i, area in enumerate(detected_data, 1):
                shape_names = [s["name"] for s in area["shapes"]]
                print(f"白領域 {i}: 位置={area['location']}, 黒図形={', '.join(shape_names)}")
        else:
            print("対象の領域または図形が見つかりませんでした。")

        if detector.last_image is not None:
            result_image = detector.draw_results(detector.last_image)
            display_image = cv2.cvtColor(result_image, cv2.COLOR_RGB2BGR)
            cv2.imshow("Detected Shapes", display_image)
            cv2.waitKey(0)
    
    finally:
        detector.close()
        #cv2.destroyAllWindows()
