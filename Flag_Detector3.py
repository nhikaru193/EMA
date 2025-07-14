import cv2
import numpy as np
import math # mathライブラリをインポート
from picamera2 import Picamera2
from time import sleep

class FlagDetector:
    """
    カメラ画像から黒い領域（フラッグ）を探し、その中に描かれた
    特定の白い図形を検出し、フラッグの位置を判定するクラス。
    重心・垂心の一致度を利用して、より高精度に図形を識別する。
    """

    def __init__(self, width=640, height=480, min_black_area=1000, triangle_tolerance=0.5):
        """
        コンストラクタ（初期化処理）
        Args:
            triangle_tolerance (float):
                三角形の重心と垂心が「一致する」と見なす距離の許容誤差。
                この値が小さいほど厳密な正三角形を、大きいほど幅広い三角形を正三角形として認識する。
        """
        # --- 設定をインスタンス変数として保存 ---
        self.width = width
        self.height = height
        self.min_black_area = min_black_area
        self.triangle_tolerance = triangle_tolerance  # 変更した

        # --- 検出結果を保持する変数を初期化 ---
        self.last_image = None
        self.detected_flags = []

        # --- カメラの準備 ---
        self.camera = Picamera2()
        config = self.camera.create_still_configuration(main={"size": (self.width, self.height)})
        self.camera.configure(config)
        self.camera.start()
        print("カメラを初期化しました。")
        sleep(2)

    # 垂心・重心について

    def _calculate_distance(self, p1, p2):
        """2点間の距離を計算する"""
        return math.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)

    def _calculate_centroid(self, points):
        """図形の重心（頂点座標の平均）を計算する"""
        return np.mean(points, axis=0)

    def _calculate_orthocenter(self, points):
        """三角形の垂心を計算する"""
        if len(points) != 3:
            return None
        (x1, y1), (x2, y2), (x3, y3) = points
        A = np.array([[x3 - x2, y3 - y2], [x1 - x3, y1 - y3]])
        B = np.array([x1 * (x3 - x2) + y1 * (y3 - y2), x2 * (x1 - x3) + y2 * (y1 - y3)])
        try:
            return np.linalg.solve(A, B)
        except np.linalg.LinAlgError:
            return None

    def _classify_shape(self, contour):
        """
        輪郭から頂点数や重心・垂心の一致度を用いて図形を判別する。
        """
        shape_name = "不明"
        epsilon = 0.04 * cv2.arcLength(contour, True)
        approx = cv2.approxPolyDP(contour, epsilon, True)
        vertices = len(approx)

        # 小さすぎる輪郭はノイズとして除外
        if cv2.contourArea(contour) < 10:
            return "不明", None

        # 凸性(Solidity)の計算
        hull = cv2.convexHull(contour)
        solidity = float(cv2.contourArea(contour)) / cv2.contourArea(hull) if cv2.contourArea(hull) > 0 else 0
        
        # 頂点座標の配列を (N, 2) の形式に整形
        points = np.squeeze(approx)

        if vertices == 3:
            centroid = self._calculate_centroid(points)
            orthocenter = self._calculate_orthocenter(points)

            if centroid is not None and orthocenter is not None:
                distance = self._calculate_distance(centroid, orthocenter)
                # 重心と垂心の距離が許容誤差内かチェック
                if distance < self.triangle_tolerance:
                    shape_name = "正三角形"
                else:
                    shape_name = "三角形"
            else:
                shape_name = "三角形" # 計算エラー時は通常の三角形とする

        elif vertices == 4 and solidity > 0.95:
            shape_name = "長方形"

        # 以前のT字・十字のロジックも残す
        elif vertices >= 7 and vertices <= 9 and solidity < 0.9:
            shape_name = "T字"
            
        elif vertices >= 10 and vertices <= 13 and solidity < 0.75:
            shape_name = "十字"

        return shape_name, approx

    def detect(self):
        """
        メインの検出処理。画像を取得し、フラッグ、図形、位置を検出する。
        """
        self.last_image = self.camera.capture_array()
        self.last_image = cv2.rotate(self.last_image, cv2.ROTATE_90_COUNTERCLOCKWISE)
        if self.last_image is None:
            print("画像が取得できませんでした。")
            return []

        self.detected_flags = []
        img = self.last_image.copy()

        # --- 1. 黒い領域を特定 ---
        hsv = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
        lower_black = np.array([0, 0, 0])
        upper_black = np.array([180, 255, 60]) # この値は環境に応じて調整　60とかにしたら明度上がって、背景の森も読み取ってしまう
        black_mask = cv2.inRange(hsv, lower_black, upper_black)
        kernel = np.ones((5, 5), np.uint8)
        black_mask = cv2.morphologyEx(black_mask, cv2.MORPH_CLOSE, kernel)
        black_contours, _ = cv2.findContours(black_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        valid_black_regions = []
        for c in black_contours:
            if cv2.contourArea(c) > self.min_black_area:
                valid_black_regions.append(c)

        # --- 2. 各黒領域内で図形を探し、位置を特定 ---
        for region_contour in valid_black_regions:
            x, y, w, h = cv2.boundingRect(region_contour)
            roi_img = img[y:y+h, x:x+w]

            gray_roi = cv2.cvtColor(roi_img, cv2.COLOR_RGB2GRAY)
            _, binary_roi = cv2.threshold(gray_roi, 100, 255, cv2.THRESH_BINARY) #80にすると、グレーを読み取る。一方で120などにすると、黒領域の白飛び部分を検出するとがある
            
            contours_in_roi, _ = cv2.findContours(binary_roi, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            shapes_in_flag = []
            for cnt in contours_in_roi:
                shape_name, approx = self._classify_shape(cnt)
                if shape_name != "不明" and approx is not None:
                    approx_global = approx + (x, y)
                    M_shape = cv2.moments(approx_global)
                    if M_shape["m00"] != 0:
                        cx_shape = int(M_shape["m10"] / M_shape["m00"])
                        cy_shape = int(M_shape["m01"] / M_shape["m00"])
                        shapes_in_flag.append({
                            "name": shape_name,
                            "contour": approx_global,
                            "center": (cx_shape, cy_shape)
                        })
            
            if shapes_in_flag:
                M_flag = cv2.moments(region_contour)
                flag_cx = int(M_flag["m10"] / M_flag["m00"]) if M_flag["m00"] != 0 else 0

                location = ""
                if flag_cx < self.width / 3:
                    location = "左"
                elif flag_cx < self.width * 2 / 3:
                    location = "中央"
                else:
                    location = "右"

                self.detected_flags.append({
                    "flag_contour": region_contour,
                    "shapes": shapes_in_flag,
                    "location": location
                })
        
        print(f"{len(self.detected_flags)}個のフラッグを検出し、{sum(len(f['shapes']) for f in self.detected_flags)}個の図形を見つけました。")
        return self.detected_flags

    def draw_results(self, image_to_draw):
        """
        検出結果を渡された画像に描画する。
        """
        if not self.detected_flags:
            return image_to_draw

        img = image_to_draw.copy()
        for flag_info in self.detected_flags:
            cv2.drawContours(img, [flag_info["flag_contour"]], -1, (255, 0, 0), 3)
            
            for shape_info in flag_info["shapes"]:
                cv2.drawContours(img, [shape_info["contour"]], -1, (0, 255, 0), 2)
                
                bx, by, _, _ = cv2.boundingRect(shape_info["contour"])
                label = f"{shape_info['name']} ({flag_info['location']})"
                cv2.putText(img, label, (bx, by - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        return img
        
    def close(self):
        """
        カメラリソースを解放する。
        """
        self.camera.close()
        print("カメラを解放しました。")

# --- クラスの使い方 ---
if __name__ == '__main__':
    # 許容誤差を調整したい場合は、ここで値を設定できます
    # 例: detector = FlagDetector(triangle_tolerance=0.8)
    detector = FlagDetector(triangle_tolerance=0.5)

    try:
        detected_data = detector.detect()

        if detected_data:
            print("\n--- 検出結果詳細 ---")
            for i, flag in enumerate(detected_data):
                # 検出された図形の名前リストを作成（正三角形も区別される）
                shape_names = [s["name"] for s in flag["shapes"]]
                print(f"フラッグ {i+1}: 位置={flag['location']}, 図形={', '.join(shape_names)}")
        else:
            print("フラッグが見つかりませんでした。")

        if detector.last_image is not None:
            result_image = detector.draw_results(detector.last_image)
            
            display_image = cv2.cvtColor(result_image, cv2.COLOR_RGB2BGR)
            cv2.imshow("Detected Shapes", display_image)
            cv2.waitKey(0)
    
    finally:
        detector.close()
        cv2.destroyAllWindows()
