import cv2
import numpy as np
import time
from picamera2 import Picamera2 # Picamera2をインポート（外部から受け取るため）

class FlagDetector:
    """
    カメラ画像からフラッグ（特定の形状と色）を検出し、その位置情報を提供するクラス。
    """
    # クラス定数 (必要に応じて調整)
    WIDTH = 640
    HEIGHT = 480
    FRAMERATE = 30 # このクラスで使用するフレームレート

    # HSV 色範囲 (例: 赤色)
    LOWER_RED1 = np.array([0, 100, 100])
    UPPER_RED1 = np.array([10, 255, 255])
    LOWER_RED2 = np.array([160, 100, 100])
    UPPER_RED2 = np.array([180, 255, 255])

    # 形状判定用の閾値 (必要に応じて調整)
    TRIANGLE_MIN_VERTICES = 3
    TRIANGLE_MAX_VERTICES = 3
    TRIANGLE_EPSILON_FACTOR = 0.04
    TRIANGLE_MIN_AREA_RATIO = 0.005

    RECTANGLE_MIN_VERTICES = 4
    RECTANGLE_MAX_VERTICES = 4
    RECTANGLE_EPSILON_FACTOR = 0.04
    RECTANGLE_ANGLE_TOLERANCE = 10
    RECTANGLE_MIN_ASPECT_RATIO = 0.5
    RECTANGLE_MAX_ASPECT_RATIO = 2.0
    RECTANGLE_MIN_AREA_RATIO = 0.005

    def __init__(self, picam2_instance): # <--- ここが変わる！picam2_instanceを受け取る
        """
        FlagDetectorのコンストラクタです。
        カメラを初期化し、検出パラメータを設定します。

        Args:
            picam2_instance (Picamera2): 既に初期化され、開始されているPicamera2のインスタンス。
        """
        self.picam2 = picam2_instance # <--- 外部から渡されたインスタンスを使う
        
        # Picamera2の解像度とフレームレートは外部で設定済みという前提
        main_stream_config = self.picam2.camera_config['main']
        self.width = main_stream_config['size'][0]
        self.height = main_stream_config['size'][1]
        self.screen_area = self.width * self.height # 画面全体のピクセル数

        print(f"✅ FlagDetector: インスタンス作成完了。カメラ設定は外部で管理されます。")

    def _get_hsv_mask(self, frame_bgr):
        # ... (変更なし) ...

    def _approximate_shape(self, contour, epsilon_factor):
        # ... (変更なし) ...

    def _is_triangle(self, num_vertices, contour_area, min_area_ratio):
        # ... (変更なし) ...

    def _is_rectangle(self, num_vertices, approx, contour_area, min_area_ratio, angle_tolerance):
        # ... (変更なし) ...
        
    def detect(self, save_debug_image=True, debug_image_path="/home/mark1/Pictures/flag_detection_debug.jpg"):
        """
        カメラからフレームを取得し、フラッグを検出します。
        """
        frame_rgb = self.picam2.capture_array() # Picamera2はデフォルトでRGB形式のNumPy配列を返す
        if frame_rgb is None:
            print("FlagDetector: 画像キャプチャ失敗: フレームがNoneです。")
            return []

        frame_bgr = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)
        blurred_frame = cv2.GaussianBlur(frame_bgr, (5, 5), 0)
        red_mask = self._get_hsv_mask(blurred_frame)

        # 輪郭検出
        contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        detected_flags = []
        debug_frame = frame_bgr.copy() # デバッグ表示用

        for contour in contours:
            contour_area = cv2.contourArea(contour)
            if contour_area < 50: # 小さすぎる輪郭は無視
                continue

            # 輪郭の中心座標
            M = cv2.moments(contour)
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
            else:
                continue

            # 画面内の位置判定
            location = "不明"
            if cx < self.width / 3:
                location = "左"
            elif cx > 2 * self.width / 3:
                location = "右"
            else:
                location = "中央"

            # 形状近似と判定
            approx_triangle, num_vertices_triangle = self._approximate_shape(contour, self.TRIANGLE_EPSILON_FACTOR)
            approx_rectangle, num_vertices_rectangle = self._approximate_shape(contour, self.RECTANGLE_EPSILON_FACTOR)

            current_flag_shapes = []
            if self._is_triangle(num_vertices_triangle, contour_area, self.TRIANGLE_MIN_AREA_RATIO):
                current_flag_shapes.append({'name': '三角形', 'approx_contour': approx_triangle})
            if self._is_rectangle(num_vertices_rectangle, approx_rectangle, contour_area, self.RECTANGLE_MIN_AREA_RATIO, self.RECTANGLE_ANGLE_TOLERANCE):
                current_flag_shapes.append({'name': '長方形', 'approx_contour': approx_rectangle})
            
            if current_flag_shapes:
                detected_flags.append({
                    'flag_contour': contour,
                    'location': location,
                    'center_x': cx,
                    'center_y': cy,
                    'area': contour_area,
                    'shapes': current_flag_shapes
                })
                cv2.drawContours(debug_frame, [contour], -1, (0, 255, 0), 2)

        if save_debug_image and detected_flags:
            directory = os.path.dirname(debug_image_path)
            if not os.path.exists(directory):
                os.makedirs(directory)
            cv2.imwrite(debug_image_path, debug_frame)
            print(f"FlagDetector: デバッグ画像を保存しました: {debug_image_path}")

        return detected_flags

    def close(self):
        """FlagDetectorのクリーンアップ処理。Picamera2は外部で管理されるためここでは停止しない。"""
        print("FlagDetector: クリーンアップ完了。")
