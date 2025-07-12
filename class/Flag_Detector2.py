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
    # 三角形
    TRIANGLE_MIN_VERTICES = 3
    TRIANGLE_MAX_VERTICES = 3
    TRIANGLE_EPSILON_FACTOR = 0.04 # 形状近似の許容誤差
    TRIANGLE_MIN_AREA_RATIO = 0.005 # 最小面積比率 (画像全体に対する割合)

    # 長方形
    RECTANGLE_MIN_VERTICES = 4
    RECTANGLE_MAX_VERTICES = 4
    RECTANGLE_EPSILON_FACTOR = 0.04 # 形状近似の許容誤差
    RECTANGLE_ANGLE_TOLERANCE = 10 # 90度からの許容誤差
    RECTANGLE_MIN_ASPECT_RATIO = 0.5 # 縦横比の最小値
    RECTANGLE_MAX_ASPECT_RATIO = 2.0 # 縦横比の最大値
    RECTANGLE_MIN_AREA_RATIO = 0.005 # 最小面積比率

    # T字、十字などの複雑な形状は、さらに詳細な幾何学的判定が必要になります。
    # 例: 面積、外接矩形との比較、重心、凸包など。
    # 現状のコードでは単純な多角形近似のみをベースにしています。

    def __init__(self, picam2_instance): # <--- ここが変わる！picam2_instanceを受け取る
        """
        FlagDetectorのコンストラクタです。
        カメラを初期化し、検出パラメータを設定します。

        Args:
            picam2_instance (Picamera2): 既に初期化され、開始されているPicamera2のインスタンス。
        """
        self.picam2 = picam2_instance # <--- 外部から渡されたインスタンスを使う
        
        # Picamera2の解像度とフレームレートは外部で設定済みという前提
        # もしPicamera2のconfigが変更される可能性があるなら、
        # self.picam2.configure(...) をここでも呼び出すが、
        # メインスクリプトで一元管理している場合は不要。
        # ここでは、既に設定済みであると仮定。
        
        # 画面サイズはインスタンスから取得
        main_stream_config = self.picam2.camera_config['main']
        self.width = main_stream_config['size'][0]
        self.height = main_stream_config['size'][1]
        self.screen_area = self.width * self.height # 画面全体のピクセル数

        print(f"✅ FlagDetector: インスタンス作成完了。カメラ設定は外部で管理されます。")

    def _get_hsv_mask(self, frame_bgr):
        """BGR画像から赤色HSVマスクを生成します。"""
        hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
        mask1 = cv2.inRange(hsv, self.LOWER_RED1, self.UPPER_RED1)
        mask2 = cv2.inRange(hsv, self.LOWER_RED2, self.UPPER_RED2)
        mask = cv2.bitwise_or(mask1, mask2)
        return mask

    def _approximate_shape(self, contour, epsilon_factor):
        """輪郭を近似し、頂点数と形状名を返します。"""
        epsilon = epsilon_factor * cv2.arcLength(contour, True)
        approx = cv2.approxPolyDP(contour, epsilon, True)
        num_vertices = len(approx)
        return approx, num_vertices

    def _is_triangle(self, num_vertices, contour_area, min_area_ratio):
        """三角形であるかを判定します。"""
        return num_vertices == self.TRIANGLE_MIN_VERTICES and \
               (contour_area / self.screen_area) >= min_area_ratio

    def _is_rectangle(self, num_vertices, approx, contour_area, min_area_ratio, angle_tolerance):
        """長方形であるかを判定します。"""
        if num_vertices == self.RECTANGLE_MIN_VERTICES and \
           (contour_area / self.screen_area) >= min_area_ratio:
            
            # 各辺が直交するかを角度で確認
            # 簡略化のため、ここでは外接矩形が四角形であることを確認するだけにします。
            # 厳密な長方形判定には、さらに各角度が90度に近いかのチェックが必要です。
            # (例: approxから辺のベクトルを計算し、内積で角度を確認)
            
            # 外接矩形を計算し、縦横比をチェック
            x, y, w, h = cv2.boundingRect(approx)
            aspect_ratio = float(w) / h if h > 0 else 0
            if self.RECTANGLE_MIN_ASPECT_RATIO <= aspect_ratio <= self.RECTANGLE_MAX_ASPECT_RATIO:
                return True
        return False
        
    def detect(self, save_debug_image=True, debug_image_path="/home/mark1/Pictures/flag_detection_debug.jpg"):
        """
        カメラからフレームを取得し、フラッグを検出します。
        検出されたフラッグ（形状、位置、輪郭）のリストを返します。
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
            
            # T字、十字などの複雑な形状検出ロジックはここに追加

            if current_flag_shapes:
                detected_flags.append({
                    'flag_contour': contour, # 元の輪郭も保存
                    'location': location,
                    'center_x': cx,
                    'center_y': cy,
                    'area': contour_area,
                    'shapes': current_flag_shapes # 検出された形状のリスト
                })
                # デバッグ表示用に輪郭を描画
                cv2.drawContours(debug_frame, [contour], -1, (0, 255, 0), 2) # 緑色で検出された輪郭

        if save_debug_image and detected_flags: # 何か検出された場合のみ保存
            directory = os.path.dirname(debug_image_path)
            if not os.path.exists(directory):
                os.makedirs(directory)
            cv2.imwrite(debug_image_path, debug_frame)
            print(f"FlagDetector: デバッグ画像を保存しました: {debug_image_path}")

        return detected_flags

    def close(self):
        """FlagDetectorのクリーンアップ処理。Picamera2は外部で管理されるためここでは停止しない。"""
        # self.picam2.stop() # Picamera2の停止はメインスクリプトで行う
        print("FlagDetector: クリーンアップ完了。")
