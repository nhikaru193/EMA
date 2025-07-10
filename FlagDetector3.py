# Flag_Detector.py (または FlagDetector クラスが定義されているファイル)

import cv2
import numpy as np
from picamera2 import Picamera2
import time

class FlagDetector:
    def __init__(self, width=640, height=480, min_black_area=2000, min_area_percent=5.0):
        """
        コンストラクタ（初期化処理）
        """
        # --- 設定をインスタンス変数として保存 ---
        self.width = width
        self.height = height
        self.min_black_area = min_black_area
        self.min_area_percent = min_area_percent # 新しい設定値を追加
        self.screen_area = self.width * self.height # 画面全体のピクセル面積を計算

        # --- 検出結果を保持する変数を初期化 ---
        self.last_image = None
        self.detected_flags = []

        # --- カメラの準備 ---
        self.camera = Picamera2()
        config = self.camera.create_still_configuration(main={"size": (self.width, self.height)})
        self.camera.configure(config)
        self.camera.start()
        print("カメラを初期化しました。")
        time.sleep(2) # カメラの安定化待ち

    def detect(self):
        """
        画像をキャプチャし、黒い旗の図形を検出する
        """
        # カメラから画像を取得
        image = self.camera.capture_array()
        if image is None:
            print("画像キャプチャに失敗しました。")
            return []

        # BGRからグレースケールに変換
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        # --- 画像の前処理と二値化の強化 ---
        # 影の影響を減らすための適応的しきい値処理
        # blockSize: 周囲の平均を計算する領域のサイズ (奇数)
        # C: 計算された平均から引かれる定数。小さすぎるとノイズを拾いやすく、大きすぎると図形が欠ける
        binary = cv2.adaptiveThreshold(gray, 255,
                                       cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                                       cv2.THRESH_BINARY_INV, 21, 5) # ここを調整してみる (21, 5は一例)

        # ノイズ除去（オープニング処理）：小さな白い点（ノイズ）を除去
        kernel = np.ones((3,3),np.uint8) # カーネルサイズを調整 (3,3は一例)
        binary = cv2.morphologyEx(binary, cv2.MORPH_OPEN, kernel)

        # クロージング処理：内部の小さな穴を埋める
        kernel_close = np.ones((5,5),np.uint8) # カーネルサイズを調整 (5,5は一例)
        binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel_close)

        # 輪郭を検出
        contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        detected_flags = []

        for contour in contours:
            area = cv2.contourArea(contour)

            # --- 既存の最小ピクセル面積によるフィルタリング ---
            if area < self.min_black_area:
                continue

            # --- 新しく追加する画面占有率によるフィルタリング ---
            area_percent = (area / self.screen_area) * 100
            if area_percent < self.min_area_percent: # ここで5%未満の図形を無視
                print(f"小さい図形をスキップしました (面積: {area_percent:.1f}%)")
                continue

            # 外接矩形と重心を計算
            x, y, w, h = cv2.boundingRect(contour)
            M = cv2.moments(contour)
            if M["m00"] == 0:
                continue # 面積がゼロの輪郭はスキップ

            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])

            # 図形（shape）の認識ロジック（ここに追加・変更）
            # ここはあなたの既存の図形認識ロジックが入ります。
            # 例:
            shapes = []
            approx = cv2.approxPolyDP(contour, 0.04 * cv2.arcLength(contour, True), True)
            num_vertices = len(approx)

            # --- 図形の判別ロジック ---
            shape_name = "不明"
            if num_vertices == 3:
                shape_name = "三角形"
            elif num_vertices == 4:
                # 矩形（長方形、正方形）の判定をさらに精密にする
                # 側面間の角度や縦横比を考慮する
                aspect_ratio = float(w)/h
                if 0.8 < aspect_ratio < 1.2: # ある程度の範囲で正方形とみなす
                     shape_name = "正方形"
                else:
                     shape_name = "長方形"
            # T字や十字などの複雑な形状は、さらに別のロジック（例: 形状マッチング、骨格化）が必要
            # ここは既存のコードのロジックに合わせてください

            # 旗の位置（左右中央）を判断
            location = ""
            if cx < self.width * 0.33:
                location = "左"
            elif cx > self.width * 0.67:
                location = "右"
            else:
                location = "中央"

            # 検出結果をリストに追加
            detected_flags.append({
                'flag_contour': contour,
                'bounding_box': (x, y, w, h),
                'center': (cx, cy),
                'area': area,
                'area_percent': area_percent, # 新しく追加
                'location': location,
                'shapes': [{'name': shape_name, 'vertices': num_vertices}] # 検出された形状情報
            })

        self.detected_flags = detected_flags
        return detected_flags

    def close(self):
        """
        カメラリソースを解放する
        """
        self.camera.stop()
        print("カメラを停止しました。")
