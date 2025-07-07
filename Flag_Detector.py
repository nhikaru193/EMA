import cv2
import numpy as np
from picamera2 import Picamera2
from time import sleep

class FlagDetector:
    """
    カメラ画像から黒い領域（フラッグ）を探し、その中に描かれた
    特定の白い図形（三角形、長方形、T字、十字）を検出するクラス。
    """
    
    def __init__(self, width=640, height=480, min_black_area=2000):
        """
        コンストラクタ（初期化処理）
        """
        # --- 設定をインスタンス変数として保存 ---
        self.width = width
        self.height = height
        self.min_black_area = min_black_area
        
        # --- 検出結果を保持する変数を初期化 ---
        self.last_image = None
        self.detected_flags = []

        # --- カメラの準備 ---
        self.camera = Picamera2()
        config = self.camera.create_still_configuration(main={"size": (self.width, self.height)})
        self.camera.configure(config)
        self.camera.start()
        print("カメラを初期化しました。")
        sleep(2) # カメラの安定化待ち

    def _classify_shape(self, contour):
        """
        輪郭から頂点数と凸性(Solidity)を用いて図形を判別する。
        （クラス内部でのみ使用するため、メソッド名の先頭に'_'を付けています）
        """
        shape_name = "不明"
        epsilon = 0.035 * cv2.arcLength(contour, True)
        approx = cv2.approxPolyDP(contour, epsilon, True)
        vertices = len(approx)

        if cv2.contourArea(contour) < 100:
            return "不明", None
        
        hull = cv2.convexHull(contour)
        solidity = 0
        if cv2.contourArea(hull) > 0:
            solidity = float(cv2.contourArea(contour)) / cv2.contourArea(hull)

        if vertices == 3:
            shape_name = "三角形"
        elif vertices == 4 and solidity > 0.95:
            shape_name = "長方形"
        elif vertices = 8 and solidity < 0.9:
            shape_name = "T字"
        elif vertices = 12 and solidity < 0.75:
            shape_name = "十字"
            
        return shape_name, approx

    def detect(self):
        """
        メインの検出処理。画像を取得し、フラッグと図形を検出。
        """
        # 画像を取得し、インスタンス変数に保存
        self.last_image = self.camera.capture_array()
        if self.last_image is None:
            print("画像が取得できませんでした。")
            return []

        # 検出結果リストをリセット
        self.detected_flags = []
        
        img = self.last_image.copy() # 元画像は変更しないようにコピーして処理

        # --- 1. 黒い領域を特定 ---
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        lower_black = np.array([0, 0, 0])
        upper_black = np.array([180, 255, 60])
        black_mask = cv2.inRange(hsv, lower_black, upper_black)
        kernel = np.ones((5, 5), np.uint8)
        black_mask = cv2.morphologyEx(black_mask, cv2.MORPH_CLOSE, kernel)
        black_contours, _ = cv2.findContours(black_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        valid_black_regions = [c for c in black_contours if cv2.contourArea(c) >= self.min_black_area]

        # --- 2. 各黒領域内で図形を探す ---
        for region_contour in valid_black_regions:
            x, y, w, h = cv2.boundingRect(region_contour)
            roi_img = img[y:y+h, x:x+w]
            
            gray_roi = cv2.cvtColor(roi_img, cv2.COLOR_BGR2GRAY)
            _, binary_roi = cv2.threshold(gray_roi, 100, 255, cv2.THRESH_BINARY)
            
            contours_in_roi, _ = cv2.findContours(binary_roi, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            shapes_in_flag = []
            for cnt in contours_in_roi:
                shape_name, approx = self._classify_shape(cnt)
                if shape_name != "不明" and approx is not None:
                    # 輪郭の座標を全体座標に変換
                    approx_global = approx + (x, y)
                    # 図形の情報を辞書として保存
                    M = cv2.moments(approx_global)
                    cx = int(M["m10"] / M["m00"]) if M["m00"] != 0 else 0
                    cy = int(M["m01"] / M["m00"]) if M["m00"] != 0 else 0
                    shapes_in_flag.append({
                        "name": shape_name,
                        "contour": approx_global,
                        "center": (cx, cy)
                    })
            
            # 黒領域（フラッグ）とその中の図形の情報を保存
            if shapes_in_flag:
                self.detected_flags.append({
                    "flag_contour": region_contour,
                    "shapes": shapes_in_flag
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
            # 黒領域（フラッグ）の外枠を描画
            cv2.drawContours(img, [flag_info["flag_contour"]], -1, (255, 0, 0), 3)
            
            # フラッグ内の各図形を描画
            for shape_info in flag_info["shapes"]:
                cv2.drawContours(img, [shape_info["contour"]], -1, (0, 255, 0), 2)
                
                bx, by, _, _ = cv2.boundingRect(shape_info["contour"])
                cv2.putText(img, shape_info["name"], (bx, by - 10),
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
    
    # 1. FlagDetectorクラスのインスタンスを作成
    detector = FlagDetector()

    try:
        # 2. 検出処理を実行
        # detect()メソッドが検出結果のリストを返す
        detected_data = detector.detect()

        # 3. 検出結果をターミナルに表示
        if detected_data:
            print("\n--- 検出結果詳細 ---")
            for i, flag in enumerate(detected_data):
                shape_names = [s["name"] for s in flag["shapes"]]
                print(f"フラッグ {i+1}: {', '.join(shape_names)}")
        else:
            print("フラッグが見つかりませんでした。")

        # 4. 元画像に検出結果を描画
        if detector.last_image is not None:
            result_image = detector.draw_results(detector.last_image)
            
            # 5. 画像を表示
            cv2.imshow("Detected Shapes", result_image)
            cv2.waitKey(0)
    
    finally:
        # 6. 終了処理
        detector.close()
        cv2.destroyAllWindows()
