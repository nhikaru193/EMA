import cv2
import numpy as np
from picamera2 import Picamera2
from time import sleep
from motor import MotorDriver
import RPi.GPIO as GPIO
import time # MotorDriver内でtime.sleepが使われているため、念のためインポート

# --- 図形分類関数 (変更なし) ---
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
    return shape_name, approx

# --- 図形検出関数 (面積フィルタリングを追加) ---
def detect_shapes_by_region(show_debug=False):
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
    grid_cols = 3
    block_w = width // grid_cols
    block_names = {0: "左", 1: "中央", 2: "右"}
    blocks_shapes = {i: [] for i in range(grid_cols)} # ここに最終的な図形を格納

    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    lower_black = np.array([0, 0, 0])
    upper_black = np.array([180, 50, 50]) # 明度(V)を調整して影の影響を減らすことも検討 (例: 50 -> 30)
    black_mask = cv2.inRange(hsv, lower_black, upper_black)
    kernel = np.ones((5,5), np.uint8)
    black_mask = cv2.morphologyEx(black_mask, cv2.MORPH_CLOSE, kernel)
    black_mask_blur = cv2.GaussianBlur(black_mask, (5,5), 0)

    min_black_area = 5000 # 黒い領域（塊）の最小面積
    contours, _ = cv2.findContours(black_mask_blur, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    valid_regions = [c for c in contours if cv2.contourArea(c) >= min_black_area]

    for region in valid_regions:
        mask_roi = np.zeros((height, width), dtype=np.uint8)
        cv2.drawContours(mask_roi, [region], -1, 255, thickness=cv2.FILLED)
        x, y, w, h = cv2.boundingRect(region)
        roi_mask = mask_roi[y:y+h, x:x+w]
        roi_img = img[y:y+h, x:x+h] # ここも元のコードの誤字を修正しました y:y+h, x:x+w
        gray_roi = cv2.cvtColor(roi_img, cv2.COLOR_BGR2GRAY)
        masked_gray = cv2.bitwise_and(gray_roi, gray_roi, mask=roi_mask)
        
        # 適応的しきい値処理を検討する場合
        # _, binary_roi = cv2.threshold(masked_gray, 50, 255, cv2.THRESH_BINARY)
        binary_roi = cv2.adaptiveThreshold(masked_gray, 255, 
                                           cv2.ADAPTIVE_THRESH_GAUSSIAN_C, # または cv2.ADAPTIVE_THRESH_MEAN_C
                                           cv2.THRESH_BINARY_INV, # 背景が白、物体が黒になるように反転
                                           11, # ブロックサイズ (奇数、調整が必要)
                                           2) # C (定数、調整が必要)

        roi_contours, _ = cv2.findContours(binary_roi, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # 各ROI（黒い領域）内で検出された図形とその面積を一時的に保存するリスト
        detected_shapes_in_roi = []

        for cnt in roi_contours:
            area = cv2.contourArea(cnt)
            # 面積が小さすぎるものは無視（ノイズ対策）
            # この100という値は、検出したい図形の最小サイズに合わせて調整してください
            if area < 100: 
                continue

            shape_name, approx = classify_by_vertex_count(cnt)
            # 頂点数が3未満のものは図形として扱わない
            if len(approx) < 3:
                continue
            
            M = cv2.moments(cnt)
            if M["m00"] == 0: # 面積が0の場合はスキップ
                continue
            
            # グローバル座標での中心点を計算
            cx = int(M["m10"] / M["m00"])
            global_cx = x + cx # ROIのオフセットを考慮

            detected_shapes_in_roi.append({
                "name": shape_name,
                "area": area,
                "global_cx": global_cx
            })
        
        # 検出された図形を面積の降順でソート
        detected_shapes_in_roi.sort(key=lambda s: s["area"], reverse=True)

        # 面積の大きい上位2つの図形のみを処理（またはそれ未満の数しか検出されなかった場合は全て）
        # min(2, ...) とすることで、検出された図形が2つ未満でもエラーになりません
        for i in range(min(2, len(detected_shapes_in_roi))):
            shape_info = detected_shapes_in_roi[i]
            
            # どのブロックに属するか判定
            block_idx = min(shape_info["global_cx"] // block_w, grid_cols - 1)
            
            # 最終結果リストに追加
            blocks_shapes.setdefault(block_idx, []).append(shape_info["name"])

    result = {}
    for i in range(grid_cols):
        region_name = block_names.get(i, "不明")
        shapes = blocks_shapes.get(i, [])
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

# --- メイン実行部分 ---
if __name__ == '__main__':
    # モータードライバのピン設定
    LEFT_MOTOR_AIN1 = 23
    LEFT_MOTOR_AIN2 = 18
    LEFT_MOTOR_PWMA = 12
    RIGHT_MOTOR_BIN1 = 16
    RIGHT_MOTOR_BIN2 = 26
    RIGHT_MOTOR_PWMB = 19
    STBY_PIN = 21

    motor = MotorDriver(LEFT_MOTOR_PWMA, LEFT_MOTOR_AIN1, LEFT_MOTOR_AIN2,
                        RIGHT_MOTOR_PWMB, RIGHT_MOTOR_BIN1, RIGHT_MOTOR_BIN2, STBY_PIN)

    # モーターの初期速度（停止状態）
    current_speed = 0

    try:
        while True:
            print("--- 新しい探索サイクル ---")
            found_on_right = False

            while not found_on_right:
                print("左に滑らかに回転します。")
                # 速度0から目標速度40まで滑らかに加速しながら左回転
                motor.changing_left(current_speed, 40)
                current_speed = 40 # 現在の速度を更新
                sleep(0.5) # 回転が完了するのを待つ (changing_left内でsleepがあるが、念のため)

                print("モーターを滑らかに停止します。")
                # 速度40から0まで滑らかに減速して停止
                motor.changing_left(current_speed, 0)
                current_speed = 0 # 現在の速度を更新
                motor.motor_stop_free() # 念のため停止コマンドも実行
                sleep(0.5) # 停止してカメラを安定させる

                print("写真を撮って図形を検出します。")
                detected_results = detect_shapes_by_region(show_debug=True)

                if "右" in detected_results and detected_results["右"]:
                    print("「右」に図形が見つかりました！")
                    found_on_right = True
                    break # 内側のループを抜けて前進

                print("「右」に図形が見つかりませんでした。再度左回転します。")
                # ループが続くので、ここで特にアクションは不要

            if found_on_right:
                print("前進します。")
                # 速度0から目標速度50まで滑らかに加速しながら前進
                motor.changing_forward(current_speed, 50)
                current_speed = 50 # 現在の速度を更新
                sleep(1.0) # 前進が完了するのを待つ (changing_forward内でsleepがあるが、念のため)

                print("モーターを滑らかに停止します。")
                # 速度50から0まで滑らかに減速して停止
                motor.changing_forward(current_speed, 0)
                current_speed = 0 # 現在の速度を更新
                motor.motor_stop_free() # 念のため停止コマンドも実行
                sleep(2) # 前進後、次の探索サイクルの準備
            # elseブロックは、今回はfound_on_rightが必ずTrueになるため不要
            # （内側のwhileループを抜ける条件がfound_on_right=Trueのみのため）

    except KeyboardInterrupt:
        print("プログラムを終了します。")
    finally:
        motor.cleanup()
