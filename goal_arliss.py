import RPi.GPIO as GPIO
import time
import pigpio
import board
import busio
import numpy as np
import cv2
from picamera2 import Picamera2
from libcamera import Transform
import sys
import os
import math

# カスタムモジュールのインポート
from motor import MotorDriver
from BNO055 import BNO055
# import following # followingモジュールはもう使いません

# --- BNO055用のラッパークラス (変更なし) ---
class BNO055Wrapper:
    def __init__(self, adafruit_bno055_sensor):
        self.sensor = adafa.sensor = adafruit_bno055_sensor

    def get_heading(self):
        heading = self.sensor.euler[0]
        if heading is None:
            wait_start_time = time.time()
            max_wait_time = 0.5
            while heading is None and (time.time() - wait_start_time < max_wait_time):
                time.sleep(0.01)
                heading = self.sensor.euler[0]
        if heading is None:
            return 0.0
        return heading

# --- 定数設定 (変更なし) ---
RX_PIN = 17

# --- 関数定義 (変更なし) ---
def save_image_for_debug(picam2_instance, path="/home/mark1/Pictures/paravo_image.jpg"):
    frame = picam2_instance.capture_array()
    if frame is None:
        print("画像キャプ手失敗：フレームがNoneです。")
        return None
    frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
    cv2.imwrite(path, frame_bgr)
    print(f"画像保存成功: {path}")
    return frame

# --- 赤色検出関数 (HSV閾値調整済み、通常画像保存に変更) ---
def detect_red_percentage(picam2_instance, save_path="/home/mark1/Pictures/red_detection_overall.jpg"):
    """
    カメラ画像をキャプチャし、画像全体における赤色ピクセルの割合を返します。
    ここではソフトウェア的に回転・反転を行います。
    エラー時は-1.0を返します。
    保存される画像は、回転後の通常のカラー画像です。
    """
    try:
        frame_rgb = picam2_instance.capture_array()
        if frame_rgb is None:
            print("画像キャプチャ失敗: フレームがNoneです。")
            return -1.0 # エラー値として-1.0を返す

        # Picamera2はRGBを返すため、BGRに変換
        frame_bgr = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)
        
        # --- ここから回転処理 ---
        # 反時計回りに90度回転 (カメラが物理的に時計回りに90度傾いている場合)
        rotated_frame_bgr = cv2.rotate(frame_bgr, cv2.ROTATE_90_COUNTERCLOCKWISE)
        # --- 回転処理ここまで ---

        # デバッグ用に通常の回転済み画像を保存
        directory = os.path.dirname(save_path)
        if not os.path.exists(directory):
            os.makedirs(directory)
        cv2.imwrite(save_path, rotated_frame_bgr) 
        print(f"通常の画像を保存しました: {save_path}")

        # 回転後のフレームの高さと幅を使用
        height, width, _ = rotated_frame_bgr.shape
        total_pixels = height * width

        # BGRからHSV色空間に変換 (回転後の画像を使用)
        hsv = cv2.cvtColor(rotated_frame_bgr, cv2.COLOR_BGR2HSV)

        # 赤色のHSV範囲を定義 (より赤色に近い色も検知するように調整済み)
        lower_red1 = np.array([0, 100, 100])   # SとVの下限を下げて、より広い範囲の赤を検出
        upper_red1 = np.array([10, 255, 255]) # 色相の上限を少し広げて、オレンジ寄りの赤も含む

        lower_red2 = np.array([170, 100, 100]) # 色相の下限を少し広げ、紫寄りの赤も含む
        upper_red2 = np.array([180, 255, 255])

        # マスクを作成し結合 (ガウシアンブラーなし)
        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask = cv2.add(mask1, mask2)

        # 赤色領域のピクセル数をカウント
        red_pixels = cv2.countNonZero(mask)

        # 赤色ピクセルの割合を計算
        red_percentage = (red_pixels / total_pixels) * 100 if total_pixels > 0 else 0.0

        print(f"検出結果: 画像全体の赤色割合: {red_percentage:.2f}%")

        return red_percentage / 100.0 # 割合は0-1の範囲で返すように調整

    except Exception as e:
        print(f"カメラ撮影・処理中にエラーが発生しました: {e}")
        return -1.0

# --- ヘルパー関数: 指定角度へ相対的に回頭する (変更なし) ---
def turn_to_relative_angle(driver, bno_sensor_instance, angle_offset_deg, turn_speed=90, angle_tolerance_deg=10.0, max_turn_attempts=100):
    """
    現在のBNO055の方位から、指定された角度だけ相対的に旋回します。
    """
    initial_heading = bno_sensor_instance.get_heading()
    if initial_heading is None:
        print("警告: turn_to_relative_angle: 初期方位が取得できませんでした。")
        return False
    
    target_heading = (initial_heading + angle_offset_deg + 360) % 360
    print(f"現在のBNO方位: {initial_heading:.2f}度, 相対目標角度: {angle_offset_deg:.2f}度 -> 絶対目標方位: {target_heading:.2f}度")

    loop_count = 0
    
    while loop_count < max_turn_attempts:
        current_heading = bno_sensor_instance.get_heading()
        if current_heading is None:
            print("警告: turn_to_relative_angle: 旋回中に方位が取得できませんでした。スキップします。")
            driver.motor_stop_brake()
            time.sleep(0.1)
            loop_count += 1
            continue

        angle_error = (target_heading - current_heading + 180 + 360) % 360 - 180

        if abs(angle_error) <= angle_tolerance_deg:
            print(f"[TURN] 相対回頭完了。最終誤差: {angle_error:.2f}度 (試行回数: {loop_count})")
            driver.motor_stop_brake()
            time.sleep(0.5)
            return True

        turn_duration_on = 0.02 + (abs(angle_error) / 180.0) * 0.2
        if angle_error < 0:
            driver.petit_left(0, turn_speed)
            driver.petit_left(turn_speed, 0)
        else:
            driver.petit_right(0, turn_speed)
            driver.petit_right(turn_speed, 0)
        
        time.sleep(turn_duration_on)
        driver.motor_stop_brake()
        time.sleep(0.05)
        
        loop_count += 1
    
    print(f"警告: turn_to_relative_angle: 最大試行回数({max_turn_attempts}回)内に目標角度に到達できませんでした。最終誤差: {angle_error:.2f}度 (試行回数: {loop_count})")
    driver.motor_stop_brake()
    time.sleep(0.5)
    return False

# --- 新しいヘルパー関数：`calculate_angle_average` (変更なし) ---
def calculate_angle_average(angles_deg):
    """
    複数の角度（度数法）の平均値を計算します。角度の連続性を考慮し、ベクトル平均を使用します。
    例: [350, 10] の平均は 0 に近い値になります。
    """
    if not angles_deg:
        return None

    # 各角度をラジアンに変換し、X成分とY成分に分解
    x_coords = [math.cos(math.radians(angle)) for angle in angles_deg]
    y_coords = [math.sin(math.radians(angle)) for angle in angles_deg]

    # X成分とY成分の合計
    sum_x = sum(x_coords)
    sum_y = sum(y_coords)

    # 合計ベクトルから平均角度を計算
    average_angle_rad = math.atan2(sum_y, sum_x)
    average_angle_deg = math.degrees(average_angle_rad)

    # 角度を0〜360度の範囲に正規化
    return (average_angle_deg + 360) % 360

# --- 周囲確認ロジック (360度スキャンに修正) ---
def perform_final_scan_and_terminate(driver, bno_sensor_instance, picam2_instance, turn_angle_step=20, final_threshold=0.15, min_red_detections_to_terminate=4, high_red_threshold=0.40):
    """
    ローバーを20度ずつ360度回転させ、設定された閾値以上の赤色を検知した方向を記録します。
    最終的にmin_red_detections_to_terminate個以上の赤色を検知し、
    かつhigh_red_threshold以上の赤色を一度でも検知した場合、後退してTrueを返します。
    それ以外の場合はFalseを返します。
    """
    print(f"\n=== 最終確認スキャンを開始します ({final_threshold:.0%}閾値、{min_red_detections_to_terminate}ヶ所検知、{high_red_threshold:.0%}高閾値で後退) ===")
    initial_heading = bno_sensor_instance.get_heading()
    if initial_heading is None:
        print("警告: 最終確認スキャン開始時に方位が取得できませんでした。")
        return False

    final_scan_detected_angles = []
    high_red_percentage_detected = False # 40%以上の赤色を検出したかどうかのフラグ

    print(f"  初回回転: {turn_angle_step}度...")
    turn_to_relative_angle(driver, bno_sensor_instance, turn_angle_step, turn_speed=90, angle_tolerance_deg=15)
    
    # 360度スキャンに変更
    for i in range(360 // turn_angle_step): 
        if i > 0:
            print(f"  --> スキャン中: さらに{turn_angle_step}度回転...")
            turn_to_relative_angle(driver, bno_sensor_instance, turn_angle_step, turn_speed=90, angle_tolerance_deg=15)
            driver.motor_stop_brake()
            time.sleep(0.5)
        
        current_scan_heading = bno_sensor_instance.get_heading()
        if current_scan_heading is None:
            print("警告: 最終確認スキャン中に方位が取得できませんでした。スキップします。")
            driver.motor_stop_brake()
            time.sleep(0.1)
            continue

        print(f"--- 最終確認スキャン中: 現在の方向: {current_scan_heading:.2f}度 ---")

        overall_red_ratio = detect_red_percentage(
            picam2_instance, 
            save_path=f"/home/mark1/Pictures/final_scan_{i*turn_angle_step + turn_angle_step:03d}.jpg"
        )

        if overall_red_ratio == -1.0:
            print("カメラ処理エラーのため、現在のスキャンステップをスキップします。")
            continue

        print(f"検出結果: 画像全体の赤色割合: {overall_red_ratio:.2%}")

        if overall_red_ratio >= final_threshold: # 15%以上の赤色を検出
            print(f"  --> 赤色を{final_threshold:.0%}以上検出！方向を記録します。")
            final_scan_detected_angles.append(current_scan_heading)
        
        if overall_red_ratio >= high_red_threshold: # 40%以上の赤色を検出
            print(f"  --> 高い赤色割合 ({high_red_threshold:.0%}) を検出しました！")
            high_red_percentage_detected = True # フラグを立てる

        driver.motor_stop_brake()
        time.sleep(0.5)

    # 最終的な判定ロジック
    if len(final_scan_detected_angles) >= min_red_detections_to_terminate and high_red_percentage_detected:
        print(f"\n  --> 最終確認スキャンで{min_red_detections_to_terminate}ヶ所以上の赤色を検出し、かつ{high_red_threshold:.0%}以上の赤色も検出しました。")
        
        target_center_angle = calculate_angle_average(final_scan_detected_angles)
        if target_center_angle is not None:
            print(f"  --> 検出された赤色の中心 ({target_center_angle:.2f}度) へ向きを調整します。")
            current_heading_at_end = bno_sensor_instance.get_heading()
            if current_heading_at_end is not None:
                angle_to_turn = (target_center_angle - current_heading_at_end + 180 + 360) % 360 - 180
                turn_to_relative_angle(driver, bno_sensor_instance, angle_to_turn, turn_speed=90, angle_tolerance_deg=15)
                driver.motor_stop_brake()
                time.sleep(1.0)
                print("  --> 中心方向への向き調整が完了しました。")
            else:
                print("警告: 最終スキャン後の中心回頭時に方位が取得できませんでした。")

        # ★後退処理
        print("\n  --> 条件を満たしたため、後退します。")
        driver.petit_petit(-5) # 後退速度を調整 (負の値を渡す)
        time.sleep(2) # 2秒間後退
        driver.motor_stop_brake()
        time.sleep(0.5)
        print("  --> 後退が完了しました。")
        
        print("  --> 最終確認スキャンを最初から再開します。")
        return True # メインループに戻り、最終確認スキャンを再開するシグナル
    else:
        print(f"\n=== 最終確認スキャンが完了しました。条件を満たしませんでした (検出箇所: {len(final_scan_detected_angles)}ヶ所, 高割合検出: {high_red_percentage_detected})。 ===")
        return False

# --- 初期アライメントスキャン関数 (270度スキャンは維持) ---
def perform_initial_alignment_scan(driver, bno_sensor_instance, picam2_instance, turn_angle_step=20, alignment_threshold=0.10):
    """
    ローバーを20度ずつ270度回転させ、20%以上の赤色を検知したらその方向で回転を停止し、向きを合わせます。
    20%以上の赤色が検知されなかった場合は、最も多くの赤が検知された方向に向きを合わせてからTrueを返します。
    戻り値: (aligned_successfully:bool, aligned_heading:float, detected_red_angles:list)
    """
    print("\n=== 初期赤色アライメントスキャンを開始します (20%閾値) ===")
    initial_heading_at_start = bno_sensor_instance.get_heading()
    if initial_heading_at_start is None:
        print("警告: 初期アライメントスキャン開始時に方位が取得できません。")
        return False, None, []

    aligned = False
    max_red_ratio = -1.0
    best_heading_for_red = initial_heading_at_start
    
    detected_red_angles = []

    # 初回回転を270度にする (変更なし)
    initial_turn_angle = 270 
    print(f"  初回回転: {initial_turn_angle}度...")
    turn_to_relative_angle(driver, bno_sensor_instance, initial_turn_angle, turn_speed=90, angle_tolerance_deg=15)
    
    # 270度スキャン (変更なし)
    for i in range(270 // turn_angle_step):
        current_relative_angle_from_start_of_scan = (i + 1) * turn_angle_step
        
        if i > 0: # 初回回転は上記で実施済みのため、2回目以降の回転
            print(f"  回転: {turn_angle_step}度...")
            turn_to_relative_angle(driver, bno_sensor_instance, turn_angle_step, turn_speed=90, angle_tolerance_deg=15)
        
        current_scan_heading = bno_sensor_instance.get_heading()
        if current_scan_heading is None:
            print("警告: 初期アライメントスキャン中に方位が取得できませんでした。スキップします。")
            driver.motor_stop_brake()
            time.sleep(0.1)
            continue

        print(f"\n--- 初期アライメントスキャン中: 現在の方向: {current_scan_heading:.2f}度 ---")
        
        overall_red_ratio = detect_red_percentage(
            picam2_instance, 
            save_path=f"/home/mark1/Pictures/initial_alignment_scan_{current_relative_angle_from_start_of_scan:03d}.jpg"
        )

        if overall_red_ratio == -1.0:
            print("初期アライメントスキャン中にカメラ処理エラー。スキップします。")
            driver.motor_stop_brake()
            time.sleep(0.5)
            continue

        print(f"検出結果: 画像全体の赤色割合: {overall_red_ratio:.2f}%")

        if overall_red_ratio > max_red_ratio:
            max_red_ratio = overall_red_ratio
            best_heading_for_red = current_scan_heading 
        
        if overall_red_ratio * 100.0 >= alignment_threshold * 100.0:
            detected_red_angles.append(current_scan_heading)
            print(f"  --> 赤色を{alignment_threshold*100.0:.0f}%以上検出！方向を記録しました。")

        driver.motor_stop_brake()
        time.sleep(0.5)

        # 最後の回転でなければ次の回転 (270度スキャンに調整) (変更なし)
        if i < (270 // turn_angle_step) - 1:
            print(f"  回転: {turn_angle_step}度...")
            turn_to_relative_angle(driver, bno_sensor_instance, turn_angle_step, turn_speed=90, angle_tolerance_deg=15)


    if not detected_red_angles:
        print(f"初期アライメントスキャンで{alignment_threshold*100.0:.0f}%以上の赤色は検出されませんでした。")
        if max_red_ratio > -1.0:
            current_heading_at_end_of_scan = bno_sensor_instance.get_heading()
            
            if current_heading_at_end_of_scan is not None:
                angle_to_turn_to_best_red = (best_heading_for_red - current_heading_at_end_of_scan + 180 + 360) % 360 - 180
                
                print(f"  --> {alignment_threshold*100.0:.0f}%"
                      f"以上は検出されませんでしたが、最も多くの赤 ({max_red_ratio:.2f}%) が検出された方向 ({best_heading_for_red:.2f}度) へアライメントします (相対回転: {angle_to_turn_to_best_red:.2f}度)。")
                turn_to_relative_angle(driver, bno_sensor_instance, angle_to_turn_to_best_red, turn_speed=90, angle_tolerance_deg=15)
                driver.motor_stop_brake()
                time.sleep(0.5)
                aligned = True
                detected_red_angles.append(best_heading_for_red)
            else:
                print("警告: スキャン終了時に方位が取得できず、最大赤色方向へのアライメントができませんでした。")
        else:
            print("初期アライメントスキャンで赤色は全く検出されませんでした。")
    else:
        aligned = True

    print("=== 初期赤色アライメントスキャンが完了しました。 ===")
    return aligned, best_heading_for_red, detected_red_angles


# --- メインシーケンス ---
if __name__ == "__main__":
    # GPIO設定
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)

    # デバイス初期化
    driver = MotorDriver(
        PWMA=12, AIN1=23, AIN2=18,
        PWMB=19, BIN1=16, BIN2=26,
        STBY=21
    )
    pi_instance = pigpio.pi()
    if not pi_instance.connected:
        print("pigpioデーモンに接続できません。終了します。")
        exit()
    
    bno_sensor = BNO055(address=0x28)
    if not bno_sensor.begin():
        print("BNO055センサーの初期化に失敗しました。終了します。")
        exit()
    bno_sensor.setMode(BNO055.OPERATION_MODE_NDOF)
    bno_sensor.setExternalCrystalUse(True)
    time.sleep(1)
    
    picam2_instance = Picamera2()
    picam2_instance.configure(picam2_instance.create_preview_configuration(
        main={"size": (640, 480)},
        controls={"FrameRate": 30},
        transform=Transform(rotation=90)
    ))
    picam2_instance.start()
    time.sleep(2)

    try:
        print("BNO055のキャリブレーション待機はスキップされました。自動操縦を開始します。")

        # メインの自律走行ループ
        while True:
            print("\n--- 新しい走行サイクル開始 ---")
            
            print("\n=== 現在方位確認 ===")
            current_bno_heading_for_info = bno_sensor.get_heading()
            if current_bno_heading_for_info is None:
                print("警告: 現在方位が取得できませんでした。")
                time.sleep(2)
                continue
            print(f"現在のBNO方位: {current_bno_heading_for_info:.2f}度")
            driver.motor_stop_brake()
            time.sleep(0.5)

            # --- 初期アライメントスキャンを実行 ---
            # 初期アライメントスキャンで2つ以上の赤色を検知した場合、次のフェーズをスキップするためのフラグ
            skip_forward_scan_phase = False 
            # best_heading_for_red を初期アライメントスキャンから取得するために返り値に追加
            aligned_in_initial_scan, initial_aligned_heading, initial_scan_detected_angles = perform_initial_alignment_scan(driver, bno_sensor, picam2_instance)

            # --- 初期アライメントで4つ以上の赤色を検知した場合、最終処理へスキップ ---
            # ここで、perform_final_scan_and_terminate の min_red_detections_to_terminate (デフォルト4) と比較
            if len(initial_scan_detected_angles) >= 4: # 4つ以上の赤色を検出
                print(f"\n=== 初期アライメントスキャンで{len(initial_scan_detected_angles)}ヶ所の赤色を検知しました。最終確認スキャンへスキップします。 ===")
                # perform_final_scan_and_terminate を呼び出し、その結果に基づいてループを終了
                # perform_final_scan_and_terminate の戻り値が True ならば、ここで continue
                # ★変更点: perform_final_scan_and_terminateの再帰呼び出しを処理するために、if文を修正
                while True: # 最終確認スキャンが成功するまで繰り返すためのループ
                    if perform_final_scan_and_terminate(driver, bno_sensor, picam2_instance, final_threshold=0.15, min_red_detections_to_terminate=4, high_red_threshold=0.40):
                        print("最終確認スキャンにより4つ以上の赤色を検知し、高割合検出後、後退しました。最終確認スキャンを再開します。")
                        # Trueが返されたので、再度perform_final_scan_and_terminateを呼び出すためにループを続ける
                        continue
                    else:
                        print("最終確認スキャンは完了しましたが、ミッション終了条件を満たしませんでした。メインループの通常フローに戻ります。")
                        break # 最終確認スキャンが条件を満たさなかった場合はこのループを抜けて、メインループの続きへ
                continue # このouter continueは、最終確認スキャンが条件を満たさなかった場合に、メインループの最初に戻るためのもの
            # --- 初期アライメントスキップ処理ここまで ---


            if aligned_in_initial_scan:
                print(f"初期アライメントスキャンによって、何らかの赤色へアライメントされました。方位: {initial_aligned_heading:.2f}度")
            else:
                print("初期アライメントスキャンで赤色は全く検出されませんでした。")
            
            # 検出された赤色が複数あった場合の処理（中心角に向いて1秒前進）
            if len(initial_scan_detected_angles) >= 2:
                target_center_angle = calculate_angle_average(initial_scan_detected_angles)
                if target_center_angle is not None:
                    print(f"\n=== 初期アライメントスキャンで複数赤色検知地点の中心 ({target_center_angle:.2f}度) へ向きを調整します ===")
                    current_heading = bno_sensor.get_heading()
                    if current_heading is not None:
                        angle_to_turn = (target_center_angle - current_heading + 180 + 360) % 360 - 180
                        turn_to_relative_angle(driver, bno_sensor, angle_to_turn, turn_speed=90, angle_tolerance_deg=15)
                        driver.motor_stop_brake()
                        time.sleep(0.5)
                        print("中心方向への向き調整が完了しました。")
                        
                        print("  --> 中心方向へ向いた後、1秒間前進します。")
                        # driver.petit_petit に変更
                        driver.petit_petit(9) # 前進速度を左右の引数に渡す (例: 90)
                        time.sleep(1) # 1秒間前進
                        driver.motor_stop_brake()
                        time.sleep(0.5)
                        print("  --> 1秒前進を完了しました。次の「周囲確認（最終確認スキャン）」へ移行します。")
                        skip_forward_scan_phase = True # ここでスキップフラグをTrueに設定
                    else:
                        print("警告: 複数赤色検知後の回頭時に現在方位が取得できませんでした。")
                        skip_forward_scan_phase = False
                else:
                    print("警告: 検出された角度からの中心角度計算に失敗しました。")
                    skip_forward_scan_phase = False
            # 初期アライメントスキャンで1か所しか検知しなかった場合の処理
            elif len(initial_scan_detected_angles) == 1:
                print(f"\n=== 赤色を1ヶ所のみ検出 ({initial_scan_detected_angles[0]:.2f}度) しました。その方向へ向きを調整済みです。")
                # スキップフラグはFalseのままにし、下のelseブロックで処理されるようにします。
                skip_forward_scan_phase = False
            else: # 赤色検知が全くなかった場合
                print("\n=== 赤色検知がなかったため、次の行動に移ります。 ===")
                skip_forward_scan_phase = False

            # skip_forward_scan_phaseがTrueの場合、次のフェーズへジャンプ
            if skip_forward_scan_phase:
                print("--- 「アライメント後、360度スキャンで赤色を探索し前進判断」フェーズをスキップします。 ---")
                pass # そのまま次の処理へ進む
            else:
                # --- アライメント後、360度スキャンで赤色を探索し前進判断 ---
                print("\n=== アライメント後、360度スキャンで赤色を探索し前進判断 ===")
                
                # このフラグは、今回のスキャン中に一度でも5%以上の赤色を検知して前進したかどうかを記録します
                any_red_detected_and_moved_this_scan = False  
                
                # スキャン中に検出された赤色の割合と方位を記録するリスト
                scan_detections = []

                # 最初に20度回転してから検知を開始
                print("  --> 前進判断のため、20度回転します...")
                turn_to_relative_angle(driver, bno_sensor, 20, turn_speed=90, angle_tolerance_deg=15)
                driver.motor_stop_brake()
                time.sleep(0.5)

                # このループで1秒前進したらbreakする (360度スキャン)
                for i in range(360 // 20): 
                    current_scan_heading_for_forward = bno_sensor.get_heading()
                    if current_scan_heading_for_forward is None:
                        print("警告: スキャン中に方位が取得できませんでした。スキップします。")
                        continue

                    print(f"--- 360度スキャン中: 現在の方向: {current_scan_heading_for_forward:.2f}度 ---")

                    current_red_percentage_scan = detect_red_percentage(
                        picam2_instance, 
                        save_path=f"/home/mark1/Pictures/forward_scan_{i*20 + 20:03d}.jpg"
                    )

                    if current_red_percentage_scan == -1.0:
                        print("カメラ処理でエラーが発生しました。現在のスキャンステップをスキップします。")
                        continue
                    
                    # 検出結果を記録 (割合と方位のペア)
                    scan_detections.append({'percentage': current_red_percentage_scan, 'heading': current_scan_heading_for_forward})

                    if current_red_percentage_scan >= 0.05: # 5%閾値
                        print(f"  --> 赤色を{0.05:.0%}以上検出！この方向に1秒前進します。")
                        # driver.petit_petit に変更
                        driver.petit_petit(4) # 前進速度を左右の引数に渡す (例: 90)
                        time.sleep(1) # 1秒間前進
                        driver.motor_stop_brake()
                        time.sleep(0.5)
                        any_red_detected_and_moved_this_scan = True  # 前進したのでフラグを立てる
                        break # 1回でも前進したらここでループを抜ける

                    # 1回も検知せず、かつ最後の回転でなければ次の回転 (360度スキャン)
                    if i < (360 // 20) - 1: 
                        print(f"  --> スキャン中: さらに20度回転...")
                        turn_to_relative_angle(driver, bno_sensor, 20, turn_speed=90, angle_tolerance_deg=15)
                        driver.motor_stop_brake()
                        time.sleep(0.5)

                # --- 360度スキャンが完了した後での判定ロジック ---
                # 初期アライメントで1か所しか検知しなかった場合の追加処理
                # かつ、この360度スキャン中に一度も5%以上の赤色を検出しなかった場合
                if not any_red_detected_and_moved_this_scan and len(initial_scan_detected_angles) == 1:
                    print("\n=== 初期アライメントスキャンで赤色を1ヶ所のみ検知。360度スキャンで2番目に赤の割合が大きかった方向へ前進します。 ===")
                    
                    # scan_detectionsから、最も割合が大きいものを除外し、残りのうちで2番目に大きいものを探す
                    if scan_detections:
                        # 割合でソート（降順）
                        sorted_detections = sorted(scan_detections, key=lambda x: x['percentage'], reverse=True)
                        
                        best_red_after_initial_alignment = None
                        if len(sorted_detections) >= 2:
                            # 初期アライメント時の方向と「十分に離れている」ものを2番目の候補とする
                            for det in sorted_detections:
                                # 検出された角度と初期アライメントでアライメントされた角度との差を計算
                                angle_diff = (det['heading'] - initial_aligned_heading + 180 + 360) % 360 - 180
                                # 検出された角度が初期アライメント時の角度からある程度離れている場合を「異なる」と判断
                                if abs(angle_diff) > 20: # 例えば20度以上離れていれば異なる方向とみなす
                                    best_red_after_initial_alignment = det
                                    break # 最初の異なる最高割合を見つけたら終了

                            if best_red_after_initial_alignment:
                                target_heading_for_second_best = best_red_after_initial_alignment['heading']
                                print(f"  --> 2番目に赤の割合が大きかった方向 ({target_heading_for_second_best:.2f}度, 割合: {best_red_after_initial_alignment['percentage']:.2%}) へ向きを調整し、1秒間前進します。")
                                current_heading_before_adjust = bno_sensor.get_heading()
                                if current_heading_before_adjust is not None:
                                    angle_to_turn = (target_heading_for_second_best - current_heading_before_adjust + 180 + 360) % 360 - 180
                                    turn_to_relative_angle(driver, bno_sensor, angle_to_turn, turn_speed=90, angle_tolerance_deg=15)
                                    driver.motor_stop_brake()
                                    time.sleep(0.5)
                                    print("向き調整が完了しました。")

                                    print("  --> 調整後、1秒間前進します。")
                                    # driver.petit_petit に変更
                                    driver.petit_petit(4) # 前進速度を左右の引数に渡す (例: 90)
                                    time.sleep(1) # 1秒間前進
                                    driver.motor_stop_brake()
                                    time.sleep(0.5)
                                    print("  --> 1秒前進を完了しました。")

                                    # 追加の要求: 1秒前進後、もう一度360度回転して、2つの赤が検知されたらその間の方向へ向く
                                    print("\n=== 1秒前進後、追加の360度スキャンと中心方向への調整を開始します ===")
                                    post_forward_scan_detected_angles = []
                                    # 最初に20度回転してから検知を開始 (新たなスキャンサイクル)
                                    print("  --> 追加スキャンのため、20度回転します...")
                                    turn_to_relative_angle(driver, bno_sensor, 20, turn_speed=90, angle_tolerance_deg=15)
                                    driver.motor_stop_brake()
                                    time.sleep(0.5)

                                    for j in range(360 // 20):
                                        current_post_forward_heading = bno_sensor.get_heading()
                                        if current_post_forward_heading is None:
                                            print("警告: 追加スキャン中に方位が取得できませんでした。スキップします。")
                                            continue

                                        print(f"--- 追加スキャン中: 現在の方向: {current_post_forward_heading:.2f}度 ---")
                                        
                                        current_red_percentage_post_forward_scan = detect_red_percentage(
                                            picam2_instance,
                                            save_path=f"/home/mark1/Pictures/post_forward_scan_{j*20 + 20:03d}.jpg"
                                        )

                                        if current_red_percentage_post_forward_scan == -1.0:
                                            print("カメラ処理エラーのため、現在の追加スキャンステップをスキップします。")
                                            continue

                                        if current_red_percentage_post_forward_scan >= 0.05: # 5%閾値
                                            print(f"  --> 追加スキャンで赤色を{0.05:.0%}以上検出！方向を記録します。")
                                            post_forward_scan_detected_angles.append(current_post_forward_heading)
                                        
                                        if j < (360 // 20) - 1:
                                            print(f"  --> 追加スキャン中: さらに20度回転...")
                                            turn_to_relative_angle(driver, bno_sensor, 20, turn_speed=90, angle_tolerance_deg=15)
                                            driver.motor_stop_brake()
                                            time.sleep(0.5)
                                    
                                    if len(post_forward_scan_detected_angles) >= 2:
                                        target_center_angle_post_forward = calculate_angle_average(post_forward_scan_detected_angles)
                                        if target_center_angle_post_forward is not None:
                                            print(f"\n=== 追加スキャンで複数赤色検知！中心 ({target_center_angle_post_forward:.2f}度) へ向きを調整します ===")
                                            current_heading_at_post_forward_end = bno_sensor.get_heading()
                                            if current_heading_at_post_forward_end is not None:
                                                angle_to_turn_post_forward = (target_center_angle_post_forward - current_heading_at_post_forward_end + 180 + 360) % 360 - 180
                                                turn_to_relative_angle(driver, bno_sensor, angle_to_turn_post_forward, turn_speed=90, angle_tolerance_deg=15)
                                                driver.motor_stop_brake()
                                                time.sleep(0.5)
                                                print("追加スキャン後の中心方向への向き調整が完了しました。")
                                            else:
                                                print("警告: 追加スキャン後の中心回頭時に方位が取得できませんでした。")
                                        else:
                                            print("警告: 追加スキャンで検出された角度からの中心角度計算に失敗しました。")
                                    else:
                                        print("\n=== 追加スキャンで赤色の複数検知はありませんでした。 ===")
                                    # 追加スキャン後の処理完了
                                else:
                                    print("警告: 2番目の方向への回頭時に現在方位が取得できませんでした。")
                            else:
                                print("警告: 2番目に適した赤色検出方向が見つかりませんでした。")
                        else:
                            print("警告: 2番目に適した赤色検出方向を特定するためのデータが不足しています。")


                # ここで、1回でも1秒前進したら、追加の2個検知スキャンと中央角への調整・前進を行う
                if any_red_detected_and_moved_this_scan: # スキャン中に一度でも赤色を検知して前進した場合
                    print("\n=== 赤色を検知し1秒前進しました。追加の2個検知スキャンを開始します ===")
                    
                    second_scan_detected_angles = [] # 2回目のスキャンで検出された角度を格納するリスト
                    
                    # 最初に20度回転してから検知を開始 (2回目のスキャン)
                    print("  --> 2回目スキャンのため、20度回転します...")
                    turn_to_relative_angle(driver, bno_sensor, 20, turn_speed=90, angle_tolerance_deg=15)
                    driver.motor_stop_brake()
                    time.sleep(0.5)

                    for i in range(360 // 20): # 2回目の360度スキャン 
                        current_scan_heading_for_second = bno_sensor.get_heading()
                        if current_scan_heading_for_second is None:
                            print("警告: 2回目スキャン中に方位が取得できませんでした。スキップします。")
                            continue

                        print(f"--- 2回目スキャン中: 現在の方向: {current_scan_heading_for_second:.2f}度 ---")

                        current_red_percentage_second_scan = detect_red_percentage(
                            picam2_instance, 
                            save_path=f"/home/mark1/Pictures/second_scan_{i*20 + 20:03d}.jpg"
                        )

                        if current_red_percentage_second_scan == -1.0:
                            print("カメラ処理でエラーが発生しました。2回目スキャンステップをスキップします。")
                            continue
                        
                        # 2回目のスキャンでは、赤色を検出したらリストに追加
                        if current_red_percentage_second_scan >= 0.05: # 同じく5%閾値で検知
                            print(f"  --> 2回目スキャンで赤色を{0.05:.0%}以上検出！方向を記録します。")
                            second_scan_detected_angles.append(current_scan_heading_for_second)

                        # 最後の回転でなければ次の回転 (360度スキャン)
                        if i < (360 // 20) - 1: 
                            print(f"  --> 2回目スキャン中: さらに20度回転...")
                            turn_to_relative_angle(driver, bno_sensor, 20, turn_speed=90, angle_tolerance_deg=15)
                            driver.motor_stop_brake()
                            time.sleep(0.5)

                    # 2回目のスキャンが完了した後、2個以上の赤色を検知したかチェック
                    if len(second_scan_detected_angles) >= 2:
                        target_center_angle_second_scan = calculate_angle_average(second_scan_detected_angles)
                        if target_center_angle_second_scan is not None:
                            print(f"\n=== 2回目スキャンで複数赤色検知！中心 ({target_center_angle_second_scan:.2f}度) へ向きを調整し、1秒前進します ===")
                            current_heading_before_adjust_second = bno_sensor.get_heading()
                            if current_heading_before_adjust_second is not None:
                                angle_to_turn_second = (target_center_angle_second_scan - current_heading_before_adjust_second + 180 + 360) % 360 - 180
                                turn_to_relative_angle(driver, bno_sensor, angle_to_turn_second, turn_speed=90, angle_tolerance_deg=15)
                                driver.motor_stop_brake()
                                time.sleep(0.5)
                                print("中心方向への向き調整が完了しました。")
                                
                                print("  --> 中心方向へ向いた後、1秒間前進します。")
                                # ★変更点: following.follow_forward を driver.petit_petit に変更
                                driver.petit_petit(8) # 前進速度を左右の引数に渡す (例: 90)
                                time.sleep(1) # 1秒間前進
                                driver.motor_stop_brake()
                                time.sleep(0.5)
                                print("  --> 1秒前進を完了しました。")
                            else:
                                print("警告: 2回目スキャン後の回頭時に現在方位が取得できませんでした。")
                        else:
                            print("警告: 2回目スキャンで検出された角度からの中心角度計算に失敗しました。")
                    else:
                        print("\n=== 2回目スキャンで赤色の複数検知はありませんでした。 ===")

            # --- 周囲確認ロジック (360度スキャン - 4つ以上の赤色検知で終了) ---
            print("\n=== 周囲確認を開始します (360度スキャン - 最終確認用) ===")
            
            # ★変更点: perform_final_scan_and_terminateがTrueを返した場合に、再度同じ関数を呼び出す
            while True:
                if perform_final_scan_and_terminate(driver, bno_sensor, picam2_instance, final_threshold=0.15, min_red_detections_to_terminate=4, high_red_threshold=0.40):
                    # Trueが返された（条件を満たして後退した）ので、再度perform_final_scan_and_terminateを呼び出すためにループを続ける
                    print("最終確認スキャン条件達成、後退。最終確認スキャンを再実行します。")
                    continue
                else:
                    # Falseが返された（条件を満たさなかった）ので、このループを抜けてメインループの続きへ
                    print("最終確認スキャンが条件を満たしませんでした。次の走行サイクルに進みます。")
                    break
            
            # 最終確認スキャンが条件を満たさなかった場合、メインループの続き（前進など）を行う
            # ここに5秒前進後の処理（アライメントから再開）を移動させる
            if not any_red_detected_and_moved_this_scan and len(initial_scan_detected_angles) == 0: # 最初のスキャンで全く前進せず、かつ初期アライメントでも赤色を検知しなかった場合のみ
                print("  --> 赤色を検出しなかったため、3秒間前進し、再度アライメントから開始します。")
                # ★変更点: following.follow_forward を driver.petit_petit に変更
                driver.petit_petit(10) # 前進速度を左右の引数に渡す (例: 90)
                time.sleep(3) # 3秒間前進
                driver.motor_stop_brake()
                time.sleep(0.5)
            
            continue # メインループの最初に戻り、アライメントから再開

    except Exception as e:
        print(f"メイン処理中に予期せぬエラーが発生しました: {e}")
        driver.motor_stop_brake()

    finally:
        if 'driver' in locals():
            driver.cleanup()
        if 'pi_instance' in locals() and pi_instance.connected:
            pi_instance.stop()
        if 'picam2_instance' in locals():
            picam2_instance.close()
        GPIO.cleanup()
        print("=== 処理を終了しました。 ===")
