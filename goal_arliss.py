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
import following

# --- BNO055用のラッパークラス (変更なし) ---
class BNO055Wrapper:
    def __init__(self, adafruit_bno055_sensor):
        self.sensor = adafruit_bno055_sensor

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
        print("画像キャプチャ失敗：フレームがNoneです。")
        return None
    frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
    cv2.imwrite(path, frame_bgr)
    print(f"画像保存成功: {path}")
    return frame

# --- 赤色検出関数 (全体割合のみに修正) ---
def detect_red_percentage(picam2_instance, save_path="/home/mark1/Pictures/red_detection_overall.jpg"):
    """
    カメラ画像をキャプチャし、画像全体における赤色ピクセルの割合を返します。
    ここではソフトウェア的に回転・反転を行います。
    エラー時は-1.0を返します。
    """
    try:
        frame_rgb = picam2_instance.capture_array()
        if frame_rgb is None:
            print("画像キャプチャ失敗: フレームがNoneです。")
            return -1.0 # エラー値として-1.0を返す

        frame_bgr = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)
        
        # ★★★ ここで画像を回転・反転させる ★★★
        # 1. 反時計回りに90度回転 (カメラが物理的に時計回りに90度傾いている場合)
        processed_frame_bgr = cv2.rotate(frame_bgr, cv2.ROTATE_90_COUNTERCLOCKWISE)
        
        # 2. 左右反転 (水平フリップ)
        processed_frame_bgr = cv2.flip(processed_frame_bgr, 1) # 1は水平フリップ (左右反転)
        
        height, width, _ = processed_frame_bgr.shape
        total_pixels = height * width

        lower_red1 = np.array([0, 100, 100])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([160, 100, 100])
        upper_red2 = np.array([180, 255, 255])

        blurred_frame = cv2.GaussianBlur(processed_frame_bgr, (5, 5), 0)
        hsv_frame = cv2.cvtColor(blurred_frame, cv2.COLOR_BGR2HSV)
        mask = cv2.bitwise_or(cv2.inRange(hsv_frame, lower_red1, upper_red1),
                              cv2.inRange(hsv_frame, lower_red2, upper_red2))
        
        red_pixels = np.count_nonzero(mask)
        red_percentage = red_pixels / total_pixels if total_pixels > 0 else 0.0

        # デバッグ用に赤色領域をハイライトした画像を保存
        debug_frame = processed_frame_bgr.copy()
        red_highlighted_frame = cv2.bitwise_and(debug_frame, debug_frame, mask=mask)

        directory = os.path.dirname(save_path)
        if not os.path.exists(directory):
            os.makedirs(directory)
        cv2.imwrite(save_path, red_highlighted_frame)
        print(f"赤色検出画像を保存しました: {save_path} (赤色割合: {red_percentage:.2%})")

        return red_percentage

    except Exception as e:
        print(f"カメラ撮影・赤色検出処理中にエラーが発生しました: {e}")
        return -1.0

# --- ヘルパー関数: 指定角度へ相対的に回頭する (変更なし) ---
def turn_to_relative_angle(driver, bno_sensor_instance, angle_offset_deg, turn_speed=40, angle_tolerance_deg=10.0, max_turn_attempts=100):
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

# --- 360度スキャンとアクションの関数 (ロジック修正なし) ---
def scan_360_for_red_and_act(driver, bno_sensor_instance, picam2_instance, turn_angle_step=20, threshold_10_percent=0.10, threshold_40_percent=0.40):
    """
    ローバーを20度ずつ360度回転させ、画像全体における赤色ピクセルの割合を検知します。
    - 10%以上の赤色を検知した場合: その方向に2秒前進。
    - その後、再度20度回転し、40%以上の赤色を検知した場合: 180度転回。
    """
    print("\n=== 360度赤色スキャンを開始します (回避/最終確認用) ===")
    initial_heading = bno_sensor_instance.get_heading()
    if initial_heading is None:
        print("警告: スキャン開始時に方位が取得できませんでした。")
        return False

    forward_movement_triggered = False

    # 最初に20度回転してから検知を開始
    turn_to_relative_angle(driver, bno_sensor_instance, turn_angle_step, turn_speed=60, angle_tolerance_deg=15)
    
    for i in range(360 // turn_angle_step):
        # 最初のループは既に20度回転済みなので、それ以降は常に20度回転
        if i > 0: 
            turn_to_relative_angle(driver, bno_sensor_instance, turn_angle_step, turn_speed=60, angle_tolerance_deg=15)
        
        current_scan_heading = bno_sensor_instance.get_heading()
        if current_scan_heading is None:
            print("警告: スキャン中に方位が取得できませんでした。スキップします。")
            continue

        print(f"\n--- スキャン中: 現在の方向: {current_scan_heading:.2f}度 ---")
        
        overall_red_ratio = detect_red_percentage(
            picam2_instance, 
            save_path=f"/home/mark1/Pictures/scan_360_overall_{i*turn_angle_step + turn_angle_step:03d}.jpg" # ファイル名も調整
        )

        if overall_red_ratio == -1.0:
            print("カメラ処理エラーのため、現在のスキャンステップをスキップします。")
            driver.motor_stop_brake()
            time.sleep(0.5)
            continue

        print(f"検出結果: 画像全体の赤色割合: {overall_red_ratio:.2%}")

        if overall_red_ratio >= threshold_10_percent and not forward_movement_triggered:
            print(f"  --> 赤色を{threshold_10_percent:.0%}以上検出！この方向に1秒前進します。")
            following.follow_forward(driver, bno_sensor_instance, base_speed=80, duration_time=1)
            driver.motor_stop_brake()
            time.sleep(0.5)
            forward_movement_triggered = True

            print("  --> 前進後、再度20度回転して赤色を再確認します...")
            turn_to_relative_angle(driver, bno_sensor_instance, turn_angle_step, turn_speed=60, angle_tolerance_deg=15)
            
            overall_red_ratio_recheck = detect_red_percentage(
                picam2_instance, 
                save_path=f"/home/mark1/Pictures/scan_360_recheck_overall_{i*turn_angle_step + 2*turn_angle_step:03d}.jpg" # ファイル名も調整
            )

            if overall_red_ratio_recheck == -1.0:
                print("再確認時のカメラ処理エラーのため、180度転回は実行されません。")
                driver.motor_stop_brake()
                time.sleep(0.5)
                continue

            print(f"  再確認結果: 画像全体の赤色割合: {overall_red_ratio_recheck:.2%}")

            if overall_red_ratio_recheck >= threshold_40_percent:
                print(f"  --> 赤色を{threshold_40_percent:.0%}以上検出！180度転回します。")
                turn_to_relative_angle(driver, bno_sensor_instance, 180, turn_speed=90, angle_tolerance_deg=10)
                driver.motor_stop_brake()
                time.sleep(1.0)
                print("  --> 180度転回を完了しました。スキャンを終了します。")
                return True

        elif overall_red_ratio >= threshold_10_percent and forward_movement_triggered:
            print(f"  (注: {threshold_10_percent:.0%}以上の赤色を検出しましたが、既定の前進行動は既に完了しています。)")
        
        driver.motor_stop_brake()
        time.sleep(0.5)

    print("\n=== 360度赤色スキャンが完了しました。 ===")
    return False

# --- 初期アライメントスキャン関数 (修正) ---
def perform_initial_alignment_scan(driver, bno_sensor_instance, picam2_instance, turn_angle_step=20, alignment_threshold=0.20):
    """
    ローバーを20度ずつ360度回転させ、20%以上の赤色を検知したらその方向で回転を停止し、向きを合わせます。
    20%以上の赤色が検知されなかった場合は、最も多くの赤が検知された方向に向きを合わせてからTrueを返します。
    """
    print("\n=== 初期赤色アライメントスキャンを開始します (20%閾値) ===")
    initial_heading = bno_sensor_instance.get_heading()
    if initial_heading is None:
        print("警告: 初期アライメントスキャン開始時に方位が取得できません。")
        return False

    aligned = False
    max_red_ratio = -1.0
    best_angle_from_initial_heading = 0 

    # 最初に20度回転してから検知を開始
    print(f"  初回回転: {turn_angle_step}度...")
    turn_to_relative_angle(driver, bno_sensor_instance, turn_angle_step, turn_speed=60, angle_tolerance_deg=15)
    
    for i in range(360 // turn_angle_step):
        current_relative_angle_from_start_of_scan = (i + 1) * turn_angle_step # 初回回転からの累積回転量
        
        # 最初のループ (i=0) は既に20度回転済みなので、それ以降 (i>0) に20度回転
        if i > 0: 
            print(f"  回転: {turn_angle_step}度...")
            turn_to_relative_angle(driver, bno_sensor_instance, turn_angle_step, turn_speed=60, angle_tolerance_deg=15)
        
        current_scan_heading = bno_sensor_instance.get_heading()
        if current_scan_heading is None:
            print("警告: 初期アライメントスキャン中に方位が取得できませんでした。スキップします。")
            driver.motor_stop_brake()
            time.sleep(0.1)
            continue

        print(f"\n--- 初期アライメントスキャン中: 現在の方向: {current_scan_heading:.2f}度 ---")
        
        overall_red_ratio = detect_red_percentage(
            picam2_instance, 
            save_path=f"/home/mark1/Pictures/initial_alignment_scan_{current_relative_angle_from_start_of_scan:03d}.jpg" # ファイル名も調整
        )

        if overall_red_ratio == -1.0:
            print("初期アライメントスキャン中にカメラ処理エラー。スキップします。")
            driver.motor_stop_brake()
            time.sleep(0.5)
            continue

        print(f"検出結果: 画像全体の赤色割合: {overall_red_ratio:.2%}")

        # 最大赤色割合とそれに対応する角度を更新
        if overall_red_ratio > max_red_ratio:
            max_red_ratio = overall_red_ratio
            best_angle_from_initial_heading = (current_scan_heading - initial_heading + 360) % 360 # 初期方位からの絶対角度

        if overall_red_ratio >= alignment_threshold: # 20%以上の赤色を検出
            print(f"  --> 赤色を{alignment_threshold:.0%}以上検出！この方向にアライメントしました。")
            aligned = True
            driver.motor_stop_brake()
            time.sleep(1.0) # 停止して向きを確定
            break # 20%以上の赤色が見つかったらスキャンを終了し、その向きで停止

        driver.motor_stop_brake()
        time.sleep(0.5)

    if not aligned:
        print(f"初期アライメントスキャンで{alignment_threshold:.0%}以上の赤色は検出されませんでした。")
        if max_red_ratio > -1.0: # 何らかの赤色が検出されていた場合
            # 最も多くの赤があった方向へ回頭
            current_heading_at_end_of_scan = bno_sensor_instance.get_heading()
            
            if current_heading_at_end_of_scan is not None:
                # 目標は (initial_heading + best_angle_from_initial_heading) の絶対方位
                target_absolute_heading = (initial_heading + best_angle_from_initial_heading + 360) % 360
                
                # 現在の向きから目標絶対方位への相対回転量を計算
                angle_to_turn_to_best_red = (target_absolute_heading - current_heading_at_end_of_scan + 180 + 360) % 360 - 180
                
                # SyntaxErrorを修正したf-string
                print(f"  --> {alignment_threshold:.0%}"
                      f"以上は検出されませんでしたが、最も多くの赤 ({max_red_ratio:.2%}) が検出された方向 ({target_absolute_heading:.2f}度) へアライメントします (相対回転: {angle_to_turn_to_best_red:.2f}度)。")
                turn_to_relative_angle(driver, bno_sensor_instance, angle_to_turn_to_best_red, turn_speed=60, angle_tolerance_deg=15)
                driver.motor_stop_brake()
                time.sleep(0.5)
                aligned = True # 最大の赤があった方向へアライメントされたとみなす
            else:
                print("警告: スキャン終了時に方位が取得できず、最大赤色方向へのアライメントができませんでした。")
        else:
            print("初期アライメントスキャンで赤色は全く検出されませんでした。")
    
    print("=== 初期赤色アライメントスキャンが完了しました。 ===")
    return aligned


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
        # === BNO055キャリブレーション待機 ===
        print("BNO055のキャリブレーション待機中...")
        while True:
            sys_cal, gyro_cal, accel_cal, mag_cal = bno_sensor.getCalibration()
            print(f"Calib → Sys:{sys_cal}, Gyro:{gyro_cal}, Acc:{accel_cal}, Mag:{mag_cal}", end='\r')
            sys.stdout.flush()
            if gyro_cal == 3 and mag_cal == 3:
                print("\nキャリブレーション完了！自動操縦を開始します。")
                break
            time.sleep(0.5)

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

            # ★★★ 初期アライメントスキャンを実行 ★★★
            aligned_in_initial_scan = perform_initial_alignment_scan(driver, bno_sensor, picam2_instance)

            # アライメント成功/失敗のログ出力
            if aligned_in_initial_scan:
                print("初期アライメントスキャンによって、何らかの赤色へアライメントされました。")
            else:
                print("初期アライメントスキャンで赤色は全く検出されませんでした。")
                
            # ★★★ アライメント後、前進判断の前に20度回転してから赤色を検知する ★★★
            print("\n=== 20度回転後、全体赤色検出と前進 (通常の10%閾値) ===")
            
            # ここで20度回転
            print("  --> 前進判断のため、20度回転します...")
            turn_to_relative_angle(driver, bno_sensor, 20, turn_speed=60, angle_tolerance_deg=15)
            driver.motor_stop_brake()
            time.sleep(0.5)

            current_red_percentage = detect_red_percentage(picam2_instance, save_path="/home/mark1/Pictures/current_overall_red.jpg")

            if current_red_percentage == -1.0:
                print("カメラ処理でエラーが発生しました。少し待機します...")
                time.sleep(2)
                continue
            elif current_red_percentage >= 0.10: # 10%以上の赤色を検出した場合
                print(f"画像全体で赤色を{current_red_percentage:.2%}検出しました！少し前進します。")
                following.follow_forward(driver, bno_sensor, base_speed=90, duration_time=3)
            else:
                print(f"画像全体で赤色を検出しませんでした (割合: {current_red_percentage:.2%})。そのまま前進します。")
                following.follow_forward(driver, bno_sensor, base_speed=90, duration_time=5)

            driver.motor_stop_brake()


            # --- 周囲確認ロジック (360度スキャン - 10%/40%閾値) ---
            print("\n=== 周囲確認を開始します (360度スキャン - 回避/最終確認用) ===")
            
            if scan_360_for_red_and_act(driver, bno_sensor, picam2_instance):
                print("360度スキャンにより180度転回が行われました。ミッションを終了します。")
                break
            else:
                print("360度スキャンが完了しましたが、180度転回は行われませんでした。次の走行サイクルに進みます。")
                following.follow_forward(driver, bno_sensor, base_speed=90, duration_time=3)
                driver.motor_stop_brake()
                time.sleep(1)
                
                continue

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
