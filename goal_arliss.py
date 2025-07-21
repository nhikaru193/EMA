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

        frame_bgr = cv2.cvtColor(frame_rgb, cv2.COLOR_BGR2HSV) # HSVに変換
        
        # ★★★ ここで画像を回転・反転させる ★★★
        # 1. 反時計回りに90度回転 (カメラが物理的に時計回りに90度傾いている場合)
        processed_frame_hsv = cv2.rotate(frame_bgr, cv2.ROTATE_90_COUNTERCLOCKWISE)
        
        # 2. 左右反転 (水平フリップ)
        processed_frame_hsv = cv2.flip(processed_frame_hsv, 1) # 1は水平フリップ (左右反転)
        
        height, width, _ = processed_frame_hsv.shape
        total_pixels = height * width

        lower_red1 = np.array([0, 100, 100])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([160, 100, 100])
        upper_red2 = np.array([180, 255, 255])

        # blurred_frame = cv2.GaussianBlur(processed_frame_hsv, (5, 5), 0) # HSVなのでHSVのまま処理
        mask = cv2.bitwise_or(cv2.inRange(processed_frame_hsv, lower_red1, upper_red1),
                              cv2.inRange(processed_frame_hsv, lower_red2, upper_red2))
        
        red_pixels = np.count_nonzero(mask)
        red_percentage = red_pixels / total_pixels if total_pixels > 0 else 0.0

        # デバッグ用に赤色領域をハイライトした画像を保存 (HSVからBGRに戻す)
        debug_frame_bgr = cv2.cvtColor(processed_frame_hsv, cv2.COLOR_HSV2BGR)
        red_highlighted_frame = cv2.bitwise_and(debug_frame_bgr, debug_frame_bgr, mask=mask)

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

# --- 周囲確認ロジック (360度スキャン - 40%閾値、180度回転で終了) ---
# この関数は独立させて、メインループの最終確認として呼び出す
def perform_final_scan_and_terminate(driver, bno_sensor_instance, picam2_instance, turn_angle_step=20, final_threshold=0.40):
    """
    ローバーを20度ずつ360度回転させ、40%以上の赤色を検知したら180度転回し、Trueを返して処理を終了します。
    検知しなかった場合はFalseを返します。
    """
    print("\n=== 最終確認スキャンを開始します (40%閾値、180度回転で終了) ===")
    initial_heading = bno_sensor_instance.get_heading()
    if initial_heading is None:
        print("警告: 最終確認スキャン開始時に方位が取得できませんでした。")
        return False

    # 最初に20度回転してから検知を開始
    print(f"  初回回転: {turn_angle_step}度...")
    turn_to_relative_angle(driver, bno_sensor_instance, turn_angle_step, turn_speed=60, angle_tolerance_deg=15)
    
    for i in range(360 // turn_angle_step):
        # 最初のループ(i=0)はすでに20度回転済みなので、それ以降(i>0)に20度回転
        if i > 0:
            print(f"  --> スキャン中: さらに20度回転...")
            turn_to_relative_angle(driver, bno_sensor_instance, turn_angle_step, turn_speed=60, angle_tolerance_deg=15)
            driver.motor_stop_brake()
            time.sleep(0.5)
        
        current_scan_heading = bno_sensor_instance.get_heading()
        if current_scan_heading is None:
            print("警告: 最終確認スキャン中に方位が取得できませんでした。スキップします。")
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

        if overall_red_ratio >= final_threshold: # 40%以上の赤色を検出
            print(f"  --> 赤色を{final_threshold:.0%}以上検出！180度転回します。")
            turn_to_relative_angle(driver, bno_sensor_instance, 180, turn_speed=90, angle_tolerance_deg=10)
            driver.motor_stop_brake()
            time.sleep(1.0)
            print("  --> 180度転回を完了しました。プログラムを終了します。")
            return True # 180度転回が行われたらTrueを返す

        driver.motor_stop_brake()
        time.sleep(0.5)

    print("\n=== 最終確認スキャンが完了しました。40%以上の赤色は検出されませんでした。 ===")
    return False # 180度転回が行われなかった場合

# --- 初期アライメントスキャン関数 ---
def perform_initial_alignment_scan(driver, bno_sensor_instance, picam2_instance, turn_angle_step=20, alignment_threshold=0.20):
    """
    ローバーを20度ずつ360度回転させ、20%以上の赤色を検知したらその方向で回転を停止し、向きを合わせます。
    20%以上の赤色が検知されなかった場合は、最も多くの赤が検知された方向に向きを合わせてからTrueを返します。
    戻り値: (aligned_successfully:bool, aligned_heading:float)
    """
    print("\n=== 初期赤色アライメントスキャンを開始します (20%閾値) ===")
    initial_heading_at_start = bno_sensor_instance.get_heading()
    if initial_heading_at_start is None:
        print("警告: 初期アライメントスキャン開始時に方位が取得できません。")
        return False, None

    aligned = False
    max_red_ratio = -1.0
    best_heading_for_red = initial_heading_at_start 

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

        # 最大赤色割合とそれに対応する絶対方位を更新
        if overall_red_ratio > max_red_ratio:
            max_red_ratio = overall_red_ratio
            best_heading_for_red = current_scan_heading 

        if overall_red_ratio >= alignment_threshold: # 20%以上の赤色を検出
            print(f"  --> 赤色を{alignment_threshold:.0%}以上検出！この方向にアライメントしました。")
            aligned = True
            driver.motor_stop_brake()
            time.sleep(1.0) # 停止して向きを確定
            # この時点の向き (current_scan_heading) がアライメントされた向き
            best_heading_for_red = current_scan_heading # 厳密にここでアライメントされた向きを記憶
            break # 20%以上の赤色が見つかったらスキャンを終了し、その向きで停止

        driver.motor_stop_brake()
        time.sleep(0.5)

        # 最初のループ (i=0) は既に20度回転済みなので、それ以降 (i>0) に20度回転
        # この回転処理はループの最後で行うことで、写真撮影と検知が終わってから次の回転に移れる
        if i < (360 // turn_angle_step) - 1: # 最後のループでは回転しない
            print(f"  回転: {turn_angle_step}度...")
            turn_to_relative_angle(driver, bno_sensor_instance, turn_angle_step, turn_speed=60, angle_tolerance_deg=15)


    if not aligned:
        print(f"初期アライメントスキャンで{alignment_threshold:.0%}以上の赤色は検出されませんでした。")
        if max_red_ratio > -1.0: # 何らかの赤色が検出されていた場合
            # 最も多くの赤があった方向へ回頭
            current_heading_at_end_of_scan = bno_sensor_instance.get_heading()
            
            if current_heading_at_end_of_scan is not None:
                # 現在の向きから最も赤があった方向への相対回転量を計算
                # best_heading_for_redは、スキャン中に最も赤があった絶対方位
                angle_to_turn_to_best_red = (best_heading_for_red - current_heading_at_end_of_scan + 180 + 360) % 360 - 180
                
                # SyntaxErrorを修正したf-string
                print(f"  --> {alignment_threshold:.0%}"
                      f"以上は検出されませんでしたが、最も多くの赤 ({max_red_ratio:.2%}) が検出された方向 ({best_heading_for_red:.2f}度) へアライメントします (相対回転: {angle_to_turn_to_best_red:.2f}度)。")
                turn_to_relative_angle(driver, bno_sensor_instance, angle_to_turn_to_best_red, turn_speed=60, angle_tolerance_deg=15)
                driver.motor_stop_brake()
                time.sleep(0.5)
                aligned = True # 最大の赤があった方向へアライメントされたとみなす
            else:
                print("警告: スキャン終了時に方位が取得できず、最大赤色方向へのアライメントができませんでした。")
        else:
            print("初期アライメントスキャンで赤色は全く検出されませんでした。")
    
    print("=== 初期赤色アライメントスキャンが完了しました。 ===")
    return aligned, best_heading_for_red # 成功フラグとアライメントした方位角を返す


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
        # BNO055キャリブレーション待機部分は削除済み
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

            # ★★★ 初期アライメントスキャンを実行 ★★★
            # アライメント成功/失敗、およびアライメントした方位角を受け取る
            aligned_in_initial_scan, initial_aligned_heading = perform_initial_alignment_scan(driver, bno_sensor, picam2_instance)

            # アライメント成功/失敗のログ出力
            if aligned_in_initial_scan:
                print(f"初期アライメントスキャンによって、何らかの赤色へアライメントされました。方位: {initial_aligned_heading:.2f}度")
            else:
                print("初期アライメントスキャンで赤色は全く検出されませんでした。")
                
            # --- アライメント後、360度スキャンで赤色を探索し前進判断 ---
            print("\n=== アライメント後、360度スキャンで赤色を探索し前進判断 ===")
            
            scanned_and_moved_after_alignment = False # このブロックで前進したか
            
            # 最初に20度回転してから検知を開始 (アライメント後の最初の動き)
            print("  --> 前進判断のため、20度回転します...")
            turn_to_relative_angle(driver, bno_sensor, 20, turn_speed=60, angle_tolerance_deg=15)
            driver.motor_stop_brake()
            time.sleep(0.5)

            # 360度を20度ずつスキャンし、赤色を検知したら1秒前進
            for i in range(360 // 20):
                # 最初のループ(i=0)はすでに20度回転済みなので、それ以降(i>0)に20度回転
                if i > 0:
                    print(f"  --> スキャン中: さらに20度回転...")
                    turn_to_relative_angle(driver, bno_sensor, 20, turn_speed=60, angle_tolerance_deg=15)
                    driver.motor_stop_brake()
                    time.sleep(0.5)
                
                current_scan_heading_for_forward = bno_sensor.get_heading()
                if current_scan_heading_for_forward is None:
                    print("警告: 360度スキャン中に方位が取得できませんでした。スキップします。")
                    continue

                print(f"--- 360度スキャン中: 現在の方向: {current_scan_heading_for_forward:.2f}度 ---")

                # 赤色を検出（10%閾値）
                current_red_percentage_scan = detect_red_percentage(
                    picam2_instance, 
                    save_path=f"/home/mark1/Pictures/forward_scan_{i*20 + 20:03d}.jpg" # 20度ずつ回転した後の画像
                )

                if current_red_percentage_scan == -1.0:
                    print("カメラ処理でエラーが発生しました。現在のスキャンステップをスキップします。")
                    continue
                
                # 10%以上の赤色を検知したら1秒前進
                if current_red_percentage_scan >= 0.10: # 10%閾値
                    print(f"  --> 赤色を{0.10:.0%}以上検出！この方向に1秒前進します。")
                    following.follow_forward(driver, bno_sensor, base_speed=90, duration_time=1) # 1秒前進
                    driver.motor_stop_brake()
                    time.sleep(0.5)
                    scanned_and_moved_after_alignment = True
                    break # 赤色を検知して前進したので、この360度スキャンを終了

            if not scanned_and_moved_after_alignment:
                print("アライメント後の360度スキャンで赤色を検出しませんでした (10%閾値未満)。5秒間前進します。")
                following.follow_forward(driver, bno_sensor, base_speed=90, duration_time=1) # 赤色なしで5秒前進
                driver.motor_stop_brake()
                time.sleep(0.5)


            # --- 周囲確認ロジック (360度スキャン - 40%閾値、180度回転で終了) ---
            # この部分は関数 perform_final_scan_and_terminate に集約します
            print("\n=== 周囲確認を開始します (360度スキャン - 最終確認用) ===")
            
            if perform_final_scan_and_terminate(driver, bno_sensor, picam2_instance):
                print("最終確認スキャンにより180度転回が行われました。ミッションを終了します。")
                break # プログラムを終了
            else:
                print("最終確認スキャンが完了しましたが、180度転回は行われませんでした。次の走行サイクルに進みます。")
                following.follow_forward(driver, bno_sensor, base_speed=90, duration_time=1)
                driver.motor_stop_brake()
                time.sleep(1)
                
                continue # メインループの最初に戻る

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
