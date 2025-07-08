# main_rover_control.py (BNO055WrapperでNone対策を完結)

import RPi.GPIO as GPIO
import time
import pigpio
import board
import busio
import adafruit_bno055
import numpy as np
import cv2
from picamera2 import Picamera2
from libcamera import Transform
import sys
import os

# カスタムモジュールのインポート
from motor import MotorDriver
import following

# --- BNO055用のラッパークラス (ここを修正します) ---
class BNO055Wrapper:
    def __init__(self, adafruit_bno055_sensor):
        self.sensor = adafruit_bno055_sensor

    def get_heading(self):
        # BNO055センサーから方位データを取得
        heading = self.sensor.euler[0]
        
        # ★★★ Noneだった場合の対策をここに集約 ★★★
        if heading is None:
            # print("警告: BNO055方位がNoneです。有効な値が取れるまで待機します。") # 頻繁に出る可能性があるのでコメントアウト
            wait_start_time = time.time()
            max_wait_time = 0.5 # 短い時間（例: 0.5秒）リトライしてみる
            
            while heading is None and (time.time() - wait_start_time < max_wait_time):
                time.sleep(0.01) # 短い間隔でポーリング
                heading = self.sensor.euler[0]
        
        if heading is None:
            # print("警告: BNO055方位が依然Noneです。今回の値は無視し、前回の値を返すか、0を返します。")
            # 継続するために、Noneのままfollowing.pyに渡すか、
            # ここで0などの仮の値を返すかを決める。
            # following.py側でNoneチェックが入っているので、そのままNoneを返してもOK
            # ただし、following.pyのNoneチェックでbreakやcontinueが発生する。
            # エラーログを見ると、計算時にNoneが渡っているはずなので、ここでは確実にfloatを返したい。
            return 0.0 # BNO055がNoneを返したら、とりあえず0を返す (注意点あり)
        
        return heading

# --- 定数設定 ---
#NICHROME_PIN = 25
#HEATING_DURATION_SECONDS = 3.0

# 目標GPS座標
destination_lat = 35.9185366
destination_lon = 139.9085042

# GPS受信ピン
RX_PIN = 17

# --- 関数定義 (既存のものを保持) ---
def convert_to_decimal(coord, direction):
    """NMEA形式のGPS座標を十進数に変換します。"""
    degrees = int(coord[:2]) if direction in ['N', 'S'] else int(coord[:3])
    minutes = float(coord[2:]) if direction in ['N', 'S'] else float(coord[3:])
    decimal = degrees + minutes / 60
    if direction in ['S', 'W']:
        decimal *= -1
    return decimal

def get_current_location(pi_instance, rx_pin):
    """GPSデータから現在の緯度と経度を取得します。
       タイムアウトした場合、None, Noneを返します。
    """
    timeout = time.time() + 5
    while time.time() < timeout:
        (count, data) = pi_instance.bb_serial_read(rx_pin)
        if count and data:
            try:
                text = data.decode("ascii", errors="ignore")
                if "$GNRMC" in text:
                    for line in text.split("\n"):
                        if "$GNRMC" in line:
                            parts = line.strip().split(",")
                            if len(parts) > 6 and parts[2] == "A":
                                lat = convert_to_decimal(parts[3], parts[4])
                                lon = convert_to_decimal(parts[5], parts[6])
                                return lat, lon
            except Exception as e:
                print(f"GPSデータ解析エラー: {e}")
                continue
        time.sleep(0.1)
    print("GPSデータの取得に失敗しました (タイムアウト)。")
    return None, None

def calculate_heading(current_lat, current_lon, dest_lat, dest_lon):
    """現在地から目的地への方位角を計算します。"""
    import math
    delta_lon = math.radians(dest_lon - current_lon)
    y = math.sin(delta_lon) * math.cos(math.radians(dest_lat))
    x = math.cos(math.radians(current_lat)) * math.sin(math.radians(dest_lat)) - \
        math.sin(math.radians(current_lat)) * math.cos(math.radians(dest_lat)) * math.cos(delta_lon)
    bearing = math.degrees(math.atan2(y, x))
    return (bearing + 360) % 360

def save_image_for_debug(picam2_instance, path="/home/mark1/Pictures/paravo_image.jpg"):
    """デバッグ用に画像を撮影して保存します。"""
    frame = picam2_instance.capture_array()
    if frame is None:
        print("画像キャプチャ失敗：フレームがNoneです。")
        return None
    frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
    cv2.imwrite(path, frame_bgr)
    print(f"画像保存成功: {path}")
    return frame

def detect_red_object(picam2_instance, min_red_percentage=0.80):
    """
    カメラ画像から赤色物体を検出し、その割合を返します。
    """
    frame = picam2_instance.capture_array()
    if frame is None:
        print("画像取得失敗: フレームがNoneです。")
        return False, 0.0
    
    frame = cv2.GaussianBlur(frame, (5, 5), 0)
    
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    lower_red1 = np.array([0, 100, 100])
    upper_red1 = np.array([10, 255, 255])
    mask1 = cv2.inRange(hsv, lower_red1, upper_red1)

    lower_red2 = np.array([160, 100, 100])
    upper_red2 = np.array([180, 255, 255])
    mask2 = cv2.inRange(hsv, lower_red2, upper_red2)

    mask = cv2.bitwise_or(mask1, mask2)
    
    red_pixel_count = np.count_nonzero(mask)
    total_pixels = frame.shape[0] * frame.shape[1]
    red_percentage = red_pixel_count / total_pixels if total_pixels > 0 else 0.0
    
    print(f"赤色ピクセル割合: {red_percentage:.2%}")

    return red_percentage >= min_red_percentage, red_percentage

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
    pi_instance.bb_serial_read_open(RX_PIN, 9600, 8)

    i2c_bus = busio.I2C(board.SCL, board.SDA)
    original_bno_sensor = adafruit_bno055.BNO055_I2C(i2c_bus)
    bno_sensor_for_following = BNO055Wrapper(original_bno_sensor) # ラッパーを作成

    picam2_instance = Picamera2()
    picam2_instance.configure(picam2_instance.create_preview_configuration(
        main={"size": (640, 480)},
        controls={"FrameRate": 30}
    ))
    picam2_instance.start()
    time.sleep(2)

    # --- メイン制御ループ ---
    try:
        # STEP 1: BNO055 キャリブレーション
        print("\n=== ステップ1: BNO055キャリブレーション開始 ===")
        print("センサーを動かしてキャリブレーションを進めてください...")
        while True:
            sys_cal, gyro_cal, accel_cal, mag_cal = original_bno_sensor.calibration_status
            print(f"Calib → Sys:{sys_cal}, Gyro:{gyro_cal}, Acc:{accel_cal}, Mag:{mag_cal}", end='\r')
            sys.stdout.flush()
            
            if gyro_cal == 3 and mag_cal == 3:
                print("\nキャリブレーション完了！")
                break
            time.sleep(0.1)

        # メインの自律走行ループ
        while True:
            print("\n--- 新しい走行サイクル開始 ---")
            
            # STEP 2: 現在地GPS取得し、目標方位計算
            print("\n=== ステップ2: GPS現在地取得と目標方位計算 ===")
            current_lat, current_lon = get_current_location(pi_instance, RX_PIN)
            
            if current_lat is None or current_lon is None:
                print("GPSデータが取得できませんでした。リトライします...")
                time.sleep(2)
                continue

            print(f"現在地：緯度={current_lat:.4f}, 経度={current_lon:.4f}")
            target_gps_heading = calculate_heading(current_lat, current_lon, destination_lat, destination_lon)
            print(f"GPSに基づく目標方位：{target_gps_heading:.2f}度")

            # STEP 3: その場で回頭 (動的調整)
            print("\n=== ステップ3: 目標方位への回頭 (動的調整) ===")
            ANGLE_THRESHOLD_DEG = 5.0
            turn_speed = 40

            max_turn_attempts = 100
            turn_attempt_count = 0

            while turn_attempt_count < max_turn_attempts:
                # ここでは original_bno_sensor を直接使用し、Noneチェックを行う
                current_bno_heading = original_bno_sensor.euler[0]
                if current_bno_heading is None:
                    print("警告: 旋回中にBNO055方位が取得できませんでした。リトライします。")
                    driver.motor_stop_brake() # 安全のため停止
                    time.sleep(1) # センサー回復を待つ
                    turn_attempt_count += 1 # リトライとしてカウント
                    continue # 次の試行へ

                angle_error = (target_gps_heading - current_bno_heading + 180 + 360) % 360 - 180
                
                if abs(angle_error) <= ANGLE_THRESHOLD_DEG:
                    print(f"[TURN] 方位調整完了。最終誤差: {angle_error:.2f}度")
                    break

                turn_duration = 0.15 + (abs(angle_error) / 180.0) * 0.2

                if angle_error < 0:
                    print(f"[TURN] 左に回頭します (誤差: {angle_error:.2f}度, 時間: {turn_duration:.2f}秒)")
                    driver.changing_left(0, turn_speed)
                else:
                    print(f"[TURN] 右に回頭します (誤差: {angle_error:.2f}度, 時間: {turn_duration:.2f}秒)")
                    driver.changing_right(0, turn_speed)
                
                time.sleep(turn_duration)
                driver.motor_stop_brake()
                time.sleep(0.5)

                turn_attempt_count += 1

            if turn_attempt_count >= max_turn_attempts and abs(angle_error) > ANGLE_THRESHOLD_DEG:
                print(f"警告: 最大回頭試行回数に達しましたが、目標方位に到達できませんでした。最終誤差: {angle_error:.2f}度")
            
            driver.motor_stop_brake()
            time.sleep(0.5)

            # STEP 4 & 5: カメラ検知と前進
            print("\n=== ステップ4&5: カメラ検知と前進 ===")
            
            red_detected_high_percentage, red_percentage_val = \
                save_and_process_single_image(picam2_instance, save_path="/home/mark1/Pictures/akairo.jpg", min_red_percentage=0.80)

            if red_detected_high_percentage:
                print(f"赤色高割合検出（割合: {red_percentage_val:.2%}） → パラシュートが覆いかぶさっている可能性あり。")
                print("10秒間待機して、風でパラシュートが飛ばされるのを待ちます...")
                
                start_wait_time = time.time()
                while time.time() - start_wait_time < 10:
                    time.sleep(1)

                print("待機終了。再度赤色検知を試みます。")
                
                red_detected_after_wait, _ = \
                    save_and_process_single_image(picam2_instance, save_path="/home/mark1/Pictures/akairo_after_wait.jpg", min_red_percentage=0.80)

                if red_detected_after_wait:
                    print("待機後も赤色を検出 → 右へ回避")
                    driver.changing_right(0, 40)
                    driver.motor_stop_brake()
                    time.sleep(0.5)
                    print("回避後、方向追従制御で前進します。(速度60, 3秒)")
                    # ここで following.follow_forward を呼び出すが、bno_sensor_for_following は get_heading() がNoneを返さないようにする
                    following.follow_forward(driver, bno_sensor_for_following, base_speed=60, duration_time=3)
                else:
                    print("待機後、赤色を検出せず → 方向追従制御で前進します。(速度80, 5秒)")
                    following.follow_forward(driver, bno_sensor_for_following, base_speed=80, duration_time=5)
                    
            else:
                print("赤なし → 方向追従制御で前進します。(速度80, 5秒)")
                following.follow_forward(driver, bno_sensor_for_following, base_speed=80, duration_time=5)

            driver.motor_stop_brake()

            print("\n=== 前進処理が完了しました。プログラムを終了します。 ===")
            break

    except Exception as e:
        print(f"メイン処理中に予期せぬエラーが発生しました: {e}")
        driver.motor_stop_brake()

    finally:
        print("\n=== 終了処理中 ===")
        if 'driver' in locals():
            driver.cleanup()
        if 'pi_instance' in locals() and pi_instance.connected:
            pi_instance.bb_serial_read_close(RX_PIN)
            pi_instance.stop()
        if 'picam2_instance' in locals():
            picam2_instance.close()
        
        GPIO.cleanup()
        print("=== 処理を終了しました。 ===")
