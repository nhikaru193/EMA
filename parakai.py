# main_rover_control.py (最新版 - 動的旋回と前進フェーズの統合)

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

# カスタムモジュールのインポート
from motor import MotorDriver
import following

# --- BNO055用のラッパークラス ---
class BNO055Wrapper:
    def __init__(self, adafruit_bno055_sensor):
        self.sensor = adafruit_bno055_sensor

    def get_heading(self):
        return self.sensor.euler[0]

# --- 定数設定 ---
#NICHROME_PIN = 25
#HEATING_DURATION_SECONDS = 3.0

# 目標GPS座標
destination_lat = 35.9194038
destination_lon = 139.9081183

# GPS受信ピン
RX_PIN = 17

# --- 関数定義 (省略 - 変更なし) ---
def convert_to_decimal(coord, direction):
    """NMEA形式のGPS座標を十進数に変換します。"""
    degrees = int(coord[:2]) if direction in ['N', 'S'] else int(coord[:3])
    minutes = float(coord[2:]) if direction in ['N', 'S'] else float(coord[3:])
    decimal = degrees + minutes / 60
    if direction in ['S', 'W']:
        decimal *= -1
    return decimal

def get_current_location(pi_instance, rx_pin):
    """GPSデータから現在の緯度と経度を取得します。"""
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
    raise TimeoutError("GPSデータの取得に失敗しました")

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
    
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    lower_red = np.array([0, 120, 70])
    upper_red = np.array([10, 255, 255])
    mask = cv2.inRange(hsv, lower_red, upper_red)
    
    red_pixel_count = np.sum(mask > 0)
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
    bno_sensor_for_following = BNO055Wrapper(original_bno_sensor)

    picam2_instance = Picamera2()
    picam2_instance.configure(picam2_instance.create_preview_configuration(
        main={"size": (640, 480)},
        controls={"FrameRate": 30},
        transform=Transform(rotation=90)
    ))
    picam2_instance.start()
    time.sleep(2)

    # --- 自律移動と回避ロジック ---
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

        # STEP 2: 現在地GPS取得し、目的地と比較
        print("\n=== ステップ2: GPS現在地取得と目標方位計算 ===")
        current_lat, current_lon = get_current_location(pi_instance, RX_PIN)
        print(f"現在地：緯度={current_lat:.4f}, 経度={current_lon:.4f}")
        target_gps_heading = calculate_heading(current_lat, current_lon, destination_lat, destination_lon)
        print(f"GPSに基づく目標方位：{target_gps_heading:.2f}度")

        # STEP 3: その場で回頭 (新しい動的旋回ロジック)
        print("\n=== ステップ3: 目標方位への回頭 (動的調整) ===")
        ANGLE_THRESHOLD_DEG = 20.0 # 許容する角度誤差（度）を厳しく
        turn_speed = 40 # 回転速度は固定 (0-100

        while turn_attempt_count < max_turn_attempts:
            current_bno_heading = original_bno_sensor.euler[0]
            if current_bno_heading is None:
                print("警告: 旋回中にBNO055方位が取得できませんでした。回頭を中断します。")
                break # または driver.motor_stop_brake()してエラーを発生させる

            # 角度誤差を計算し、-180から180の範囲に調整
            angle_error = (target_gps_heading - current_bno_heading + 180 + 360) % 360 - 180
            
            # 誤差が許容範囲内であればループを抜ける
            if abs(angle_error) <= ANGLE_THRESHOLD_DEG:
                print(f"[TURN] 方位調整完了。最終誤差: {angle_error:.2f}度")
                break

            # 誤差が大きい場合のみ旋回
            # angle_errorは-180から180の範囲なので、360-angle_errorのチェックは不要
            # if abs(angle_error) > ANGLE_THRESHOLD_DEG: # これは冗長
            
            # 旋回時間を動的に計算 (ご提供いただいた計算式を適用)
            # min(abs(angle_error), 360 - abs(angle_error)) を使用して、常に短い方の角度差を基準にする
            # ただし、angle_errorは既に-180から180なので、abs(angle_error)を使う
            turn_duration = 0.15 + (abs(angle_error) / 180.0) * 0.2
            # 最小時間を確保しつつ、角度に応じて線形に増加

            if angle_error < 0: # 現在向いている方向が目標より左 (反時計回り)
                print(f"[TURN] 左に回頭します (誤差: {angle_error:.2f}度, 時間: {turn_duration:.2f}秒)")
                driver.changing_left(0, turn_speed)
                driver.changing_left(turn_speed, 0)# 左旋回
            else: # 現在向いている方向が目標より右 (時計回り)
                print(f"[TURN] 右に回頭します (誤差: {angle_error:.2f}度, 時間: {turn_duration:.2f}秒)")
                driver.changing_right(0, turn_speed)
                driver.changing_right(turn_speed, 0)# 右旋回
            
            time.sleep(turn_duration) # 計算された時間だけ旋回
            driver.motor_stop_brake() # 確実な停止 (以前の motor_stop_free より良い)
            time.sleep(0.5) # 回転後の安定待ち

            turn_attempt_count += 1 # 試行回数をカウント

        # もし最大試行回数に達してしまった場合
        if turn_attempt_count >= max_turn_attempts and abs(angle_error) > ANGLE_THRESHOLD_DEG:
            print(f"警告: 最大回頭試行回数に達しましたが、目標方位に到達できませんでした。最終誤差: {angle_error:.2f}度")
        
        driver.motor_stop_brake() # 念のため停止
        time.sleep(0.5)

        # STEP 4 & 5: カメラで撮影し色検知 → 前進
        print("\n=== ステップ4&5: カメラ検知と前進 ===")
        save_image_for_debug(picam2_instance) # デバッグ用画像を保存

        red_detected_high_percentage, red_percentage_val = detect_red_object(picam2_instance, min_red_percentage=0.80)

        if red_detected_high_percentage:
            print(f"赤色高割合検出（割合: {red_percentage_val:.2%}） → パラシュートが覆いかぶさっている可能性あり。")
            print("10秒間待機して、風でパラシュートが飛ばされるのを待ちます...")
            
            start_wait_time = time.time()
            while time.time() - start_wait_time < 10:
                time.sleep(1)

            print("待機終了。再度赤色検知を試みます。")
            
            red_detected_after_wait, _ = detect_red_object(picam2_instance, min_red_percentage=0.80)

            if red_detected_after_wait:
                print("待機後も赤色を検出 → 右へ回避")
                driver.changing_right(0, 40)
                driver.motor_stop_brake()
                time.sleep(0.5)
                print("回避後、方向追従制御で前進します。(速度60, 3秒)")
                following.follow_forward(driver, bno_sensor_for_following, base_speed=60, duration_time=3)
            else:
                print("待機後、赤色を検出せず → 方向追従制御で前進します。(速度80, 5秒)")
                following.follow_forward(driver, bno_sensor_for_following, base_speed=80, duration_time=5)
                
        else:
            print("赤なし → 方向追従制御で前進します。(速度80, 5秒)")
            following.follow_forward(driver, bno_sensor_for_following, base_speed=80, duration_time=5)

        driver.motor_stop_brake()

    except TimeoutError as e:
        print(f"メイン処理中にエラーが発生しました (GPS取得タイムアウト): {e}")
        driver.motor_stop_brake()
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
