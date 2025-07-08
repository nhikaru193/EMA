# main_rover_control.py (カメラ撮影と赤色検知を新しい関数に統合)

import RPi.GPIO as GPIO
import time
import pigpio
import board
import busio
import adafruit_bno055
import numpy as np
import cv2
from picamera2 import Picamera2
# from libcamera import Transform # transformを削除したので不要になります
import sys
import os # osモジュールはディレクトリ作成のために必要です

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

# --- 新しいカメラ撮影・保存・赤色検知関数 ---
def save_and_process_single_image(picam2_instance, save_path="/home/mark1/Pictures/akairo.jpg", min_red_percentage=0.80):
    """
    カメラから一度だけ画像をキャプチャし、指定されたパスに保存します。
    保存後、その画像に対して赤色検知処理を行い、結果を返します。
    キャプチャした画像を反時計回りに90度回転させてから処理します。

    Args:
        picam2_instance: Picamera2のインスタンス。
        save_path (str): 画像を保存するフルパス。
        min_red_percentage (float): 赤色と判断するピクセル割合の閾値 (0.0〜1.0)。

    Returns:
        tuple: (赤色が高割合で検出されたか(bool), 赤色の実際のピクセル割合(float))
    """
    try:
        # ディレクトリが存在するか確認し、なければ作成
        directory = os.path.dirname(save_path)
        if not os.path.exists(directory):
            os.makedirs(directory)
            print(f"ディレクトリを作成しました: {directory}")

        print(f"画像をキャプチャし、{save_path}に保存します...")
        
        # 画像をキャプチャ (Picamera2はデフォルトでRGB形式のNumPy配列を返す)
        frame_rgb = picam2_instance.capture_array()
        
        if frame_rgb is None:
            print("画像キャプチャ失敗: フレームがNoneです。")
            return False, 0.0
            
        # RGBからBGRに変換 (OpenCVがBGRを期待するため)
        frame_bgr = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)

        # --- 回転処理を追加 ---
        # 時計回りに90度傾いているので、反時計回りに90度（または時計回りに270度）回転させる
        rotated_frame_bgr = cv2.rotate(frame_bgr, cv2.ROTATE_90_COUNTERCLOCKWISE)
        print("画像を反時計回りに90度回転させました。")
        # --- 回転処理ここまで ---

        # 回転した画像を保存
        cv2.imwrite(save_path, rotated_frame_bgr)
        print(f"画像を保存しました: {save_path}")

        # --- 保存した画像 (回転後の画像) に対して赤色検知処理を行う ---
        print("保存された画像に対して赤色検知を開始します...")

        # ガウシアンブラーを適用してノイズを減らす
        blurred_frame = cv2.GaussianBlur(rotated_frame_bgr, (5, 5), 0)
        
        # BGRからHSV色空間に変換 (回転後の画像を使用)
        hsv = cv2.cvtColor(blurred_frame, cv2.COLOR_BGR2HSV)

        # 赤色のHSV範囲を定義 (ご希望の範囲)
        lower_red1 = np.array([0, 100, 100])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([160, 100, 100]) # 一般的な赤ではない範囲なので、意図をご確認ください
        upper_red2 = np.array([180, 255, 255])

        # マスクを作成し結合
        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask = cv2.bitwise_or(mask1, mask2)

        # 赤色領域のピクセル数をカウント
        red_pixels = cv2.countNonZero(mask)

        # 赤色ピクセルの割合を計算
        height, width, _ = rotated_frame_bgr.shape
        total_pixels = height * width
        
        red_percentage = (red_pixels / total_pixels) if total_pixels > 0 else 0.0
        print(f"赤色ピクセル割合: {red_percentage:.2%}")

        # 結果をウィンドウ表示（自動実行時はコメントアウト推奨）
        # res = cv2.bitwise_and(rotated_frame_bgr, rotated_frame_bgr, mask=mask)
        # cv2.imshow('Captured Original (Rotated)', rotated_frame_bgr)
        # cv2.imshow('Red Mask', mask)
        # cv2.imshow('Red Detected', res)
        # cv2.waitKey(0)
        # cv2.destroyAllWindows()

        return red_percentage >= min_red_percentage, red_percentage # フラグと割合を返す

    except Exception as e:
        print(f"カメラ撮影・処理中にエラーが発生しました: {e}")
        return False, 0.0 # エラー時は検出失敗として返す


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
    # Picamera2のconfigureからtransformを削除 (ソフトウェア回転に任せるため)
    picam2_instance.configure(picam2_instance.create_preview_configuration(
        main={"size": (640, 480)},
        controls={"FrameRate": 30}
    ))
    picam2_instance.start()
    time.sleep(2)

    # --- メイン制御ループ ---
    try:
        # STEP 1: BNO055 キャリブレーション (プログラム開始時に一度だけ実行)
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
        while True: # GPSが取れるまで、または目標達成まで継続
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
                current_bno_heading = original_bno_sensor.euler[0]
                if current_bno_heading is None:
                    print("警告: 旋回中にBNO055方位が取得できませんでした。回頭を中断します。")
                    driver.motor_stop_brake()
                    time.sleep(1)
                    continue

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

            # STEP 4 & 5: カメラで撮影し色検知 → 前進
            print("\n=== ステップ4&5: カメラ検知と前進 ===")
            # save_image_for_debug(picam2_instance) # この呼び出しは新しい関数に置き換えられる

            # 新しい関数で画像処理と赤色検知を行う
            red_detected_high_percentage, red_percentage_val = \
                save_and_process_single_image(picam2_instance, save_path="/home/mark1/Pictures/akairo.jpg", min_red_percentage=0.80)

            if red_detected_high_percentage:
                print(f"赤色高割合検出（割合: {red_percentage_val:.2%}） → パラシュートが覆いかぶさっている可能性あり。")
                print("10秒間待機して、風でパラシュートが飛ばされるのを待ちます...")
                
                start_wait_time = time.time()
                while time.time() - start_wait_time < 10:
                    time.sleep(1)

                print("待機終了。再度赤色検知を試みます。")
                
                # 待機後に再度、新しい関数で赤色検知
                red_detected_after_wait, _ = \
                    save_and_process_single_image(picam2_instance, save_path="/home/mark1/Pictures/akairo_after_wait.jpg", min_red_percentage=0.80)

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
