# main_rover_control.py
import RPi.GPIO as GPIO
import time
import pigpio
import board
import busio
import adafruit_bno055
import numpy as np
import cv2
from picamera2 import Picamera2

# カスタムモジュールのインポート
from motor import MotorDriver   # motor.py から MotorDriver クラスをインポート
import following                # following.py モジュールをインポート

# --- 定数設定 ---
#NICHROME_PIN = 25              # ニクロム線が接続されているGPIOピン
#HEATING_DURATION_SECONDS = 3.0 # ニクロム線をオンにしておく時間（秒）

# 目標GPS座標
destination_lat = 40.47
destination_lon = 119.42

# GPS受信ピン
RX_PIN = 17

# --- 関数定義 ---

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
    timeout = time.time() + 5 # 5秒のタイムアウト
    while time.time() < timeout:
        (count, data) = pi_instance.bb_serial_read(rx_pin)
        if count and data:
            try:
                text = data.decode("ascii", errors="ignore")
                if "$GNRMC" in text:
                    for line in text.split("\n"):
                        if "$GNRMC" in line:
                            parts = line.strip().split(",")
                            if len(parts) > 6 and parts[2] == "A": # "A"はデータが有効であることを示す
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
    import math # 関数内でインポート
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
    
    red_pixel_count = np.sum(mask > 0) # マスク内の非ゼロピクセル数をカウント
    total_pixels = frame.shape[0] * frame.shape[1] # 画像全体のピクセル数
    red_percentage = red_pixel_count / total_pixels if total_pixels > 0 else 0.0
    
    print(f"赤色ピクセル割合: {red_percentage:.2%}")

    return red_percentage >= min_red_percentage, red_percentage

# --- メインシーケンス ---
if __name__ == "__main__":
    # GPIO設定
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False) # 警告を非表示に

    # ニクロム線溶断シーケンス
    #GPIO.setup(NICHROME_PIN, GPIO.OUT, initial=GPIO.LOW)
    #print("ニクロム線溶断シーケンスを開始します。")
    #try:
        #print(f"GPIO{NICHROME_PIN} をHIGHに設定し、ニクロム線をオンにします。")
        #GPIO.output(NICHROME_PIN, GPIO.HIGH)
        #print(f"{HEATING_DURATION_SECONDS}秒間、加熱します...")
        #time.sleep(HEATING_DURATION_SECONDS)
        #print(f"GPIO{NICHROME_PIN} をLOWに設定し、ニクロム線をオフにします。")
        #GPIO.output(NICHROME_PIN, GPIO.LOW)
        #print("シーケンスが正常に完了しました。")
    #except KeyboardInterrupt:
        #print("ニクロム線溶断シーケンスが中断されました。")
        #GPIO.output(NICHROME_PIN, GPIO.LOW)
    #finally:
        #print("GPIO25の溶断ピンのクリーンアップを実行しました。")
        # GPIO.cleanup() は最後にまとめて実行

    # デバイス初期化
    driver = MotorDriver(
        PWMA=12, AIN1=23, AIN2=18,    # 左モーター
        PWMB=19, BIN1=16, BIN2=26,    # 右モーター
        STBY=21                       # STBYピン
    )
    pi_instance = pigpio.pi()
    if not pi_instance.connected:
        print("pigpioデーモンに接続できません。終了します。")
        exit()
    pi_instance.bb_serial_read_open(RX_PIN, 9600, 8)

    i2c_bus = busio.I2C(board.SCL, board.SDA)
    bno_sensor = adafruit_bno055.BNO055_I2C(i2c_bus) # BNO055センサーオブジェクト

    picam2_instance = Picamera2()
    picam2_instance.configure(picam2_instance.create_preview_configuration(main={"size": (640, 480)}, controls={"FrameRate": 30}), rotation=90) picam2_instance.start()
    time.sleep(2) # カメラ起動待ち

    # --- 自律移動と回避ロジック ---
    try:
        # デバッグ用画像を保存
        save_image_for_debug(picam2_instance)

        # GPSで現在地と目標方位を取得
        current_lat, current_lon = get_current_location(pi_instance, RX_PIN)
        print(f"現在地：緯度={current_lat:.4f}, 経度={current_lon:.4f}")
        target_gps_heading = calculate_heading(current_lat, current_lon, destination_lat, destination_lon)
        print(f"GPSに基づく目標方位：{target_gps_heading:.2f}度")

        # 現在のBNO055方位と比較し、旋回
        current_bno_heading = bno_sensor.euler[0]
        if current_bno_heading is None:
            print("警告: BNO055から現在の方位が取得できませんでした。0度を仮定します。")
            current_bno_heading = 0
        print(f"BNO055に基づく現在の方位：{current_bno_heading:.2f}度")

        # GPS目標方位とBNO055方位の差を計算して旋回
        # 簡易的な旋回ロジック。より正確にはPID制御で旋回することも可能
        diff_heading = (target_gps_heading - current_bno_heading + 360) % 360
        print(f"方位差: {diff_heading:.2f}度")

        if 10 < diff_heading < 180: # 10度以上の右ズレ
            print("方位調整: 右旋回")
            driver.changing_right(0, 40)
            time.sleep(2) # 旋回のための固定時間
            driver.motor_stop_brake()
            time.sleep(0.5)
        elif diff_heading >= 180 or diff_heading < -10: # 180度以上のズレ、または-10度以下の左ズレ
            print("方位調整: 左旋回")
            driver.changing_left(0, 40)
            time.sleep(2) # 旋回のための固定時間
            driver.motor_stop_brake()
            time.sleep(0.5)
        else:
            print("方位調整: OK (誤差±10度以内)")
            driver.motor_stop_brake()
            time.sleep(0.5)

        # --- 赤色検知とパラシュート回避ロジック ---
        red_detected_high_percentage, red_percentage_val = detect_red_object(picam2_instance, min_red_percentage=0.80)

        if red_detected_high_percentage:
            print(f"赤色高割合検出（割合: {red_percentage_val:.2%}） → パラシュートが覆いかぶさっている可能性あり。")
            print("10秒間待機して、風でパラシュートが飛ばされるのを待ちます...")
            
            start_wait_time = time.time()
            while time.time() - start_wait_time < 10: # 10秒間待機
                time.sleep(1) # 1秒ごとにチェック
                # 待機中に何か特別なことをするならここに追加

            print("待機終了。再度赤色検知を試みます。")
            
            # 待機後に再度赤色検知
            red_detected_after_wait, _ = detect_red_object(picam2_instance, min_red_percentage=0.80)

            if red_detected_after_wait:
                print("待機後も赤色を検出 → 右へ回避")
                driver.changing_right(0, 40) # 回避旋回
                driver.motor_stop_brake()
                time.sleep(0.5)
                # 回避後の前進に方向追従制御を使用
                print("回避後、方向追従制御で前進します。(速度60, 3秒)")
                following.follow_forward(driver, bno_sensor, base_speed=60, duration_time=3) 
            else:
                print("待機後、赤色を検出せず → 方向追従制御で前進します。(速度80, 5秒)")
                # 赤色がなくなれば、通常の前進に方向追従制御を使用
                following.follow_forward(driver, bno_sensor, base_speed=80, duration_time=5)
                
        else:
            print("赤なし → 方向追従制御で前進します。(速度80, 5秒)")
            # 赤色が検出されなければ、通常の前進に方向追従制御を使用
            following.follow_forward(driver, bno_sensor, base_speed=80, duration_time=5)

        driver.motor_stop_brake() # すべての動作完了後に停止

    except TimeoutError as e:
        print(e)
        print("GPS取得失敗しましたが、画像は保存されています。")
        driver.motor_stop_brake() # エラー時もモーター停止
    except Exception as e:
        print(f"メイン処理中に予期せぬエラーが発生しました: {e}")
        driver.motor_stop_brake() # エラー時もモーター停止

    finally:
        print("\n=== 終了処理中 ===")
        # 各デバイスのリソースを解放
        if 'driver' in locals():
            driver.cleanup()
        if 'pi_instance' in locals() and pi_instance.connected:
            pi_instance.bb_serial_read_close(RX_PIN)
            pi_instance.stop()
        if 'picam2_instance' in locals():
            picam2_instance.close()
        
        GPIO.cleanup() # 最後にまとめてGPIOをクリーンアップ
        print("=== 処理を終了しました。 ===")
