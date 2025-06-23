import RPi.GPIO as GPIO
import time
import pigpio
import board
import busio
import adafruit_bno055
import numpy as np
import cv2
from picamera2 import Picamera2
from motor import MotorDriver
import math

# --- 定数設定 ---
NICHROME_PIN = 25
HEATING_DURATION_SECONDS = 3.0
DESTINATION_LAT = 40.47
DESTINATION_LON = 119.42
GPS_RX_PIN = 17

# --- 関数定義 ---

def convert_to_decimal(coord, direction):
    degrees = int(coord[:2]) if direction in ['N', 'S'] else int(coord[:3])
    minutes = float(coord[2:]) if direction in ['N', 'S'] else float(coord[3:])
    decimal = degrees + minutes / 60
    if direction in ['S', 'W']:
        decimal *= -1
    return decimal

def get_current_location(pi, rx_pin):
    timeout = time.time() + 5
    while time.time() < timeout:
        (count, data) = pi.bb_serial_read(rx_pin)
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
                print(f"GPSデータ解析中にエラー: {e}")
                continue
        time.sleep(0.1)
    raise TimeoutError("GPSデータの取得に失敗しました")

def calculate_heading(current_lat, current_lon, dest_lat, dest_lon):
    delta_lon = math.radians(dest_lon - current_lon)
    y = math.sin(delta_lon) * math.cos(math.radians(dest_lat))
    x = math.cos(math.radians(current_lat)) * math.sin(math.radians(dest_lat)) - \
        math.sin(math.radians(current_lat)) * math.cos(math.radians(dest_lat)) * math.cos(delta_lon)
    bearing = math.degrees(math.atan2(y, x))
    return (bearing + 360) % 360

def save_and_detect_red(picam2):
    """一度だけ画像を取得し、保存と赤色検出の両方を行う"""
    frame_rgb = picam2.capture_array()
    if frame_rgb is None:
        print("画像取得失敗")
        return None, False

    # --- 画像保存処理 ---
    frame_bgr = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)
    # ファイル名はタイムスタンプでユニークに
    timestr = time.strftime("%Y%m%d-%H%M%S")
    image_path = f"/home/mark1/Pictures/paravo_image_{timestr}.jpg"
    cv2.imwrite(image_path, frame_bgr)
    print(f"画像保存成功: {image_path}")

    # --- 赤色検出処理 ---
    hsv = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2HSV)
    lower_red1 = np.array([0, 120, 70])
    upper_red1 = np.array([10, 255, 255])
    mask1 = cv2.inRange(hsv, lower_red1, upper_red1)

    lower_red2 = np.array([170, 120, 70])
    upper_red2 = np.array([180, 255, 255])
    mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
    
    mask = mask1 + mask2
    
    is_red_detected = np.sum(mask) > 5000
    return frame_rgb, is_red_detected

def activate_nichrome(pin, duration):
    """ニクロム線溶断"""
    print("ニクロム線溶断を開始します。")
    print(f"GPIO{pin} をHIGHに設定し、ニクロム線をオンにします。")
    GPIO.output(pin, GPIO.HIGH)
    
    print(f"{duration}秒間、加熱します...")
    time.sleep(duration)
    
    print(f"GPIO{pin} をLOWに設定し、ニクロム線をオフにします。")
    GPIO.output(pin, GPIO.LOW)
    print("溶断回路が正常に完了しました。")

def align_to_heading(driver, sensor, target_heading):
    """
    指定された目標方位に機体が向くまで、その場で旋回を続ける関数。
    成功した場合はTrue、タイムアウトした場合はFalseを返す。
    """
    print(f"\n目標方位 {target_heading:.1f} への旋回を開始します。")
    turn_timeout = 30  # タイムアウト時間（秒）
    turn_start_time = time.time()

    while True:
        heading = sensor.euler[0]
        if heading is None:
            print("警告: 方位取得失敗。リトライします。")
            time.sleep(0.5)
            continue

        diff = (target_heading - heading + 360) % 360
        print(f"目標方位: {target_heading:.1f}, 現在方位: {heading:.1f}, 差: {diff:.1f}")

        if 10 < diff < 180:
            driver.changing_right(0, 50)
            time.sleep(0.1)
            driver.motor_stop_brake()
        elif diff >= 180:
            driver.changing_left(0, 50)
            time.sleep(0.1)
            driver.motor_stop_brake()
        else:
            print("方位OK。")
            driver.motor_stop_brake()
            return True  # 成功して終了

        if time.time() - turn_start_time > turn_timeout:
            print("エラー: 旋回がタイムアウトしました。")
            driver.motor_stop_brake()
            return False # 失敗して終了
            
        time.sleep(0.2)


# --- メイン実行ブロック ---
def main():
    # --- 1. 初期化フェーズ ---
    driver = None
    pi = None
    picam2 = None

    try:
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        driver = MotorDriver(PWMA=12, AIN1=23, AIN2=18, PWMB=19, BIN1=16, BIN2=26, STBY=21)
        GPIO.setup(NICHROME_PIN, GPIO.OUT, initial=GPIO.LOW)
        pi = pigpio.pi()
        pi.bb_serial_read_open(GPS_RX_PIN, 9600, 8)
        i2c = busio.I2C(board.SCL, board.SDA)
        sensor = adafruit_bno055.BNO055_I2C(i2c)
        picam2 = Picamera2()
        picam2.configure(picam2.create_preview_configuration(main={"size": (640, 480)}))
        picam2.start()
        time.sleep(2)

        print("全ての初期化が完了しました。")

        # --- 2. メイン処理フェーズ ---
        
        # 2-1. ニクロム線溶断
        activate_nichrome(NICHROME_PIN, HEATING_DURATION_SECONDS)
        time.sleep(1)

        # 2-2. GPSベースのナビゲーション（目標方位へ初めて向き直す）
        target_heading = 0 # スコープ外でも使えるように初期化
        try:
            current_lat, current_lon = get_current_location(pi, GPS_RX_PIN)
            print("現在地：", current_lat, current_lon)
            target_heading = calculate_heading(current_lat, current_lon, DESTINATION_LAT, DESTINATION_LON)
            
            # ループする方位合わせ関数を呼び出す 
            align_to_heading(driver, sensor, target_heading)

        except TimeoutError as e:
            print(e)
            print("GPSが取得できなかったため、ナビゲーションをスキップします。")

        # 2-3. 画像認識と回避・最終動作
        print("\n障害物検知フェーズを開始します...")
        frame, is_red_detected = save_and_detect_red(picam2)
        
        path_is_clear = not is_red_detected

        if not path_is_clear:
            # --- 回避モード ---
            print("赤色障害物を検出！回避行動を開始します。")
            avoid_timeout = 25 # 回避のタイムアウト（秒）
            avoid_start_time = time.time()

            while time.time() - avoid_start_time < avoid_timeout:
                print("クリアな進路を探して右に旋回...")
                driver.changing_right(0, 45)
                time.sleep(0.3)
                driver.motor_stop_brake()
                time.sleep(0.5)

                _, is_red_still_detected = save_and_detect_red(picam2)
                
                if not is_red_still_detected:
                    print("進路クリア！回避行動を完了します。")
                    path_is_clear = True
                    break # 回避ループを抜ける
                else:
                    print("まだ障害物が見えます。探索を継続。")
            
            if not path_is_clear:
                print("タイムアウト：クリアな進路が見つかりませんでした。")

        # --- 最終的な前進処理 ---
        if path_is_clear:
            if not is_red_detected:
                print("前方クリア。そのまま前進します。")
            else:
                # 回避行動の直後なので、再度、目的地へ向き直す
                print("\n回避完了。再度、目標方位に機体を向け直します。")
                align_to_heading(driver, sensor, target_heading)

            # 最終的な前進
            print("\n最終シーケンス：目標へ向けて前進！")
            driver.changing_forward(0, 80)
            time.sleep(2)
        else:
            print("\n回避失敗。安全のため、これ以上前進しません。")
        
        driver.motor_stop_brake()
        print("\nミッション完了。")

    except KeyboardInterrupt:
        print("\nプログラムがユーザーによって中断されました。")
    except Exception as e:
        print(f"\n予期せぬエラーが発生しました: {e}")
    finally:
        # --- 3. 終了処理フェーズ ---
        print("終了処理を開始します...")
        if driver:
            driver.motor_stop_brake()
            driver.cleanup()
            print("モーターを停止し、クリーンアップしました。")
        if pi:
            pi.bb_serial_read_close(GPS_RX_PIN)
            pi.stop()
            print("pigpioを停止しました。")
        if picam2:
            picam2.close()
            print("カメラを解放しました。")
        
        GPIO.cleanup()
        print("GPIOのクリーンアップを実行しました。")
        print("全ての処理を終了しました。")


if __name__ == "__main__":
    main()
```
