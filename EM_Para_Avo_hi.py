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
    # (元のコードと同じだが、引数を追加)
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
    image_path = "/home/mark1/Pictures/paravo_image.jpg"
    cv2.imwrite(image_path, frame_bgr)
    print(f"画像保存成功: {image_path}")

    # --- 赤色検出処理 ---
    hsv = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2HSV) # BGRではなくRGBから変換
    # 赤色の範囲を2つに分けると、より検出精度が上がることがあります
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
    """ニクロム線溶断シーケンス"""
    print("ニクロム線溶断シーケンスを開始します。")
    print(f"GPIO{pin} をHIGHに設定し、ニクロム線をオンにします。")
    GPIO.output(pin, GPIO.HIGH)
    
    print(f"{duration}秒間、加熱します...")
    time.sleep(duration)
    
    print(f"GPIO{pin} をLOWに設定し、ニクロム線をオフにします。")
    GPIO.output(pin, GPIO.LOW)
    print("溶断シーケンスが正常に完了しました。")


# --- メイン実行ブロック ---
def main():
    # --- 1. 初期化フェーズ ---
    driver = None
    pi = None
    picam2 = None

    try:
        # GPIOモード設定（プログラム開始時に一度だけ）
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)

        # モータードライバー
        driver = MotorDriver(PWMA=12, AIN1=23, AIN2=18, PWMB=19, BIN1=16, BIN2=26, STBY=21)
        
        # ニクロム線ピン設定
        GPIO.setup(NICHROME_PIN, GPIO.OUT, initial=GPIO.LOW)

        # GPS (pigpio)
        pi = pigpio.pi()
        pi.bb_serial_read_open(GPS_RX_PIN, 9600, 8)

        # BNO055（方位センサー）
        i2c = busio.I2C(board.SCL, board.SDA)
        sensor = adafruit_bno055.BNO055_I2C(i2c)

        # Picamera2
        picam2 = Picamera2()
        picam2.configure(picam2.create_preview_configuration(main={"size": (640, 480)}))
        picam2.start()
        time.sleep(2) # カメラの安定待ち

        print("全ての初期化が完了しました。")

        # --- 2. メイン処理フェーズ ---
        
        # 2-1. ニクロム線溶断
        activate_nichrome(NICHROME_PIN, HEATING_DURATION_SECONDS)
        time.sleep(1) # 溶断後の待機

        # 2-2. GPSベースのナビゲーション
        try:
            current_lat, current_lon = get_current_location(pi, GPS_RX_PIN)
            print("現在地：", current_lat, current_lon)

            target_heading = calculate_heading(current_lat, current_lon, DESTINATION_LAT, DESTINATION_LON)
            print("目標方位：", target_heading)

            heading = sensor.euler[0]
            if heading is None:
                print("警告: 方位が取得できませんでした。0として扱います。")
                heading = 0
            print("現在の方位：", heading)

            diff = (target_heading - heading + 360) % 360
            if 10 < diff < 180:
                print("目標へ向けて右旋回")
                driver.changing_right(0, 40)
            elif diff >= 180:
                print("目標へ向けて左旋回")
                driver.changing_left(0, 40)
            else:
                print("方位OK")
                driver.motor_stop_brake()
            
            time.sleep(2) # 旋回後の待機

        except TimeoutError as e:
            print(e)
            print("GPSが取得できなかったため、ナビゲーションをスキップします。")

        # 2-3. 画像認識と最終動作
        frame, is_red_detected = save_and_detect_red(picam2)
        
        if is_red_detected:
            print("赤色検出 → 右へ回避")
            driver.changing_right(0, 40) # 短く旋回
            time.sleep(0.5)
            driver.motor_stop_brake()
            time.sleep(0.5)
            driver.changing_forward(0, 80)
            time.sleep(1)
        else:
            print("赤なし → 前進")
            driver.changing_forward(0, 80)
            time.sleep(2)

        driver.motor_stop_brake()
        print("ミッション完了。")

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
        
        # 最後に一度だけGPIO全体をクリーンアップ
        GPIO.cleanup()
        print("GPIOのクリーンアップを実行しました。")
        print("全ての処理を終了しました。")


if __name__ == "__main__":
    main()
