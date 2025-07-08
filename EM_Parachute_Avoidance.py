#溶断回路
import RPi.GPIO as GPIO
import time
#回避関係
import pigpio
import board
import busio
import adafruit_bno055
import RPi.GPIO as GPIO
import numpy as np
import cv2
from picamera2 import Picamera2
from motor import MotorDriver
import fusing

#溶断回路動作
#fusing.circuit()
    
# GPIO モーター制御設定
driver = MotorDriver(
    PWMA=12, AIN1=23, AIN2=18,   # 左モーター用（モータA）
    PWMB=19, BIN1=16, BIN2=26,   # 右モーター用（モータB）
    STBY=21                      # STBYピン
)

# GPS (pigpio)
RX_PIN = 17
pi = pigpio.pi()
pi.bb_serial_read_open(RX_PIN, 9600, 8)

destination_lat = 40.47
destination_lon = 119.42

def convert_to_decimal(coord, direction):
    degrees = int(coord[:2]) if direction in ['N', 'S'] else int(coord[:3])
    minutes = float(coord[2:]) if direction in ['N', 'S'] else float(coord[3:])
    decimal = degrees + minutes / 60
    if direction in ['S', 'W']:
        decimal *= -1
    return decimal

def get_current_location():
    timeout = time.time() + 5
    while time.time() < timeout:
        (count, data) = pi.bb_serial_read(RX_PIN)
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
            except:
                continue
        time.sleep(0.1)
    raise TimeoutError("GPSデータの取得に失敗しました")

# 方位計算（GPS→目的地）
def calculate_heading(current_lat, current_lon, dest_lat, dest_lon):
    import math
    delta_lon = math.radians(dest_lon - current_lon)
    y = math.sin(delta_lon) * math.cos(math.radians(dest_lat))
    x = math.cos(math.radians(current_lat)) * math.sin(math.radians(dest_lat)) - \
        math.sin(math.radians(current_lat)) * math.cos(math.radians(dest_lat)) * math.cos(delta_lon)
    bearing = math.degrees(math.atan2(y, x))
    return (bearing + 360) % 360

def save_image_before_detection(picam2):
    # 画像を撮影
    frame = picam2.capture_array()
    frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
    
    # 撮影した画像を保存
    image_path = "/home/mark1/Pictures/paravo_image.jpg"
    cv2.imwrite(image_path, frame_bgr)                            #メモリ上の画像データをファイルとして保存する
    print(f"画像保存成功: {image_path}")                   #出力          
    
    return frame

# 赤色検出（Picamera2 + OpenCV）
def detect_red_object(picam2):
    frame = picam2.capture_array()
    
    if frame is None:
        print("画像取得失敗")
        return False
    
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    lower_red = np.array([0, 120, 70])
    upper_red = np.array([10, 255, 255])
    mask = cv2.inRange(hsv, lower_red, upper_red)
    
    return np.sum(mask) > 5000

# 初期化
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# BNO055（方位センサー）
i2c = busio.I2C(board.SCL, board.SDA)
sensor = adafruit_bno055.BNO055_I2C(i2c)

# Picamera2 設定
picam2 = Picamera2()
picam2.configure(picam2.create_preview_configuration(main={"size": (640, 480)}))
picam2.start()
time.sleep(2)

# 画像保存処理（赤色検出の前に実行）
frame = save_image_before_detection(picam2)  # ここで画像を保存

# GPS取得処理
try:
    current_lat, current_lon = get_current_location()
    print("現在地：", current_lat, current_lon)

    target_heading = calculate_heading(current_lat, current_lon, destination_lat, destination_lon)
    print("目標方位：", target_heading)

    heading = sensor.euler[0]
    if heading is None:
        heading = 0
    print("現在の方位：", heading)

    diff = (target_heading - heading + 360) % 360
    if 10 < diff < 180:
        print("右旋回")
        driver.changing_right(0, 40)
    
    elif diff >= 180:
        print("左旋回")
        driver.changing_left(0, 40)
        
    else:
        print("方位OK")
        driver.motor_stop_brake()

    time.sleep(2)

    if detect_red_object(picam2):
        print("赤色検出 → 右へ回避")
        driver.changing_right(0, 40)
        driver.motor_stop_brake()
        driver.changing_forward(0, 80)
        time.sleep(1)
    
    else:
        print("赤なし → 前進")
        driver.changing_forward(0, 80)
        time.sleep(2)

    driver.motor_stop_brake()

except TimeoutError as e:
    print(e)
    # GPSの取得に失敗した場合でも画像は保存済み
    print("GPS取得失敗しましたが、画像は保存されています。")

finally:
    print("パラシュート回避の終了処理中...")
    driver.cleanup()
    pi.bb_serial_read_close(RX_PIN)
    pi.stop()
    picam2.close()
    GPIO.cleanup()
    print("パラシュート回避の処理を終了しました。")
