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
import fusing

class ParachuteAvoidance:
    def __init__(self):
        # GPIO setup
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)

        # Motor driver initialization
        self.driver = MotorDriver(
            PWMA=12, AIN1=23, AIN2=18,
            PWMB=19, BIN1=16, BIN2=26,
            STBY=21
        )

        # GPS (pigpio) setup
        self.RX_PIN = 17
        self.pi = pigpio.pi()
        self.pi.bb_serial_read_open(self.RX_PIN, 9600, 8)

        # Destination coordinates
        self.destination_lat = 40.47
        self.destination_lon = 119.42

        # BNO055 initialization
        i2c = busio.I2C(board.SCL, board.SDA)
        self.sensor = adafruit_bno055.BNO055_I2C(i2c)

        # Picamera2 setup
        self.picam2 = Picamera2()
        self.picam2.configure(
            self.picam2.create_preview_configuration(main={"size": (640, 480)})
        )
        self.picam2.start()
        time.sleep(2)

    def convert_to_decimal(self, coord, direction):
        degrees = int(coord[:2]) if direction in ['N', 'S'] else int(coord[:3])
        minutes = float(coord[2:]) if direction in ['N', 'S'] else float(coord[3:])
        decimal = degrees + minutes / 60
        if direction in ['S', 'W']:
            decimal *= -1
        return decimal

    def get_current_location(self):
        timeout = time.time() + 5
        while time.time() < timeout:
            count, data = self.pi.bb_serial_read(self.RX_PIN)
            if count and data:
                try:
                    text = data.decode("ascii", errors="ignore")
                    if "$GNRMC" in text:
                        for line in text.split("\n"):
                            if "$GNRMC" in line:
                                parts = line.strip().split(",")
                                if len(parts) > 6 and parts[2] == "A":
                                    lat = self.convert_to_decimal(parts[3], parts[4])
                                    lon = self.convert_to_decimal(parts[5], parts[6])
                                    return lat, lon
                except Exception:
                    continue
            time.sleep(0.1)
        raise TimeoutError("GPSデータの取得に失敗しました")

    def calculate_heading(self, current_lat, current_lon, dest_lat, dest_lon):
        import math
        delta_lon = math.radians(dest_lon - current_lon)
        y = math.sin(delta_lon) * math.cos(math.radians(dest_lat))
        x = (math.cos(math.radians(current_lat)) * math.sin(math.radians(dest_lat)) -
             math.sin(math.radians(current_lat)) * math.cos(math.radians(dest_lat)) * math.cos(delta_lon))
        bearing = math.degrees(math.atan2(y, x))
        return (bearing + 360) % 360

    def save_image_before_detection(self):
        frame = self.picam2.capture_array()
        frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        image_path = "/home/mark1/Pictures/paravo_image.jpg"
        cv2.imwrite(image_path, frame_bgr)
        print(f"画像保存成功: {image_path}")
        return frame

    def detect_red_object(self):
        frame = self.picam2.capture_array()
        if frame is None:
            print("画像取得失敗")
            return False
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        lower_red = np.array([0, 120, 70])
        upper_red = np.array([10, 255, 255])
        mask = cv2.inRange(hsv, lower_red, upper_red)
        return np.sum(mask) > 5000

    def run(self):
        # Save image before detection
        self.save_image_before_detection()

        try:
            current_lat, current_lon = self.get_current_location()
            print("現在地：", current_lat, current_lon)

            target_heading = self.calculate_heading(
                current_lat, current_lon,
                self.destination_lat, self.destination_lon
            )
            print("目標方位：", target_heading)

            heading = self.sensor.euler[0] or 0
            print("現在の方位：", heading)

            diff = (target_heading - heading + 360) % 360
            if 10 < diff < 180:
                print("右旋回")
                self.driver.changing_right(0, 40)
            elif diff >= 180:
                print("左旋回")
                self.driver.changing_left(0, 40)
            else:
                print("方位OK")
                self.driver.motor_stop_free()

            time.sleep(2)

            if self.detect_red_object():
                print("赤色検出 → 右へ回避")
                self.driver.changing_right(0, 40)
                self.driver.motor_stop_brake()
                self.driver.changing_forward(0, 80)
                time.sleep(1)
            else:
                print("赤なし → 前進")
                self.driver.changing_forward(0, 80)
                time.sleep(2)

            self.driver.motor_stop_brake()

        except TimeoutError as e:
            print(e)
            print("GPS取得失敗しましたが、画像は保存されています。")

        finally:
            print("パラシュート回避の終了処理中...")
            self.driver.cleanup()
            self.pi.bb_serial_read_close(self.RX_PIN)
            self.pi.stop()
            self.picam2.close()
            GPIO.cleanup()
            print("パラシュート回避の処理を終了しました。")

if __name__ == "__main__":
    pa = ParachuteAvoidance()
    pa.run()
