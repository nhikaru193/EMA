import cv2
import numpy as np
import time
from picamera2 import Picamera2
from motor import MotorDriver
import camera
import following
from BNO055 import BNO055
import math
from collections import deque
import pigpio

class GDA:
    def __init__(self, bno: BNO055, counter_max: int=50):
        self.driver = MotorDriver(
            PWMA=12, AIN1=23, AIN2=18,
            PWMB=19, BIN1=16, BIN2=26,
            STBY=21
        )
        self.bno = bno
        self.picam2 = Picamera2()
        config = self.picam2.create_still_configuration(main={"size": (320, 480)})
        self.picam2.configure(config)
        self.picam2.start()
        time.sleep(1)
        self.counter_max = counter_max
        self.lower_red1 = np.array([0, 100, 100])
        self.upper_red1 = np.array([10, 255, 255])
        self.lower_red2 = np.array([160, 100, 100])
        self.upper_red2 = np.array([180, 255, 255])
        self.pi = pigpio.pi()
        if not self.pi.connected:
            raise RuntimeError("pigpioデーモンに接続できません。`sudo pigpiod`を実行して確認してください。")
        
    def get_percentage(self, frame):
        frame = cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)
        frame = cv2.GaussianBlur(frame, (5, 5), 0)
        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask1 = cv2.inRange(hsv, self.lower_red1, self.upper_red1)
        mask2 = cv2.inRange(hsv, self.lower_red2, self.upper_red2)
        mask = cv2.bitwise_or(mask1, mask2)
        red_area = np.count_nonzero(mask)
        total_area = frame.shape[0] * frame.shape[1]
        percentage = (red_area / total_area) * 100
        print(f"検知割合は{percentage}%です")
        return percentage
    
    def run(self):
        left_a = 90
        right_a = 80
        counter = self.counter_max
        percentage = 0
        try:
            heading_list = deque(maxlen=5)
            counter = self.counter_max
            print("ゴール誘導を開始します")
            while True:
                if counter <= 0:
                    print("赤コーンが近くにありません。探索を行います")
                    counter = self.counter_max
                    while True: # 探索モードのループ (ここから直接「その場での回転探索」が始まる)
                        print("探索中")
                        # 広範囲な移動による探索 (changing_moving_forward) の部分は削除されました
                        
                        before_heading = self.bno.get_heading()
                        delta_heading = 20
                        while delta_heading > 5:
                            print("この場所でのコーン探索を行います")
                            frame = self.picam2.capture_array()
                            time.sleep(0.2)
                            percentage = self.get_percentage(frame)
                            if percentage > 15:
                                print("赤コーンの探索に成功しました")
                                break
                            print("視野角内にコーンを検知できませんでした。左回頭を行います")
                            self.driver.petit_left(0, 90)
                            self.driver.motor_stop_brake()
                            time.sleep(0.2)
                            after_heading = self.bno.get_heading()
                            
                            delta_heading = min((after_heading -  before_heading) % 360, (before_heading -  after_heading) % 360)
                        else:
                            print("付近にはコーンを検知できなかったため、再度探索を行います")
                        if percentage > 10:
                            print("10%以上の赤面積を検知したため、探索プログラムを終えます")
                            break    
                frame = self.picam2.capture_array()
                time.sleep(0.2)
                percentage = self.get_percentage(frame)
                time.sleep(0.2)
                print(f"赤割合: {percentage:2f}%です ")

                if percentage >= 90:
                    print("percentageでのゴール判定")
                    break
                elif percentage > 15:
                    print("赤コーンを検知しました。接近します。")
                    if percentage > 40:
                        print("非常に近いので、ゆっくり前進します (petit_petit 2回)")
                        self.driver.petit_petit(2)
                    elif percentage > 20:
                        print("近いので、少し前進します (petit_petit 3回)")
                        self.driver.petit_petit(3)
                    else:
                        print("遠いので、前進します (follow_forward)")
                        following.follow_forward(self.driver, self.bno, 70, 1)
                    counter = self.counter_max
                
                counter = counter - 1
                c_heading = self.bno.get_heading()
                heading_list.append(c_heading)
                if len(heading_list) == 5:
                    print("スタック判定を行います")
                    a = abs((heading_list[4] - heading_list[3] + 180) % 360 - 180)
                    b = abs((heading_list[3] - heading_list[2] + 180) % 360 - 180)
                    c = abs((heading_list[2] - heading_list[1] + 180) % 360 - 180)
                    if a < 1.5 and b < 1.5 and c < 1.5:
                        print("スタック判定です")
                        print("スタック離脱を行います")
                        self.driver.changing_right(0, 90)
                        time.sleep(3)
                        self.driver.changing_right(90, 0)
                        time.sleep(0.5)
                        self.driver.changing_left(0, 90)
                        time.sleep(3)
                        self.driver.changing_left(90, 0)
                        time.sleep(0.5)
                        self.driver.changing_forward(0, 90)
                        time.sleep(0.5)
                        self.driver.changing_forward(90, 0)
                        time.sleep(0.5)
                        print("スタック離脱を終了します")
                        heading_list.clear()
        finally:
            self.picam2.close()
            self.pi.bb_serial_read_close(17)
            self.picam2.close()
            print("カメラを閉じました。")
            print("ゴール判定")
            self.driver.cleanup()
            print("GPIOクリーンアップが終了しました。プログラムを終了します")
