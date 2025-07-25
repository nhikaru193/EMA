import math
from collections import deque
import numpy as np
import cv2
import time
from picamera2 import Picamera2
from motor import MotorDriver
import camera
import following
from BNO055 import BNO055

class GDA:
    def __init__(
        self,
        bno: BNO055
    ):
        self.bno = bno
        self.picam2 = Picamera2()
        config = self.picam2.create_still_configuration(main={"size": (320, 480)})
        self.picam2.configure(config)
        self.picam2.start()
        self.lower_red1 = np.array([0, 100, 100])
        self.upper_red1 = np.array([10, 255, 255])
        self.lower_red2 = np.array([160, 100, 100])
        self.upper_red2 = np.array([180, 255, 255])

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

    def get_block_number_by_density(self, frame):
        frame = cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)
        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        frame = cv2.GaussianBlur(frame, (5, 5), 0)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask1 = cv2.inRange(hsv, self.lower_red1, self.upper_red1)
        mask2 = cv2.inRange(hsv, self.lower_red2, self.upper_red2)
        mask = cv2.bitwise_or(mask1, mask2)
        height, width = mask.shape
        block_width = width // 5
        red_ratios = []
        for i in range(5):
            x_start = i * block_width
            x_end = (i + 1) * block_width if i < 4 else width
            block_mask = mask[:, x_start:x_end]
            red_count = np.count_nonzero(block_mask)
            total_count = block_mask.size
            ratio = red_count / total_count
            red_ratios.append(ratio)
        for i, r in enumerate(red_ratios):
            print(f"[DEBUG] ブロック{i+1}の赤密度: {r:.2%}")
        max_ratio = max(red_ratios)
        if max_ratio < 0.08:
            print("❌ 赤色が検出されません（全ブロックで密度低）")
            return None  # 全体的に赤が少なすぎる場合
        else:
            print(f"一番密度の高いブロックは{red_ratios.index(max_ratio) + 1}です")
            return red_ratios.index(max_ratio) + 1

    def run():
        heading_list = deque(maxlen=4)
        percent_list = deque(maxlen=4)
        turn_counter = 4
        start_heading = self.bno.get_heading()
        a_time = time.time()
        #ゴール内部からのスタートであることに注意
        #以下のwhile True構文はコーンの情報把握
        while True:
            frame = self.picam2.capture_array()
            time.sleep(0.2)
            percentage = self.get_percentage(frame)
            time.sleep(0.2)
            number = self.get_block_number_by_density(frame)
            time.sleep(0.2)
            if number == 1:
                self.driver.petit_left(0, 100)
                self.driver.motor_stop_brake()
                time.sleep(0.6)
            elif number == 2:
                self.driver.petit_left(0, 100)
                self.driver.motor_stop_brake()
                time.sleep(0.6)
            elif number == 4:
                self.driver.petit_right(0, 100)
                self.driver.motor_stop_brake()
                time.sleep(0.6)
            elif number == 5:
                self.driver.petit_right(0, 100)
                self.driver.motor_stop_brake()
                time.sleep(0.6)
            #以下は中央にある場合であるので情報を記憶し格納する
            else:
                heading = self.bno.get_heading()
                #スタート位置との差が小さければループを抜ける
                b_time = time.time()
                deltaa_time = b_time - a_time
                if deltaa_time > 10:
                    deltaa_heading = ((heading - start_heading + 180) % 360 - 180)
                    if deltaa_heading < 10:
                        break
                heading_list.append(heading)
                percent_list.append(percentage)
                turn_counter = turn_counter - 1
                #すべてのコーンを格納済み
                if turn_counter == 0:
                    print("すべてのコーンの割合と方位角の情報を保持しました")
                #次のコーンを検知するまで回頭
                print("コーンの探索を行います")
                start_time = time.time()
                while True:
                    self.driver.petit_left(0, 100)
                    self.driver.motor_stop_brake()
                    time.sleep(0.6)
                    current_time = time.time()
                    if number == 2:
                        print("次のコーンを検知しました。")
                        break
                    elif number == 1:
                        print("次のコーンを検知しました。")
                        break
                    delta_time = current_time - start_time
                    if delta_time > 4:
                        if number == 3:
                            print("次のコーンを検知しました。")
                            break
            
