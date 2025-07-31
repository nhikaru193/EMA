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

    def get_density_on_each_block(self, frame):
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
       return red_ratios

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
            return -1  # 全体的に赤が少なすぎる場合
        else:
            print(f"一番密度の高いブロックは{red_ratios.index(max_ratio) + 1}です")
            return red_ratios.index(max_ratio) + 1
            
    def run(self):
        try:
            heading_list = deque(maxlen=4)
            percent_list = deque(maxlen=4)
            turn_counter = 4

            #大まかな接近を行う。おおよそ正方形外怪から2 m前後まで接近
            while True:
                frame = self.picam2.capture_array()
                per = self.get_percentage(frame)
                if per < 8:
                    self.driver.petit_petit(4)
                    time.sleep(1)
                else:
                    print("大まかな接近に成功しました。")
                    break
            
            #ブロックごとの割合を検知して、45%を超えると70 cmと検知する方針。まずはある一個に近づく
            while True:
                self.driver.petit_left(0, 90)
                self.driver.motor_stop_brake()
                time.sleep(0.5)
                frame = self.picam2.capture_array()
                RED = self.get_density_on_each_block(frame)
                max_RED = max(RED)
                max_RED_number = RED.index(max_RED)
                print(f"赤色検知最大は{max_RED_number}です")
                #一番高い要素の割合が45%でbreak
                if max_RED > 45:
                    print("赤色球体1個に接近完了しました")
                    break
                if max_RED_number == 2:
                    print("中央にとらえることができたので、前進します")
                    if max_RED > 30:
                        self.driver.petit_petit(2)
                    else:
                        self.driver.petit_petit(5)
                elif max_RED_number == 0:
                    self.driver.petit_left(0, 90)
                elif max_RED_number == 1:
                    self.driver.petit_left(0, 80)
                elif max_RED_number == 3:
                    self.driver.petit_right(0, 70)
                elif max_RED_number == 4:
                    self.driver.petit_right(0, 80)
                
            #中心部に入るコード
            while True:
                self.driver.petit_right(0, 90)
                self.driver.motor_stop_brake()
                frame = self.picam2.capture_array()
                number = self.get_block_number_by_density(frame)
                if number ==
                """
                time.sleep(0.5)
                frame = self.picam2.capture_array()
                RED = self.get_density_on_each_block(frame)
                max_RED = max(RED)
                max_RED_number = RED.index(max_RED)
                current_heading = self.bno.get_heading()
                check = 0
                if heading_list = None:
                for i in range(len(heading_list)):
                    d_heading = abs(((heading_list[i] - current_heading + 180) % 360) - 180)
                    if d_heading < 10:
                if check == 100:
                    print("登録済みの赤色球体です。スキップします")
                    if max_RED > 50:
                        if max_RED_number == 2:
                            heading = self.bno.get_heading() 
                            if check == 100:
                                print("登録済みの赤色球体です。スキップします")
                            elif check == 0:
                                print("登録していない赤色球体です。登録をお願いします")
                                heading_list.append(heading)
                """
        finally:
            self.driver.cleanup()
            self.picam2.close()
