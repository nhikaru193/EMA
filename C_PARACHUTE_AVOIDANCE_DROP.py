import math
import time
import serial
import pigpio
import RPi.GPIO as GPIO
from motor import MotorDriver      # ユーザーのMotorDriverクラスを使用
from BNO055 import BNO055
import smbus
import struct
import following
import cv2
import numpy as np
from picamera2 import Picamera2
import camera

class PAD:
    def __init__(self, bno: BNO055):
        self.driver = MotorDriver(
            PWMA=12, AIN1=23, AIN2=18,
            PWMB=19, BIN1=16, BIN2=26,
            STBY=21
        )
        self.picam2 = Picamera2()
        config = self.picam2.create_still_configuration(main={"size": (320, 480)})
        self.picam2.configure(config)
        self.picam2.start()
        self.bno = bno
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

    def detective_red(self):
        frame = self.picam2.capture_array()
        percentage = self.get_percentage(frame)
        if percentage > 10:
            return True
            print("この方向にパラシュートを検知しました")
        else:
            return False
            print("この方向にパラシュートは検知できませんでした")

    #左n度回頭はdegree負の値、右はその逆
    def degree_rotation(self, degree, threshold_deg = 5, sleeping = 0.01):
        before_heading = self.bno.getVector(BNO055.VECTOR_EULER)[0]
        target_heading = (before_heading + degree) % 360
        while True:
            current_heading = self.bno.getVector(BNO055.VECTOR_EULER)[0]
            delta_heading = ((target_heading - current_heading + 180) % 360) - 180
            if abs(delta_heading) <= threshold_deg:
                break
            elif delta_heading < -1 * threshold_deg:
                self.driver.petit_left(0, 90)
                time.sleep(sleeping)
                time.sleep(0.05)
                self.driver.motor_stop_brake()
                time.sleep(0.5)
            elif delta_heading > threshold_deg:
                self.driver.petit_right(0, 99)
                time.sleep(sleeping)
                time.sleep(0.05)
                self.driver.motor_stop_brake()
                time.sleep(0.5)


    def run(self):
        while True:
            if self.detective_red:
                print("前方にパラシュートが検知できたので回頭します")
                self.driver.changing_Lforward(0, 90)
                self.driver.changing_Lforward(90, 0)
            else:
                following.follow_forward(self.driver, self.bno, 90, 4)
                print("パラシュートの回避を終了します")
                break
