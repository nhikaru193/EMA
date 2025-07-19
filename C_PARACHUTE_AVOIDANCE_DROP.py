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
        
        """
        self.lower_red1 = np.array([0, 100, 100])
        self.upper_red1 = np.array([10, 255, 255])
        self.lower_red2 = np.array([160, 100, 100])
        self.upper_red2 = np.array([180, 255, 255])
        """
        self.lower_red1 = np.array([5, 150, 150])
        self.upper_red1 = np.array([30, 255, 255])
        """
        self.lower_red2 = np.array([160, 100, 100])
        self.upper_red2 = np.array([180, 255, 255])
        """
        
    def get_percentage(self, frame):
        frame = cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)
        frame = cv2.GaussianBlur(frame, (5, 5), 0)
        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.lower_red1, self.upper_red1)
        #mask2 = cv2.inRange(hsv, self.lower_red2, self.upper_red2)
        #mask = cv2.bitwise_or(mask1, mask2)
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
            print(f"検知割合は{percentage}%です")
            print("この方向にパラシュートを検知しました")
        else:
            return False
            print("この方向にパラシュートは検知できませんでした")

    def run(self):
        while True:
            if self.detective_red():
                print("前方にパラシュートが検知できたので回頭します")
                self.driver.changing_Lforward(0, 90)
                self.driver.changing_Lforward(90, 0)
            else:
                following.follow_forward(self.driver, self.bno, 90, 4)
                print("パラシュートの回避を終了します")
                break
        self.picam2.close()
        self.driver.cleanup()

if __name__ == '__main__':
    # 許容誤差を調整したい場合は、ここで値を設定できます
    # 例: detector = FlagDetector(triangle_tolerance=0.8)
    bno = BNO055()
    time.sleep(0.5)
    if not bno.begin():    
        print("bnoが始まりませんでした")
        exit()
    time.sleep(0.5)
    bno.setMode(BNO055.OPERATION_MODE_NDOF)
    time.sleep(0.5)
    
    while True:
        sys, gyro, accel, mag = bno.getCalibration()
        print(f"Calib → Sys:{sys}, Gyro:{gyro}, Acc:{accel}, Mag:{mag}", end='\r\n')
        if gyro == 3:
            print("キャリブレーション完了")
            break
    AVO = PAD(bno)
    AVO.run()
