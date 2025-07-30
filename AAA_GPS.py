import cv2
import numpy as np
import time
import camera
import smbus
from picamera2 import Picamera2
import struct
import RPi.GPIO as GPIO
import math
import pigpio

#作成ファイルのインポート
import fusing
import BME280
import following
from BNO055 import BNO055
from motor import MotorDriver
from Flag_B import Flag_B

#ミッション部分
from C_RELEASE import RD
from C_Landing_Detective import LD
from C_PARACHUTE_AVOIDANCE import PA
from Flag_Navi import FN
from C_Servo import SM
from C_excellent_GPS import GPS
from C_GOAL_DETECTIVE_NOSHIRO import GDN


Flag_location = [35.9242090, 139.9113949]
Goal_location = [35.9243095, 139.9113758]

#BNO055の初期設定
bno = BNO055()
bno.begin()
time.sleep(1)
bno.setMode(BNO055.OPERATION_MODE_NDOF)
time.sleep(1)
bno.setExternalCrystalUse(True)

while True:
    sys, gyro, accel, mag = bno.getCalibration()
    print(f"gyro:{gyro}")
    if gyro == 3 and mag == 3:
        print("BNO055のキャリブレーション終了")
        break

def degree_rotation(degree, threshold_deg = 5, sleeping = 0.01):
    before_heading = bno.getVector(BNO055.VECTOR_EULER)[0]
    target_heading = (before_heading + degree) % 360
    while True:
        current_heading = bno.getVector(BNO055.VECTOR_EULER)[0]
        delta_heading = ((target_heading - current_heading + 180) % 360) - 180
        if abs(delta_heading) <= threshold_deg:
            break
        elif delta_heading < -1 * threshold_deg:
            driver.petit_left(0, 90)
            time.sleep(sleeping)
            time.sleep(0.05)
            driver.motor_stop_brake()
            time.sleep(0.5)
        elif delta_heading > threshold_deg:
            driver.petit_right(0, 99)
            time.sleep(sleeping)
            time.sleep(0.05)
            driver.motor_stop_brake()
            time.sleep(0.5)

GPS_StoF = GPS(bno, goal_location = Flag_location)
GPS_StoF.run()

GPS_FtoG = GPS(bno, goal_location = Goal_location)
GPS_FtoG.run()

print("クラス呼び出し完了です")
