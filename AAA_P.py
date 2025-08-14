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
from Flag_Detector3 import FlagDetector

#ミッション部分
from C_RELEASE import RD
from C_Landing_Detective import LD
from C_PARACHUTE_AVOIDANCE import PA
from Flag_Navi import FN
#from C_Servo import SM
from C_excellent_GPS import GPS
from GDA2 import GDA

GPS_FtoG = GPS(bno, goal_location = Goal_location)
GPS_FtoG.run()

Flag_location = [35.9242012, 139.9114341]
Goal_location = [40.1426175, 139.9876533]

def set_servo_duty(duty):
    pwm.ChangeDutyCycle(duty)
    time.sleep(0.5)

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

GPS_FtoG = GPS(bno, goal_location = Goal_location)
GPS_FtoG.run()

GOAL = GDA(bno, 30)
GOAL.run()
