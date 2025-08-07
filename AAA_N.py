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
import Servo
from C_excellent_GPS import GPS
from C_GOAL_DETECTIVE_NOSHIRO import GDN

#初期設定
Flag_location = [35.9241083, 139.9112913]
Goal_location = [35.9241933, 139.9115140]

#BNO055の初期設定
bno = BNO055()
bno.begin()
time.sleep(1)
bno.setMode(BNO055.OPERATION_MODE_NDOF)
time.sleep(1)
bno.setExternalCrystalUse(True)

while True:
    sys, gyro, accel, mag = bno.getCalibration()
    print(f"gyro:{gyro}, mag:{mag}")
    if gyro == 3 and mag == 3:
        print("BNO055のキャリブレーション終了")
        break

RELEASE = RD(bno)
RELEASE.run()

LAND = LD(bno) 
LAND.run()

time.sleep(3)

print("パラシュート回避を始めます")
time.sleep(1)

AVOIDANCE = PA(bno, goal_location = Flag_location) #ok
AVOIDANCE.run()

GPS_StoF = GPS(bno, goal_location = Flag_location)
GPS_StoF.run()


FLAG = FN(bno, flag_location = Flag_location) 
FLAG.run()

Servo.release()

GPS_FtoG = GPS(bno, goal_location = Goal_location)
GPS_FtoG.run()

GOAL = GDN(bno, 30)
GOAL.run()

print("Mission Complete")
