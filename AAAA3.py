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
from Flag_Detector2 import FlagDetector

#ミッション部分
from C_RELEASE import RD
from C_Landing_Detective import LD
from C_PARACHUTE_AVOIDANCE_DROP import PAD
from C_Flag_Navi import FN
from C_excellent_GPS import GPS
from C_GOAL_DETECTIVE_NOSHIRO import GDN

#BNO055の初期設定
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
    if gyro == 3 and accel == 3:
        print("キャリブレーション完了")
        break

RELEASE = RD(bno)
RELEASE.run()

LAND = LD(bno)
LAND.run()

AVOIDANCE = PAD(bno)
AVOIDANCE.run()
