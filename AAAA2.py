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
from C2_release import RD
from C2_Landing_Detective import LD
from C2_PARACHUTE_AVOIDANCE import PA
from C2_Flag_Navi import FN
from C2_excellent_GPS import GPS
from C2_GOAL_DETECTIVE_NOSHIRO import GDN

#おそらく未使用のモジュール
"""
import numpy
import busio
from C_Parachute_Avoidance import Parakai
"""

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
    if gyro == 3:
        print("BNO055のキャリブレーション終了")
        break

#関数のインスタンス作成
RELEASE = RD(bno) #ok
RELEASE.run()

LAND = Landing(bno) 
LAND.run()

AVOIDANCE = PA(bno, goal_location = [35.9240852, 139.9112008]) #ok
AVOIDANCE.run()

GPS_StoF = GPS(bno, goal_location = [35.9240852, 139.9112008])
GPS_StoF.run()

FLAG = FN(bno)
FLAG.run()

GPS_FtoG = GPS(bno, goal_location = [35.9241086 ,139.9113731])
GPS_FtoG.run()

GOAL = GDN(bno, 30)
GOAL.run()

#実行文
print("クラス呼び出し完了です")
