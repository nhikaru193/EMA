import cv2
import numpy as np
import time
from picamera2 import Picamera2
from motor import MotorDriver
import camera
import smbus
import following
import BME280
from BNO055 import BNO055
from C_release import Release
from C_Landing_Detective import Landing
from C_Parachute_Avoidance import Parakai
from C_PARACHUTE_AVOIDANCE import PA
from C_Flag_Navi import FlagNavigator
from C_excellent_GPS import Amaging_GPS
from C_GOAL_DETECTIVE_NOSHIRO import GDN
import fusing
import struct
import RPi.GPIO as GPIO
import math
import numpy
from Flag_Detector2 import FlagDetector
import pigpio
import busio

"""
#モータの初期化
driver = MotorDriver(
    PWMA=12, AIN1=23, AIN2=18,   # 左モーター用（モータA）
    PWMB=19, BIN1=16, BIN2=26,   # 右モーター用（モータB）
    STBY=21                      # STBYピン
)
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
        
"""
# カメラ初期化と設定
picam2 = Picamera2()
config = picam2.create_still_configuration(main={"size": (320, 240)})
picam2.configure(config)
picam2.start()
time.sleep(1)
"""
#関数のインスタンス作成
RELEASE = Release(bno) #ok
RELEASE.run()

LAND = Landing(bno) 
LAND.run()

AVOIDANCE = PA(bno, goal_location = [35.9240852, 139.9112008]) #ok
AVOIDANCE.run()

GPS_StoF = Amaging_GPS(bno, goal_location = [35.9240852, 139.9112008])
GPS_StoF.run()

FLAG = FLAGNAVIGATOR(bno)
FLAG.run()

GPS_FtoG = Amaging_GPS(bno, goal_location = [35.9241086 ,139.9113731])
GPS_FtoG.run()

GOAL = GDN(bno, 30)
GOAL.run()

#実行文
print("クラス呼び出し完了です")
