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
from C_release import RD
from C_Landing_Detective import LD
from C_PARACHUTE_AVOIDANCE import PA
from C_Flag_Navi import FN
from C_excellent_GPS import GPS
from C_GOAL_DETECTIVE_NOSHIRO import GDN

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
    if gyro == 3 and mag == 3:
        print("BNO055のキャリブレーション終了")
        break

#関数のインスタンス作成
RELEASE = RD(bno) #ok
RELEASE.run()

LAND = LD(bno) 
LAND.run()

AVOIDANCE = PA(bno, goal_location = [35.9240619, 139.9112387]) #ok
AVOIDANCE.run()

GPS_StoF = GPS(bno, goal_location = [35.9240619, 139.9112387])
GPS_StoF.run()

FLAG = FN(bno)
FLAG.run()

SERVO_PIN = 13  # GPIO13を使用
GPIO.setmode(GPIO.BCM)
GPIO.setup(SERVO_PIN, GPIO.OUT)
pwm = GPIO.PWM(SERVO_PIN, 50)
pwm.start(0)
print("逆回転（速い）")
set_servo_duty(4.0)
time.sleep(7)
#set_servo_duty(12.5)
pwm.stop()
GPIO.cleanup()

GPS_FtoG = GPS(bno, goal_location = [35.9241528 ,139.9112682])
GPS_FtoG.run()

GOAL = GDN(bno, 30)
GOAL.run()

#実行文
print("クラス呼び出し完了です")
