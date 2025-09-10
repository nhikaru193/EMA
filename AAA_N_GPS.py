#ARLISS本番前にGPS誘導を確認するためのコード

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
import sys

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
from GDA2 import GDA
from GPS_ARLISS_FINAL import GPS_SENDING

#初期設定
Flag_location_a = [40.879698, -119.114899]
Flag_location_b = [40.879774, -119.114969]
Goal_location = [40.8841111, -119.1182222]
t = 1

#BNO055の初期設定
bno = BNO055()
bno.begin()
time.sleep(1)
bno.setMode(BNO055.OPERATION_MODE_NDOF)
time.sleep(1)
bno.setExternalCrystalUse(True)
GPIO.setmode(GPIO.BCM)

while True:
    sys, gyro, accel, mag = bno.getCalibration()
    print(f"gyro:{gyro}, mag:{mag}")
    if gyro == 3 and mag == 3:
        print("BNO055のキャリブレーション終了")
        break
    #driver.cleanup()
    time.sleep(0.3)

#ここのタイムスリープは収納待ちのタイムスリープ
time.sleep(t)


gps_transmitter = GPS_Transmitter(pi)
try:
    RELEASE = RD(bno)
    RELEASE.run()

    LAND = LD(bno) 
    LAND.run()
    time.sleep(3)
    
    print("着地が完了しました。GPS送信を開始します。")
    GPS_ARLISS_FINAL.start()

    
    print("パラシュート回避を始めます")
    time.sleep(1)
    
    AVOIDANCE = PA(bno, goal_location = Flag_location_a) #ok
    AVOIDANCE.run()
    
    
    GPS_StoE = GPS(bno, goal_location = Flag_location_a)
    GPS_StoE.run()
    
    GPS_StoF = GPS(bno, goal_location = Flag_location_b)
    GPS_StoF.run()
    
    
    FLAG = FN(bno, flag_location = Flag_location_b) 
    FLAG.run()
    
    Servo.release()
    
    
    GPS_FtoG = GPS(bno, goal_location = Goal_location)
    GPS_FtoG.run()
    
    GOAL = GDA(bno, 30)
    GOAL.run()

    print("Mission Complete")

finally:
    # プログラム終了時に必ず呼び出すクリーンアップ処理
    print("クリーンアップ処理を開始します。")
    GPS_ARLISS_FINAL.stop()  # GPS送信スレッドを停止
