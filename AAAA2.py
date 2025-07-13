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

#driverのインスタンス作成
driver = MotorDriver(
    PWMA=12, AIN1=23, AIN2=18,   # 左モーター用（モータA）
    PWMB=19, BIN1=16, BIN2=26,   # 右モーター用（モータB）
    STBY=21                      # STBYピン
)

#BNO055の初期設定
bno = BNO055()
bno.begin()
time.sleep(1)
bno.setMode(BNO055.OPERATION_MODE_NDOF)
time.sleep(1)
bno.setExternalCrystalUse(True)

# === GPSピン設定 ===
RX_PIN = 17
BAUD = 9600
# === pigpio 初期化 ===
pi = pigpio.pi()
if not pi.connected:
    print("pigpio デーモンに接続できてないよ。sudo pigpiod を実行してください。")
    exit(1)
err = pi.bb_serial_read_open(RX_PIN, BAUD, 8)
if err != 0:
    print(f"ソフトUART RX の設定に失敗：GPIO={RX_PIN}, {BAUD}bps")
    pi.stop()
    exit(1)

#picam2初期設定
picam2 = Picamera2()
config = picam2.create_still_configuration(main={"size": (320, 480)})
picam2.configure(config)
picam2.start()
time.sleep(1)

while True:
    sys, gyro, accel, mag = bno.getCalibration()
    print(f"gyro:{gyro}")
    if gyro == 3:
        print("BNO055のキャリブレーション終了")
        break

#関数のインスタンス作成
RELEASE = RD(bno) #ok
RELEASE.run()

LAND = LD(driver, bno) 
LAND.run()

AVOIDANCE = PA(driver, bno, pi, picam2, goal_location = [35.9240852, 139.9112008]) #ok
AVOIDANCE.run()

GPS_StoF = GPS(driver, bno, pi, goal_location = [35.9240852, 139.9112008])
GPS_StoF.run()

FLAG = FN(driver, bno)
FLAG.run()

GPS_FtoG = GPS(driver, bno, pi, goal_location = [35.9241086 ,139.9113731])
GPS_FtoG.run()

GOAL = GDN(driver, bno, picam2, 30)
GOAL.run()

#実行文
print("クラス呼び出し完了です")
