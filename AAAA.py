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
from C_GOAL_DETECTIVE_NOSHIRO import GDN
from C_excellent_GPS import Amaging_GPS
from C_release import Release
from C_Landing_Detective import Landing
import fusing
import struct
import RPi.GPIO as GPIO


#モータの初期化
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

# カメラ初期化と設定
picam2 = Picamera2()
config = picam2.create_still_configuration(main={"size": (320, 240)})
picam2.configure(config)
picam2.start()
time.sleep(1)

#関数のインスタンス作成
RELEASE = Release(bno)
LAND = Landing(driver, bno)
#GPS = Amaging_GPS(driver, bno, GOAL_LOCATION=[x ,y])
GOAL = GDN(driver, bno, picam2, 30)

#実行文
LAND.run()
"""
RELEASE.run()
GPS.run()
GOAL.run()
"""
print("クラス呼び出し完了です")
