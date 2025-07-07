import RPi.GPIO as GPIO
import time
from motor import MotorDriver
from BNO055 import BNO055
import smbus
import struct
import following

#モータードライバのインスタンス作成
driver = MotorDriver(
    PWMA=12, AIN1=23, AIN2=18,   # 左モーター用（モータA）
    PWMB=19, BIN1=16, BIN2=26,   # 右モーター用（モータB）
    STBY=21                      # STBYピン
)

#九軸センサのインスタンス作成
bno = BNO055()
bno.begin()
time.sleep(1)
bno.setMode(BNO055.OPERATION_MODE_NDOF)
time.sleep(1)
bno.setExternalCrystalUse(True)

following.follow_forward(driver, bno, 70, 5)
following.follow_forward(driver, bno, 50, 5)
