import time
import math
import RPi.GPIO as GPIO
from motor import MotorDriver  # ユーザーのMotorDriverクラスを使用
from BNO055 import BNO055
import smbus
import struct
import serial
import pigpio

try:
    driver = MotorDriver(
        PWMA=12, AIN1=23, AIN2=18,
        PWMB=19, BIN1=16, BIN2=26,
        STBY=21
    )
    while True:
        driver.petit_left(0, 90)
        driver.petit_left(90, 0)
        driver.motor_stop_brake()
        time.sleep(0.5)

except KeyboardInterrupt:
    print("中断します")

finally:
    driver.cleanup()
    print("プログラムを終了します")

