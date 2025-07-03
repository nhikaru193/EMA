import RPi.GPIO as GPIO
import time
import BNO055
import smbus 
import struct

driver = MotorDriver(
    PWMA=12, AIN1=23, AIN2=18,   # 左モーター用（モータA）
    PWMB=19, BIN1=16, BIN2=26,   # 右モーター用（モータB）
    STBY=21                      # STBYピン
)

driver.changing_right(0, 100)
time.sleep(5)
driver.changing_right(100, 0)
driver.cleanup()
