import RPi.GPIO as GPIO
import time
from motor import MotorDriver
import smbus
import struct

driver = MotorDriver(
    PWMA=12, AIN1=23, AIN2=18,   # 左モーター用（モータA）
    PWMB=19, BIN1=16, BIN2=26,   # 右モーター用（モータB）
    STBY=21                      # STBYピン
)

bno = BNO055()
if not bno.begin():
    print("Error initializing device")
    exit()
time.sleep(1)
bno.setMode(BNO055.OPERATION_MODE_NDOF)
time.sleep(1)
bno.setExternalCrystalUse(True)
time.sleep(1)


def correction_heading(before_heading, Lspeed, Rspeed):
    while True:
        after_heading = bno.get_heading()
        delta = (after_heading - before_heading + 180) % 360 -180
        
        if delta == 0:
            break
        elif delta < 20:
            Rspeed = Rspeed - 1
            driver.motor_Rforward(Rspeed)
        elif delta < 0:
            Rspeed = Rspeed - 0.5
            driver.motor_Rforward(Rspeed)
        elif delta > -20:
            Lspeed = Lspeed - 1
            driver.motor_Lforward(Lspeed)
        elif delta > 0:
            Lspeed = Lspeed - 0.5
            driver.motor_Lforward(Lspeed)

before_heading = bno.get_heading()
driver.changing_forward(0, 100)
correction_heading(before_heading, 100, 100)
        
                
                
                
    


