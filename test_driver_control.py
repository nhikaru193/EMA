import RPi.GPIO as GPIO
import time
from motor import MotorDriver
from BNO055 import BNO055
import smbus
import struct

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

try:
    before_heading = bno.get_heading()
    driver.changing_forward(0, 80)
    Va1 = 80
    Va2 = 80
    Vb1 = 80
    Vb2 = 80
    while True:
        after_heading = bno.get_heading()
        delta_heading = (after_heading - before_heading) 
        if delta_heading >= 180:
            delta_heading = delta_heading - 180
        if delta_heading >= 20:
            Vb2 = Vb2 + 0.5
            Va2 = Va2 - 0.5
            driver.changing_Lforward(Va1, Va2)
            driver.changing_Rforward(Vb1, Vb2)
            Va1 = Va2
            Vb1 = Vb2
            time.sleep(0.02)
        elif delta_heading >= 10:
            Vb2 = Vb2 + 0.25
            Va2 = Va2 - 0.25
            driver.changing_Lforward(Va1, Va2)
            driver.changing_Rforward(Vb1, Vb2)
            Va1 = Va2
            Vb1 = Vb2
            time.sleep(0.02)
        elif delta_heading <= -20:
            Vb2 = Vb2 - 0.25
            Va2 = Va2 + 0.25
            driver.changing_Lforward(Va1, Va2)
            driver.changing_Rforward(Vb1, Vb2)
            Va1 = Va2
            Vb1 = Vb2
            time.sleep(0.02)
        elif delta_heading <= -10:
            Vb2 = Vb2 - 0.5
            Va2 = Va2 + 0.5
            driver.changing_Lforward(Va1, Va2)
            driver.changing_Rforward(Vb1, Vb2)
            Va1 = Va2
            Vb1 = Vb2
            time.sleep(0.02)
        else:
            break
    
finally:
    driver.cleanup()
