import RPi.GPIO as GPIO
import time
from motor import MotorDriver

driver = MotorDriver(
    PWMA=12, AIN1=23, AIN2=18,   # 左モーター用（モータA）
    PWMB=19, BIN1=16, BIN2=26,   # 右モーター用（モータB）
    STBY=21                      # STBYピン
)

try:
    for i in range (30):
        k = 100 - i
        print(f"duty比{k}です")
        driver.changing_moving_forward(0, 100, 0, k)
        print("準備完了です")
        time.sleep(4)
        print("減速を開始します")
        driver.changing_moving_forward(100, 0, k, 0)

finally:
    driver.cleanup()
