import RPi.GPIO as GPIO
import time
from motor import MotorDriver

driver = MotorDriver(
    PWMA=12, AIN1=23, AIN2=18,   # 左モーター用（モータA）
    PWMB=19, BIN1=16, BIN2=26,   # 右モーター用（モータB）
    STBY=21                      # STBYピン
)

try:
    for i in range (15):
        L = 85
        k = L - i
        print(f"duty比{k}です")
        driver.changing_moving_forward(0, L, 0, k)
        print("準備完了です")
        time.sleep(4)
        print("減速を開始します")
        driver.changing_moving_forward(L, 0, k, 0)
        time.sleep(0.2)

finally:
    driver.cleanup()
