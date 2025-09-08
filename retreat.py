from motor import MotorDriver
import RPi.GPIO as GPIO
import time

driver = MotorDriver(
    PWMA=12, AIN1=23, AIN2=18,   # 左モーター用（モータA）
    PWMB=19, BIN1=16, BIN2=26,   # 右モーター用（モータB）
    STBY=21                      # STBYピン
)

driver.petit_petit_retreat(6)


GPIO.cleanup()
print("GPIOのクリーンアップを実行しました。")
