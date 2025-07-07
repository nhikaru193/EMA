import time
from motor import MotorDriver
import RPi.GPIO as GPIO

# === モーターインスタンス作成===
driver = MotorDriver(
    PWMA=12, AIN1=23, AIN2=18,    # 左モーター
    PWMB=19, BIN1=16, BIN2=26,    # 右モーター
    STBY=21
)
try:
    start_time = time.time()
    while time.time() - start_time < 14000:
        driver.changing_forward(0, 90)
        elapsed = int(time.time() - start_time)
        hours = elapsed // 3600
        minutes = (elapsed % 3600) // 60
        seconds = elapsed % 60
        print(f"現在の経過時間は{hours}時間{minutes}分{seconds}秒です")
        time.sleep(30)
        driver.changing_forward(90, 0)
        time.sleep(5)
        driver.changing_right(0, 40)
        driver.changing_right(40, 0)
        driver.changing_left(0, 40)
        driver.changing_left(40, 0)
        time.sleep(5)
    time.sleep(2)
except keyboardInterrupt:
    print("計測を終了します")

finally:
    driver.cleanup()
