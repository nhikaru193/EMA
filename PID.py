import time
from BNO055 import BNO055
from motor import MotorDriver
import smbus
import struct
import RPi.GPIO as GPIO

# --- 初期化 ---
bno = BNO055()
if not bno.begin():
    print("BNO055の初期化失敗")
    exit()
bno.setMode(BNO055.OPERATION_MODE_NDOF)
time.sleep(1)

motor = MotorDriver(PWMA=12, AIN1=23, AIN2=18,
                    PWMB=19, BIN1=16, BIN2=26, STBY=21)

# --- 目標方位を取得 ---
target_heading = bno.get_heading()
print(f"目標方位: {target_heading:.2f}°")

# --- 制御パラメータ ---
base_speed = 50      # ベースとなるPWMデューティ（0～100）
Kp = 1.06             # 比例ゲイン（要チューニング）
kd = 0.01
loop_interval = 0.08  # 制御周期[s]

prev_err = 0.0

try:
    while True:
        current = bno.get_heading()
        err = (current - target_heading + 180) % 360 - 180

        der = (err - prev_err) / loop_interval

        correction = Kp * err + kd * der
        
        ls = max(0, min(100, base_speed - correction))
        rs = max(0, min(100, base_speed + correction))

        motor.motor_Lforward(ls)
        motor.motor_Rforward(rs)

        # デバッグ出力
        print(f"現在: {current:.2f}°  誤差: {err:.2f}°  L:{ls:.1f}  R:{rs:.1f}")

        time.sleep(loop_interval)

except KeyboardInterrupt:
    motor.cleanup()
    print("終了")
