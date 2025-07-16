import time
from BNO055 import BNO055
from motor import MotorDriver
import smbus
import struct
import RPi.GPIO as GPIO

#100付近にはしないこと。制御ができなくはならないけど、追従が遅くなる。
def follow_forward(driver, bno, base_speed, duration_time):
    target = bno.get_heading()
    
    #パラメータ
    base_speed = base_speed
    Kp = 0.80
    Kd = 0
    loop_interval = 0.10
    prev_err = 0.0
    derr = 0
    ls = 0
    rs = 0
    start_time = time.time()
    driver.changing_forward(0, base_speed)

    try:
        while True:
            current = bno.get_heading()
            err = (current - target + 180) % 360 - 180
            correction = Kp * err + Kd * derr
            ls = max(0, min(100, base_speed - correction))
            rs = max(0, min(100, base_speed + correction))
            driver.motor_Lforward(ls)
            driver.motor_Rforward(rs)
            after = bno.get_heading()
            time.sleep(loop_interval)
            derr = (after - current) / loop_interval
            delta_time = time.time() - start_time
            if delta_time > duration_time:
                for i in range (1, 100):
                    d_ls = ls / 100
                    d_rs = rs / 100
                    ls = ls - i * d_ls
                    rs = rs - i * d_rs
                    driver.motor_Lforward(ls)
                    driver.motor_Rforward(rs)
                    time.sleep(0.03)
                break
    finally:
        print("誘導終了")

#petit_forward0.1秒所要、減速0.2秒所要、
def follow_petit_forward(driver, bno, base_speed, duration_time):
    target = bno.get_heading()
    
    #パラメータ
    base_speed = base_speed
    Kp = 0.80
    Kd = 0
    loop_interval = 0.10
    prev_err = 0.0
    derr = 0
    ls = 0
    rs = 0
    start_time = time.time()
    driver.petit_forward(0, base_speed)

    try:
        while True:
            current = bno.get_heading()
            err = (current - target + 180) % 360 - 180
            correction = Kp * err + Kd * derr
            ls = max(0, min(100, base_speed - correction))
            rs = max(0, min(100, base_speed + correction))
            driver.motor_Lforward(ls)
            driver.motor_Rforward(rs)
            after = bno.get_heading()
            time.sleep(loop_interval)
            derr = (after - current) / loop_interval
            delta_time = time.time() - start_time
            if delta_time > duration_time:
                for i in range (1, 20):
                    d_ls = ls / 20
                    d_rs = rs / 20
                    ls = ls - i * d_ls
                    rs = rs - i * d_rs
                    driver.motor_Lforward(ls)
                    driver.motor_Rforward(rs)
                    time.sleep(0.01)
                break
    finally:
        print("誘導終了")

if __name__ == "__main__":
    try:
        driver = MotorDriver{
            PWMA=12, AIN1=23, AIN2=18,   # 左モーター用（モータA）
            PWMB=19, BIN1=16, BIN2=26,   # 右モーター用（モータB）
            STBY=21                      # STBYピン
        }
    
        bno = BNO055()
        bno.begin()
        time.sleep(1)
        bno.setMode(BNO055.OPERATION_MODE_NDOF)
        time.sleep(1)
        bno.setExternalCrystalUse(True)
        
        follow_forward(driver, bno, 90, 10)

    finally:
        driver.cleanup()
