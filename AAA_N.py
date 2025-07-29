import cv2
import numpy as np
import time
import camera
import smbus
from picamera2 import Picamera2
import struct
import RPi.GPIO as GPIO
import math
import pigpio

#作成ファイルのインポート
import fusing
import BME280
import following
from BNO055 import BNO055
from motor import MotorDriver
from Flag_B import Flag_B

#ミッション部分
from C_RELEASE import RD
from C_Landing_Detective import LD
from C_PARACHUTE_AVOIDANCE import PA
from Flag_Navi import FN
from C_Servo import SM
from C_excellent_GPS import GPS
from C_GOAL_DETECTIVE_NOSHIRO import GDN

#おそらく未使用のモジュール
"""
import numpy
import busio
from C_Parachute_Avoidance import Parakai
"""
def set_servo_duty(duty):
    pwm.ChangeDutyCycle(duty)
    time.sleep(0.5)

#初期設定
Flag_location = [35.9181526, 139.9083178]
Goal_location = [35.9181214, 139.9082591]

#BNO055の初期設定
bno = BNO055()
bno.begin()
time.sleep(1)
bno.setMode(BNO055.OPERATION_MODE_NDOF)
time.sleep(1)
bno.setExternalCrystalUse(True)

while True:
    sys, gyro, accel, mag = bno.getCalibration()
    print(f"gyro:{gyro}")
    if gyro == 3 and mag == 3:
        print("BNO055のキャリブレーション終了")
        break

def degree_rotation(degree, threshold_deg = 5, sleeping = 0.01):
    before_heading = bno.getVector(BNO055.VECTOR_EULER)[0]
    target_heading = (before_heading + degree) % 360
    while True:
        current_heading = bno.getVector(BNO055.VECTOR_EULER)[0]
        delta_heading = ((target_heading - current_heading + 180) % 360) - 180
        if abs(delta_heading) <= threshold_deg:
            break
        elif delta_heading < -1 * threshold_deg:
            driver.petit_left(0, 90)
            time.sleep(sleeping)
            time.sleep(0.05)
            driver.motor_stop_brake()
            time.sleep(0.5)
        elif delta_heading > threshold_deg:
            driver.petit_right(0, 99)
            time.sleep(sleeping)
            time.sleep(0.05)
            driver.motor_stop_brake()
            time.sleep(0.5)
#関数のインスタンス作成
"""
RELEASE = RD(bno) #ok
RELEASE.run()

LAND = LD(bno) 
LAND.run()
"""
time.sleep(3)

driver = MotorDriver(
        PWMA=12, AIN1=23, AIN2=18,
        PWMB=19, BIN1=16, BIN2=26,
        STBY=21
    )
degree_rotation(-90, threshold_deg = 10)
time.sleep(0.3)
driver.motor_stop_brake()
time.sleep(1)
following.follow_forward(driver, bno, 80, 2)
driver.motor_stop_free()
time.sleep(2)
degree_rotation(90, threshold_deg = 10)
driver.motor_stop_brake()
time.sleep(1)
following.follow_forward(driver, bno, 80, 4)
driver.motor_stop_free()
time.sleep(2)

driver.cleanup()
"""
AVOIDANCE = PA(bno, goal_location = Flag_location) #ok
AVOIDANCE.run()

GPS_StoF = GPS(bno, goal_location = Flag_location)
GPS_StoF.run()
"""

FLAG = FN(bno, flag_location = Flag_location) 
FLAG.run()
"""
SERVO = SM(6)
SERVO.run()
"""
SERVO_PIN = 13  # GPIO13を使用
GPIO.setmode(GPIO.BCM)
GPIO.setup(SERVO_PIN, GPIO.OUT)
pwm = GPIO.PWM(SERVO_PIN, 50)
pwm.start(0)
print("逆回転（速い）")
set_servo_duty(4.0)
time.sleep(7)
pwm.stop()
GPIO.cleanup()

"""
GPS_FtoG = GPS(bno, goal_location = Goal_location)
GPS_FtoG.run()
"""

GOAL = GDN(bno, 30)
GOAL.run()

print("クラス呼び出し完了です")
