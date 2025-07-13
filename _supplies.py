import RPi.GPIO as GPIO
import time

SERVO_PIN = 13  # GPIO13を使用

GPIO.setmode(GPIO.BCM)
GPIO.setup(SERVO_PIN, GPIO.OUT)

# 50Hz の PWM波形（サーボ用）
pwm = GPIO.PWM(SERVO_PIN, 50)
pwm.start(0)

def set_servo_duty(duty):
    pwm.ChangeDutyCycle(duty)
    # 少し待たないとデューティが反映されないことがある
    time.sleep(0.02)

def rotate_cw(speed=1.0, duration=1.0):
    """
    連続回転サーボを時計回り（CW）に回す
    speed:  0.0～1.0（1.0が最大速）
    duration: 動作時間（秒）
    """
    # ニュートラル7.5% から最大2.5%上振れ
    duty = 7.5 + 2.5 * max(0.0, min(speed, 1.0))
    set_servo_duty(duty)
    time.sleep(duration)
    set_servo_duty(7.5)  # 停止

def rotate_ccw(speed=1.0, duration=1.0):
    """
    連続回転サーボを反時計回り（CCW）に回す
    speed:  0.0～1.0（1.0が最大速）
    duration: 動作時間（秒）
    """
    # ニュートラル7.5% から最大2.5%下振れ
    duty = 7.5 - 2.5 * max(0.0, min(speed, 1.0))
    set_servo_duty(duty)
    time.sleep(duration)
    set_servo_duty(7.5)  # 停止

try:
    # 例：正回転で1秒、逆回転で1秒
    rotate_cw(speed=0.8, duration=1.0)
    time.sleep(0.5)
    rotate_ccw(speed=0.5, duration=1.0)

finally:
    pwm.stop()
    GPIO.cleanup()
