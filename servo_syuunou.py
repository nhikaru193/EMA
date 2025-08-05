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
    time.sleep(0.5)
    
try:
    print("収納")
    t = 12.5
    set_servo_duty(t)
    time.sleep(6)

finally:
    pwm.stop()
    GPIO.cleanup()
