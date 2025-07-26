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

def changing_servo_reverse(before, after):
    global speed
    for i in range (1, 100):
        delta_speed = (after - before) / 100
        speed = before + i * delta_speed
        set_servo_duty(speed)
        time.sleep(0.1)
    
try:
    """
    print("サーボ停止")
    set_servo_duty(7.5)

    print("正回転（速い）")
    changing_servo_reverse(7.5, 10)
    time.sleep(3)
    
    """
    t = 4
    print(f"放出デューティ比{t}")
    set_servo_duty(t)
    time.sleep(6)
    time.sleep(3)
    """

    print("停止")
    changing_servo_reverse(10, 7.5)
    """

    set

finally:
    pwm.stop()
    GPIO.cleanup()
