import RPi.GPIO as GPIO
import time

def set_servo_duty(duty):
    pwm.ChangeDutyCycle(duty)
    time.sleep(0.5)

#------------------------#
def install(duty=12.5, duration=6):
    try:
        print("物資を格納するため、サーボモータの起動を行います")
        SERVO_PIN = 13
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(SERVO_PIN, GPIO.OUT)
        pwm = GPIO.PWM(SERVO_PIN, 50)
        pwm.start(0)
        print(f"物資をデューティ比{duty}、格納時間{duration}で設置します")
        set_servo_duty(duty)
        time.sleep(duration)

    except keyboardInterupt:
        print("プログラムの中断が行われました")

    finally:
        self.pwm.stop()
        GPIO.cleanup()
        print("物資の格納が正常に終了しました")
#------------------------#

#------------------------#
def release(duty=2.5, duration=6):
    try:
        print("物資を設置するため、サーボモータの起動を行います")
        SERVO_PIN = 13
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(SERVO_PIN, GPIO.OUT)
        pwm = GPIO.PWM(SERVO_PIN, 50)
        pwm.start(0)
        print(f"物資をデューティ比{duty}、設置時間{duration}で設置します")
        set_servo_duty(duty)
        time.sleep(duration)

    except keyboardInterupt:
        print("プログラムの中断が行われました")

    finally:
        self.pwm.stop()
        GPIO.cleanup()
        print("物資の設置が正常に終了しました")
#------------------------#
