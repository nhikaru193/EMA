import RPi.GPIO as GPIO
import time

class SM:
    def __init__(self, time_install):
        self.time_install = time_install
        self.SERVO_PIN = 13
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.SERVO_PIN, GPIO.OUT)   
        self.pwm = GPIO.PWM(self.SERVO_PIN, 50)

    def set_servo_duty(self.duty):
        self.pwm.ChangeDutyCycle(duty)
        time.sleep(0.5)
    
    def run():
        try:
            print("物資を設置するため、サーボモータの起動を行います")
            self.set_servo_duty(4.0)
            time.sleep(self.time_install)

        except keyboardInterupt:
            print("プログラムの中断が行われました")

        finally:
            self.pwm.stop()
            GPIO.cleanup()
            
