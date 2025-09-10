#import RPi.GPIO as GPIO # 削除
import pigpio # pigpioのインポート
import time
import BNO055
import smbus
import struct

class MotorDriver():
    def __init__(self, pi, PWMA, AIN1, AIN2, PWMB, BIN1, BIN2, STBY, freq=1000):
        self.pi = pi  # pigpioオブジェクトをインスタンス変数に保存
        self.pi.set_mode(AIN1, pigpio.OUTPUT)
        self.pi.set_mode(AIN2, pigpio.OUTPUT)
        self.pi.set_mode(PWMA, pigpio.OUTPUT)
        self.pi.set_mode(BIN1, pigpio.OUTPUT)
        self.pi.set_mode(BIN2, pigpio.OUTPUT)
        self.pi.set_mode(PWMB, pigpio.OUTPUT)
        self.pi.set_mode(STBY, pigpio.OUTPUT)
        
        # モータ起動 (STBYピンをHIGHにしてモータードライバを有効化)
        self.pi.write(STBY, 1)

        # self型の関数に格納
        self.A1, self.A2 = AIN1, AIN2
        self.B1, self.B2 = BIN1, BIN2
        self.PWMA = PWMA
        self.PWMB = PWMB
        self.STBY = STBY
        
        # PWM周波数の設定
        self.pi.set_PWM_frequency(self.PWMA, freq)
        self.pi.set_PWM_frequency(self.PWMB, freq)
        
        # motorの起動：デューティ比0⇒停止
        self.pi.set_PWM_dutycycle(self.PWMA, 0)
        self.pi.set_PWM_dutycycle(self.PWMB, 0)

    # 速度をpigpioのデューティ比に変換するヘルパー関数
    def _convert_speed(self, speed):
        return (speed * 255) // 100

    # 右回頭
    def motor_right(self, speed):
        duty = self._convert_speed(speed)
        self.pi.write(self.A1, 0)
        self.pi.write(self.A2, 1)
        self.pi.write(self.B1, 0)
        self.pi.write(self.B2, 1)
        self.pi.set_PWM_dutycycle(self.PWMA, duty)
        self.pi.set_PWM_dutycycle(self.PWMB, duty)

    # 左回頭
    def motor_left(self, speed):
        duty = self._convert_speed(speed)
        self.pi.write(self.A1, 1)
        self.pi.write(self.A2, 0)
        self.pi.write(self.B1, 1)
        self.pi.write(self.B2, 0)
        self.pi.set_PWM_dutycycle(self.PWMA, duty)
        self.pi.set_PWM_dutycycle(self.PWMB, duty)

    # 後退
    def motor_retreat(self, speed):
        duty = self._convert_speed(speed)
        self.pi.write(self.A1, 1)
        self.pi.write(self.A2, 0)
        self.pi.write(self.B1, 0)
        self.pi.write(self.B2, 1)
        self.pi.set_PWM_dutycycle(self.PWMA, duty)
        self.pi.set_PWM_dutycycle(self.PWMB, duty)
    
    # モータのトルクでブレーキをかける
    def motor_stop_free(self):
        self.pi.set_PWM_dutycycle(self.PWMA, 0)
        self.pi.set_PWM_dutycycle(self.PWMB, 0)
        self.pi.write(self.A1, 0)
        self.pi.write(self.A2, 0)
        self.pi.write(self.B1, 0)
        self.pi.write(self.B2, 0)
    
    # ガチブレーキ
    def motor_stop_brake(self):
        self.pi.set_PWM_dutycycle(self.PWMA, 0)
        self.pi.set_PWM_dutycycle(self.PWMB, 0)
        self.pi.write(self.A1, 1)
        self.pi.write(self.A2, 1)
        self.pi.write(self.B1, 1)
        self.pi.write(self.B2, 1)

    # 雑なキャリブレーション (リソース解放)
    def cleanup(self):
        self.pi.set_PWM_dutycycle(self.PWMA, 0)
        self.pi.set_PWM_dutycycle(self.PWMB, 0)
        self.pi.write(self.STBY, 0) # STBYをLOWにしてモータードライバを無効化
    
    # 前進
    def motor_forward(self, speed):
        duty = self._convert_speed(speed)
        self.pi.write(self.A1, 0)
        self.pi.write(self.A2, 1)
        self.pi.write(self.B1, 1)
        self.pi.write(self.B2, 0)
        self.pi.set_PWM_dutycycle(self.PWMA, duty)
        self.pi.set_PWM_dutycycle(self.PWMB, duty)
    
    def motor_Lforward(self, speed):
        duty = self._convert_speed(speed)
        self.pi.write(self.A1, 0)
        self.pi.write(self.A2, 1)
        self.pi.set_PWM_dutycycle(self.PWMA, duty)
    
    def motor_Rforward(self, speed):
        duty = self._convert_speed(speed)
        self.pi.write(self.B1, 1)
        self.pi.write(self.B2, 0)
        self.pi.set_PWM_dutycycle(self.PWMB, duty)
            
    # 前進：回転数制御(異なる回転数へ変化するときに滑らかに遷移するようにする)
    def changing_forward(self, before, after):
        # global speed # 不要なため削除
        for i in range(1, 100):
            delta_speed = (after - before) / 100
            speed = before + i * delta_speed
            self.motor_forward(speed)
            time.sleep(0.02)

    def changing_Lforward(self, before, after):
        # global speed # 不要なため削除
        for i in range(1, 100):
            delta_speed = (after - before) / 100
            speed = before + i * delta_speed
            self.motor_Lforward(speed)
            time.sleep(0.03)
            
    def changing_Rforward(self, before, after):
        # global speed # 不要なため削除
        for i in range(1, 100):
            delta_speed = (after - before) / 100
            speed = before + i * delta_speed
            self.motor_Rforward(speed)
            time.sleep(0.03)
            
    # 右折：回転数制御
    def changing_right(self, before, after):
        # global speed # 不要なため削除
        for i in range(50):
            delta_speed = (after - before) / 50
            speed = before + i * delta_speed
            self.motor_right(speed)
            time.sleep(0.03)
    
    # 左折
    def changing_left(self, before, after):
        # global speed # 不要なため削除
        for i in range(50):
            delta_speed = (after - before) / 50
            speed = before + i * delta_speed
            self.motor_left(speed)
            time.sleep(0.03)

    # 後退：回転数制御
    def changing_retreat(self, before, after):
        # global speed # 不要なため削除
        for i in range(50):
            delta_speed = (after - before) / 50
            speed = before + i * delta_speed
            self.motor_retreat(speed)
            time.sleep(0.03)
            
    def quick_right(self, before, after):
        # global speed # 不要なため削除
        for i in range(10):
            delta_speed = (after - before) / 10
            speed = before + i * delta_speed
            self.motor_right(speed)
            time.sleep(0.02)

    def quick_left(self, before, after):
        # global speed # 不要なため削除
        for i in range(10):
            delta_speed = (after - before) / 10
            speed = before + i * delta_speed
            self.motor_left(speed)
            time.sleep(0.02)
    
    def changing_moving_forward(self, Lmotor_b, Lmotor_a ,Rmotor_b, Rmotor_a):
        # global speed # 不要なため削除
        for i in range(1, 20):
            delta_speed_L = (Lmotor_a - Lmotor_b) / 20
            delta_speed_R = (Rmotor_a - Rmotor_b) / 20
            speed_L = Lmotor_b + i * delta_speed_L
            speed_R = Rmotor_b + i * delta_speed_R
            self.motor_Lforward(speed_L)
            self.motor_Rforward(speed_R)
            time.sleep(0.02)

    def petit_forward(self, before, after):
        for i in range (1, 5):
            delta_speed = (after - before) / 5
            speed = before + i * delta_speed
            self.motor_forward(speed)
            time.sleep(0.02)

    def petit_back(self, before, after):
        for i in range (1, 5):
            delta_speed = (after - before) / 5
            speed = before + i * delta_speed
            self.motor_retreat(speed)
            time.sleep(0.02)
            
    def petit_left(self, before, after):
        for i in range (1, 5):
            delta_speed = (after - before) / 5
            speed = before + i * delta_speed
            self.motor_left(speed)
            time.sleep(0.02)

    def petit_right(self, before, after):
        for i in range (1, 5):
            delta_speed = (after - before) / 5
            speed = before + i * delta_speed
            self.motor_right(speed)
            time.sleep(0.02)
            
    def petit_petit(self, count):
        for i in range (1, count):
            self.petit_forward(0, 90)
            self.petit_forward(90, 0)
            time.sleep(0.2)

    def petit_petit_retreat(self, count):
        for i in range (1, count):
            self.petit_back(0, 90)
            self.petit_back(90, 0)
            time.sleep(0.2)
