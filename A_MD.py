
import RPi.GPIO as GPIO
import time
import BNO055
import smbus
import struct

class MotorDriver():
    # 初期設定関数の定義
    def __init__(self,
                 PWMA, AIN1, AIN2,
                 PWMB, BIN1, BIN2, STBY,
                 freq = 1000):
        
        # GPIO初期化
        GPIO.setmode(GPIO.BCM)
        # すべてのpinの出力を開始(使えるようになる)
        GPIO.setup([AIN1, AIN2, PWMA, BIN1, BIN2, PWMB, STBY], GPIO.OUT)
        
        # モータ起動 (STBYピンをHIGHにしてモータードライバを有効化)
        GPIO.output(STBY, GPIO.HIGH)

        # self型の関数に格納
        # なぜか？⇒self型に格納しないと同クラス内で関数の情報が引き継がない(スコープの問題)
        self.A1, self.A2 = AIN1, AIN2
        self.B1, self.B2 = BIN1, BIN2
        self.pwma = GPIO.PWM(PWMA, freq)
        self.pwmb = GPIO.PWM(PWMB, freq)
        
        # motorの起動：デューティ比0⇒停止
        self.pwma.start(0)
        self.pwmb.start(0)

    # 右回頭
    def motor_right(self, speed):
        GPIO.output(self.A1, GPIO.LOW)  # 左モーター後退
        GPIO.output(self.A2, GPIO.HIGH)
        GPIO.output(self.B1, GPIO.LOW)  # 右モーター後退
        GPIO.output(self.B2, GPIO.HIGH)
        self.pwma.ChangeDutyCycle(speed)
        self.pwmb.ChangeDutyCycle(speed)

    # 左回頭
    def motor_left(self, speed):
        GPIO.output(self.A1, GPIO.HIGH) # 左モーター前進
        GPIO.output(self.A2, GPIO.LOW)
        GPIO.output(self.B1, GPIO.HIGH) # 右モーター前進
        GPIO.output(self.B2, GPIO.LOW)
        self.pwma.ChangeDutyCycle(speed)
        self.pwmb.ChangeDutyCycle(speed)

    # 後退
    def motor_retreat(self, speed):
        GPIO.output(self.A1, GPIO.HIGH)
        GPIO.output(self.A2, GPIO.LOW)
        GPIO.output(self.B1, GPIO.LOW)
        GPIO.output(self.B2, GPIO.HIGH)
        self.pwma.ChangeDutyCycle(speed)
        self.pwmb.ChangeDutyCycle(speed)
    
    # モータのトルクでブレーキをかける (実際はピンをLOWにするだけ)
    def motor_stop_free(self):
        self.pwma.ChangeDutyCycle(0)
        self.pwmb.ChangeDutyCycle(0)
        GPIO.output(self.A1, GPIO.LOW)
        GPIO.output(self.A2, GPIO.LOW)
        GPIO.output(self.B1, GPIO.LOW)
        GPIO.output(self.B2, GPIO.LOW)
    
    # ガチブレーキ(恐らく)
    def motor_stop_brake(self):
        self.pwma.ChangeDutyCycle(0)
        self.pwmb.ChangeDutyCycle(0)
        GPIO.output(self.A1, GPIO.HIGH) # 両方のピンをHIGHにしてブレーキ
        GPIO.output(self.A2, GPIO.HIGH)
        GPIO.output(self.B1, GPIO.HIGH)
        GPIO.output(self.B2, GPIO.HIGH)

    # 雑なキャリブレーション (リソース解放)
    def cleanup(self):
        self.pwma.stop()
        self.pwmb.stop()
        GPIO.cleanup()
    
    # 前進：任意
    def motor_forward(self, speed):
        GPIO.output(self.A1, GPIO.LOW)
        GPIO.output(self.A2, GPIO.HIGH)
        GPIO.output(self.B1, GPIO.HIGH)
        GPIO.output(self.B2, GPIO.LOW)
        self.pwma.ChangeDutyCycle(speed)
        self.pwmb.ChangeDutyCycle(speed)
    
    def motor_Lforward(self, speed):
        GPIO.output(self.A1, GPIO.LOW)
        GPIO.output(self.A2, GPIO.HIGH)
        self.pwma.ChangeDutyCycle(speed)
            
    def motor_Rforward(self, speed):
        GPIO.output(self.B1, GPIO.HIGH)
        GPIO.output(self.B2, GPIO.LOW)
        self.pwmb.ChangeDutyCycle(speed)
            
    # 前進：回転数制御(異なる回転数へ変化するときに滑らかに遷移するようにする)
    def changing_forward(self, before, after):
        global speed # main_rover_control.pyから呼び出される際にglobal speedが定義されていないとエラーになる可能性
        for i in range(1, 100):
            delta_speed = (after - before) / 100
            speed = before + i * delta_speed
            self.motor_forward(speed)
            time.sleep(0.02)

    def changing_Lforward(self, before, after):
        global speed # main_rover_control.pyから呼び出される際にglobal speedが定義されていないとエラーになる可能性
        for i in range(1, 100):
            delta_speed = (after - before) / 100
            speed = before + i * delta_speed
            self.motor_Lforward(speed)
            time.sleep(0.03)
            
    def changing_Rforward(self, before, after):
        global speed # main_rover_control.pyから呼び出される際にglobal speedが定義されていないとエラーになる可能性
        for i in range(1, 100):
            delta_speed = (after - before) / 100
            speed = before + i * delta_speed
            self.motor_Rforward(speed)
            time.sleep(0.03)
            
    # 右折：回転数制御(基本は停止してから使いましょう)
    def changing_right(self, before, after):
        global speed # main_rover_control.pyから呼び出される際にglobal speedが定義されていないとエラーになる可能性
        for i in range(50):
            delta_speed = (after - before) / 50
            speed = before + i * delta_speed
            self.motor_right(speed)
            time.sleep(0.03)
    
    # 左折（同様）
    def changing_left(self, before, after):
        global speed # main_rover_control.pyから呼び出される際にglobal speedが定義されていないとエラーになる可能性
        for i in range(50):
            delta_speed = (after - before) / 50
            speed = before + i * delta_speed
            self.motor_left(speed)
            time.sleep(0.03)

    # 後退：回転数制御(異なる回転数へ変化するときに滑らかに遷移するようにする)
    def changing_retreat(self, before, after):
        global speed # main_rover_control.pyから呼び出される際にglobal speedが定義されていないとエラーになる可能性
        for i in range(50):
            delta_speed = (after - before) / 50
            speed = before + i * delta_speed
            self.motor_retreat(speed)
            time.sleep(0.03)
            
    def quick_right(self, before, after):
        global speed # main_rover_control.pyから呼び出される際にglobal speedが定義されていないとエラーになる可能性
        for i in range(10):
            delta_speed = (after - before) / 10
            speed = before + i * delta_speed
            self.motor_right(speed)
            time.sleep(0.02)

    def quick_left(self, before, after):
        global speed # main_rover_control.pyから呼び出される際にglobal speedが定義されていないとエラーになる可能性
        for i in range(10):
            delta_speed = (after - before) / 10
            speed = before + i * delta_speed
            self.motor_left(speed)
            time.sleep(0.02)
    
    def changing_moving_forward(self, Lmotor_b, Lmotor_a ,Rmotor_b, Rmotor_a):
        global speed # main_rover_control.pyから呼び出される際にglobal speedが定義されていないとエラーになる可能性
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

    def petit_back(self, before, after):#ほたがついか
        for i in range (1, 5):
            delta_speed = (after - before) / 5
            speed = before + i * delta_speed
            self.motor_retreat(speed)
            time.sleep(0.02)
            
    def petit_left(self, before, after):
        for i in range (1, 5):
            delta_speed = (after - before) / 5
            speed = before + i * delta_speed
            self.motor_left(speed) # notor_left を motor_left に修正
            time.sleep(0.02)

    def petit_right(self, before, after):
        for i in range (1, 5):
            delta_speed = (after - before) / 5
            speed = before + i * delta_speed
            self.motor_right(speed) # notor_right を motor_right に修正
            time.sleep(0.02)
            
    def petit_petit(self, count):
        for i in range (1, count):
            self.petit_forward(0, 90)
            self.petit_forward(90, 0)
            time.sleep(0.2)

    def petit_petit_retreat(self, count):#ほたが追加
            for i in range (1, count):
                self.petit_back(0, 90)
                self.petit_back(90, 0)
                time.sleep(0.2)
