import RPi.GPIO as GPIO # GPIO.cleanup()のために残しますが、ピン設定はpigpioで行いません
import time
import pigpio # pigpioを使うように変更

class MotorDriver:
    """
    pigpioライブラリを使用してモータードライバーを制御するクラス。
    全てのGPIOピン設定とPWM制御はpigpioで行います。
    """

    def __init__(self, PWMA, AIN1, AIN2,
                 PWMB, BIN1, BIN2, STBY,
                 freq=1000):
        
        # pigpioインスタンスはメインスクリプトから受け取るべきですが、
        # MotorDriverは低レベル制御なので、ここでは内部でpiインスタンスを作成します。
        # (ただし、メインスクリプトで既にpigpio.pi()が呼ばれていれば、
        # pigpioは既存のデーモンに接続するだけなので問題ありません)
        self.pi = pigpio.pi()
        if not self.pi.connected:
            raise IOError("pigpio daemon not connected in MotorDriver.")

        self.A1, self.A2 = AIN1, AIN2
        self.B1, self.B2 = BIN1, BIN2
        self.PWMA_PIN, self.PWMB_PIN = PWMA, PWMB # PWMピンの番号を保存
        self.STBY_PIN = STBY

        # --- pigpioによるGPIO初期化 ---
        # 全てのピンを出力モードに設定
        self.pi.set_mode(self.A1, pigpio.OUTPUT)
        self.pi.set_mode(self.A2, pigpio.OUTPUT)
        self.pi.set_mode(self.PWMA_PIN, pigpio.OUTPUT)
        self.pi.set_mode(self.B1, pigpio.OUTPUT)
        self.pi.set_mode(self.B2, pigpio.OUTPUT)
        self.pi.set_mode(self.PWMB_PIN, pigpio.OUTPUT)
        self.pi.set_mode(self.STBY_PIN, pigpio.OUTPUT)

        # モーター起動 (STBYピンをHIGHにしてモータードライバを有効化)
        self.pi.write(self.STBY_PIN, 1) # pigpioでHIGHは1

        # --- pigpioによるPWM初期化 ---
        self.pwm_freq = freq # 周波数

        # PWMA/PWMBピンのPWM周波数を設定
        self.pi.set_PWM_frequency(self.PWMA_PIN, self.pwm_freq)
        self.pi.set_PWM_frequency(self.PWMB_PIN, self.pwm_freq)

        # PWMの範囲を設定 (デューティサイクルの最大値)。0-255 の範囲で指定
        self.pi.set_PWM_range(self.PWMA_PIN, 255) # 0-255の範囲に設定 (RPi.GPIOの0-100に合わせるなら100にする)
        self.pi.set_PWM_range(self.PWMB_PIN, 255) # 0-255の範囲に設定

        # motorの起動：デューティ比0⇒停止
        self.pi.set_PWM_dutycycle(self.PWMA_PIN, 0)
        self.pi.set_PWM_dutycycle(self.PWMB_PIN, 0)

        # 最大速度をRPi.GPIOの100%に合わせるため、MAX_SPEEDを定義
        self.MAX_SPEED = 255 # pigpioのデューティサイクル範囲の最大値

        print("✅ MotorDriver: インスタンス作成完了 (pigpioベース)。")

    # 右回頭
    def motor_right(self, speed):
        duty = int(speed / 100 * self.MAX_SPEED) # 0-100%を0-MAX_SPEEDに変換
        self.pi.write(self.A1, 0) # pigpio LOWは0
        self.pi.write(self.A2, 1) # pigpio HIGHは1
        self.pi.write(self.B1, 0)
        self.pi.write(self.B2, 1)
        self.pi.set_PWM_dutycycle(self.PWMA_PIN, duty)
        self.pi.set_PWM_dutycycle(self.PWMB_PIN, duty)

    # 左回頭
    def motor_left(self, speed):
        duty = int(speed / 100 * self.MAX_SPEED)
        self.pi.write(self.A1, 1)
        self.pi.write(self.A2, 0)
        self.pi.write(self.B1, 1)
        self.pi.write(self.B2, 0)
        self.pi.set_PWM_dutycycle(self.PWMA_PIN, duty)
        self.pi.set_PWM_dutycycle(self.PWMB_PIN, duty)

    # 後退
    def motor_retreat(self, speed):
        duty = int(speed / 100 * self.MAX_SPEED)
        self.pi.write(self.A1, 1)
        self.pi.write(self.A2, 0)
        self.pi.write(self.B1, 0)
        self.pi.write(self.B2, 1)
        self.pi.set_PWM_dutycycle(self.PWMA_PIN, duty)
        self.pi.set_PWM_dutycycle(self.PWMB_PIN, duty)
    
    # モータのトルクでブレーキをかける (実際はピンをLOWにするだけ)
    def motor_stop_free(self):
        self.pi.set_PWM_dutycycle(self.PWMA_PIN, 0)
        self.pi.set_PWM_dutycycle(self.PWMB_PIN, 0)
        self.pi.write(self.A1, 0)
        self.pi.write(self.A2, 0)
        self.pi.write(self.B1, 0)
        self.pi.write(self.B2, 0)
    
    # ガチブレーキ
    def motor_stop_brake(self):
        self.pi.set_PWM_dutycycle(self.PWMA_PIN, 0)
        self.pi.set_PWM_dutycycle(self.PWMB_PIN, 0)
        self.pi.write(self.A1, 1)
        self.pi.write(self.A2, 1)
        self.pi.write(self.B1, 1)
        self.pi.write(self.B2, 1)

    # 前進：任意
    def motor_forward(self, speed):
        duty = int(speed / 100 * self.MAX_SPEED)
        self.pi.write(self.A1, 0)
        self.pi.write(self.A2, 1)
        self.pi.write(self.B1, 1)
        self.pi.write(self.B2, 0)
        self.pi.set_PWM_dutycycle(self.PWMA_PIN, duty)
        self.pi.set_PWM_dutycycle(self.PWMB_PIN, duty)
    
    def motor_Lforward(self, speed):
        duty = int(speed / 100 * self.MAX_SPEED)
        self.pi.write(self.A1, 0)
        self.pi.write(self.A2, 1)
        self.pi.set_PWM_dutycycle(self.PWMA_PIN, duty)
            
    def motor_Rforward(self, speed):
        duty = int(speed / 100 * self.MAX_SPEED)
        self.pi.write(self.B1, 1)
        self.pi.write(self.B2, 0)
        self.pi.set_PWM_dutycycle(self.PWMB_PIN, duty)
            
    # 前進：回転数制御(異なる回転数へ変化するときに滑らかに遷移するようにする)
    def changing_forward(self, before, after):
        # global speed はこのクラスのメソッド内では不要。speedはループ内のローカル変数で良い。
        for i in range(1, 100):
            delta_speed = (after - before) / 100
            speed = before + i * delta_speed
            self.motor_forward(speed)
            time.sleep(0.02)

    def changing_Lforward(self, before, after):
        for i in range(1, 100):
            delta_speed = (after - before) / 100
            speed = before + i * delta_speed
            self.motor_Lforward(speed)
            time.sleep(0.03)
            
    def changing_Rforward(self, before, after):
        for i in range(1, 100):
            delta_speed = (after - before) / 100
            speed = before + i * delta_speed
            self.motor_Rforward(speed)
            time.sleep(0.03)
            
    # 右折：回転数制御(基本は停止してから使いましょう)
    def changing_right(self, before, after):
        for i in range(50):
            delta_speed = (after - before) / 50
            speed = before + i * delta_speed
            self.motor_right(speed)
            time.sleep(0.03)
    
    # 左折（同様）
    def changing_left(self, before, after):
        for i in range(50):
            delta_speed = (after - before) / 50
            speed = before + i * delta_speed
            self.motor_left(speed)
            time.sleep(0.03)

    # 後退：回転数制御
    def changing_retreat(self, before, after):
        for i in range(50):
            delta_speed = (after - before) / 50
            speed = before + i * delta_speed
            self.motor_retreat(speed)
            time.sleep(0.03)
            
    def quick_right(self, before, after):
        for i in range(10):
            delta_speed = (after - before) / 10
            speed = before + i * delta_speed
            self.motor_right(speed)
            time.sleep(0.02)

    def quick_left(self, before, after):
        for i in range(10):
            delta_speed = (after - before) / 10
            speed = before + i * delta_speed
            self.motor_left(speed)
            time.sleep(0.02)
    
    def changing_moving_forward(self, Lmotor_b, Lmotor_a ,Rmotor_b, Rmotor_a):
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
        for i in range (1, count + 1): # count回実行するために +1
            self.petit_forward(0, 90)
            self.petit_forward(90, 0)
            time.sleep(0.2)
            
    # モータードライバのクリーンアップ (pigpioピンをクリア)
    def cleanup(self):
        # PWMを停止し、ピンを出力から入力に戻す
        self.pi.set_PWM_dutycycle(self.PWMA_PIN, 0)
        self.pi.set_PWM_dutycycle(self.PWMB_PIN, 0)
        self.pi.set_mode(self.A1, pigpio.INPUT)
        self.pi.set_mode(self.A2, pigpio.INPUT)
        self.pi.set_mode(self.PWMA_PIN, pigpio.INPUT)
        self.pi.set_mode(self.B1, pigpio.INPUT)
        self.pi.set_mode(self.B2, pigpio.INPUT)
        self.pi.set_mode(self.PWMB_PIN, pigpio.INPUT)
        self.pi.set_mode(self.STBY_PIN, pigpio.INPUT)
        # pi.stop() はメインで呼ぶ
        print("MotorDriver: クリーンアップ完了。")
