import time
import pigpio # pigpioを使うように変更
import RPi.GPIO as GPIO # GPIO.cleanup()のために残すが、ピン設定は行わない

class ServoController:
    """
    pigpioライブラリのPWM機能を使用してサーボモーターを制御するクラス。
    ピンのセットアップやPWM制御はすべてpigpioで行います。
    """

    def __init__(self, pi_instance, servo_pin=13, pwm_frequency=50):
        """
        ServoControllerのコンストラクタです。

        Args:
            pi_instance (pigpio.pi): 既に初期化されたpigpioのインスタンス。
            servo_pin (int): サーボモーターが接続されているGPIOピンの番号 (BCMモード)。
            pwm_frequency (int): PWM信号の周波数 (Hz)。標準的なサーボは50Hzを使用します。
        """
        self.pi = pi_instance # pigpioインスタンスを使用
        self.servo_pin = servo_pin
        self.pwm_frequency = pwm_frequency

        # --- pigpioによるサーボPWMの初期化 ---
        # 1. ピンを出力モードに設定
        self.pi.set_mode(self.servo_pin, pigpio.OUTPUT)
        
        # 2. PWM周波数を設定 (通常50Hz)
        self.pi.set_PWM_frequency(self.servo_pin, self.pwm_frequency)
        
        # 3. PWMの範囲を設定 (デューティサイクルの最大値)。通常 0-1,000,000 ですが、
        #    ここではデューティサイクルをパーセンテージで受け取りやすいため、
        #    10000 などの値を設定し、後に100%の値を10000と対応させます。
        #    一般的なサーボのデューティサイクル(2.5%～12.5%)をそのまま使用できるよう、
        #    RPi.GPIOのPWM_rangeが0-100なので、それに対応させます。
        self.pi.set_PWM_range(self.servo_pin, 100) # デューティサイクルを0-100で指定できるように設定
        
        # 4. 初期デューティサイクルを設定 (サーボ停止位置: 通常0)
        self.pi.set_PWM_dutycycle(self.servo_pin, 0) 
        
        print(f"✅ GPIO{self.servo_pin} でサーボをpigpio PWMで初期化しました (周波数: {self.pwm_frequency}Hz)。")

    def set_duty_cycle(self, duty_cycle):
        """
        サーボモーターのデューティサイクルを設定します。
        pigpioのデューティサイクル範囲 (0-PWM_range) に合わせて設定します。

        Args:
            duty_cycle (float): 設定するデューティサイクル値 (0.0 から 100.0)。
                                一般的にサーボは2.5 (反時計回り最大) から 12.5 (時計回り最大) の範囲。
        """
        if not (0.0 <= duty_cycle <= 100.0):
            print(f"警告: 不正なデューティサイクル値 ({duty_cycle}) です。0.0から100.0の範囲で指定してください。")
            return
        
        # pigpioのデューティサイクルは0からPWM_range (ここでは100) なので、そのまま渡せる
        self.pi.set_PWM_dutycycle(self.servo_pin, int(duty_cycle))
        time.sleep(0.5) # サーボが位置に到達するのを待つ
        print(f"デューティサイクルを {duty_cycle:.1f}% に設定しました。")

    def gradually_change_duty_cycle(self, start_duty, end_duty, steps=50, delay_per_step=0.05):
        """
        サーボのデューティサイクルを`start_duty`から`end_duty`まで徐々に変化させます。
        pigpioのデューティサイクル範囲に合わせて設定します。
        """
        print(f"デューティサイクルを {start_duty:.1f}% から {end_duty:.1f}% へ徐々に変化させます。")
        for i in range(steps + 1):
            current_duty = start_duty + (end_duty - start_duty) * i / steps
            self.pi.set_PWM_dutycycle(self.servo_pin, int(current_duty))
            time.sleep(delay_per_step)
        print(f"デューティサイクル変化完了。最終値: {end_duty:.1f}%")

    def stop_pwm(self):
        """
        PWM信号の出力を停止し、サーボへの制御を終了します。
        """
        if self.pi:
            self.pi.set_PWM_dutycycle(self.servo_pin, 0) # デューティサイクルを0に
            self.pi.set_mode(self.servo_pin, pigpio.INPUT) # ピンを入力モードに戻して解放
            print("PWM信号を停止し、ピンを解放しました。")

    def cleanup(self):
        """
        サーボ制御に関連するリソースをクリーンアップします。
        pigpioインスタンスはメインスクリプトで一括して行われるため、ここでは呼び出しません。
        """
        self.stop_pwm()
        print("ServoController: クリーンアップ完了。")
