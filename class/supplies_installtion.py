import RPi.GPIO as GPIO # PWMを使うためにRPi.GPIOは必要
import time

class ServoController:
    """
    RPi.GPIOのPWM機能を使用してサーボモーターを制御するクラス。
    メインスクリプトからpigpioインスタンスを受け取りますが、
    GPIOピンの直接設定はRPi.GPIOのPWM機能に任せます。
    """

    def __init__(self, pi_instance, servo_pin=13, pwm_frequency=50):
        """
        ServoControllerのコンストラクタです。

        Args:
            pi_instance (pigpio.pi): 既に初期化されたpigpioのインスタンス。
                                     (RPi.GPIOのPWM機能を使うため、このクラス内では直接使用しませんが、
                                      今後の拡張性や整合性のため引数として受け取ります。)
            servo_pin (int): サーボモーターが接続されているGPIOピンの番号 (BCMモード)。
            pwm_frequency (int): PWM信号の周波数 (Hz)。標準的なサーボは50Hzを使用します。
        """
        self.pi = pi_instance # pigpioインスタンスを受け取るが、このクラスでは直接使用しない
                               # RPi.GPIO.PWMが内部でピンを管理するため

        self.servo_pin = servo_pin
        self.pwm_frequency = pwm_frequency

        # RPi.GPIOのPWMを初期化します。
        # メインスクリプトで GPIO.setmode(GPIO.BCM) が一度だけ呼ばれていることを前提とします。
        # GPIO.PWM() は、内部で必要な GPIO.setup() を行いますので、ここでは明示的に呼び出しません。
        self.pwm = GPIO.PWM(self.servo_pin, self.pwm_frequency)
        self.pwm.start(0) # 初期デューティサイクルを0に設定してPWMを開始
        print(f"GPIO{self.servo_pin} でサーボを初期化しました (周波数: {self.pwm_frequency}Hz)。")

    def set_duty_cycle(self, duty_cycle):
        """
        サーボモーターのデューティサイクルを設定します。

        Args:
            duty_cycle (float): 設定するデューティサイクル値 (0.0 から 100.0)。
                                一般的にサーボは2.5 (反時計回り最大) から 12.5 (時計回り最大) の範囲。
        """
        if not (0.0 <= duty_cycle <= 100.0):
            print(f"警告: 不正なデューティサイクル値 ({duty_cycle}) です。0.0から100.0の範囲で指定してください。")
            return
        self.pwm.ChangeDutyCycle(duty_cycle)
        time.sleep(0.5) # サーボが位置に到達するのを待つ
        print(f"デューティサイクルを {duty_cycle:.1f}% に設定しました。")

    def gradually_change_duty_cycle(self, start_duty, end_duty, steps=50, delay_per_step=0.05):
        """
        サーボのデューティサイクルを`start_duty`から`end_duty`まで徐々に変化させます。
        これにより、滑らかな動きを実現できます。

        Args:
            start_duty (float): 変化開始時のデューティサイクル。
            end_duty (float): 変化終了時のデューティサイクル。
            steps (int): 変化を分割するステップ数。
            delay_per_step (float): 各ステップ間の遅延時間 (秒)。
        """
        print(f"デューティサイクルを {start_duty:.1f}% から {end_duty:.1f}% へ徐々に変化させます。")
        for i in range(steps + 1):
            current_duty = start_duty + (end_duty - start_duty) * i / steps
            self.pwm.ChangeDutyCycle(current_duty)
            time.sleep(delay_per_step)
        print(f"デューティサイクル変化完了。最終値: {end_duty:.1f}%")

    def stop_pwm(self):
        """
        PWM信号の出力を停止し、サーボへの制御を終了します。
        プログラム終了前に必ず呼び出してください。
        """
        if self.pwm:
            self.pwm.stop()
            print("PWM信号を停止しました。")

    def cleanup(self):
        """
        サーボ制御に関連するリソースをクリーンアップします。
        GPIO.cleanup()はメインスクリプトで一括して行われるため、ここでは呼び出しません。
        """
        self.stop_pwm()
        print("ServoController: クリーンアップ完了。")
