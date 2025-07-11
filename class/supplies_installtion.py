import RPi.GPIO as GPIO
import time

class ServoController:
    """
    RPi.GPIOとPWMを使用してサーボモーターを制御するクラス。
    主に特定のデューティサイクルを設定したり、
    デューティサイクルを徐々に変化させたりする機能を提供します。
    """

    def __init__(self, servo_pin=13, pwm_frequency=50):
        """
        ServoControllerのコンストラクタです。

        Args:
            servo_pin (int): サーボモーターが接続されているGPIOピンの番号 (BCMモード)。
            pwm_frequency (int): PWM信号の周波数 (Hz)。標準的なサーボは50Hzを使用します。
        """
        self.servo_pin = servo_pin
        self.pwm_frequency = pwm_frequency
        self.pwm = None # PWMオブジェクトは後で初期化

        # GPIOの初期設定
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.servo_pin, GPIO.OUT)

        # PWMの初期化
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
        # 短い遅延を入れてサーボが位置に到達するのを待つ
        time.sleep(0.5)
        print(f"デューティサイクルを {duty_cycle:.1f}% に設定しました。")

    def gradually_change_duty_cycle(self, start_duty, end_duty, steps=100, delay_per_step=0.1):
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
        for i in range(steps + 1): # start_dutyとend_dutyを含むようにsteps+1回ループ
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
        GPIOピンをクリーンアップし、全てのサーボ制御を終了します。
        プログラム終了時に呼び出す必要があります。
        """
        self.stop_pwm()
        GPIO.cleanup()
        print("GPIOをクリーンアップしました。")

---

### **クラス化のポイント**

1.  **`ServoController` クラス**:
    * サーボモーターの操作に関連するすべての機能（初期化、デューティサイクル設定、段階的変化、クリーンアップ）をカプセル化します。
    * **コンストラクタ `__init__`**:
        * サーボが接続されている **GPIOピン番号**と**PWM周波数**を引数で受け取るようにしました。これにより、複数のサーボを異なるピンや設定で制御することが簡単になります。
        * `RPi.GPIO`の初期設定とPWMの開始もここで行われます。
    * **`set_duty_cycle(self, duty_cycle)`**:
        * 特定のデューティサイクルを設定し、サーボがその位置に移動するのを待つためのシンプルなメソッドです。
        * デューティサイクルが有効な範囲内にあるかどうかの基本的なチェックを追加しました。
    * **`gradually_change_duty_cycle(self, start_duty, end_duty, steps, delay_per_step)`**:
        * 元の`changing_servo_reverse`関数をより汎用的にしました。`start_duty`から`end_duty`までを**指定されたステップ数**で徐々に変化させ、**各ステップ間の遅延**も調整できるようにしました。
        * 関数名も、より動作が分かりやすいように変更しています。
    * **`stop_pwm(self)`**:
        * PWM信号の出力を停止するためのメソッドです。これにより、サーボモーターがフリーな状態になります。
    * **`cleanup(self)`**:
        * `stop_pwm()`を呼び出した後、`GPIO.cleanup()`を実行してGPIOピンを適切に解放します。これにより、次回プログラムを実行する際にGPIO関連のエラーを防ぎます。

### **使い方**

このクラスを使うには、以下の手順に従います。

1.  **インスタンスの作成**:
    サーボが接続されているGPIOピン番号（例: `13`）とPWM周波数（通常 `50` Hz）を指定して、`ServoController`のインスタンスを作成します。
    ```python
    servo_control = ServoController(servo_pin=13, pwm_frequency=50)
    ```

2.  **サーボの制御**:
    * 特定のデューティサイクルを設定するには：
        ```python
        print("サーボを逆回転（速い）位置へ設定")
        servo_control.set_duty_cycle(4.0) # デューティサイクルを4.0%に設定
        time.sleep(10) # 10秒間その位置を維持
        ```
    * デューティサイクルを徐々に変化させるには：
        ```python
        print("デューティサイクルを7.5%から10%へ徐々に変化（正回転方向へ）")
        servo_control.gradually_change_duty_cycle(7.5, 10.0, steps=50, delay_per_step=0.05)
        time.sleep(3)

        print("デューティサイクルを10%から7.5%へ徐々に変化（停止方向へ）")
        servo_control.gradually_change_duty_cycle(10.0, 7.5, steps=50, delay_per_step=0.05)
        time.sleep(3)
        ```
    * 元のコードにあった`set_servo_duty(12.5)`は、例として徐々に変化させる方法の代わりに、直接最大値に設定する用途として残しています。

3.  **クリーンアップ**:
    プログラムの終了時や、サーボ制御が不要になった際には、必ず`cleanup()`メソッドを呼び出してリソースを解放してください。
    ```python
    servo_control.cleanup()
    ```

### **完全な実行例**

```python
import RPi.GPIO as GPIO
import time

class ServoController:
    """
    RPi.GPIOとPWMを使用してサーボモーターを制御するクラス。
    主に特定のデューティサイクルを設定したり、
    デューティサイクルを徐々に変化させたりする機能を提供します。
    """

    def __init__(self, servo_pin=13, pwm_frequency=50):
        """
        ServoControllerのコンストラクタです。

        Args:
            servo_pin (int): サーボモーターが接続されているGPIOピンの番号 (BCMモード)。
            pwm_frequency (int): PWM信号の周波数 (Hz)。標準的なサーボは50Hzを使用します。
        """
        self.servo_pin = servo_pin
        self.pwm_frequency = pwm_frequency
        self.pwm = None # PWMオブジェクトは後で初期化

        # GPIOの初期設定
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.servo_pin, GPIO.OUT)

        # PWMの初期化
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
        # 短い遅延を入れてサーボが位置に到達するのを待つ
        time.sleep(0.5)
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
        for i in range(steps + 1): # start_dutyとend_dutyを含むようにsteps+1回ループ
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
        GPIOピンをクリーンアップし、全てのサーボ制御を終了します。
        プログラム終了時に呼び出す必要があります。
        """
        self.stop_pwm()
        GPIO.cleanup()
        print("GPIOをクリーンアップしました。")

if __name__ == "__main__":
    # サーボコントローラーのインスタンスを作成
    servo_control = ServoController(servo_pin=13, pwm_frequency=50)

    try:
        # 例1: 特定のデューティサイクルに直接設定
        print("\n--- サーボ停止位置に設定 ---")
        servo_control.set_duty_cycle(7.5) # 7.5%は多くの場合、サーボのニュートラル位置
        time.sleep(2)

        # 例2: 逆回転（速い）に直接設定し、10秒維持
        print("\n--- サーボ逆回転（速い）に設定し10秒維持 ---")
        servo_control.set_duty_cycle(4.0) # 4.0%は反時計回り方向の回転
        time.sleep(10)

        # 例3: 徐々にデューティサイクルを変化させる
        print("\n--- 徐々に正回転（速い）へ変化 ---")
        servo_control.gradually_change_duty_cycle(4.0, 10.0, steps=50, delay_per_step=0.1)
        time.sleep(3)

        print("\n--- 徐々に停止位置へ変化 ---")
        servo_control.gradually_change_duty_cycle(10.0, 7.5, steps=50, delay_per_step=0.1)
        time.sleep(2)

        print("\n--- プログラム実行例終了 ---")

    except KeyboardInterrupt:
        print("\nユーザーによって中断されました。")
    except Exception as e:
        print(f"エラーが発生しました: {e}")
    finally:
        # プログラム終了前に必ずクリーンアップ
        servo_control.cleanup()
