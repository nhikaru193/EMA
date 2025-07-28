import time
import sys
import RPi.GPIO as GPIO
import board # BNO055の初期化に必要
import busio # BNO055の初期化に必要

# あなたのカスタムBNO055クラスとGDAクラスをインポートします
from BNO055 import BNO055 # BNO055.py が同じディレクトリにあることを確認してください
from C_GOAL_DETECTIVE_ARLISS import GDA # C_GOAL_DETECTIVE_ARLISS.py が同じディレクトリにあることを確認してください

def main():
    """
    自律走行ロボットのメイン実行関数。
    BNO055センサーを初期化し、GDAロボットインスタンスに渡して
    HAT_TRICKミッションを開始します。
    """
    bno_sensor = None
    rover = None
    bno_sensor_address = 0x28 # BNO055のI2Cアドレス

    try:
        print("--- ロボットシステムを初期化中 ---")

        # BNO055センサーの初期化
        try:
            bno_sensor = BNO055(address=bno_sensor_address)
            if not bno_sensor.begin():
                raise RuntimeError("BNO055センサーの初期化に失敗しました。プログラムを終了します。")
            
            # BNO055の動作モードを設定 (NDOFモードは方位、加速度、ジャイロ、磁力計を含む)
            bno_sensor.setMode(BNO055.OPERATION_MODE_NDOF)
            # 外部クリスタルを使用するように設定 (精度向上)
            bno_sensor.setExternalCrystalUse(True)
            time.sleep(1) # センサーが安定するのを待つ
            print("BNO055センサーが正常に初期化されました。")

        except Exception as e:
            print(f"BNO055センサーの初期化中にエラーが発生しました: {e}")
            print("センサー関連のエラーによりプログラムを終了します。")
            sys.exit(1) # センサーの初期化に失敗したら終了

        # GDAロボットクラスのインスタンス化
        # 初期化したBNO055センサーインスタンスをGDAに渡します
        rover = GDA(
            motor_pwma_pin=12, motor_ain1_pin=23, motor_ain2_pin=18,
            motor_pwmb_pin=19, motor_bin1_pin=16, motor_bin2_pin=26,
            motor_stby_pin=21, 
            bno_sensor_instance=bno_sensor, # ここでBNO055インスタンスを渡す
            rx_pin=17
        )
        print("GDAロボットインスタンスが作成されました。")

        # ロボットのメインミッションを開始
        print("\n=== HAT_TRICKミッションを開始します！ ===")
        rover.HAT_TRICK()

    except KeyboardInterrupt:
        print("\nCtrl+C が押されました。プログラムを安全に終了します。")
    except Exception as e:
        print(f"メインプログラムで予期せぬエラーが発生しました: {e}")
    finally:
        # roverインスタンスが作成されている場合にのみcleanupを呼び出す
        if rover:
            rover.cleanup()
        else:
            # roverが作成されていない場合は、GPIOのみクリーンアップを試みる
            print("GDAインスタンスが作成されなかったため、MotorDriverのクリーンアップはスキップします。")
            GPIO.cleanup()
            print("GPIO cleaned up.")
            # pigpioは直接参照できないため、デーモンは手動で停止する必要がある場合があります。
        print("--- プログラムを終了しました。 ---")

if __name__ == "__main__":
    main()
