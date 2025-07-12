import RPi.GPIO as GPIO
import time
import pigpio
import board # Adafruit CircuitPython I2C (BNO055用)
import busio # Adafruit CircuitPython I2C (BNO055用)
import threading
import smbus # BME280用
from motor import MotorDriver
from BNO055 import BNO055
import following
from Flag_Detector2 import FlagDetector
from release import RoverReleaseDetector 
from land import RoverLandingDetector 
from GPS_datalink import GpsIm920Communicator
from excellent_gps import RoverGPSNavigator
from Flagseeker import FlagSeeker
from supplies_installtion import ServoController
from Goal_Detective_Noshiro import RedConeNavigator
from picamera2 import Picamera2

import cv2
import numpy as np
import math
import sys
import os

# --- グローバル定数設定 ---
# GPS受信ピン (pigpioソフトUART)
GPS_RX_PIN = 17
GPS_BAUD_RATE = 9600

# モータードライバピン設定
MOTOR_PINS = {
    'PWMA': 12, 'AIN1': 23, 'AIN2': 18,
    'PWMB': 19, 'BIN1': 16, 'BIN2': 26,
    'STBY': 21
}

# BNO055 IMU設定
BNO055_I2C_ADDRESS = 0x28

# BME280 気圧センサー設定
BME280_I2C_BUS = 1
BME280_ADDRESS = 0x76

# IM920 無線通信設定
IM920_PORT = '/dev/serial0'
IM920_BAUD = 19200
IM920_WIRELESS_CTRL_PIN = 22 # ワイヤレスグラウンド制御用GPIO

# サーボモーター設定 (放出・物資設置アクション用)
SERVO_PIN_ACTION = 13 # 放出と物資設置で同じサーボを使う想定
SERVO_PWM_FREQUENCY = 50

# カメラ設定
CAMERA_RESOLUTION = (640, 480)

# --- ミッションステージのパラメータ ---
# 放出判定ステージ (EjectionDetectorのデフォルト設定を使用)
EJECTION_PRESSURE_CHANGE_THRESHOLD = 0.3 # hPa (初期気圧からの変化量)
EJECTION_ACC_Z_THRESHOLD_ABS = 4.0 # m/s^2 (Z軸加速度絶対値)
EJECTION_CONSECUTIVE_CHECKS = 3
EJECTION_TIMEOUT_S = 60 # 秒

# 着地安定性判定ステージ (LandingStabilityDetectorのデフォルト設定を使用)
LANDING_STABILITY_PRESSURE_CHANGE_THRESHOLD = 0.1 # hPa (直前の気圧からの変化量)
LANDING_STABILITY_ACC_THRESHOLD_ABS = 0.5 # m/s^2 (各軸の加速度絶対値)
LANDING_STABILITY_GYRO_THRESHOLD_ABS = 0.5 # °/s (各軸の角速度絶対値)
LANDING_STABILITY_CONSECUTIVE_CHECKS = 3
LANDING_STABILITY_TIMEOUT_S = 120 # 秒

# パラシュート回避ステージ
PARACHUTE_AVOID_GOAL = [35.9248066, 139.9112360] # 回避後の進路目標 (例: GPS航行の初期地点など)
PARACHUTE_AVOID_DISTANCE_M = 10.0 # この距離を進んで回避する

# フラッグまでGPS誘導ステージ
FLAG_GPS_GOAL_LOCATION = [35.9186248, 139.9112360] # フラッグ付近のGPS目標地点
FLAG_GPS_THRESHOLD_M = 5.0 # ゴールとみなす距離 (メートル)
FLAG_GPS_ANGLE_ADJUST_THRESHOLD_DEG = 15.0 # 角度誤差許容範囲 (度)
FLAG_GPS_TURN_SPEED = 45 # 回頭速度
FLAG_GPS_MOVE_SPEED = 80 # 前進速度
FLAG_GPS_MOVE_DURATION_S = 1.5 # 一回の前進時間 (秒)

# フラッグ探索・追跡ステージ
FLAG_TARGET_SHAPES = ["三角形", "長方形"] # 探索するフラッグの形状リスト
FLAG_AREA_THRESHOLD_PERCENT = 20.0 # 接近完了とみなす画面占有率（パーセント）

# 物資設置ステージ (サーボ動作のデューティサイクル例)
SUPPLIES_INSTALL_DUTY_CYCLE = 4.0 # 物資を放出するサーボのデューティサイクル
SUPPLIES_RETURN_DUTY_CYCLE = 7.5 # 物資設置後のサーボの初期位置デューティサイクル

# ゴールまでGPS誘導ステージ
GOAL_GPS_LOCATION = [35.9185000, 139.9110000] # 最終的なゴール地点
GOAL_GPS_THRESHOLD_M = 1.0 # 最終ゴールとみなす距離 (メートル)
GOAL_GPS_ANGLE_ADJUST_THRESHOLD_DEG = 10.0 # 角度誤差許容範囲 (度)
GOAL_GPS_TURN_SPEED = 40 # 回頭速度
GOAL_GPS_MOVE_SPEED = 70 # 前進速度
GOAL_GPS_MOVE_DURATION_S = 1.0 # 一回の前進時間 (秒)

# ゴール検知ステージ (赤コーン追跡がゴール検知を兼ねる)
RED_CONE_GOAL_PERCENTAGE = 90 # 赤コーンに到達したとみなす画面占有率
RED_CONE_LOST_MAX_COUNT = 5 # コーンを見失う許容回数

# --- グローバル変数 (インスタンスとスレッド) ---
pi_instance = None
bno_sensor_main = None
i2c_bus_main = None 
motor_driver = None
picam2_instance = None
gps_im920_comm = None
gps_comm_thread = None
ejection_detector = None # 放出判定用
landing_stability_detector = None # 着地安定性判定用
gps_navigator = None
flag_seeker = None
servo_controller_action = None
red_cone_navigator = None

# --- ヘルパー関数 ---
def wait_for_bno055_calibration(bno_sensor):
    """
    BNO055のキャリブレーションを待機します。
    ここでは、メインで初期化されたBNOセンサーを対象とします。
    """
    print("⚙️ 主制御用BNO055キャリブレーション待機中...")
    if not bno_sensor.begin():
        print("🔴 主制御用BNO055センサーの初期化に失敗しました。")
        raise IOError("Main BNO055 sensor initialization failed.")
    
    bno_sensor.setExternalCrystalUse(True)
    bno_sensor.setMode(BNO055.OPERATION_MODE_NDOF)

    calibration_start_time = time.time()
    while True:
        sys_cal, gyro_cal, accel_cal, mag_cal = bno_sensor.getCalibration()
        print(f"Calib → Sys:{sys_cal}, Gyro:{gyro_cal}, Acc:{accel_cal}, Mag:{mag_cal} ", end='\r')
        if gyro_cal == 3: # 全てレベル3を待機
            print("\n✅ 主制御用BNO055ジャイロセンサーキャリブレーション完了！")
            break
        time.sleep(0.5)
    print(f"キャリブレーションにかかった時間: {time.time() - calibration_start_time:.1f}秒\n")


def cleanup_all_resources():
    """
    プログラム終了時に使用した全てのハードウェアリソースを解放します。
    """
    print("\n--- 全てのシステムをクリーンアップしています ---")
    global pi_instance, bno_sensor_main, i2c_bus_main, motor_driver, picam2_instance, \
           gps_im920_comm, gps_comm_thread, ejection_detector, landing_stability_detector, \
           gps_navigator, flag_seeker, servo_controller_action, red_cone_navigator

    # GPS通信スレッドを停止し、終了を待つ
    if gps_im920_comm:
        gps_im920_comm.stop()
        if gps_comm_thread and gps_comm_thread.is_alive():
            print("GPS通信スレッドの終了を待機中...")
            gps_comm_thread.join(timeout=10) # 10秒待機
            if gps_comm_thread.is_alive():
                print("警告: GPS通信スレッドがタイムアウト内に終了しませんでした。強制終了します。")
    
    # 個々の機能クラスのクリーンアップ (独自に初期化したリソースがあれば)
    if servo_controller_action:
        servo_controller_action.cleanup()
    if gps_im920_comm:
        gps_im920_comm.cleanup() # IM920の通信リソースをクリーンアップ (ワイヤレスグラウンドOFFも含む)

    # 共有リソースのクリーンアップ
    if picam2_instance:
        picam2_instance.close()
        print("カメラを閉じました。")
    if motor_driver:
        motor_driver.cleanup()
    if bno_sensor_main:
        # BNO055ライブラリには明示的なクローズがないことが多い
        pass
    if i2c_bus_main:
        # SMBusは明示的なクローズメソッドがないが、PythonのGCが管理する
        pass
    if pi_instance and pi_instance.connected:
        pi_instance.stop()
        print("pigpioデーモンとの接続を切断しました。")
    GPIO.cleanup()
    print("✅ 全てのシステムクリーンアップ完了。")
    print("\n=== ローバーミッションシステムを終了します ===")

# --- メインミッション実行ブロック ---
if __name__ == "__main__":
    try:
        print("🚀 ローバーミッション制御スクリプトを開始しています...")

        # --- 共通リソースの初期化 ---
        # pigpioデーモンへの接続
        pi_instance = pigpio.pi()
        if not pi_instance.connected:
            print("🔴 pigpioデーモンに接続できません。'sudo pigpiod'を実行してください。")
            sys.exit(1)
        print("✅ pigpioデーmonに接続しました。")

        # BNO055センサーの初期化 (他のフェーズ用。RoverReleaseDetector/RoverLandingDetectorは独自に初期化する)
        bno_sensor_main = BNO055(address=BNO055_I2C_ADDRESS)
        # BNOセンサーのbegin()はwait_for_bno055_calibration()で呼ばれる
        print("✅ BNO055センサーインスタンス作成 (後続フェーズ用)。")

        # BME280 気圧センサー用のI2Cバス初期化 (RoverReleaseDetector/RoverLandingDetectorは内部で直接I2Cバスを初期化するため、この変数は利用されない)
        # i2c_bus_main = smbus.SMBus(BME280_I2C_BUS) # この行も不要なのでコメントアウトまたは削除
        # print(f"✅ BME280 I2Cバス (バス{BME280_I2C_BUS}) 初期化完了 (後続フェーズ用)。")

        # MotorDriverの初期化
        motor_driver = MotorDriver(
            PWMA=MOTOR_PINS['PWMA'], AIN1=MOTOR_PINS['AIN1'], AIN2=MOTOR_PINS['AIN2'],
            PWMB=MOTOR_PINS['PWMB'], BIN1=MOTOR_PINS['BIN1'], BIN2=MOTOR_PINS['BIN2'],
            STBY=MOTOR_PINS['STBY']
        )
        print("✅ モータードライバー初期化完了。")

        # Picamera2の初期化
        picam2_instance = Picamera2()
        config = picam2_instance.create_still_configuration(
            main={"size": CAMERA_RESOLUTION}
        )
        picam2_instance.configure(config)
        picam2_instance.start()
        time.sleep(1)
        print(f"✅ カメラ初期化完了。解像度: {CAMERA_RESOLUTION[0]}x{CAMERA_RESOLUTION[1]}")

        # --- 各機能クラスのインスタンス化 ---
        # 1. 放出判定（RoverReleaseDetector）：**オリジナルのままのコンストラクタを使用**
        # RoverReleaseDetectorは自身の内部でBNO055とSMBusを初期化します。
        ejection_detector = RoverReleaseDetector(
            pressure_change_threshold=EJECTION_PRESSURE_CHANGE_THRESHOLD,
            acc_z_threshold_abs=EJECTION_ACC_Z_THRESHOLD_ABS,
            consecutive_checks=EJECTION_CONSECUTIVE_CHECKS,
            timeout=EJECTION_TIMEOUT_S
        )
        print("✅ RoverReleaseDetector (放出判定用) インスタンス作成。")

        # 2. 着地安定性判定（RoverLandingDetector）：**オリジナルのままのコンストラクタを使用**
        # RoverLandingDetectorは自身の内部でBNO055とSMBusを初期化します。
        landing_stability_detector = RoverLandingDetector(
            pressure_change_threshold=LANDING_STABILITY_PRESSURE_CHANGE_THRESHOLD,
            acc_threshold_abs=LANDING_STABILITY_ACC_THRESHOLD_ABS,
            gyro_threshold_abs=LANDING_STABILITY_GYRO_THRESHOLD_ABS,
            consecutive_checks=LANDING_STABILITY_CONSECUTIVE_CHECKS,
            timeout=LANDING_STABILITY_TIMEOUT_S,
            calibrate_bno055=False # メインでBNOキャリブレーションを行うため、ここではスキップ
        )
        print("✅ RoverLandingDetector (着地安定性判定用) インスタンス作成。")

        # GpsIm920Communicator は、ここではインスタンス化のみで、後で activate() を呼ぶ
        gps_im920_comm = GpsIm920Communicator(
            pi_instance=pi_instance,
            rx_pin=GPS_RX_PIN,
            gps_baud=GPS_BAUD_RATE,
            wireless_ctrl_pin=IM920_WIRELESS_CTRL_PIN,
            im920_port=IM920_PORT,
            im920_baud=IM920_BAUD,
            target_node_id=0x0003
        )
        # GPS通信用のスレッドを準備 (activated後にstartされる)
        gps_comm_thread = threading.Thread(target=gps_im920_comm.start_communication_loop, daemon=True)
        print("✅ GpsIm920Communicator インスタンスとスレッド準備完了。")

        # RoverGPSNavigator
        gps_navigator = RoverGPSNavigator(
            driver_instance=motor_driver,
            bno_instance=bno_sensor_main, # RoverMissionControllerで作成したBNOインスタンスを渡す
            pi_instance=pi_instance,
            rx_pin=GPS_RX_PIN,
            gps_baud=GPS_BAUD_RATE,
            goal_location=[0.0, 0.0], # 初期値はダミー
            goal_threshold_m=FLAG_GPS_THRESHOLD_M, # 後で再設定
            angle_adjust_threshold_deg=FLAG_GPS_ANGLE_ADJUST_THRESHOLD_DEG, # 後で再設定
            turn_speed=FLAG_GPS_TURN_SPEED, # 後で再設定
            move_speed=FLAG_GPS_MOVE_SPEED, # 後で再設定
            move_duration_s=FLAG_GPS_MOVE_DURATION_S # 後で再設定
        )
        print("✅ RoverGPSNavigator インスタンス作成。")

        # FlagSeeker
        flag_seeker = FlagSeeker(
            driver_instance=motor_driver,
            bno_instance=bno_sensor_main,
            picam2_instance=picam2_instance,
            target_shapes=FLAG_TARGET_SHAPES,
            area_threshold_percent=FLAG_AREA_THRESHOLD_PERCENT
        )
        print("✅ FlagSeeker インスタンス作成。")
        
        # ServoController
        servo_controller_action = ServoController(
            servo_pin=SERVO_PIN_ACTION,
            pwm_frequency=SERVO_PWM_FREQUENCY
        )
        print("✅ ServoController (アクション用) インスタンス作成。")

        # RedConeNavigator
        red_cone_navigator = RedConeNavigator(
            driver_instance=motor_driver,
            bno_instance=bno_sensor_main,
            picam2_instance=picam2_instance,
            cone_lost_max_count=RED_CONE_LOST_MAX_COUNT,
            goal_percentage_threshold=RED_CONE_GOAL_PERCENTAGE
        )
        print("✅ RedConeNavigator インスタンス作成。")

        print("✅ 全てのローバーコンポーネントの初期化完了。")

    except Exception as e:
        print(f"\n🚨 ミッション中に予期せぬエラーが発生しました: {e}")
    finally:
        cleanup_all_resources()

        # --- メインミッション開始 ---

        # === フェーズ1: 放出判定 ===
        print("\n--- フェーズ1: 放出判定（気圧上昇と加速度上昇の検出）を開始します ---")
        is_ejected = ejection_detector.check_landing()
        if is_ejected:
            print("🎉 放出を確認しました！次のフェーズへ移行します。")
        else:
            print("⚠️ 放出が確認できませんでした。ミッションを続行します。")
            # 必要に応じてここでミッション中止 (sys.exit(1))

        # === GPS通信開始 (ミッション終了まで継続) ===
        print("\n--- GPSデータリンクを初期化し、並行して開始します ---")
        gps_im920_comm.activate() # GPSソフトUARTオープン、IM920シリアルオープン、ワイヤレスグラウンドON
        gps_comm_thread.start()
        print("✅ GPSデータリンクスレッドがバックグラウンドで起動しました。")
        time.sleep(2) # スレッドが完全に起動するまで少し待機

        # === フェーズ2: 着地判定 ===
        print("\n--- フェーズ2: 着地判定（気圧・加速度・角速度の安定性検出）を開始します ---")
        # 着地安定性判定器を使用
        is_landed_stable = landing_stability_detector.check_landing()
        if is_landed_stable:
            print("🎉 ローバーの着地と安定を確認しました！次のフェーズへ移行します。")
        else:
            print("⚠️ ローバーの着地と安定が確認できませんでした。ミッションを続行します。")
            # 必要に応じてここでミッション中止 (sys.exit(1))

        # === フェーズ3: パラシュート回避 ===
        print("\n--- フェーズ3: パラシュート回避を開始します ---")
        print("パラシュート回避のため、少し前進し、周辺を確認します...")
        motor_driver.move_forward(motor_driver.MAX_SPEED * 0.5)
        time.sleep(5)
        motor_driver.motor_stop_brake()
        print("✅ パラシュート回避行動完了。")

        # === フェーズ4: フラッグまでGPS誘導 ===
        gps_navigator.set_goal_location(FLAG_GPS_GOAL_LOCATION)
        gps_navigator.set_goal_threshold(FLAG_GPS_THRESHOLD_M)
        gps_navigator.set_angle_adjust_threshold(FLAG_GPS_ANGLE_ADJUST_THRESHOLD_DEG)
        gps_navigator.set_turn_speed(FLAG_GPS_TURN_SPEED)
        gps_navigator.set_move_speed(FLAG_GPS_MOVE_SPEED)
        gps_navigator.set_move_duration(FLAG_GPS_MOVE_DURATION_S)
        
        print(f"\n--- フェーズ4: フラッグまでGPS誘導 ({FLAG_GPS_GOAL_LOCATION}) を開始します ---")
        gps_navigator.navigate_to_goal()
        print("🎉 フラッグ付近へのGPS誘導が完了しました！")

        # === フェーズ5: フラッグ検知 & 誘導 ===
        print("\n--- フェーズ5: フラッグの検知と誘導を開始します ---")
        flag_seeker.seek_and_approach()
        print("🎉 フラッグの検知と誘導が完了しました！")

        # === フェーズ6: 物資設置 ===
        print("\n--- フェーズ6: 物資設置アクションを実行します ---")
        print("サーボを物資設置位置に移動させます。")
        servo_controller_action.set_duty_cycle(SUPPLIES_INSTALL_DUTY_CYCLE)
        time.sleep(5)
        servo_controller_action.set_duty_cycle(SUPPLIES_RETURN_DUTY_CYCLE)
        time.sleep(1)
        print("✅ 物資設置アクション完了。")

        # === フェーズ7: ゴールまでGPS誘導 ===
        gps_navigator.set_goal_location(GOAL_GPS_LOCATION)
        gps_navigator.set_goal_threshold(GOAL_GPS_THRESHOLD_M)
        gps_navigator.set_angle_adjust_threshold(GOAL_GPS_ANGLE_ADJUST_THRESHOLD_DEG)
        gps_navigator.set_turn_speed(GOAL_GPS_TURN_SPEED)
        gps_navigator.set_move_speed(GOAL_GPS_MOVE_SPEED)
        gps_navigator.set_move_duration(GOAL_GPS_MOVE_DURATION_S)
        
        print(f"\n--- フェーズ7: 最終ゴール地点 ({GOAL_GPS_LOCATION}) までGPS誘導を開始します ---")
        gps_navigator.navigate_to_goal()
        print("🎉 最終ゴール地点へのGPS誘導が完了しました！")

        # === フェーズ8: ゴール検知（赤コーン追跡） ===
        print("\n--- フェーズ8: ゴール検知（赤コーン追跡）を開始します ---")
        red_cone_navigator.navigate_to_cone()
        print("🎉 赤コーンへの追跡（最終ゴール検知）が完了しました！")

        print("\n=== ミッション完了！ ===\nローバーの全タスクが完了しました。")

    except Exception as e:
        print(f"\n🚨 ミッション中に予期せぬエラーが発生しました: {e}")
    finally:
        cleanup_all_resources()
