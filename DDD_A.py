import cv2
import numpy as np
import time
import camera
import smbus
from picamera2 import Picamera2
import struct
import RPi.GPIO as GPIO
import math
import pigpio

# 作成ファイルのインポート
import fusing
import BME280
import following
from BNO055 import BNO055
from motor import MotorDriver
from Flag_Detector3 import FlagDetector

# ミッション部分
from C_RELEASE import RD
from C_Landing_Detective import LD
from C_PARACHUTE_AVOIDANCE import PA
from C_Flag_Navi import FN
from C_Servo import SM # このモジュールは使用されていないようです
from C_excellent_GPS import GPS
from C_GOAL_DETECTIVE_ARLISS import GDA

def set_servo_duty(duty):
    """サーボのデューティサイクルを設定し、少し待機する関数"""
    try:
        pwm.ChangeDutyCycle(duty)
        time.sleep(0.5)
    except Exception as e:
        print(f"Error setting servo duty cycle: {e}")

# BNO055の初期設定
bno = BNO055()
bno.begin()
time.sleep(1)
bno.setMode(BNO055.OPERATION_MODE_NDOF)
time.sleep(1)
bno.setExternalCrystalUse(True)

# BNO055のキャリブレーション待機
print("Waiting for BNO055 calibration...")
while True:
    sys, gyro, accel, mag = bno.getCalibration()
    print(f"gyro:{gyro}, mag:{mag}") # magのキャリブレーション状態も確認すると良いでしょう
    if gyro == 3 and mag == 3: # ジャイロと地磁気の両方が完全にキャリブレーションされたら終了
        print("BNO055のキャリブレーション終了")
        break
    time.sleep(0.1) # ポーリング間隔を短くして負荷を減らす

# GPIOとPWMオブジェクトの初期化（後で確実にクリーンアップするため、Noneで初期化）
pwm = None

# 各ミッションフェーズの実行
try:
    # --- RELEASEフェーズ ---
    try:
        print("\n--- Starting RELEASE phase ---")
        RELEASE = RD(bno)
        RELEASE.run()
        print("RELEASE phase completed.")
    except Exception as e:
        print(f"!!! Error during RELEASE phase: {e}")
        import traceback
        traceback.print_exc()
        # RELEASEでエラーが発生した場合の追加処理（例: 緊急着陸フェーズへ移行など）

    # --- LANDフェーズ ---
    try:
        print("\n--- Starting LANDING DETECTIVE phase ---")
        LAND = LD(bno)
        LAND.run()
        print("LANDING DETECTIVE phase completed.")
    except Exception as e:
        print(f"!!! Error during LANDING DETECTIVE phase: {e}")
        import traceback
        traceback.print_exc()

    # --- PARACHUTE AVOIDANCEフェーズ ---
    try:
        print("\n--- Starting PARACHUTE AVOIDANCE phase ---")
        AVOIDANCE = PA(bno, goal_location=[35.9175612, 139.9087922])
        AVOIDANCE.run()
        print("PARACHUTE AVOIDANCE phase completed.")
    except Exception as e:
        print(f"!!! Error during PARACHUTE AVOIDANCE phase: {e}")
        import traceback
        traceback.print_exc()

    # --- GPS (Start to Flag) フェーズ ---
    try:
        print("\n--- Starting GPS (Start to Flag) phase ---")
        GPS_StoF = GPS(bno, goal_location=[35.9175612, 139.9087922])
        GPS_StoF.run()
        print("GPS (Start to Flag) phase completed.")
    except Exception as e:
        print(f"!!! Error during GPS (Start to Flag) phase: {e}")
        import traceback
        traceback.print_exc()

    # --- FLAG NAVIフェーズ ---
    try:
        print("\n--- Starting FLAG NAVIGATION phase ---")
        # FIXME: flag_locationが空のリストです。適切な緯度・経度を設定してください。
        FLAG = FN(bno, flag_location=[35.9175612, 139.9087922]) # 仮の値を設定
        FLAG.run()
        print("FLAG NAVIGATION phase completed.")
    except Exception as e:
        print(f"!!! Error during FLAG NAVIGATION phase: {e}")
        import traceback
        traceback.print_exc()

    # --- サーボ操作フェーズ ---
    try:
        print("\n--- Starting SERVO OPERATION phase ---")
        SERVO_PIN = 13  # GPIO13を使用
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(SERVO_PIN, GPIO.OUT)
        pwm = GPIO.PWM(SERVO_PIN, 50) # PWMオブジェクトを初期化
        pwm.start(0)
        
        print("逆回転（速い）")
        set_servo_duty(4.0) # set_servo_duty関数を呼び出し
        time.sleep(7)
        print("停止（待機）")
        set_servo_duty(12.5) # set_servo_duty関数を呼び出し
        print("SERVO OPERATION phase completed.")
    except Exception as e:
        print(f"!!! Error during SERVO OPERATION phase: {e}")
        import traceback
        traceback.print_exc()
    finally:
        # サーボの停止とGPIOクリーンアップは、サーボ操作のfinallyブロックで実行
        if pwm is not None:
            pwm.stop()
        # GPIO.cleanup()は一番外側のfinallyで確実に実行するため、ここでは呼び出さない

    # --- GPS (Flag to Goal) フェーズ ---
    try:
        print("\n--- Starting GPS (Flag to Goal) phase ---")
        GPS_FtoG = GPS(bno, goal_location=[35.9243464, 139.9113269])
        GPS_FtoG.run()
        print("GPS (Flag to Goal) phase completed.")
    except Exception as e:
        print(f"!!! Error during GPS (Flag to Goal) phase: {e}")
        import traceback
        traceback.print_exc()

    # --- GOAL DETECTIVE ARLISSフェーズ ---
    try:
        print("\n--- Starting GOAL DETECTIVE ARLISS phase ---")
        GOAL = GDA(bno, 30)
        GOAL.run()
        print("GOAL DETECTIVE ARLISS phase completed.")
    except Exception as e:
        print(f"!!! Error during GOAL DETECTIVE ARLISS phase: {e}")
        import traceback
        traceback.print_exc()

    print("\nすべてのクラス呼び出しが完了しました。")

# 全体で予期せぬエラーが発生した場合のキャッチ
except Exception as e:
    print(f"\n!!! A critical unexpected error occurred in the main script: {e}")
    import traceback
    traceback.print_exc() # 詳細なエラー情報（スタックトレース）を出力

finally:
    # プログラムがどのように終了しても（正常終了またはエラー終了）、
    # ここに記述された処理は必ず実行されます。
    print("\n--- Cleaning up resources ---")
    try:
        if GPIO.getmode() is not None: # GPIOモードが設定されているか確認
            GPIO.cleanup()
            print("GPIO resources cleaned up.")
    except Exception as cleanup_e:
        print(f"!!! Error during GPIO cleanup: {cleanup_e}")
    
    print("Program terminated.")
