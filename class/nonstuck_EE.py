import RPi.GPIO as GPIO
import time
import pigpio
import board
import busio
import threading

# 外部クラスのインポート
from motor import MotorDriver
from BNO055 import BNO055
import following
from Flag_Detector2 import FlagDetector
from rover_landing_detector import RoverLandingDetector
from gps_im920_communicator import GpsIm920Communicator # 修正版を使用
from rover_gps_navigator import RoverGPSNavigator
from flag_seeker import FlagSeeker
from servo_controller import ServoController
from red_cone_navigator import RedConeNavigator
from picamera2 import Picamera2

import cv2
import numpy as np
import math
import sys
import os

class RoverMissionController:
    # ... (クラス定数は変更なし)

    def __init__(self):
        # ... (共通リソースの初期化は変更なし)
        # pigpio, BNO055, MotorDriver, Picamera2 の初期化

        # --- 各機能クラスのインスタンス化 (共有リソースを引数で渡す) ---
        self.landing_detector = RoverLandingDetector(
            # driver_instance=self.motor_driver, # 着地検出器がモーターを直接制御しないなら不要
            bno_sensor=self.bno_sensor,
            pressure_change_threshold=self.LANDING_PRESSURE_CHANGE_THRESHOLD,
            acc_z_threshold_abs=self.LANDING_ACC_Z_THRESHOLD_ABS,
            consecutive_checks=self.LANDING_CONSECUTIVE_CHECKS,
            timeout=self.LANDING_TIMEOUT_S
        )
        print("✅ RoverLandingDetector インスタンス作成。")

        # GpsIm920Communicator は、ここではインスタンス化のみで、後で activate() を呼ぶ
        self.gps_im920_comm = GpsIm920Communicator(
            pi_instance=self.pi,
            rx_pin=self.GPS_RX_PIN,
            gps_baud=self.GPS_BAUD_RATE,
            wireless_ctrl_pin=self.IM920_WIRELESS_CTRL_PIN,
            im920_port=self.IM920_PORT,
            im920_baud=self.IM920_BAUD,
            target_node_id=0x0003
        )
        # GPS通信用のスレッドを準備 (activated後にstartされる)
        self.gps_comm_thread = threading.Thread(target=self.gps_im920_comm.start_communication_loop, daemon=True)
        print("✅ GpsIm920Communicator インスタンスとスレッド準備完了。")

        self.gps_navigator = RoverGPSNavigator(
            driver_instance=self.motor_driver,
            bno_instance=self.bno_sensor,
            pi_instance=self.pi,
            rx_pin=self.GPS_RX_PIN,
            gps_baud=self.GPS_BAUD_RATE,
            goal_location=[0.0, 0.0], # 初期値はダミー
            goal_threshold_m=self.EXCELLENT_GPS_THRESHOLD_M,
            angle_adjust_threshold_deg=self.EXCELLENT_GPS_ANGLE_ADJUST_THRESHOLD_DEG,
            turn_speed=self.EXCELLENT_GPS_TURN_SPEED,
            move_speed=self.EXCELLENT_GPS_MOVE_SPEED,
            move_duration_s=self.EXCELLENT_GPS_MOVE_DURATION_S
        )
        print("✅ RoverGPSNavigator インスタンス作成。")

        self.flag_seeker = FlagSeeker(
            driver_instance=self.motor_driver,
            bno_instance=self.bno_sensor,
            picam2_instance=self.picam2,
            target_shapes=self.FLAG_TARGET_SHAPES,
            area_threshold_percent=self.FLAG_AREA_THRESHOLD_PERCENT
        )
        print("✅ FlagSeeker インスタンス作成。")
        
        self.servo_controller_action = ServoController(
            servo_pin=self.SERVO_PIN_ACTION,
            pwm_frequency=self.SERVO_PWM_FREQUENCY
        )
        print("✅ ServoController (アクション用) インスタンス作成。")

        self.red_cone_navigator = RedConeNavigator(
            driver_instance=self.motor_driver,
            bno_instance=self.bno_sensor,
            picam2_instance=self.picam2,
            cone_lost_max_count=self.RED_CONE_LOST_MAX_COUNT,
            goal_percentage_threshold=self.RED_CONE_GOAL_PERCENTAGE
        )
        print("✅ RedConeNavigator インスタンス作成。")

        print("✅ ローバーミッションコントローラー初期化完了。")

    def _wait_for_bno055_calibration(self):
        # ... (変更なし)

    def start_mission(self):
        print("\n--- ミッション開始 ---")
        self._wait_for_bno055_calibration()

        try:
            # === フェーズ1: 放出アクション ===
            print("\n=== フェーズ1: 放出アクションを実行します ===")
            print("サーボを放出位置に移動させます。")
            self.servo_controller_action.set_duty_cycle(self.SUPPLIES_INSTALL_DUTY_CYCLE)
            time.sleep(3)
            self.servo_controller_action.set_duty_cycle(self.SUPPLIES_RETURN_DUTY_CYCLE)
            time.sleep(1)
            print("✅ 放出アクション完了。")

            # === GPSデータリンクの初期化と開始 (並行処理) ===
            # 放出アクション完了後、ここで初めてGPSハードウェアを初期化し、通信ループを開始
            print("\n=== GPSデータリンクを初期化し、並行して開始します ===")
            self.gps_im920_comm.activate() # GPSソフトUARTオープン、IM920シリアルオープン、ワイヤレスグラウンドON
            self.gps_comm_thread.start()
            print("✅ GPSデータリンクスレッドがバックグラウンドで起動しました。")
            time.sleep(2) # スレッドが完全に起動するまで少し待機

            # === フェーズ2: 着地判定 ===
            print("\n=== フェーズ2: 着地判定を開始します ===")
            if self.landing_detector.check_landing():
                print("🎉 着地を確認しました！次のフェーズへ移行します。")
            else:
                print("⚠️ 着地が確認できませんでした。ミッションを続行します。")

            # === フェーズ3: パラシュート回避 ===
            print("\n=== フェーズ3: パラシュート回避を開始します ===")
            print("パラシュート回避のため、少し前進し、周辺を確認します...")
            self.motor_driver.move_forward(self.motor_driver.MAX_SPEED * 0.5)
            time.sleep(5)
            self.motor_driver.motor_stop_brake()
            print("✅ パラシュート回避行動完了。")

            # === フェーズ4: Excellent GPS (精密GPS航行) ===
            self.gps_navigator.set_goal_location(self.EXCELLENT_GPS_GOAL_LOCATION_1)
            self.gps_navigator.set_goal_threshold(self.EXCELLENT_GPS_THRESHOLD_M)
            
            print(f"\n=== フェーズ4: 最初の精密GPS目標地点 ({self.EXCELLENT_GPS_GOAL_LOCATION_1}) への航行を開始します ===")
            self.gps_navigator.navigate_to_goal()
            print("🎉 最初の精密GPS目標地点への航行が完了しました！")

            # === フェーズ5: フラッグシーカー (FlagSeeker) ===
            print("\n=== フェーズ5: フラッグの探索と接近を開始します ===")
            self.flag_seeker.seek_and_approach()
            print("🎉 フラッグの探索と接近が完了しました！")

            # === フェーズ6: 物資設置 ===
            print("\n=== フェーズ6: 物資設置アクションを実行します ===")
            print("サーボを物資設置位置に移動させます。")
            self.servo_controller_action.set_duty_cycle(self.SUPPLIES_INSTALL_DUTY_CYCLE)
            time.sleep(5)
            self.servo_controller_action.set_duty_cycle(self.SUPPLIES_RETURN_DUTY_CYCLE)
            time.sleep(1)
            print("✅ 物資設置アクション完了。")

            # === フェーズ7: Goal Excellent GPS (最終目標精密GPS航行) ===
            self.gps_navigator.set_goal_location(self.GOAL_EXCELLENT_GPS_LOCATION)
            self.gps_navigator.set_goal_threshold(self.GOAL_EXCELLENT_GPS_THRESHOLD_M)
            
            print(f"\n=== フェーズ7: 最終ゴール地点 ({self.GOAL_EXCELLENT_GPS_LOCATION}) への精密GPS航行を開始します ===")
            self.gps_navigator.navigate_to_goal()
            print("🎉 最終ゴール地点への航行が完了しました！")

            # === フェーズ8: 赤コーン追跡 ===
            print("\n=== フェーズ8: 赤コーンの追跡を開始します ===")
            self.red_cone_navigator.navigate_to_cone()
            print("🎉 赤コーンへの追跡が完了しました！")

            print("\n=== ミッション完了！ ===\nローバーの全タスクが完了しました。")

        except Exception as e:
            print(f"\n🚨 ミッション中に予期せぬエラーが発生しました: {e}")
        finally:
            self.cleanup()

    def cleanup(self):
        # ... (変更なし)
        # GPS通信スレッドの停止とクリーンアップ、他リソースのクリーンアップ
