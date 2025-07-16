import time
import smbus
import struct
import cv2
import math
import numpy as np
from picamera2 import Picamera2
from BNO055 import BNO055
# motor.py から MotorDriver クラスをインポート
from motor import MotorDriver 
from Flag_Detector3 import FlagDetector
import RPi.GPIO as GPIO

class FN:
    # --- クラスの初期化メソッド ---
    def __init__(self, bno: BNO055):

        # --- 設定値 ---
        self.TARGET_SHAPES = ["三角形", "長方形"]
        self.AREA_THRESHOLD_PERCENT = 23.0

        # === P制御関連のパラメータ ===
        self.KP_TURN = 0.35
        self.KP_APPROACH = 2.0
        self.CENTER_TOLERANCE_PX = 20
        self.MAX_TURN_SPEED = 70
        self.MAX_APPROACH_SPEED = 80
        self.MIN_APPROACH_SPEED = 45

        # --- 初期化処理 ---
        self.detector = FlagDetector()
        # MotorDriverクラスを初期化
        self.driver = MotorDriver(
            PWMA=12, AIN1=23, AIN2=18,
            PWMB=19, BIN1=16, BIN2=26,
            STBY=21
        )
        self.screen_width = self.detector.width
        self.screen_height = self.detector.height
        self.screen_area = self.screen_width * self.screen_height
        self.screen_center_x = self.screen_width / 2
        
        # === BNO055 初期化 ===
        self.bno = bno

    def find_target_flag(self, detected_data, target_name):
        """検出データから指定された図形(target_name)のフラッグを探して返す"""
        for flag_data in detected_data:
            if 'shapes' in flag_data and flag_data['shapes']:
                for shape in flag_data['shapes']:
                    if shape['name'] == target_name:
                        flag_data.update(shape)
                        return flag_data
        return None

    def left_20_degree_rotation(self):
        """BNO055センサーを使い、正確に左へ20度回転する"""
        before_heading = self.bno.getVector(BNO055.VECTOR_EULER)[0]
        target_heading = (before_heading - 20) % 360
        
        delta_heading = target_heading - self.bno.getVector(BNO055.VECTOR_EULER)[0]
        if delta_heading > 180: delta_heading -= 360
        if delta_heading < -180: delta_heading += 360
            
        while abs(delta_heading) > 3:
            current_heading = self.bno.getVector(BNO055.VECTOR_EULER)[0]
            delta_heading = target_heading - current_heading
            if delta_heading > 180: delta_heading -= 360
            if delta_heading < -180: delta_heading += 360

            # P制御で計算した値でモーターを回す
            turn_power = np.clip(delta_heading * 2.5, -60, 60)
            # MotorDriverのset_speedsメソッドを使用
            self.driver.set_speeds(-turn_power, turn_power) # 左回転
            time.sleep(0.01)

        # MotorDriverのmotor_stop_brakeメソッドを使用
        self.driver.motor_stop_brake()


    def run(self):
        """
        全てのターゲットフラッグを探索し、接近するメインのタスクを実行
        """
        for target_name in self.TARGET_SHAPES:
            print(f"\n---====== 新しい目標: [{target_name}] の探索を開始します ======---")
            
            task_completed = False
            search_attempts = 0 # 全体の探索試行回数
            while not task_completed and search_attempts < 5: # 5回試行しても見つからなければ諦める
                
                # --- 探索 ---
                print(f"[{target_name}] を探しています... (試行 {search_attempts + 1}回目)")
                detected_data = self.detector.detect()
                target_flag = self.find_target_flag(detected_data, target_name)

                # 見つからない場合は「回転して探索」→「少し前進」を繰り返す
                if target_flag is None:
                    print(f"[{target_name}] が見つかりません。回転して探索します。")
                    rotation_count = 0
                    while target_flag is None and rotation_count < 20:
                        self.left_20_degree_rotation()
                        time.sleep(0.3)
                        detected_data = self.detector.detect()
                        target_flag = self.find_target_flag(detected_data, target_name)
                        rotation_count += 1
                        if target_flag:
                            break
                    
                    # 回転しても見つからなかったら、少し前進して再試行
                    if target_flag is None:
                        print("周辺に見つかりません。少し前進して再探索します。")
                        # MotorDriverのpetit_petitメソッドで少し前進
                        self.driver.petit_petit(2)
                        search_attempts += 1
                        continue # while not task_completed の先頭に戻る

                # --- P制御による追跡フェーズ ---
                print(f"[{target_name}] を発見！P制御で追跡を開始します。")
                while target_flag:
                    error_x = self.screen_center_x - target_flag['center'][0]
                    area_percent = (target_flag['area'] / self.screen_area) * 100
                    
                    # 1. 向きの調整（P制御）
                    if abs(error_x) > self.CENTER_TOLERANCE_PX:
                        turn_speed = self.KP_TURN * error_x
                        turn_speed = np.clip(turn_speed, -self.MAX_TURN_SPEED, self.MAX_TURN_SPEED)
                        print(f"位置調整中... 誤差: {error_x:.1f}px, 回転速度: {turn_speed:.1f}")
                        # MotorDriverのset_speedsメソッドで旋回
                        self.driver.set_speeds(turn_speed, -turn_speed)

                    # 2. 接近（P制御）
                    else:
                        if area_percent >= self.AREA_THRESHOLD_PERCENT:
                            self.driver.motor_stop_brake()
                            print(f"[{target_name}] に接近完了！ (画面占有率: {area_percent:.1f}%)")
                            task_completed = True
                            time.sleep(1)
                            break
                        
                        error_area = self.AREA_THRESHOLD_PERCENT - area_percent
                        approach_speed = self.KP_APPROACH * error_area
                        approach_speed = np.clip(approach_speed, self.MIN_APPROACH_SPEED, self.MAX_APPROACH_SPEED)
                        print(f"中央に補足。接近中... (占有率: {area_percent:.1f}%), 速度: {approach_speed:.1f}")
                        # MotorDriverのset_speedsメソッドで前進
                        self.driver.set_speeds(approach_speed, approach_speed)

                    time.sleep(0.02)
                    detected_data = self.detector.detect()
                    target_flag = self.find_target_flag(detected_data, target_name)
                    
                    if not target_flag:
                        self.driver.motor_stop_brake()
                        print(f"追跡中に [{target_name}] を見失いました。再探索します。")
                        break
            
            if not task_completed:
                print(f"探索を5回試行しましたが、[{target_name}] のタスクは完了できませんでした。")

        print("\n---====== 全ての目標の探索が完了しました ======---")

    def cleanup(self):
        print("--- 制御を終了します ---")
        self.driver.cleanup()
        self.detector.close()
        GPIO.cleanup()

# メインの実行ブロック (変更なし)
if __name__ == '__main__':
    bno = None
    robot = None
    try:
        bno = BNO055()
        if not bno.begin():
            raise RuntimeError("BNO055の起動に失敗しました。接続を確認してください。")
        print("BNO055の準備ができました。")
        
        robot = FN(bno)
        robot.run()

    except RuntimeError as e:
        print(f"実行時エラー: {e}")
    except KeyboardInterrupt:
        print("\nプログラムが中断されました。")
    finally:
        if robot:
            robot.cleanup()
        else:
            GPIO.cleanup()
