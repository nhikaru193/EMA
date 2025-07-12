import cv2
import numpy as np
import time
from picamera2 import Picamera2
from motor import MotorDriver # MotorDriverクラス (型ヒント用、直接は使わない)
from BNO055 import BNO055 # BNO055クラス (型ヒント用、直接は使わない)
from Flag_Detector2 import FlagDetector # Flag_Detector2 クラス
import RPi.GPIO as GPIO # GPIOクリーンアップのため

import following # <- この行は FlagSeeker のメソッド内で使用されるため、ここでインポートしておきます。
                 #    もし FlagSeeker のメソッド内で直接呼び出されている場合は重複になりません。

class FlagSeeker:
    """
    カメラとIMU (BNO055)、モータードライバーを使用して、
    指定された形状のフラッグを探索し、モーターを制御してフラッグに接近するクラス。
    """

    # --- クラス定数 (設定値) ---
    DEFAULT_TARGET_SHAPES = ["三角形", "長方形"] # デフォルトの目標図形リスト
    AREA_THRESHOLD_PERCENT = 20.0 # フラッグ接近完了とみなす画面占有率の閾値（パーセント）

    def __init__(self, driver_instance, bno_instance, picam2_instance,
                 target_shapes=None, area_threshold_percent=None):
        """
        FlagSeekerのコンストラクタです。

        Args:
            driver_instance (MotorDriver): 既に初期化されたMotorDriverのインスタンス。
            bno_instance (BNO055): 既に初期化されたBNO055のインスタンス。
            picam2_instance (Picamera2): 既に初期化されたPicamera2のインスタンス。
            target_shapes (list, optional): 探索する図形のリスト。
            area_threshold_percent (float, optional): フラッグ接近完了とみなす画面占有率の閾値。
        """
        # 設定値の初期化
        self.target_shapes = target_shapes if target_shapes is not None else self.DEFAULT_TARGET_SHAPES
        self.area_threshold_percent = area_threshold_percent if area_threshold_percent is not None else self.AREA_THRESHOLD_PERCENT

        # 共有ハードウェアインスタンスを受け取る
        self.driver = driver_instance
        self.bno = bno_instance
        self.picam2 = picam2_instance

        # FlagDetectorの初期化 (picam2_instanceを渡す)
        # FlagDetectorの__init__もpicam2_instanceを受け取るように修正されている必要があります
        self.detector = FlagDetector(picam2_instance=self.picam2)
        
        # 画面サイズ (FlagDetectorから取得)
        self.screen_area = self.detector.width * self.detector.height
        print(f"✅ FlagSeeker: カメラ解像度: {self.detector.width}x{self.detector.height}, 画面総ピクセル数: {self.screen_area}")
        print("✅ FlagSeeker: インスタンス作成完了。")

    def _find_target_flag_in_data(self, detected_data, target_name):
        """検出データから指定された図形(target_name)のフラッグを探して返す。"""
        for flag in detected_data:
            for shape in flag['shapes']:
                if shape['name'] == target_name:
                    return flag
        return None

    def seek_and_approach(self):
        """
        指定された目標形状のフラッグを順番に探索し、接近するメインシーケンスを実行します。
        """
        try:
            for target_name in self.target_shapes:
                print(f"\n---====== FlagSeeker: 新しい目標: [{target_name}] の探索を開始します ======---")
                
                task_completed = False
                while not task_completed:
                    # --- 探索フェーズ ---
                    print(f"FlagSeeker: [{target_name}] を探しています...")
                    detected_data = self.detector.detect()
                    target_flag = self._find_target_flag_in_data(detected_data, target_name)

                    if target_flag is None:
                        print(f"FlagSeeker: [{target_name}] が見つかりません。回転して探索します。")
                        search_attempt_count = 0
                        max_search_attempts = 40 # 探索回転の最大試行回数

                        # 少し前進してから全方位探索を行うロジック
                        # following.pyのfollow_forwardがdriverとbnoを直接受け取ることを前提
                        print("FlagSeeker: 探索のため少し前進します。")
                        following.follow_forward(self.driver, self.bno, base_speed=60, duration_time=1.0) 
                        self.driver.motor_stop_brake()
                        time.sleep(0.5)

                        # 全方位を探索 (左回頭)
                        print("FlagSeeker: この場所で全方位コーン探索を行います。")
                        rotation_time_per_step = 0.2 # 1回の旋回時間
                        turn_speed = 70 # 旋回速度
                        
                        # 最大旋回ステップ数を調整（約360度回るため）
                        # 1回の petit_left(0, 70) で約5-10度回転すると仮定
                        approx_angle_per_step = 7 # 1ステップあたりの概算角度 (要調整)
                        max_rotation_steps = int(360 / approx_angle_per_step) + 5 # 余裕を持たせる

                        for step in range(max_rotation_steps):
                            # 検出を試みる
                            detected_data = self.detector.detect()
                            target_flag = self._find_target_flag_in_data(detected_data, target_name)
                            if target_flag:
                                print(f"FlagSeeker: 回転中に [{target_name}] を見つけました！")
                                break # ターゲットが見つかったので、回転ループを抜ける
                            
                            print(f"FlagSeeker: 視野角内に [{target_name}] を検知できませんでした。左回頭を行います (ステップ {step+1}/{max_rotation_steps})")
                            self.driver.petit_left(0, turn_speed) # 左旋回開始
                            # MotorDriverの petit_left は、pigpioベースのMotorDriverに修正済みであることを前提
                            self.driver.petit_left(turn_speed, 0) 
                            self.driver.motor_stop_brake()
                            time.sleep(rotation_time_per_step) # 短く回頭
                            time.sleep(0.2) # 停止してセンサー安定化

                        if target_flag is None:
                            print(f"FlagSeeker: 探索しましたが [{target_name}] は見つかりませんでした。次の目標に移ります。")
                            break # while not task_completed ループを抜ける (次のターゲットへ)

                    # --- 追跡フェーズ（中央寄せ＆接近）---
                    print(f"FlagSeeker: [{target_name}] を発見！追跡を開始します。")
                    while target_flag: # フラッグが見つかっている間は追跡を続ける
                        # --- 中央寄せ ---
                        if target_flag['location'] != '中央':
                            print(f"FlagSeeker: 位置を調整中... (現在位置: {target_flag['location']})")
                            turn_speed_adjust = 60 # 調整時の旋回速度
                            if target_flag['location'] == '左':
                                self.driver.petit_right(0, turn_speed_adjust) # 右に小刻み旋回
                                self.driver.petit_right(turn_speed_adjust, 0)
                                self.driver.motor_stop_brake()
                                time.sleep(0.5) # 短い待機
                            elif target_flag['location'] == '右':
                                self.driver.petit_left(0, turn_speed_adjust) # 左に小刻み旋回
                                self.driver.petit_left(turn_speed_adjust, 0)
                                self.driver.motor_stop_brake()
                                time.sleep(0.5) # 短い待機
                            
                            # 動かした直後に再検出して、位置を再評価
                            print("FlagSeeker: 位置調整後、再検出中...")
                            detected_data = self.detector.detect()
                            target_flag = self._find_target_flag_in_data(detected_data, target_name)
                            
                            if not target_flag:
                                print(f"FlagSeeker: 調整中に [{target_name}] を見失いました。再探索します。")
                                break # 追跡ループを抜けて、外側の探索ループに戻る
                            
                            continue # 位置を再評価するため、追跡ループの最初に戻る
                        
                        # --- 接近 ---
                        else: # 中央にいる場合
                            flag_area = cv2.contourArea(target_flag['flag_contour'])
                            area_percent = (flag_area / self.screen_area) * 100
                            print(f"FlagSeeker: 中央に補足。接近中... (画面占有率: {area_percent:.1f}%)")

                            if area_percent >= self.area_threshold_percent:
                                print(f"\n✅ FlagSeeker: [{target_name}] に接近完了！画面占有率が閾値({self.area_threshold_percent:.1f}%)を超えました。")
                                task_completed = True # このターゲットのタスク完了
                                self.driver.motor_stop_brake()
                                time.sleep(1) # 完了後の待機
                                break # 追跡ループを抜ける
                            else:
                                # しきい値未満なら、PD制御で直進しつつフラッグを追従して前進
                                print(f"FlagSeeker: 目標に接近するため前進します。")
                                # `following.follow_forward`にdriverとbnoを渡す
                                following.follow_forward(self.driver, self.bno, base_speed=40, duration_time=1.0)
                                self.driver.motor_stop_brake() # 短い前進後に停止
                                time.sleep(0.2) # 停止してセンサー安定化
                        
                        # 前進後に再検出（正しい位置にいるか確認し、次のループへ）
                        print("FlagSeeker: 接近動作後、再検出中...")
                        detected_data = self.detector.detect()
                        target_flag = self._find_target_flag_in_data(detected_data, target_name)
                        
                        if not target_flag:
                            print(f"FlagSeeker: 接近中に [{target_name}] を見失いました。再探索します。")
                            break # 追跡ループを抜けて、外側の探索ループに戻る
                
                if task_completed:
                    continue # for target_name in self.target_shapes ループの次の要素へ

            print("\n---====== FlagSeeker: 全ての目標の探索が完了しました ======---")

        except KeyboardInterrupt:
            print("\n[STOP] FlagSeeker: 手動で停止されました。")
        except Exception as e:
            print(f"\n[FATAL] FlagSeeker: 予期せぬエラーが発生しました: {e}")
        finally:
            self.driver.motor_stop_brake() # 念のため停止

    def cleanup(self):
        """
        FlagSeeker独自のクリーンアップ処理（現在はモーター停止のみ。Picamera2は外部で管理）
        """
        if self.driver:
            self.driver.motor_stop_brake()
        print("FlagSeeker: クリーンアップ完了。")
