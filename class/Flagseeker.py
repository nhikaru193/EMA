import time
import smbus # BNO055用
import struct # このコードでは直接使われていないが、structをimportしていたので残す
import following # 別のファイルに定義された方向追従制御関数 (PD制御ロジックを内包)
import cv2
import math
import numpy as np
from picamera2 import Picamera2 # FlagDetectorクラス内で使用される
from BNO055 import BNO055
from motor import MotorDriver
from Flag_Detector2 import FlagDetector # ユーザーのFlagDetectorクラスを使用
import RPi.GPIO as GPIO

class FlagSeeker:
    """
    カメラとIMU (BNO055) を使用して、指定された形状のフラッグを探索し、
    モーターを制御してフラッグに接近するクラス。
    """

    # --- クラス定数 (設定値) ---
    DEFAULT_TARGET_SHAPES = ["三角形", "長方形"] # デフォルトの目標図形リスト
    AREA_THRESHOLD_PERCENT = 20.0 # フラッグ接近完了とみなす画面占有率の閾値（パーセント）
    # モーターピン設定 (MotorDriverクラス内で定義されているが、ここに定数として持たせることも可能)
    # PWMA = 12
    # ...

    def __init__(self, target_shapes=None, area_threshold_percent=None):
        """
        FlagSeekerのコンストラクタです。

        Args:
            target_shapes (list, optional): 探索する図形のリスト。
                                            指定しない場合はDEFAULT_TARGET_SHAPESを使用。
            area_threshold_percent (float, optional): フラッグ接近完了とみなす画面占有率の閾値。
                                                      指定しない場合はAREA_THRESHOLD_PERCENTを使用。
        """
        # 設定値の初期化
        self.target_shapes = target_shapes if target_shapes is not None else self.DEFAULT_TARGET_SHAPES
        self.area_threshold_percent = area_threshold_percent if area_threshold_percent is not None else self.AREA_THRESHOLD_PERCENT

        # ハードウェアインスタンス
        self.detector = None
        self.driver = None
        self.bno = None

        # 画面サイズ (FlagDetectorから取得)
        self.screen_area = 0

        self._initialize_hardware()

    def _initialize_hardware(self):
        """
        必要なハードウェア (FlagDetector, MotorDriver, BNO055) を初期化します。
        """
        # GPIO設定はMotorDriverとFlagDetector内で適切に行われる前提
        # 必要であればここでGPIO.setmode()などを明示的に呼び出す

        # FlagDetectorの初期化 (内部でPicamera2も初期化される)
        self.detector = FlagDetector()
        # 画面サイズを取得
        self.screen_area = self.detector.width * self.detector.height
        print(f"カメラ解像度: {self.detector.width}x{self.detector.height}, 画面総ピクセル数: {self.screen_area}")

        # MotorDriverの初期化
        self.driver = MotorDriver(
            PWMA=12, AIN1=23, AIN2=18,
            PWMB=19, BIN1=16, BIN2=26,
            STBY=21
        )

        # BNO055 IMUの初期化
        self.bno = BNO055()
        if not self.bno.begin():
            print("🔴 BNO055の初期化に失敗しました。プログラムを終了します。")
            self.cleanup() # 失敗時はクリーンアップ
            exit(1)
        time.sleep(1) # センサー安定化のための待機
        self.bno.setExternalCrystalUse(True)
        self.bno.setMode(BNO055.OPERATION_MODE_NDOF)
        time.sleep(1) # モード設定後の待機
        print("✅ センサー類の初期化完了。")

        # BNO055キャリブレーション待機
        self._wait_for_bno055_calibration()

    def _wait_for_bno055_calibration(self):
        """BNO055センサーの完全キャリブレーションを待機します。"""
        print("BNO055のキャリブレーション待機中...")
        calibration_start_time = time.time()
        while True:
            sys_cal, gyro_cal, accel_cal, mag_cal = self.bno.getCalibration()
            print(f"Calib → Sys:{sys_cal}, Gyro:{gyro_cal}, Acc:{accel_cal}, Mag:{mag_cal}", end='\r')
            # ジャイロ、地磁気がレベル3になればOKとする（または全て3にする）
            if gyro_cal == 3 and mag_cal == 3: # accel_cal == 3 も追加するとより厳密
                print("\n✅ キャリブレーション完了！フラッグ探索を開始します。")
                break
            time.sleep(0.5) # 0.5秒ごとに状態を確認
        print(f"キャリブレーションにかかった時間: {time.time() - calibration_start_time:.1f}秒\n")

    def _find_target_flag_in_data(self, detected_data, target_name):
        """検出データから指定された図形(target_name)のフラッグを探して返す。
        これはクラスのプライベートヘルパーメソッド。
        """
        for flag in detected_data:
            for shape in flag['shapes']:
                if shape['name'] == target_name:
                    # 見つかった場合は、そのフラッグ全体情報を返す
                    return flag
        return None

    def seek_and_approach(self):
        """
        指定された目標形状のフラッグを順番に探索し、接近するメインシーケンスを実行します。
        """
        try:
            # --- 全てのターゲットに対してループ ---
            for target_name in self.target_shapes:
                print(f"\n---====== 新しい目標: [{target_name}] の探索を開始します ======---")
                
                task_completed = False
                while not task_completed:
                    # --- 探索フェーズ ---
                    print(f"[{target_name}] を探しています...")
                    detected_data = self.detector.detect()
                    target_flag = self._find_target_flag_in_data(detected_data, target_name)

                    # 見つからない場合は回転して探索
                    if target_flag is None:
                        print(f"[{target_name}] が見つかりません。回転して探索します。")
                        search_attempt_count = 0
                        max_search_attempts = 40 # 探索回転の最大試行回数（元のコードの`search_count < 40`から）

                        # 少し前進してから全方位探索を行うロジック
                        # `driver.changing_moving_forward` は `motor.py` に存在しないため、
                        # `following.follow_forward` か他の前進関数に置き換えが必要です。
                        # ここでは仮に `following.follow_forward` を使用します。
                        print("探索のため少し前進します。")
                        # 速度と時間については適宜調整してください
                        following.follow_forward(self.driver, self.bno, base_speed=60, duration_time=1.0) 
                        self.driver.motor_stop_brake()
                        time.sleep(0.5)

                        initial_heading = self.bno.getVector(BNO055.VECTOR_EULER)[0]
                        if initial_heading is None:
                            print("警告: 探索開始時の方位が取得できません。")
                            initial_heading = 0 # フォールバック

                        current_heading = initial_heading
                        # 全方位を探索するループ
                        while search_attempt_count < max_search_attempts:
                            print(f"視野角内に [{target_name}] を検知できませんでした。左回頭を行います。")
                            self.driver.petit_left(0, 70) # 左旋回開始 (速度70)
                            self.driver.petit_left(70, 0)
                            time.sleep(0.1) # 短い時間回頭
                            self.driver.motor_stop_brake()
                            time.sleep(0.2) # 停止してセンサー安定化

                            current_heading = self.bno.getVector(BNO055.VECTOR_EULER)[0]
                            if current_heading is None:
                                print("警告: 探索旋回中に方位が取得できません。")
                                current_heading = initial_heading # 前回の値を使うか、適当なフォールバック

                            # 検出を試みる
                            detected_data = self.detector.detect()
                            target_flag = self._find_target_flag_in_data(detected_data, target_name)
                            if target_flag:
                                print(f"回転中に [{target_name}] を見つけました！")
                                break # ターゲットが見つかったので、回転ループを抜ける

                            search_attempt_count += 1
                            # 360度を回りきったかどうかの簡易判定 (これは厳密ではないので注意)
                            # if search_attempt_count % (360 // (回転角度の目安)) == 0:
                            #     print("一周回った可能性がありますが、見つかりません。")

                        if target_flag is None:
                            print(f"探索しましたが [{target_name}] は見つかりませんでした。次の目標に移ります。")
                            break # while not task_completed ループを抜ける (次のターゲットへ)

                    # --- 追跡フェーズ（中央寄せ＆接近）---
                    print(f"[{target_name}] を発見！追跡を開始します。")
                    while target_flag: # フラッグが見つかっている間は追跡を続ける
                        # --- 中央寄せ ---
                        if target_flag['location'] != '中央':
                            print(f"位置を調整中... (現在位置: {target_flag['location']})")
                            if target_flag['location'] == '左':
                                self.driver.petit_right(0, 60) # 右に小刻み旋回
                                self.driver.petit_right(60, 0)
                                self.driver.motor_stop_brake()
                                time.sleep(0.5) # 短い待機
                            elif target_flag['location'] == '右':
                                self.driver.petit_left(0, 60) # 左に小刻み旋回
                                self.driver.petit_left(60, 0)
                                self.driver.motor_stop_brake()
                                time.sleep(0.5) # 短い待機
                            
                            # 動かした直後に再検出して、位置を再評価
                            print("  位置調整後、再検出中...")
                            detected_data = self.detector.detect()
                            target_flag = self._find_target_flag_in_data(detected_data, target_name)
                            
                            if not target_flag:
                                print(f"調整中に [{target_name}] を見失いました。再探索します。")
                                break # 追跡ループを抜けて、外側の探索ループに戻る
                            
                            continue # 位置を再評価するため、追跡ループの最初に戻る
                        
                        # --- 接近 ---
                        else: # 中央にいる場合
                            flag_area = cv2.contourArea(target_flag['flag_contour'])
                            area_percent = (flag_area / self.screen_area) * 100
                            print(f"中央に補足。接近中... (画面占有率: {area_percent:.1f}%)")

                            if area_percent >= self.area_threshold_percent:
                                print(f"\n✅ [{target_name}] に接近完了！画面占有率が閾値({self.area_threshold_percent:.1f}%)を超えました。")
                                task_completed = True # このターゲットのタスク完了
                                self.driver.motor_stop_brake()
                                time.sleep(1) # 完了後の待機
                                break # 追跡ループを抜ける
                            else:
                                # しきい値未満なら、PD制御で直進しつつフラッグを追従して前進
                                print(f"目標に接近するため前進します。")
                                # `petit_petit(2)` はmotor.pyに存在しないため、`following.follow_forward`に置き換えます。
                                # 速度と時間については適宜調整してください
                                following.follow_forward(self.driver, self.bno, base_speed=40, duration_time=1.0)
                                self.driver.motor_stop_brake() # 短い前進後に停止
                                time.sleep(0.2) # 停止してセンサー安定化
                        
                        # 前進後に再検出（正しい位置にいるか確認し、次のループへ）
                        print("  接近動作後、再検出中...")
                        detected_data = self.detector.detect()
                        target_flag = self._find_target_flag_in_data(detected_data, target_name)
                        
                        if not target_flag:
                            print(f"接近中に [{target_name}] を見失いました。再探索します。")
                            break # 追跡ループを抜けて、外側の探索ループに戻る
                
                # このターゲットのタスクが完了していれば、次のターゲットへ進む
                if task_completed:
                    continue # for target_name in self.target_shapes ループの次の要素へ

            print("\n---====== 全ての目標の探索が完了しました ======---")

        except KeyboardInterrupt:
            print("\n[STOP] 手動で停止されました。")
        except Exception as e:
            print(f"\n[FATAL] 予期せぬエラーが発生しました: {e}")
        finally:
            self.cleanup()

    def cleanup(self):
        """
        プログラム終了時に使用したリソースを解放します。
        """
        print("--- 制御を終了します ---")
        if self.driver:
            self.driver.cleanup()
        if self.detector:
            self.detector.close() # Picamera2をクローズ
        # BNO055は明示的なクローズは不要な場合が多い
        # if self.bno:
        #     self.bno.end() # BNO055ライブラリに終了メソッドがあれば
        
        # GPIOクリーンアップは最後に
        GPIO.cleanup()
        cv2.destroyAllWindows()
        print("プログラムを終了しました。")

# --- メイン実行ブロック ---
if __name__ == '__main__':
    # === 制御パラメータの設定 ===
    # 探索するターゲット形状のリスト（例: "T字", "十字" も追加可能）
    # DEFAULT_TARGET_SHAPES = ["三角形", "長方形", "T字", "十字"] などのように
    # Flag_Detector2.pyで定義されている形状名と一致させる必要があります。
    my_target_shapes = ["三角形", "長方形"]
    
    # フラッグ接近完了とみなす画面占有率（パーセント）
    my_area_threshold = 20.0 

    # FlagSeekerのインスタンスを作成
    seeker = FlagSeeker(
        target_shapes=my_target_shapes,
        area_threshold_percent=my_area_threshold
    )

    # 探索・接近シーケンスを開始
    seeker.seek_and_approach()
