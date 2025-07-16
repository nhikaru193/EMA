import time
import smbus
import struct
import cv2
import math
import numpy as np
from picamera2 import Picamera2
from BNO055 import BNO055
from motor import MotorDriver
from Flag_Detector3 import FlagDetector # 'center', 'area'を返すように修正が必要
import RPi.GPIO as GPIO

class FN:
    # --- クラスの初期化メソッド ---
    def __init__(self, bno: BNO055):

        # --- 設定値 ---
        self.TARGET_SHAPES = ["三角形", "長方形"]
        self.AREA_THRESHOLD_PERCENT = 23.0  # この占有率に達したら接近完了
        
        # === P制御関連のパラメータ ===
        self.KP_TURN = 0.35                 # 回転速度の比例ゲイン 調整です
        self.KP_APPROACH = 2.0              # 接近速度の比例ゲイン 調整です
        self.CENTER_TOLERANCE_PX = 20       # 中心とみなす許容範囲 (ピクセル単位)
        self.MAX_TURN_SPEED = 70            # 回転時の最大速度
        self.MAX_APPROACH_SPEED = 80        # 接近時の最大速度
        self.MIN_APPROACH_SPEED = 45        # 接近時の最低速度

        # --- 初期化処理 ---
        self.detector = FlagDetector()
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
            # `shapes`キーが存在し、そのリストが空でないことを確認
            if 'shapes' in flag_data and flag_data['shapes']:
                 # shapesリスト内の各図形をチェック
                for shape in flag_data['shapes']:
                    if shape['name'] == target_name:
                        # 元のflag_dataにshapeの情報を追加して返す
                        # これにより、'center'や'area'にアクセスできる
                        flag_data.update(shape) 
                        return flag_data
        return None

    def left_20_degree_rotation(self):
        """BNO055センサーを使い、正確に左へ20度回転する"""
        before_heading = self.bno.getVector(BNO055.VECTOR_EULER)[0]
        target_heading = (before_heading - 20) % 360
        
        # 回転方向を決定するための差分計算
        delta_heading = target_heading - self.bno.getVector(BNO055.VECTOR_EULER)[0]
        if delta_heading > 180: delta_heading -= 360
        if delta_heading < -180: delta_heading += 360
            
        while abs(delta_heading) > 3: # 許容誤差3度
            current_heading = self.bno.getVector(BNO055.VECTOR_EULER)[0]
            delta_heading = target_heading - current_heading
            if delta_heading > 180: delta_heading -= 360
            if delta_heading < -180: delta_heading += 360

            # P制御的な回転
            turn_power = np.clip(delta_heading * 2.5, -60, 60)
            self.driver.set_speeds(-turn_power, turn_power) # 左回転
            time.sleep(0.01)

        self.driver.motor_stop_brake()


    def run(self):
        """
        全てのターゲットフラッグを探索し、接近するメインのタスクを実行
        """
        for target_name in self.TARGET_SHAPES:
            print(f"\n---====== 新しい目標: [{target_name}] の探索を開始します ======---")
            
            task_completed = False
            while not task_completed:
                
                # --- 探索 ---
                print(f"[{target_name}] を探しています...")
                detected_data = self.detector.detect()
                target_flag = self.find_target_flag(detected_data, target_name)

                # 見つからない場合は回転して探索
                if target_flag is None:
                    print(f"[{target_name}] が見つかりません。回転して探索します。")
                    search_count = 0
                    while target_flag is None and rotation_count < 20: # 360度/20度 = 18回 + α
                        self.driver.petit_petit(2)
                        detected_data = self.detector.detect()
                        target_flag = self.find_target_flag(detected_data, target_name)
                        time.sleep(0.5)
                        search_count += 1
                        rotation_count = 0
                        while target_flag is None and rotation_count < 23:
                            self.left_20_degree_rotation()
                            time.sleep(0.5) #7/16追加
                            detected_data = self.detector.detect()
                            target_flag = self.find_target_flag(detected_data, target_name)
                            time.sleep(0.5)
                            rotation_count += 1
                        if target_flag:
                            break
                    
                    # 回転しても見つからなかったら、このターゲットは諦める
                    if target_flag is None:
                        print(f"探索しましたが [{target_name}] は見つかりませんでした。次の目標に移ります。")
                        break # while not task_completed ループを抜ける

                # --- P制御による追跡フェーズ ---
                print(f"[{target_name}] を発見！P制御で追跡を開始します。")
                while target_flag:
                    # 誤差（中心からのズレ）を計算
                    error_x = self.screen_center_x - target_flag['center'][0]
                    
                    # 画面占有率を計算
                    area_percent = (target_flag['area'] / self.screen_area) * 100
                    
                    # 1. 向きの調整（P制御）
                    if abs(error_x) > self.CENTER_TOLERANCE_PX:
                        turn_speed = self.KP_TURN * error_x
                        # 速度が最大値を超えないように制限（クリッピング）
                        turn_speed = np.clip(turn_speed, -self.MAX_TURN_SPEED, self.MAX_TURN_SPEED)
                        
                        print(f"位置調整中... 誤差: {error_x:.1f}px, 回転速度: {turn_speed:.1f}")
                        # 誤差に応じて左右のモーター速度を決定（右がプラス、左がマイナス）
                        self.driver.set_speeds(turn_speed, -turn_speed)

                    # 2. 接近（P制御）
                    else:
                        # 面積がしきい値に達したらタスク完了
                        if area_percent >= self.AREA_THRESHOLD_PERCENT:
                            self.driver.motor_stop_brake()
                            print(f"[{target_name}] に接近完了！ (画面占有率: {area_percent:.1f}%)")
                            task_completed = True
                            time.sleep(1)
                            break # 追跡ループを抜ける
                        
                        # 誤差（目標面積との差）から前進速度を決定
                        error_area = self.AREA_THRESHOLD_PERCENT - area_percent
                        approach_speed = self.KP_APPROACH * error_area
                        # 速度を最小値と最大値の間に制限
                        approach_speed = np.clip(approach_speed, self.MIN_APPROACH_SPEED, self.MAX_APPROACH_SPEED)
                        
                        print(f"中央に補足。接近中... (占有率: {area_percent:.1f}%), 速度: {approach_speed:.1f}")
                        self.driver.set_speeds(approach_speed, approach_speed)

                    # 動作後に再検出
                    time.sleep(0.02) # CPU負荷を軽減
                    detected_data = self.detector.detect()
                    target_flag = self.find_target_flag(detected_data, target_name)
                    
                    if not target_flag:
                        self.driver.motor_stop_brake()
                        print(f"追跡中に [{target_name}] を見失いました。再探索します。")
                        break # 追跡ループを抜けて探索フェーズに戻る
            
            if not task_completed:
                print(f"[{target_name}]のタスクは完了しませんでした。")

        print("\n---====== 全ての目標の探索が完了しました ======---")


    def cleanup(self):
        """プログラム終了時にリソースを解放します。"""
        print("--- 制御を終了します ---")
        self.driver.cleanup()
        self.detector.close()
        GPIO.cleanup()

# メインの実行ブロック
if __name__ == '__main__':
    bno = None
    robot = None
    try:
        # BNO055（9軸センサー）の初期化
        bno = BNO055()
        if not bno.begin():
            raise RuntimeError("BNO055の起動に失敗しました。接続を確認してください。")
        print("BNO055の準備ができました。")
        
        # メインクラスのインスタンスを作成
        robot = FN(bno)
        # メインの処理を実行
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
