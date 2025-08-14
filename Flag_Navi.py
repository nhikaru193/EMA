import time
import smbus
import struct
import os #save fileのときに使用
import cv2
import math
import numpy as np
from picamera2 import Picamera2
from BNO055 import BNO055
from motor import MotorDriver
from Flag_B import Flag_B
import RPi.GPIO as GPIO
from collections import deque

class FN:
    # --- クラスの初期化メソッド ---
    def __init__(self, bno: BNO055, flag_location):
        
        # --- 設定値 ---
        self.TARGET_SHAPES = ["三角形", "長方形", "T字", "十字"] #, "T字", "十字"] #"三角形", "長方形", "T字", "十字"を追加する
        self.AREA_THRESHOLD_PERCENT = 3.5
        self.turn_speed = 95
        self.F_lat = flag_location[0]
        self.F_lon = flag_location[1]

        # --- 初期化処理 ---
        self.detector = Flag_B(triangle_tolerance=0.5)
        self.driver = MotorDriver(
            PWMA=12, AIN1=23, AIN2=18,    
            PWMB=19, BIN1=16, BIN2=26,    
            STBY=21                      
        )
        self.screen_area = self.detector.width * self.detector.height
        
        # === BNO055 初期化 ===
        self.bno = bno

    def find_target_flag(self, detected_data, target_name):
        """検出データから指定された図形(target_name)のフラッグを探して返す"""
        for flag in detected_data:
            for shape in flag['shapes']:
                if shape['name'] == target_name:
                    return flag
        return None

    # === 追加したヘルパー関数 ===
    def get_shape_from_flag(self, flag_data, shape_name):
        """
        指定されたフラッグデータの中から、目的の図形データを検索して返す
        """
        if 'shapes' in flag_data:
            for shape in flag_data['shapes']:
                if shape['name'] == shape_name:
                    return shape
        return None

    """
    def left_10_degree_rotation(self):
        before_heading = self.bno.getVector(BNO055.VECTOR_EULER)[0]
        target_heading = (before_heading - 20) % 360
        while True:
            current_heading = self.bno.getVector(BNO055.VECTOR_EULER)[0]
            delta_heading = ((target_heading - current_heading + 180) % 360) - 180 #値を代入すればわかる
            if abs(delta_heading) <= 3:
                break
            elif delta_heading < -5:
                self.driver.petit_left(0, 70)
                self.driver.motor_stop_brake()
            elif delta_heading > 5:
                self.driver.petit_right(0, 60) #値が変わるのは右回りでも左回りでも同じだけ回ると同じ場所しか見れないため
                self.driver.motor_stop_brake()
                """
            
    def run(self):
        """
        全てのターゲットフラッグを探索し、接近するメインのタスクを実行
        """
        # --- 全てのターゲットに対してループ ---
        for target_name in self.TARGET_SHAPES:
            print(f"\n---====== 新しい目標: [{target_name}] の探索を開始します ======---")
            
            task_completed = False
            # スタック判定のために方位角を保存するdeque
            heading_history = deque(maxlen=4) # 直近3回の回転後の方位を記録
            while not task_completed:
                
                # --- 探索 ---
                print(f"[{target_name}] を探しています...")
                detected_data = self.detector.detect()
                target_flag = self.find_target_flag(detected_data, target_name)

                # 見つからない場合は回転して探索
                if target_flag is None:
                    print(f"[{target_name}] が見つかりません。回転して探索します。")
                    search_count = 0
                    """
                    while target_flag is None and search_count < 50: # タイムアウト設定
                        self.driver.petit_right(0, self.turn_speed)
                        self.driver.petit_right(self.turn_speed, 0)
                        self.driver.motor_stop_brake()
                        time.sleep(1.0)
                        detected_data = self.detector.detect()
                        target_flag = self.find_target_flag(detected_data, target_name)
                        time.sleep(0.5)
                        
                        #GPS_StoF = Amaging_GPS(driver, bno, goal_location = [self.flag_lat, self.flag_lon])
                        #GPS_StoF.run()
                        
                        detected_data = self.detector.detect()
                        target_flag = self.find_target_flag(detected_data, target_name)
                        time.sleep(0.5)
                        search_count += 1
                    """ 
                        
                    while target_flag is None and search_count < 70:
                        self.driver.petit_petit(4)
                        detected_data = self.detector.detect()
                        target_flag = self.find_target_flag(detected_data, target_name)
                        time.sleep(0.5)
                        search_count += 1
                        
                        rotation_count = 0
                        while target_flag is None and rotation_count < 45:
                            
                            """
                            self.driver.petit_right(0, 95)
                            self.driver.motor_stop_brake()
                            time.sleep(0.5)
                            """
                            
                            self.driver.changing_right(0, 90)
                            time.sleep(3)
                            self.driver.changing_right(90, 0)
                            time.sleep(0.5)
                            detected_data = self.detector.detect()
                            target_flag = self.find_target_flag(detected_data, target_name)
                            time.sleep(0.5)
                            rotation_count += 1
                            
                # 回転しても見つからなかったら、このターゲットは諦めて次の輪郭検知　ここむずい　by中川
                if target_flag is None:
                    print(f"探索しましたが [{target_name}] は見つかりませんでした。次の目標に移ります。")
                    break # while not task_completed ループを抜ける

                # --- 追跡（中央寄せ＆接近）---
                print(f"[{target_name}] を発見！追跡を開始します。")
                while target_flag:
                    # --- 中央寄せ ---
                    if target_flag['location'] != '中央':
                        print(f"位置を調整中... (現在位置: {target_flag['location']})")
                        if target_flag['location'] == '左':
                            
                            """
                            self.driver.petit_left(0, self.turn_speed)
                            self.driver.petit_left(self.turn_speed, 0)
                            self.driver.motor_stop_brake()
                            time.sleep(1.0)
                            """
                            self.driver.changing_left(0, 90)
                            time.sleep(3)
                            self.driver.changing_left(90, 0)
                            time.sleep(0.5)

                        elif target_flag['location'] == '右':
                            
                            """
                            self.driver.petit_right(0, self.turn_speed)
                            self.driver.petit_right(self.turn_speed, 0)
                            self.driver.motor_stop_brake()
                            time.sleep(1.0)
                            """
                            
                            self.driver.changing_right(0, 90)
                            time.sleep(3)
                            self.driver.changing_right(90, 0)
                            time.sleep(0.5)
                            
                        # 動かした直後に再検出
                        print("  再検出中...")
                        detected_data = self.detector.detect()
                        target_flag = self.find_target_flag(detected_data, target_name)
                        
                        if not target_flag:
                            print(f"調整中に [{target_name}] を見失いました。")
                            break # 追跡ループを抜ける
                        
                        # 位置を再評価するため、ループの最初に戻る
                        continue
                    
                    # --- 接近 ---
                    else: # 中央にいる場合
                        # ===== ここからが修正箇所 =====
                        # 目的の図形（'T字'など）のデータを取得する
                        target_shape_data = self.get_shape_from_flag(target_flag, target_name)

                        # 図形データとその輪郭('contour')が存在するかを安全にチェック
                        if target_shape_data and 'contour' in target_shape_data:
                            # ★★★ 白領域ではなく、対象の黒図形の面積を計算 ★★★
                            shape_area = cv2.contourArea(target_shape_data['contour'])
                            area_percent = (shape_area / self.screen_area) * 100
                            print(f"中央に補足。接近中... (図形 '{target_name}' の画面占有率: {area_percent:.1f}%)")

                            # 面積の比較
                            if area_percent >= self.AREA_THRESHOLD_PERCENT:
                                print(f"[{target_name}] に接近完了！")
                                task_completed = True
                                time.sleep(1)
                                break # 追跡ループを抜ける
                            else:
                                # しきい値未満なら、前進
                                self.driver.petit_petit(5)
                        
                        else:
                            # 'contour' がないという予期せぬ事態に対応
                            print(f"エラー: [{target_name}]は中央にありますが、その図形データがありません。再探索します。")
                            break # 追跡ループを抜けて探索からやり直す
                        # ===== ここまでが修正箇所 =====

                    # 動作後に再検出
                    print("  再検出中...")
                    detected_data = self.detector.detect()
                    target_flag = self.find_target_flag(detected_data, target_name)
                    
                    if not target_flag:
                        print(f"追跡中に [{target_name}] を見失いました。再探索します。")
                        break # 追跡ループ(while target_flag)を抜ける

        print("\n---====== 全ての目標の探索が完了しました ======---")
        print("--- 制御を終了します ---")
        self.driver.cleanup()
        self.detector.close()
        GPIO.cleanup()
        
    def cleanup(self):
        
        #プログラム終了時にリソースを解放します。
        
        print("--- 制御を終了します ---")
        self.driver.cleanup()
        self.detector.close()
        self.pi.bb_serial_read_close(self.RX_PIN)
        self.pi.stop()
        #cv2.destroyAllWindows()

"""
# メインの実行ブロック
if __name__ == '__main__':
    # --- 各クラスのインスタンスを作成 ---
    # 1. モータードライバの初期化
    
    # 2. BNO055（9軸センサー）の初期化
    try:
        bno = BNO055()
        if not bno.begin():
            raise RuntimeError("BNO055の起動に失敗しました。接続を確認してください。")
        # 必要に応じてキャリブレーションステータスの確認などをここに追加
        print("BNO055の準備ができました。")
    except Exception as e:
        print(f"BNO055の初期化中にエラーが発生しました: {e}")
        # BNO055が使えない場合はプログラムを終了
    

    # --- メインクラスのインスタンスを作成し、実行 ---
    # 3. FlagNavigatorにbnoを渡してインスタンス化
    flag_location = [35.9241360, 139.9113158]
    robot = FN(bno, flag_location)

    try:
        # 4. メインの処理を実行
        robot.run()
    except KeyboardInterrupt:
        # Ctrl+Cが押されたら、安全に終了処理を行う
        print("\nプログラムが中断されました。")
    finally:
        # 5. 終了処理（クリーンアップ）を呼び出す
        robot.cleanup()
"""
