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
        self.TARGET_SHAPES = ["三角形", "長方形"] #"三角形", "長方形", "T字", "十字"を追加する
        self.AREA_THRESHOLD_PERCENT = 30.0
        self.turn_speed = 45
        self.F_lat = flag_location[0]
        self.F_lon = flag_location[1]

        # --- 初期化処理 ---
        self.detector = FlagDetector(triangle_tolerance=0.5)
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

    def left_20_degree_rotation(self):
        before_heading = self.bno.getVector(BNO055.VECTOR_EULER)[0]
        target_heading = (before_heading - 20) % 360
        while True:
            current_heading = self.bno.getVector(BNO055.VECTOR_EULER)[0]
            delta_heading = ((target_heading - current_heading + 180) % 360) - 180
            if abs(delta_heading) <= 3:
                break
            elif delta_heading < -3:
                self.driver.petit_left(0, 90)
                self.driver.motor_stop_brake()
            elif delta_heading > 3:
                self.driver.petit_right(0, 80)
                self.driver.motor_stop_brake()
            
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
                        self.driver.petit_petit(2)
                        detected_data = self.detector.detect()
                        target_flag = self.find_target_flag(detected_data, target_name)
                        time.sleep(0.5)
                        search_count += 1
                        
                        rotation_count = 0
                        while target_flag is None and rotation_count < 23:
                            self.left_20_degree_rotation()

                            # ===== ここからスタック判定処理 =====
                            current_heading = self.bno.getVector(BNO055.VECTOR_EULER)[0]
                            heading_history.append(current_heading)

                            # 履歴が3つ溜まったらスタック判定を行う
                            if len(heading_history) == 4:
                                # 2回前と1回前、1回前と現在の角度差を計算
                                a = abs((heading_history[0] - heading_history[1] + 180) % 360 - 180)
                                b = abs((heading_history[1] - heading_history[2] + 180) % 360 - 180)
                                c = abs((heading_history[2] - heading_history[3] + 180) % 360 - 180)

                                # 2回連続で角度の変化が5度未満ならスタックと判断
                                if a < 3 and b < 3 and c < 3:
                                    print("スタックを検知しました！回避行動を開始します。")
                                    # 前後左右に動いてスタックからの脱出を試みる
                                    self.driver.changing_right(0, 90)
                                    time.sleep(3)
                                    self.driver.changing_right(90, 0)
                                    time.sleep(0.5)
                                    self.driver.changing_left(0, 90)
                                    time.sleep(3)
                                    self.driver.changing_left(90, 0)
                                    time.sleep(0.5)
                                    self.driver.changing_forward(0, 90)
                                    time.sleep(1)
                                    self.driver.changing_forward(90, 0)
                                    time.sleep(0.5)
                                    print(" 回避行動を終了しました。探索を再開します。")
                                    heading_history.clear()
                                    
                                    self.driver.cleanup()
                                    self.pi.bb_serial_read_close(self.RX_PIN)
                                    self.pi.stop()

                                    GPS_StoF = GPS(bno, goal_location = [self.F_lat, self.F_lon])
                                    GPS_StoF.run()
                                    

                             # ===== スタック判定処理ここまで =====
                            
                            time.sleep(0.5) #7/16追加
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
                            self.driver.petit_left(0, self.turn_speed)
                            self.driver.petit_left(self.turn_speed, 0)
                            self.driver.motor_stop_brake()
                            time.sleep(1.0)
                        elif target_flag['location'] == '右':
                            self.driver.petit_right(0, self.turn_speed)
                            self.driver.petit_right(self.turn_speed, 0)
                            self.driver.motor_stop_brake()
                            time.sleep(1.0)
                        
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
                        flag_area = cv2.contourArea(target_flag['flag_contour'])
                        area_percent = (flag_area / self.screen_area) * 100
                        print(f"中央に補足。接近中... (画面占有率: {area_percent:.1f}%)")

                        # 面積の比較
                        if area_percent >= self.AREA_THRESHOLD_PERCENT:
                            print(f"[{target_name}] に接近完了！")

                            """
                            if self.detector.last_image is not None:
                            # 1. 保存先のフォルダ名を決める
                                save_folder = '/home/EM/saved_images'
                                
                                # 2. フォルダが存在しなければ作成する
                                os.makedirs(save_folder, exist_ok=True)
    
                                # 3. ファイル名とフォルダパスを結合する
                                base_filename = f"success_{target_name}_{int(time.time())}.png"
                                full_path = os.path.join(save_folder, base_filename)
                                
                                # OpenCVはBGR形式で画像を保存するため、RGBから変換する
                                image_to_save = cv2.cvtColor(self.detector.last_image, cv2.COLOR_RGB2BGR)
                                
                                # フルパスを指定して保存
                                cv2.imwrite(full_path, image_to_save)
                                print(f"✅ 検出画像を {full_path} として保存しました。")
                                """
                            
                            task_completed = True
                            time.sleep(1)
                            break # 追跡ループを抜ける
                        else:
                            # しきい値未満なら、前進
                            self.driver.petit_petit(2)
                    
                    # 動作後に再検出（正しい位置）
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
        #cv2.destroyAllWindows()

# メインの実行ブロック
if __name__ == '__main__':
    # --- 各クラスのインスタンスを作成 ---
    # 1. モータードライバの初期化
    """
    driver = MotorDriver(PWMA=12, AIN1=23, AIN2=18,   # 左モーター用（モータA）
    PWMB=19, BIN1=16, BIN2=26,   # 右モーター用（モータB）
    STBY=21   )
    """
    
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
        """
        driver.cleanup()
        GPIO.cleanup()
        exit()
        """

    # --- メインクラスのインスタンスを作成し、実行 ---
    # 3. FlagNavigatorにdriverとbnoを渡してインスタンス化
    flag_location = None
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
