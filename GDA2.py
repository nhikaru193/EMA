import cv2
import numpy as np
import time
from picamera2 import Picamera2
from motor import MotorDriver
import camera
import following 
from BNO055 import BNO055 
import math
from collections import deque
import pigpio

class GDA:
    def __init__(self, bno: BNO055, counter_max: int=50):
        self.driver = MotorDriver(
            PWMA=12, AIN1=23, AIN2=18,
            PWMB=19, BIN1=16, BIN2=26,
            STBY=21
        )
        self.bno = bno
        self.picam2 = Picamera2()
        config = self.picam2.create_still_configuration(main={"size": (320, 480)})
        self.picam2.configure(config)
        self.picam2.start()
        time.sleep(1)
        self.counter_max = counter_max
        self.lower_red1 = np.array([0, 100, 100])
        self.upper_red1 = np.array([10, 255, 255])
        self.lower_red2 = np.array([160, 100, 100])
        self.upper_red2 = np.array([180, 255, 255])
        self.pi = pigpio.pi()
        self.percentage = 0
        if not self.pi.connected:
            raise RuntimeError("pigpioデーモンに接続できません。`sudo pigpiod`を実行して確認してください。")
        
    def get_percentage(self, frame):
        frame = cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)
        frame = cv2.GaussianBlur(frame, (5, 5), 0)
        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask1 = cv2.inRange(hsv, self.lower_red1, self.upper_red1)
        mask2 = cv2.inRange(hsv, self.lower_red2, self.upper_red2)
        mask = cv2.bitwise_or(mask1, mask2)
        red_area = np.count_nonzero(mask)
        total_area = frame.shape[0] * frame.shape[1]
        percentage = (red_area / total_area) * 100
        print(f"検知割合は{percentage}%です")
        return percentage

    def turn_to_heading(self, target_heading, speed): #get_headingで現在の向きを取得してから目標方位に回転させるやつ
        print(f"目標方位: {target_heading:.2f}° に向かって調整開始")
        while True:
            current_heading = self.bno.get_heading()
            
            # 角度差
            delta_heading = target_heading - current_heading
            if delta_heading > 180:
                delta_heading -= 360
            elif delta_heading < -180:
                delta_heading += 360
            
            # 許容範囲内であれば停止
            if abs(delta_heading) < 10: # 誤差10度以内
                print("目標方位に到達しました。")
                self.driver.motor_stop_brake()
                time.sleep(0.5)
                break
            
            # 向きに応じて左右に回転
            if delta_heading > 0:
                self.driver.petit_right(0, speed) # 目標が現在より右なら右へ
            else:
                self.driver.petit_left(speed, 0) # 目標が現在より左なら左へ
            
            time.sleep(0.05) # 制御を安定させるために少し待機

    def perform_360_degree_search(self):
                print("赤コーンが近くにありません。360度回転して最も良い方向を探索します。")
                
                best_percentage = 0.0
                best_heading = None
                
                print("360度探索を開始...")
                
                red_detection_data = [] # (percentage, heading) のタプルを保存

                num_steps = 36 # 10度ずつ回ると仮定して36ステップ
                
                for i in range(num_steps + 4):
                    self.driver.petit_right(0, 70)
                    self.driver.petit_right(70, 0)
                    self.driver.motor_stop_brake()
                    time.sleep(1.0)

                    frame = self.picam2.capture_array()
                    current_percentage = self.get_percentage(frame)
                    current_heading = self.bno.get_heading()
                    
                    red_detection_data.append((current_percentage, current_heading))

                    if current_percentage > best_percentage:
                        best_percentage = current_percentage
                        best_heading = current_heading
                        print(f"[探索中] 新しい最高の赤割合: {best_percentage:.2f}% @ 方位: {best_heading:.2f}°")
                    
                print(f"360度探索完了。最高赤割合: {best_percentage:.2f}% @ 方位: {best_heading:.2f}°")

                if best_heading is not None and best_percentage > 1: 
                    print(f"最適な方向 ({best_heading:.2f}°)に調整します。")
                    self.turn_to_heading(target_heading, 70)
                    
                    print("赤コーンの割合が15%になるまで前進します。")
                    
                    while True:
                        frame = self.picam2.capture_array()
                        current_percentage = self.get_percentage(frame)
                        print(f"前進中... 現在の赤割合: {current_percentage:.2f}%")
                        
                        if current_percentage >= 15:
                            print("赤割合が15%に達しました。前進を停止し、追従モードに戻ります。")
                            self.driver.motor_stop_brake()
                            time.sleep(0.5)
                            return True 
                        
                        if current_percentage < 5: 
                            print("前進中に赤コーンを見失いました。停止し、再探索します。")
                            self.driver.motor_stop_brake()
                            time.sleep(0.5)
                            return False 

                        self.driver.petit_petit(5) 
                        self.driver.motor_stop_brake()
                        time.sleep(0.2) 
                else:
                    print("360度探索でもコーンを明確に検知できませんでした。")
                    return False
    
    def run(self):
        search_successful = self.perform_360_degree_search()
        
        if not search_successful:
            print("初期探索でコーンが見つかりませんでした。プログラムを終了します。")
            self.driver.cleanup()
            return
        try:
            print("ゴール誘導を開始します")

            while True:
                print("赤色15%まで近づけたので2つのボールの間に行くぜベイベー")
                high_percentage_detections = [] 
                
                start_scan_heading = self.bno.get_heading()
                
                scan_steps = 36 
                high_red_count = 0 

                for _ in range(scan_steps + 4): 
                    self.driver.petit_right(0, 70)
                    self.driver.petit_right(70, 0)
                    self.driver.motor_stop_brake()
                    time.sleep(1.0)

                    frame = self.picam2.capture_array()
                    current_percentage_scan = self.get_percentage(frame)
                    
                    if current_percentage_scan >= 5 and current_percentage_scan < 15:
                        self.turn_to_heading(target_heading, 70)
                        if current_percentage >= 10:
                            print("赤割合が10%に達しました。前進を停止するよ。")
                            self.driver.motor_stop_brake()
                            time.sleep(0.5)
                            return True
                        self.driver.petit_petit(5) 
                        self.driver.motor_stop_brake()
                        time.sleep(0.2)


            while True:
                # --- 新しいゴール判定ロジック (唯一のゴール判定) ---
                print("現在の位置で最終ゴール判定のための360度スキャン兼4つのボールの中に入ります。")
                high_percentage_detections = [] 
                
                start_scan_heading = self.bno.get_heading()
                
                scan_steps = 36 
                high_red_count = 0 

                for _ in range(scan_steps + 4): 
                    self.driver.petit_right(0, 70)
                    self.driver.petit_right(70, 0)
                    self.driver.motor_stop_brake()
                    time.sleep(1.0)

                    frame = self.picam2.capture_array()
                    current_percentage_scan = self.get_percentage(frame)
                    
                    if current_percentage_scan > 15: 
                        high_percentage_detections.append(current_percentage_scan)
                        high_red_count += 1
                    
                print(f"360度スキャン完了。高い赤色検出数: {high_red_count}個")

                if high_red_count >= 4:
                    if len(high_percentage_detections) > 1:
                        max_val = max(high_percentage_detections)
                        min_val = min(high_percentage_detections)
                        
                        if (max_val - min_val) <= 10: 
                            print("🎉 360度ゴール判定に成功しました！複数の方向で均等な高い赤色を検知。")
                            self.driver.motor_stop_brake()
                            time.sleep(2)
                            break 
                        else:
                            print(f"高い赤色検出は複数ありますが、割合のばらつきが大きすぎます (Max:{max_val:.2f}%, Min:{min_val:.2f}%).")
                            low_detections_with_headings = [d for d in scan_data if d['percentage'] > 15]

                    # 割合が最も低い2つの方位を見つける
                            low_detections_with_headings.sort(key=lambda x: x['percentage'])
                            if len(low_detections_with_headings) >= 2:
                                heading1 = low_detections_with_headings[0]['heading']
                                heading2 = low_detections_with_headings[1]['heading']
                                angle_diff = (heading2 - heading1 + 360) % 360
                                if angle_diff > 180:
                                    target_heading = (heading1 + (angle_diff - 360) / 2) % 360 #逆方向計算
                                else:
                                    target_heading = (heading1 + angle_diff / 2) % 360 #順方向計算
                                    
                                if target_heading < 0:
                                    target_heading += 360
                                    
                                print(f"最も低い2つの赤色検知方位は {heading1:.2f}° と {heading2:.2f}° です。")
                                print(f"その中間方位 ({target_heading:.2f}°) に向かって前進します。")
                        
                            # 中間方位にロボットの向きを調整
                                self.turn_to_heading(target_heading, 70)
                            
                            # 短く前進する
                                self.driver.petit_petit(4)
                                self.driver.motor_stop_brake()
                                time.sleep(0.5)
                elif high_red_count >= 2 and high_red_count < 4:
                    print("⚠️ 赤色検知が2個以上4個未満です。ボールの間に向かって前進します。")
                    
                    # 検出された高い割合のデータだけを抽出
                    high_detections_with_headings = [d for d in scan_data if d['percentage'] > 15]

                    # 割合が最も高い2つの方位を見つける
                    high_detections_with_headings.sort(key=lambda x: x['percentage'], reverse=True)
                    if len(high_detections_with_headings) >= 2:
                        heading3 = high_detections_with_headings[0]['heading']
                        heading4 = high_detections_with_headings[1]['heading']
                        
                        # 2つの方位の中間点を計算する
                        angle_diff = (heading4 - heading3 + 360) % 360
                        if angle_diff > 180:
                            # 逆方向に計算
                            target_heading = (heading3 + (angle_diff - 360) / 2) % 360
                        else:
                            # 順方向に計算
                            target_heading = (heading3 + angle_diff / 2) % 360

                        # マイナス値になった場合の調整
                        if target_heading < 0:
                            target_heading += 360
                            
                        print(f"最も高い2つの赤色検知方位は {heading3:.2f}° と {heading4:.2f}° です。")
                        print(f"その中間方位 ({target_heading:.2f}°) に向かって前進します。")
                        
                        # 中間方位にロボットの向きを調整
                        self.turn_to_heading(target_heading, 70)
                        
                        # 短く前進する
                        self.driver.petit_petit(2)
                        self.driver.motor_stop_brake()
                        time.sleep(0.5)
                        
                        # カウンターをリセットし、次のループへ
                        print("ボールの間を前進後、再度360度ゴール判定スキャンを開始します。")
                        counter = self.counter_max
                        continue
                    else:
                        print("検知した赤色が2個未満のため、通常追従モードに戻ります。")

                else:
                    print("360度スキャンでは、ゴールと判断できるほどの赤色検知がありませんでした。")
        finally:
            self.picam2.close()
            print("カメラを閉じました。")
            print("ゴール判定")
            self.driver.cleanup()
            print("GPIOクリーンアップが終了しました。プログラムを終了します")
