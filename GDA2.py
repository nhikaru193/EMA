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
    
    def run(self):
        left_a = 90
        right_a = 80
        counter = self.counter_max
        percentage = 0
        try:
            heading_list = deque(maxlen=5)
            print("ゴール誘導を開始します")
            
            # --- 探索モードの関数化 ---
            def perform_360_degree_search():
                nonlocal percentage 
                print("赤コーンが近くにありません。360度回転して最も良い方向を探索します。")
                
                best_percentage = 0.0
                best_heading = None
                
                print("360度探索を開始...")
                
                red_detection_data = [] # (percentage, heading) のタプルを保存

                num_steps = 36 # 10度ずつ回ると仮定して36ステップ
                
                for i in range(num_steps + 4): 
                    self.driver.petit_right(0, 70) 
                    time.sleep(0.2) 
                    self.driver.motor_stop_brake()

                    frame = self.picam2.capture_array()
                    current_percentage = self.get_percentage(frame)
                    current_heading = self.bno.get_heading()
                    
                    red_detection_data.append((current_percentage, current_heading))

                    if current_percentage > best_percentage:
                        best_percentage = current_percentage
                        best_heading = current_heading
                        print(f"[探索中] 新しい最高の赤割合: {best_percentage:.2f}% @ 方位: {best_heading:.2f}°")
                    
                print(f"360度探索完了。最高赤割合: {best_percentage:.2f}% @ 方位: {best_heading:.2f}°")

                if best_heading is not None and best_percentage > 5: 
                    print(f"最適な方向 ({best_heading:.2f}°)に調整します。")
                    self.bno.turn_to_heading(self.driver, best_heading, 70) 
                    self.driver.motor_stop_brake()
                    time.sleep(1.0)
                    
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

                        driver.motor.petit_petit(5) 
                        self.driver.motor_stop_brake()
                        time.sleep(0.2) 
                else:
                    print("360度探索でもコーンを明確に検知できませんでした。")
                    return False 
            # --- 探索モードの関数化ここまで ---


            while True:
                # --- 新しいゴール判定ロジック (唯一のゴール判定) ---
                print("現在の位置で最終ゴール判定のための360度スキャンを開始します。")
                high_percentage_detections = [] 
                
                start_scan_heading = self.bno.get_heading()
                
                scan_steps = 36 
                high_red_count = 0 

                for _ in range(scan_steps + 4): 
                    self.driver.petit_right(0, 50) 
                    time.sleep(0.2)
                    self.driver.motor_stop_brake()

                    frame = self.picam2.capture_array()
                    current_percentage_scan = self.get_percentage(frame)
                    
                    if current_percentage_scan > 30: 
                        high_percentage_detections.append(current_percentage_scan)
                        high_red_count += 1
                    
                print(f"360度スキャン完了。高い赤色検出数: {high_red_count}個")

                if high_red_count >= 4:
                    if len(high_percentage_detections) > 1:
                        max_val = max(high_percentage_detections)
                        min_val = min(high_percentage_detections)
                        
                        if (max_val - min_val) <= 20: 
                            print("🎉 360度ゴール判定に成功しました！複数の方向で均等な高い赤色を検知。")
                            self.driver.motor_stop_brake()
                            time.sleep(2)
                            break 
                        else:
                            print(f"高い赤色検出は複数ありますが、割合のばらつきが大きすぎます (Max:{max_val:.2f}%, Min:{min_val:.2f}%).")
                    elif len(high_percentage_detections) == 1 and high_percentage_detections[0] >= 90:
                        print("🎉 360度スキャンで1つの非常に高い赤色検出をしました（単一コーンゴール）。")
                        self.driver.motor_stop_brake()
                        time.sleep(2)
                        break
                    else:
                        print("高い赤色検出は複数ありますが、検出数が不十分か、まだ十分な範囲ではありません。")
                else:
                    print("360度スキャンでは、ゴールと判断できるほどの赤色検知がありませんでした。")
                # --- 新しいゴール判定ロジックここまで ---


                if counter <= 0:
                    search_successful = perform_360_degree_search()
                    if not search_successful:
                        counter = self.counter_max
                        continue 
                    else:
                        counter = self.counter_max


                # --- 通常の追従ロジック (従来の90%ゴール判定は削除されました) ---
                frame = self.picam2.capture_array()
                time.sleep(0.2)
                percentage = self.get_percentage(frame)
                time.sleep(0.2)
                print(f"赤割合: {percentage:2f}%です ")

                # 従来の if percentage >= 90: ゴール判定は削除されました
                
                elif percentage > 15:
                    print("赤コーンを検知しました。接近します。")
                    if percentage > 40:
                        print("非常に近いので、ゆっくり前進します (petit_petit 2回)")
                        self.driver.petit_petit(2)
                    elif percentage > 20:
                        print("近いので、少し前進します (petit_petit 3回)")
                        self.driver.petit_petit(3)
                    else: 
                        print("遠いので、前進します (follow_forward)")
                        following.follow_forward(self.driver, self.bno, 70, 1)
                    counter = self.counter_max
                
                counter = counter - 1
                c_heading = self.bno.get_heading()
                heading_list.append(c_heading)
                if len(heading_list) == 5:
                    print("スタック判定を行います")
                    a = abs((heading_list[4] - heading_list[3] + 180) % 360 - 180)
                    b = abs((heading_list[3] - heading_list[2] + 180) % 360 - 180)
                    c = abs((heading_list[2] - heading_list[1] + 180) % 360 - 180)
                    if a < 1.5 and b < 1.5 and c < 1.5:
                        print("スタック判定です")
                        print("スタック離脱を行います")
                        self.driver.changing_right(0, 90)
                        time.sleep(3)
                        self.driver.changing_right(90, 0)
                        time.sleep(0.5)
                        self.driver.changing_left(0, 90)
                        time.sleep(3)
                        self.driver.changing_left(90, 0)
                        time.sleep(0.5)
                        self.driver.changing_forward(0, 90)
                        time.sleep(0.5)
                        self.driver.changing_forward(90, 0)
                        time.sleep(0.5)
                        print("スタック離脱を終了します")
                        heading_list.clear()
        finally:
            self.picam2.close()
            print("カメラを閉じました。")
            print("ゴール判定")
            self.driver.cleanup()
            print("GPIOクリーンアップが終了しました。プログラムを終了します")
