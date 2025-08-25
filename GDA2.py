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
        try:
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
                    self.driver.petit_right(0, 60)
                    self.driver.petit_right(60, 0)
                    self.driver.motor_stop_brake()
                    time.sleep(1.0)
                else:
                    self.driver.petit_left(0, 60)
                    self.driver.petit_left(60, 0)
                    self.driver.motor_stop_brake()
                    time.sleep(1.0)
                
                time.sleep(0.05) # 制御を安定させるために少し待

    def perform_360_degree(self, target_heading, speed):
        best_percentage = 0.0
        search_speed = 60
        while True:
            current_heading = self.bno.get_heading()
            angle_diff = (current_heading - start_heading + 360) % 360
            if angle_diff >= 350:
                break
            frame = self.picam2.capture_array()
            current_percentage = self.get_percentage(frame)
            if current_percentage > best_percentage:
                best_percentage = current_percentage
                best_heading = current_heading
                print(f"[探索中] 新しい最高の赤割合: {best_percentage:.2f}% @ 方位: {best_heading:.2f}°")
                
        self.driver.petit_right(0, search_speed)
        self.driver.petit_right(search_speed, 0)
        self.driver.motor_stop_brake()
        time.sleep(1.0)
        # BNO055の計測値に基づき、360度回転したかを判断するロジック
        start_heading = self.bno.get_heading()
        start_heading = self.bno.get_heading()
        print(f"360度探索完了。最高赤割合: {best_percentage:.2f}% @ 方位: {best_heading:.2f}°")
        
        if best_percentage > 1: # わずかでも検出できていれば方位を返す
            return best_heading
        else:
            return None # コーンが見つからなかった場合はNoneを返す

    
    def rotate_search_red_ball(self):
        try:
            scan_data = []
            self.driver.petit_right(0, 60)
            self.driver.petit_right(60, 0)
            self.driver.motor_stop_brake()
            time.sleep(1.0)
            start_heading = self.bno.get_heading()
            while True:
                current_heading = self.bno.get_heading()
                angle_diff = (current_heading - start_heading + 360) % 360
                if angle_diff >= 350:
                    break
                frame = self.picam2.capture_array()
                current_percentage_scan = self.get_percentage(frame)
                current_heading_scan = self.bno.get_heading()
                scan_data.append({'percentage': current_percentage_scan, 'heading': current_heading_scan})
            self.driver.motor_stop_brake()
            return scan_data

    
    def run(self):
        try:
            current_state = "SEARCH"
            best_heading = None
            scan_data = []
    
            while True:
                # --- フェーズ1: 探索 ---
                if current_state == "SEARCH":
                    print("\n[状態: 探索] 赤コーンを探索します。")
                    best_heading = self.perform_360_degree()
                    
                    if best_heading is not None:
                        print(f"赤コーンが見つかりました。追従モードに移行します。")
                        self.turn_to_heading(best_heading, 70) # 見つけた方向へ向きを調整
                        current_state = "FOLLOW"
                    else:
                        print("コーンが見つかりませんでした。とりあえず前に進みます。")
                        self.driver.petit_petit(5)
                        self.driver.motor_stop_brake()
                        time.sleep(0.2)
                # --- フェーズ2: 追従 ---
                elif current_state == "FOLLOW":
                    print("\n[状態: 追従] 赤コーンに向かって前進します。")
                    frame = self.picam2.capture_array()
                    current_percentage = self.get_percentage(frame)
                    
                    if current_percentage >= 15:
                        print("赤割合が15%に達しました。2個目のボール探索に移行します。")
                        current_state = "2ndBall"
                        self.driver.motor_stop_brake()
                        time.sleep(1.0)
                    elif current_percentage < 1:
                        print("コーンを見失いました。探索モードに戻ります。")
                        current_state = "SEARCH"
                        self.driver.motor_stop_brake()
                    else:
                        print(f"コーンを追従中...現在の赤割合: {current_percentage:.2f}%")
                        self.driver.petit_petit(5)
                        self.driver.motor_stop_brake()
                        time.sleep(0.2)
    
                elif current_state == "2ndBall":
                   print("360度回転して2個目のボールを探して前進します。")
                   scan_data =self.rotate_search_red_ball()
                   # 赤色の割合が5%から10%の間にあるコーンを探す
                   found_2nd_ball = None
                   for data in scan_data:
                       if 5 <= data['percentage'] < 10:
                           found_2nd_ball = data
                           break # 最初のコーンを見つけたらループを抜ける
                           
                   if found_2nd_ball:
                       target_heading = found_2nd_ball['heading']
                       print(f"2つ目のボールを方位 {target_heading:.2f}° で検知しました。")
                       self.turn_to_heading(target_heading, 70)
                       print("赤割合が10%になるまで前進します。")
                       while True:
                           frame = self.picam2.capture_array()
                           current_percentage = self.get_percentage(frame)
                           if current_percentage >= 10:
                               print("赤割合が10%に達しました。前進を停止し、最終ゴール判定に移行します。")
                               self.driver.motor_stop_brake()
                               time.sleep(0.5)
                               current_state = "GOAL_CHECK"
                               break # 前進ループを抜ける
                           elif current_percentage < 2:
                               print("2つ目のボールを見失いました。再度探索します。")
                               self.driver.motor_stop_brake()
                               time.sleep(0.5)
                               current_state = "SEARCH"
                               break # 前進ループを抜けて、外側のwhileループに戻る
                           else:
                               # 前進を続ける
                               self.driver.petit_petit(5)
                               self.driver.motor_stop_brake()
                               time.sleep(0.2)
                   else:
                       print("2つ目のボールが見つかりませんでした。探索モードに戻ります。")
                       current_state = "SEARCH"
                        
    
                elif current_state == "GOAL_CHECK":
                    print("\n[状態: ゴール判定] 最終判定のための360度スキャンを開始します。")
                    
                    scan_data = self.rotate_search_red_ball()
                    
                    high_red_count = len([d for d in scan_data if d['percentage'] > 15])
                    
                    if high_red_count >= 4:
                        print("ゴール条件を満たしました！")
                        self.driver.motor_stop_brake()
                        time.sleep(2)
                        break # ゴール確定でループ終了
                    elif high_red_count >= 2:
                        print("ボールの間に進む必要があります。")
                        # 中間点計算ロジック（元のコードから流用）
                        high_detections_with_headings = [d for d in scan_data if d['percentage'] > 15]
                        high_detections_with_headings.sort(key=lambda x: x['percentage'], reverse=True)
                        if len(high_detections_with_headings) >= 2:
                            heading3 = high_detections_with_headings[0]['heading']
                            heading4 = high_detections_with_headings[1]['heading']
                            angle_diff = (heading4 - heading3 + 360) % 360
                            target_heading = (heading3 + angle_diff / 2) % 360 if angle_diff <= 180 else (heading3 + (angle_diff - 360) / 2) % 360
                            if target_heading < 0: target_heading += 360
                            print(f"中間方位 ({target_heading:.2f}°) に向かって前進します。")
                            self.turn_to_heading(target_heading, 70)
                            self.driver.petit_petit(2)
                            self.driver.motor_stop_brake()
                            time.sleep(0.5)
                            current_state = "GOAL_CHECK" # 再度ゴールチェック
                    else:
                        print("ゴールと判断できませんでした。追従モードに戻ります。")
                        current_state = "GOAL_CHECK" # 追従に戻る
                        
        finally:
            self.picam2.close()
            self.driver.cleanup()
            print("\nプログラムを終了します。")
