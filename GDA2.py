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
import RPi.GPIO as GPIO

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
        self.lower_red1 = np.array([0, 150, 120])
        self.upper_red1 = np.array([5, 255, 255])
        self.lower_red2 = np.array([175, 150, 120])
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
                self.driver.petit_right(0, 70)
                self.driver.petit_right(70, 0)
                self.driver.motor_stop_brake()
                time.sleep(1.0)
            else:
                self.driver.petit_left(0, 70)
                self.driver.petit_left(70, 0)
                self.driver.motor_stop_brake()
                time.sleep(1.0)
            
            time.sleep(0.05) # 制御を安定させるために少し待

    def perform_360_degree(self):
        self.driver.petit_right(0, 90)
        self.driver.petit_right(90, 0)
        self.driver.motor_stop_brake()
        time.sleep(1.0)
        start_heading = self.bno.get_heading()
        best_percentage = 0.0
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
                print(f"[探索中] 新しい最高の割合: {best_percentage:.2f}% @ 方位: {best_heading:.2f}")
            if best_percentage > 1: # わずかでも検出できていれば方位を返
                print(f"360度スキャン完了。最も高い割合 ({best_percentage:.2f}%) を検出した方位を返します。")
                return best_heading
            else:
                return None # ボールが見つからなかった場合はNoneを返す

    def perform_360_degree2(self):
        self.driver.petit_right(0, 90)
        self.driver.petit_right(90, 0)
        self.driver.motor_stop_brake()
        time.sleep(1.0)
        start_heading = self.bno.get_heading()
        best_percentage = 0.0
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
                print(f"[探索中] 新しい最高の割合: {best_percentage:.2f}% @ 方位: {best_heading:.2f}")
            if 1 < best_percentage < 10: # 1つ目を誤反応させないように範囲を決める
                print(f"360度スキャン完了。最も高い割合 ({best_percentage:.2f}%) を検出した方位を返します。")
                return best_heading
            else:
                return None # ボールが見つからなかった場合はNoneを返す

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
                self.turn_to_relative_angle(turn_angle_step, turn_speed=90, angle_tolerance_deg=15)
   

    def rotate_search_red_ball(self):
        print("\n[360度スキャン開始] 赤いボールを探します。")
        scan_data = []
        self.driver.motor_stop_brake()
        time.sleep(1.0)
        start_heading = self.bno.get_heading()
        # 20度ずつ回転するためのループ
        for i in range(18): # 360度 / 20度 = 18回
            # 目標となる相対的な回転角度を計算
            target_heading = (start_heading + (i + 1) * 20) % 360
            print(f"[{i+1}/18] 目標方位 {target_heading:.2f}° に向かって回転中...")
            self.turn_to_heading(target_heading, speed=70)
            # カメラで撮影し、赤色の割合を取得
            frame = self.picam2.capture_array()
            current_percentage = self.get_percentage(frame)
            # 検出したデータをリストに追加
            scan_data.append({
                'percentage': current_percentage,
                'heading': self.bno.get_heading()
            })
            
        self.driver.motor_stop_brake()
        print("[360度スキャン終了] データ収集完了。")

        return scan_data
    
    def run(self):
        try:
            current_state = "SEARCH"
            best_heading = None
            scan_data = []
    
            while True:
                # --- フェーズ1: 探索 ---
                if current_state == "SEARCH":
                    print("\n[状態: 探索] 赤ボールを探索します。")
                    best_heading = self.perform_360_degree()
                    
                    if best_heading is not None:
                        print(f"赤ボールが見つかりました。追従モードに移行します。")
                        self.turn_to_heading(best_heading, 70) # 見つけた方向へ向きを調整
                        current_state = "FOLLOW"
                    else:
                        print("ボールが見つかりませんでした。見つかるまで回転します。")
                        self.perform_360_degree()
                        time.sleep(0.2)
                # --- フェーズ2: 追従 ---
                elif current_state == "FOLLOW":
                    print("\n[状態: 追従] 赤ボールに向かって前進します。")
                    frame = self.picam2.capture_array()
                    frame = cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)
                    frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
                    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
                    
                    # 画像を左・中央・右の3つの領域に分割
                    height, width, _ = frame.shape
                    center_start = int(width / 3)
                    center_end = int(width * 2 / 3)
                    
                    # 各領域のHSVマスクを生成
                    mask1 = cv2.inRange(hsv, self.lower_red1, self.upper_red1)
                    mask2 = cv2.inRange(hsv, self.lower_red2, self.upper_red2)
                    full_mask = cv2.bitwise_or(mask1, mask2)
                    
                    left_mask = full_mask[:, :center_start]
                    center_mask = full_mask[:, center_start:center_end]
                    right_mask = full_mask[:, center_end:]
                    
                    # 各領域での赤色のピクセル数をカウント
                    left_red_pixels = np.count_nonzero(left_mask)
                    center_red_pixels = np.count_nonzero(center_mask)
                    right_red_pixels = np.count_nonzero(right_mask)
                    
                    # 赤いピクセルの総数を計算して割合を判断
                    total_red_pixels = np.count_nonzero(full_mask)
                    current_percentage = (total_red_pixels / (width * height)) * 100
                    
                    time.sleep(1.0)
                    
                    if 20 <= current_percentage <= 25:
                        print("赤割合が20%に達しました。2個目のボール探索に移行します。")
                        current_state = "2ndBall"
                        self.driver.motor_stop_brake()
                        time.sleep(1.0)
                    elif current_percentage < 1:
                        print("ボールを見失いました。探索モードに戻ります。")
                        current_state = "SEARCH"
                        self.driver.motor_stop_brake()

                    elif current_percentage > 30:
                        print("近づきすぎたので後退します")
                        self.driver.petit_petit_retreat(3)
                        self.driver.motor_stop_brake()
                        time.sleep(1.0)
                        
                    else:
                        print(f"ボールを追従中...現在の赤割合: {current_percentage:.2f}%")
        
                        # 3つの領域での赤色ピクセル数を比較して方向を決定
                        if left_red_pixels > center_red_pixels and left_red_pixels > right_red_pixels:
                            print("ボールが左にあります。左に旋回します。")
                            self.driver.petit_left(0, 70)
                            self.driver.petit_left(70, 0)
                            self.driver.motor_stop_brake()
                            time.sleep(1.0)
                        elif right_red_pixels > center_red_pixels and right_red_pixels > left_red_pixels:
                            print("ボールが右にあります。右に旋回します。")
                            self.driver.petit_right(0, 70)
                            self.driver.petit_right(70, 0)
                            self.driver.motor_stop_brake()
                            time.sleep(1.0)
                        else:
                            print("ボールは中央です。前進します。")
                            self.driver.petit_petit(3)
                
                        self.driver.motor_stop_brake()
                        time.sleep(1.0)
                elif current_state == "2ndBall":
                   print("360度回転して2個目のボールを探して前進します。")
                   best_heading = self.perform_360_degree2()
                    
                   if best_heading is not None:
                       print(f"赤ボールが見つかりました。追従モードに移行します。")
                       self.turn_to_heading(best_heading, 70) # 見つけた方向へ向きを調整
                       current_state = "FOLLOW2"
                   else:
                       print("ボールが見つかりませんでした。見つかるまで回転します。")
                       self.perform_360_degree()
                       time.sleep(0.2)

                elif current_state == "FOLLOW2":
                    print("\n[状態: 追従] 赤ボールに向かって前進します。")
                    frame = self.picam2.capture_array()
                    frame = cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)
                    frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
                    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
                    
                    # 画像を左・中央・右の3つの領域に分割
                    height, width, _ = frame.shape
                    center_start = int(width / 3)
                    center_end = int(width * 2 / 3)
                    
                    # 各領域のHSVマスクを生成
                    mask1 = cv2.inRange(hsv, self.lower_red1, self.upper_red1)
                    mask2 = cv2.inRange(hsv, self.lower_red2, self.upper_red2)
                    full_mask = cv2.bitwise_or(mask1, mask2)
                    
                    left_mask = full_mask[:, :center_start]
                    center_mask = full_mask[:, center_start:center_end]
                    right_mask = full_mask[:, center_end:]
                    
                    # 各領域での赤色のピクセル数をカウント
                    left_red_pixels = np.count_nonzero(left_mask)
                    center_red_pixels = np.count_nonzero(center_mask)
                    right_red_pixels = np.count_nonzero(right_mask)
                    
                    # 赤いピクセルの総数を計算して割合を判断
                    total_red_pixels = np.count_nonzero(full_mask)
                    current_percentage = (total_red_pixels / (width * height)) * 100
                    
                    time.sleep(1.0)
                    
                    if 2 < current_percentage <= 15:
                        print("赤割合が15%に達しました。ゴール検知に移るよ")
                        current_state = "GOAL_CHECK"
                        self.driver.motor_stop_brake()
                        time.sleep(1.0)
                    elif current_percentage < 1:
                        print("ボールを見失いました。探索モードに戻ります。")
                        current_state = "2ndBall"
                        self.driver.motor_stop_brake()
                    elif current_percentage > 20:
                        print("近づきすぎたので後退します")
                        self.driver.petit_petit_retreat(5)
                        self.driver.motor_stop_brake()
                        time.sleep(1.0)
                    else:
                        print(f"ボールを追従中...現在の赤割合: {current_percentage:.2f}%")
                        if left_red_pixels > center_red_pixels and left_red_pixels > right_red_pixels:
                            print("ボールが左にあります。左に旋回します。")
                            self.driver.petit_left(0, 70)
                            self.driver.petit_left(70, 0)
                            self.driver.motor_stop_brake()
                            time.sleep(1.0)
                        elif right_red_pixels > center_red_pixels and right_red_pixels > left_red_pixels:
                            print("ボールが右にあります。右に旋回します。")
                            self.driver.petit_right(0, 70)
                            self.driver.petit_right(70, 0)
                            self.driver.motor_stop_brake()
                            time.sleep(1.0)
                        else:
                            print("ボールは中央です。前進します。")
                            self.driver.petit_petit(3)
                
                        self.driver.motor_stop_brake()
                        time.sleep(1.0)
                        
    
                elif current_state == "GOAL_CHECK":
                    print("\n[状態: ゴール判定] 最終判定のための360度スキャンを開始します。")
                    scan_data = self.rotate_search_red_ball()
                    high_detections = [d for d in scan_data if d['percentage'] > 30]
                    high_red_count = len(high_detections)
                    if high_red_count >= 4:
                        # 検出された方角のリストを作成
                        high_headings = [d['heading'] for d in high_detections]
                        
                        # 角度差を計算
                        max_angle_diff = 0
                        if len(high_headings) > 1:
                            for i in range(len(high_headings)):
                                for j in range(i + 1, len(high_headings)):
                                    # 2つの角度間の最小の差を計算（0〜180度）
                                    diff = abs(high_headings[i] - high_headings[j])
                                    angle_diff = min(diff, 360 - diff)
                                    if angle_diff > max_angle_diff:
                                        max_angle_diff = angle_diff
                        
                        # 条件判定
                        if max_angle_diff >= 40:
                            print("ゴール条件を満たしました！")
                            print(f"検出数: {high_red_count}、最大角度差: {max_angle_diff:.2f}°")
                            self.driver.motor_stop_brake()
                            time.sleep(2)
                            break # ゴール確定でループ終了
                        else:
                            print(f"検出数は満たしましたが、最大角度差が足りません ({max_angle_diff:.2f}°) 。")
                            print("ボールの間に進む必要があります。")
                            # 元の中間点計算ロジック
                            high_detections.sort(key=lambda x: x['percentage'], reverse=True)
                            if len(high_detections) >= 2:
                                heading3 = high_detections[0]['heading']
                                heading4 = high_detections[1]['heading']
                                angle_diff = (heading4 - heading3 + 360) % 360
                                target_heading = (heading3 + angle_diff / 2) % 360 if angle_diff <= 180 else (heading3 + (angle_diff - 360) / 2) % 360
                                if target_heading < 0: target_heading += 360
                                current_state = "GOAL_CHECK" # 再度ゴールチェック
                    elif len(scan_data) >= 2:
                        print("ボールの間に進む必要があります。")
                        # 中間点計算ロジック（元のコードから流用）
                        scan_data.sort(key=lambda x: x['percentage'], reverse=True)
                        if len(scan_data) >= 2:
                            heading3 = scan_data[0]['heading']
                            heading4 = scan_data[1]['heading']
                            angle_diff = (heading4 - heading3 + 360) % 360
                            target_heading = (heading3 + angle_diff / 2) % 360 if angle_diff <= 180 else (heading3 + (angle_diff - 360) / 2) % 360
                            if target_heading < 0: target_heading += 360
                            print(f"中間方位 ({target_heading:.2f}°) に向かって前進します。")
                            self.turn_to_heading(target_heading, 70)
                            self.driver.petit_petit(15)
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
