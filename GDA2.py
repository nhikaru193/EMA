import cv2
import os
import csv
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
                self.driver.petit_right(0, 90)
                self.driver.petit_right(90, 0)
                self.driver.motor_stop_brake()
                time.sleep(1.0)
            else:
                self.driver.petit_left(0, 90)
                self.driver.petit_left(90, 0)
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
            if best_percentage > 0.1: # わずかでも検出できていれば方位を返
                print(f"360度スキャン完了。最も高い割合 ({best_percentage:.2f}%) を検出した方位を返します。")
                return best_heading
            else:
                return None # ボールが見つからなかった場合はNoneを返す

   
    def rotate_search_red_ball(self):
        print("\n[360度スキャン開始] 突撃のための赤いボールを探します。")
        scan_data = []
        self.driver.motor_stop_brake()
        time.sleep(1.0)
        start_heading = self.bno.get_heading()
        # 20度ずつ回転するためのループ
        for i in range(12): # 360度 / 30度 = 12回
            # 目標となる相対的な回転角度を計算
            target_heading = (start_heading + (i + 1) * 30) % 360
            print(f"[{i+1}/12] 目標方位 {target_heading:.2f}° に向かって回転中...")
            self.turn_to_heading(target_heading, speed=90)
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

    def rotate_search_red_ball2(self):
        print("\n[360度スキャン開始] ゴールのための赤いボールを探します。")
        scan_data = []
        self.driver.motor_stop_brake()
        time.sleep(1.0)
        start_heading = self.bno.get_heading()
        # 15度ずつ回転するためのループ
        for i in range(24): # 360度 / 15度 = 18回
            # 目標となる相対的な回転角度を計算
            target_heading = (start_heading + (i + 1) * 15) % 360
            print(f"[{i+1}/24] 目標方位 {target_heading:.2f}° に向かって回転中...")
            self.turn_to_heading(target_heading, speed=90)
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

    def beyblade(self):
        print("スタック判定を行います")
        self.driver.changing_right(0, 90)
        time.sleep(3)
        self.driver.changing_right(90, 0)
        time.sleep(0.5)
        self.driver.changing_left(0, 90)
        time.sleep(3)
        self.driver.changing_left(90, 0)
        time.sleep(0.5)
        print("スタック離脱を終了します")

    def run(self, timeout_seconds=1500):
        current_time_str = time.strftime("%m%d-%H%M%S") #現在時刻をファイル名に含める
        path_to = "/home/EMA/_csv"
        os.makedirs(path_to, exit_ok=True)
        filename = os.path.join(path_to, f"GDA_{current_time_str}.csv")
        f = open(filename, "w", newline='')
        writer = csv.writer(f)
        writer.writerow(["current_state", "max_angle_diff", "max_percentage"])
        try:
            current_state = "SEARCH"
            best_heading = None
            scan_data = []
            program_start_time = time.time()
            # 各状態のタイムアウト時間を設定（秒）
            timeout_search = 90
            timeout_follow = 360
            timeout_assault = 120
            timeout_goal_check = 120
            
            state_start_time = time.time()
            
            while True:
                #タイムアウト20分
                if time.time() - program_start_time > timeout_seconds:
                    print(f"\n[終了] 全体のタイムアウト ({timeout_seconds}秒) に達しました。プログラムを終了します。")
                    break

                current_time_in_state = time.time() - state_start_time
                if current_state == "SEARCH" and current_time_in_state > timeout_search:
                    print(f"\n[タイムアウト] SEARCH状態が{timeout_search}秒を超えました。スタック離脱に入るよ。")
                    self.beyblade()
                    current_state = "SEARCH"
                    state_start_time = time.time() # 新しい状態の開始時間をリセット
                    self.driver.motor_stop_brake()
                    continue
                elif current_state == "FOLLOW" and current_time_in_state > timeout_follow:
                    print(f"\n[タイムアウト] FOLLOW状態が{timeout_follow}秒を超えました。スタック離脱に入るよ。")
                    self.beyblade()
                    current_state = "FOLLOW"
                    state_start_time = time.time() # 新しい状態の開始時間をリセット
                    self.driver.motor_stop_brake()
                    continue
                elif current_state == "Assault_Double_Ball" and current_time_in_state > timeout_assault:
                    print(f"\n[タイムアウト] Assault_Double_Ball状態が{timeout_assault}秒を超えました。スタック離脱に入るよ。")
                    self.beyblade()
                    current_state = "Assault_Double_Ball"
                    state_start_time = time.time() # 新しい状態の開始時間をリセット
                    self.driver.motor_stop_brake()
                    continue
                elif current_state == "Assault_Double_Ball2" and current_time_in_state > timeout_assault:
                    print(f"\n[タイムアウト] Assault_Double_Ball2状態が{timeout_assault}秒を超えました。スタック離脱に入るよ。")
                    self.beyblade()
                    current_state = "Assault_Double_Ball2"
                    state_start_time = time.time() # 新しい状態の開始時間をリセット
                    self.driver.motor_stop_brake()
                    continue
                elif current_state == "GOAL_CHECK" and current_time_in_state > timeout_goal_check:
                    print(f"\n[タイムアウト] GOAL_CHECK状態が{timeout_goal_check}秒を超えました。スタック離脱に入るよ。")
                    self.beyblade()
                    current_state = "GOAL_CHECK"
                    state_start_time = time.time() # 新しい状態の開始時間をリセット
                    self.driver.motor_stop_brake()
                    continue
                # --- フェーズ1: 探索 ---
                if current_state == "SEARCH":
                    print("\n[状態: 探索] 赤ボールを探索します。")
                    best_heading = self.perform_360_degree()
                    
                    if best_heading is not None:
                        print(f"赤ボールが見つかりました。FOLLOWに移行します。")
                        self.turn_to_heading(best_heading, 90) # 見つけた方向へ向きを調整
                        current_state = "FOLLOW"
                        state_start_time = time.time()
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
                    
                    if 15 <= current_percentage <= 20:
                        print("赤割合が20%に達しました。突撃に移行します。")
                        current_state = "Assault_Double_Ball"
                        state_start_time = time.time()
                        self.driver.motor_stop_brake()
                        time.sleep(1.0)
                    elif current_percentage < 0.1:
                        print("ボールを見失いました。SEARCHに戻ります。")
                        current_state = "SEARCH"
                        self.driver.motor_stop_brake()

                    elif current_percentage > 30:
                        print("近づきすぎたので後退します")
                        self.driver.petit_petit_retreat(6)
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
                            self.driver.petit_petit(5)

                        self.driver.motor_stop_brake()
                        time.sleep(1.0)
                

                elif current_state == "Assault_Double_Ball":
                    print("\n[状態: 突撃] 2つのボールの間に突撃します。")
                    scan_data = self.rotate_search_red_ball()
                    max_percentage = 0
                    if scan_data:
                        max_percentage = max(d['percentage'] for d in scan_data)
                
                    # 修正：赤色の割合が45%を超えた場合の処理を追加
                    if max_percentage > 45:
                        print(f"最大赤割合が45%を超えました ({max_percentage:.2f}%)。ボールに近づきすぎたため後退します。")
                        max_detection = None
                        if scan_data:
                            max_percentage = max(d['percentage'] for d in scan_data)
                            # 最も高い割合のデータを特定
                            for d in scan_data:
                                if d['percentage'] == max_percentage:
                                    max_detection = d
                                    break
                        if max_detection:
                            target_heading = max_detection['heading']
                            print(f"最も高い割合を検知した方位 ({target_heading:.2f}°) に向いてから後退します。")
                            self.turn_to_heading(target_heading, 90)
                        self.turn_to_heading(target_heading, 90)
                        self.driver.petit_petit_retreat(6)
                        self.driver.motor_stop_brake()
                        time.sleep(1.0)
                        current_state = "Assault_Double_Ball" # 後退後に再度突撃
                        state_start_time = time.time()
                        continue # ループの先頭に戻る
                    high_detections = [d for d in scan_data if d['percentage'] > 3]
                    high_red_count = len(high_detections)
                    if high_red_count >= 2:
                        print("ボールの間に向かって前進します。")
                        # 複数のボールの平均的な中間方向を計算
                        sum_sin = 0
                        sum_cos = 0
                        for d in high_detections:
                            heading_rad = math.radians(d['heading'])
                            sum_sin += math.sin(heading_rad)
                            sum_cos += math.cos(heading_rad)
                        avg_heading_rad = math.atan2(sum_sin, sum_cos)
                        target_heading = math.degrees(avg_heading_rad)
                        if target_heading < 0:
                            target_heading += 360
                        print(f"全てのボールの中間方位 ({target_heading:.2f}°) に向かって前進します。")
                        self.turn_to_heading(target_heading, 90)
                        self.driver.petit_petit(15)
                        self.driver.motor_stop_brake()
                        time.sleep(0.5)
                        current_state = "GOAL_CHECK" # ゴールチェック行くよ
                        state_start_time = time.time()
                    else:
                        print("突撃できませんでした再度突撃を試みます。")
                        current_state = "Assault_Double_Ball"
                        state_start_time = time.time()

                elif current_state == "Assault_Double_Ball2":
                    print("\n[状態: 突撃2] 2つのボールの間に突撃2します。")
                    scan_data = self.rotate_search_red_ball()
                    max_percentage = 0
                    if scan_data:
                        max_percentage = max(d['percentage'] for d in scan_data)
                
                    # 修正：赤色の割合が45%を超えた場合の処理を追加
                    if max_percentage > 45:
                        print(f"最大赤割合が45%を超えました ({max_percentage:.2f}%)。ボールに近づきすぎたため後退します。")
                        max_detection = None
                        if scan_data:
                            max_percentage = max(d['percentage'] for d in scan_data)
                            # 最も高い割合のデータを特定
                            for d in scan_data:
                                if d['percentage'] == max_percentage:
                                    max_detection = d
                                    break
                        if max_detection:
                            target_heading = max_detection['heading']
                            print(f"最も高い割合を検知した方位 ({target_heading:.2f}°) に向いてから後退します。")
                            self.turn_to_heading(target_heading, 90)
                        self.turn_to_heading(target_heading, 90)
                        self.driver.petit_petit_retreat(3)
                        self.driver.motor_stop_brake()
                        time.sleep(1.0)
                        current_state = "Assault_Double_Ball2" # 後退後に再度突撃2
                        state_start_time = time.time()
                        continue # ループの先頭に戻る
                    high_detections = [d for d in scan_data if d['percentage'] > 3]
                    high_red_count = len(high_detections)
                    if high_red_count >= 2:
                        print("ボールの間に向かって前進します。")
                        # 複数のボールの平均的な中間方向を計算
                        sum_sin = 0
                        sum_cos = 0
                        for d in high_detections:
                            heading_rad = math.radians(d['heading'])
                            sum_sin += math.sin(heading_rad)
                            sum_cos += math.cos(heading_rad)
                        avg_heading_rad = math.atan2(sum_sin, sum_cos)
                        target_heading = math.degrees(avg_heading_rad)
                        if target_heading < 0:
                            target_heading += 360
                        print(f"全てのボールの中間方位 ({target_heading:.2f}°) に向かって前進します。")
                        self.turn_to_heading(target_heading, 90)
                        self.driver.petit_petit(7)
                        self.driver.motor_stop_brake()
                        time.sleep(0.5)
                        current_state = "GOAL_CHECK" # 再度ゴールチェック
                        state_start_time = time.time()
                    else:
                        print("突撃できませんでした再度突撃2を試みます。")
                        current_state = "Assault_Double_Ball2"
                        state_start_time = time.time()
                            
    
                elif current_state == "GOAL_CHECK":
                    print("\n[状態: ゴール判定] 最終判定のための360度スキャンを開始します。")
                    scan_data = self.rotate_search_red_ball2()
                    max_percentage = 0
                    if scan_data:
                        max_percentage = max(d['percentage'] for d in scan_data)
                
                    # 修正：赤色の割合が40%を超えた場合の処理を追加
                    if max_percentage > 40:
                        print(f"最大赤割合が40%を超えました ({max_percentage:.2f}%)。ボールに近づきすぎたため後退します。")
                        max_detection = None
                        if scan_data:
                            max_percentage = max(d['percentage'] for d in scan_data)
                            # 最も高い割合のデータを特定
                            for d in scan_data:
                                if d['percentage'] == max_percentage:
                                    max_detection = d
                                    break
                        if max_detection:
                            target_heading = max_detection['heading']
                            print(f"最も高い割合を検知した方位 ({target_heading:.2f}°) に向いてから後退します。")
                            self.turn_to_heading(target_heading, 90)
                        self.turn_to_heading(target_heading, 90)
                        self.driver.petit_petit_retreat(6)
                        self.driver.motor_stop_brake()
                        time.sleep(1.0)
                        current_state = "GOAL_CHECK" # 後退後に再度ゴールチェック
                        state_start_time = time.time()
                        continue # ループの先頭に戻る
                    high_detections = [d for d in scan_data if d['percentage'] > 15]
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
                            writer.writerow([current_state, max_angle_diff, max_percentage])
                            f.flush()
                            break # ゴール確定でループ終了
                        else:
                            print(f"検出数は満たしましたが、最大角度差が足りません ({max_angle_diff:.2f}°) 。")
                            max_detection = None
                            if high_detections:
                                max_percentage = max(d['percentage'] for d in high_detections)
                                for d in high_detections:
                                    if d['percentage'] == max_percentage:
                                        max_detection = d
                                        break
                            # 最も赤の割合が高かった方向に向いて後退
                            if max_detection:
                                target_heading = max_detection['heading']
                                print(f"最も高い割合 ({max_percentage:.2f}%) を検知した方位 ({target_heading:.2f}°) に向いて後退します。")
                                self.turn_to_heading(target_heading, 90)
                                self.driver.petit_petit_retreat(5)
                                self.driver.motor_stop_brake()
                                time.sleep(0.5)
                                current_state = "GOAL_CHECK" # 再度ゴールチェック
                                state_start_time = time.time()
                    else:
                        print("ゴールと判断できませんでした。突撃に戻ります。")
                        current_state = "Assault_Double_Ball2" # 突撃に戻る
                        state_start_time = time.time()

                writer.writerow([current_state, max_angle_diff, max_percentage])
                f.flush()
                            
                        
        finally:
            self.picam2.close()
            self.driver.cleanup()
            print("\nプログラムを終了します。")
