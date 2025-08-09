import os
import csv
import math
import time
import serial
import pigpio
import RPi.GPIO as GPIO
from motor import MotorDriver      # ユーザーのMotorDriverクラスを使用
from BNO055 import BNO055
import smbus
import struct
import following
from collections import deque

def convert_to_decimal(coord, direction):
    if not coord: return 0.0
    if direction in ['N', 'S']:
        degrees = int(coord[:2])
        minutes = float(coord[2:])
    else:
        degrees = int(coord[:3])
        minutes = float(coord[3:])
    decimal = degrees + minutes / 60.0
    if direction in ['S', 'W']:
        decimal *= -1
    return decimal

def get_bearing_to_goal(current, goal):
    if current is None or goal is None: return None
    lat1, lon1 = math.radians(current[0]), math.radians(current[1])
    lat2, lon2 = math.radians(goal[0]), math.radians(goal[1])
    delta_lon = lon2 - lon1
    y = math.sin(delta_lon) * math.cos(lat2)
    x = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(delta_lon)
    bearing_rad = math.atan2(y, x)
    return (math.degrees(bearing_rad) + 360) % 360

def get_distance_to_goal(current, goal):
    if current is None or goal is None: return float('inf')
    lat1, lon1 = math.radians(current[0]), math.radians(current[1])
    lat2, lon2 = math.radians(goal[0]), math.radians(goal[1])
    radius = 6378137.0  # 地球の半径 (メートル)
    dlat = lat2 - lat1
    dlon = lon2 - lon1
    a = math.sin(dlat / 2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2)**2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    dist = radius * c
    return dist

def GPS_navigate(goal_location, bno, driver=None, pi=None):
    try:
        #---ここでdriverの引き渡しがあるかどうかの確認。なければ、driverを設定し直す---#
        if driver is None:
            driver = MotorDriver(
                PWMA=12, AIN1=23, AIN2=18,
                PWMB=19, BIN1=16, BIN2=26,
                STBY=21
            )
            driver_checker = 0
        else:
            driver_checker = 1
        #-------------------------------------------------------------------------#
        #---ここでpiの引き渡しがあるかどうかの確認。なければ、piを設定し直す---#
        if pi is None:
            pi = pigpio.pi()
            pi_checker = 0
        else:
            pi_checker = 1
        #----------------------------------------------------------------#
        #初期設定#
        GOAL_THRESHOLD_M: float = 3.0
        ANGLE_THRESHOLD_DEG: float = 9.0
        RX_PIN = 17
        BAUD = 9600
        turn_speed = 95
        #-------#
        current_time_str = time.strftime("%m%d-%H%M%S") #現在時刻をファイル名に含める
        path_to = "/home/EM/_csv"
        filename = os.path.join(path_to, f"stuck_GPS_NAVIGATE_{current_time_str}.csv")
        f = open(filename, "w", newline='')
        writer = csv.writer(f)
        writer.writerow(["latitude", "longitude", "heading"])
        heading_list = deque(maxlen=5)
        while True:
            # 1. 状態把握
            (count, data) = pi.bb_serial_read(RX_PIN)
            current_location = None
            
            if count and data:
                try:
                    text = data.decode("ascii", errors="ignore")
                    if "$GNRMC" in text:
                        lines = text.split("\n")
                        for line in lines:
                            if line.startswith("$GNRMC"):
                                parts = line.strip().split(",")
                                if len(parts) > 6 and parts[2] == "A":
                                    lat = convert_to_decimal(parts[3], parts[4])
                                    lon = convert_to_decimal(parts[5], parts[6])
                                    current_location = [lat, lon]
                                    break
                except Exception as e:
                    print(f"GPSデコードエラー: {e}")
            
            if not current_location:
                print("[WARN] GPS位置情報を取得できません。リトライします...")
                driver.motor_stop_brake()
                time.sleep(1)
                continue

            heading = bno.getVector(BNO055.VECTOR_EULER)[0]
            if heading is None:
                print("[WARN] BNO055から方位角を取得できません。リトライします...")
                driver.motor_stop_brake()
                time.sleep(1)
                continue

            # 2. 計算
            dist_to_goal = get_distance_to_goal(current_location, goal_location)
            bearing_to_goal = get_bearing_to_goal(current_location, goal_location)
            angle_error = (bearing_to_goal - heading + 360) % 360

            print(f"[INFO] 距離:{dist_to_goal: >6.1f}m | 目標方位:{bearing_to_goal: >5.1f}° | 現在方位:{heading: >5.1f}°")

            # 3. ゴール判定
            if dist_to_goal <= GOAL_THRESHOLD_M:
                print(f"[GOAL] 目標地点に到達しました！ (距離: {dist_to_goal:.2f}m)")
                driver.motor_stop_free()
                break

            # 4. 方向調整フェーズ
            if angle_error > ANGLE_THRESHOLD_DEG and angle_error < (360 - ANGLE_THRESHOLD_DEG):
        
                if angle_error > 180: # 反時計回り（左）に回る方が近い
                    print(f"[TURN] 左に回頭します ")
                    driver.petit_left(0, 75) 
                    driver.petit_left(75, 0) 
                    driver.motor_stop_brake()
                    time.sleep(0.3)
                    #------簡単なスタック判定の追加-------#
                    heading = bno.getVector(BNO055.VECTOR_EULER)[0]
                    heading_list.append(heading) #listの末尾にタプル形式でデータ蓄積　最終項を呼び出すときは[-1]
                    if heading is None:
                        print("[WARN] BNO055から方位角を取得できません。リトライします...")
                        driver.motor_stop_brake()
                        time.sleep(1)
                        continue
                    if len(heading_list) == 5:
                        print("スタック判定を行います")
                        a_delta = abs((heading_list[2] - heading_list[3] + 180) % 360 - 180)
                        b_delta = abs((heading_list[3] - heading_list[4] + 180) % 360 - 180)
                        c_delta = abs((heading_list[1] - heading_list[2] + 180) % 360 - 180)
                        if a_delta < 1.5 and b_delta < 1.5 and c_delta < 1.5:
                            print("スタック判定です")
                            print("スタック離脱を行います")
                            driver.changing_right(0, 90)
                            time.sleep(3)
                            driver.changing_right(90, 0)
                            time.sleep(0.5)
                            driver.changing_left(0, 90)
                            time.sleep(3)
                            driver.changing_left(90, 0)
                            time.sleep(0.5)
                            driver.changing_forward(0, 90)
                            time.sleep(1)
                            driver.changing_forward(90, 0)
                            time.sleep(0.5)
                            print("スタック離脱を終了します")
                            heading_list.clear()
                        else:
                            print("長時間のスタックはしていないため、GPS誘導を継続します")
                    #----------------------------#
                    
                else: # 時計回り（右）に回る方が近い
                    print(f"[TURN] 右に回頭します")
                    driver.petit_right(0, 95) 
                    driver.petit_right(90, 0)
                    driver.motor_stop_brake()
                    time.sleep(0.3)
                    #------簡単なスタック判定の追加-------#
                    heading = bno.getVector(BNO055.VECTOR_EULER)[0]
                    heading_list.append(heading) #listの末尾にタプル形式でデータ蓄積　最終項を呼び出すときは[-1]
                    if heading is None:
                        print("[WARN] BNO055から方位角を取得できません。リトライします...")
                        driver.motor_stop_brake()
                        time.sleep(1)
                        continue
                    if len(heading_list) == 5:
                        print("スタック判定を行います")
                        a_delta = abs((heading_list[2] - heading_list[3] + 180) % 360 - 180)
                        b_delta = abs((heading_list[3] - heading_list[4] + 180) % 360 - 180)
                        c_delta = abs((heading_list[1] - heading_list[2] + 180) % 360 - 180)
                        if a_delta < 1.5 and b_delta < 1.5 and c_delta < 1.5:
                            print("スタック判定です")
                            print("スタック離脱を行います")
                            driver.changing_right(0, 90)
                            time.sleep(3)
                            driver.changing_right(90, 0)
                            time.sleep(0.5)
                            driver.changing_left(0, 90)
                            time.sleep(3)
                            driver.changing_left(90, 0)
                            time.sleep(0.5)
                            driver.changing_forward(0, 90)
                            time.sleep(1)
                            driver.changing_forward(90, 0)
                            time.sleep(0.5)
                            print("スタック離脱を終了します")
                            heading_list.clear()
                        else:
                            print("長時間のスタックはしていないため、GPS誘導を継続します")
                    #----------------------------#
                
                driver.motor_stop_free() # 確実な停止
                time.sleep(0.5) # 回転後の安定待ち
                continue # 方向調整が終わったら、次のループで再度GPSと方位を確認

            if dist_to_goal > 15:
                # 5. 前進フェーズ (PD制御による直進維持)
                print(f"[MOVE] 方向OK。PD制御で前進します。")
                following.follow_forward(driver, bno, 70, 9)
                heading_list.clear()

            else:
                # 5. 前進フェーズ (PD制御による直進維持)
                print(f"[MOVE] 方向OK。PD制御で前進します。")
                following.follow_forward(driver, bno, 70, 3)
                heading_list.clear()

            #------csvファイルの書き込み------#
            writer.writerow([lat, lon, heading])
            f.flush()

    except KeyboardInterrupt:
        print("キーボード割り込みによって作業が中断されました")
        
        """
        if pi_checker = 0:
            pi.stop()
        if driver_checker = 0:
            driver.cleanup()
        """
        
    finally:
        print("スタック判定のgps誘導を終了しました")
        if pi_checker = 0:
            pi.stop()
        if driver_checker = 0:
            driver.cleanup()
        f.close()
