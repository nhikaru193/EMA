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

class Amaging_GPS:
    def __init__(
        self,
        bno: BNO055,
        goal_location: list,
        GOAL_THRESHOLD_M: float = 5.0,
        ANGLE_THRESHOLD_DEG: float = 15.0,
    ):
        self.GOAL_LOCATION     = goal_location
        self.GOAL_THRESHOLD_M  = GOAL_THRESHOLD_M
        self.ANGLE_THRESHOLD_DEG   = ANGLE_THRESHOLD_DEG
        self.driver = MotorDriver(
            PWMA=12, AIN1=23, AIN2=18,
            PWMB=19, BIN1=16, BIN2=26,
            STBY=21
        )
        self.bno    = bno
        self.RX_PIN = 17
        self.BAUD = 9600
        self.turn_speed = 90
        self.pi = pigpio.pi()
        if not self.pi.connected:
            raise RuntimeError("pigpio デーモンに接続できません。sudo pigpiod を起動してください。")
        err = self.pi.bb_serial_read_open(self.RX_PIN, self.BAUD, 8)
        if err != 0:
            self.pi.stop()
            raise RuntimeError(f"ソフトUART RX 設定失敗: GPIO={self.RX_PIN}, {self.BAUD}bps")
        
        
        
    def convert_to_decimal(self, coord, direction):
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

    def get_bearing_to_goal(self, current, goal):
        if current is None or goal is None: return None
        lat1, lon1 = math.radians(current[0]), math.radians(current[1])
        lat2, lon2 = math.radians(goal[0]), math.radians(goal[1])
        delta_lon = lon2 - lon1
        y = math.sin(delta_lon) * math.cos(lat2)
        x = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(delta_lon)
        bearing_rad = math.atan2(y, x)
        return (math.degrees(bearing_rad) + 360) % 360

    def get_distance_to_goal(self, current, goal):
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

    def run(self):
        try:
            while True:
                # 1. 状態把握
                (count, data) = self.pi.bb_serial_read(self.RX_PIN)
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
                                        lat = self.convert_to_decimal(parts[3], parts[4])
                                        lon = self.convert_to_decimal(parts[5], parts[6])
                                        current_location = [lat, lon]
                                        break
                    except Exception as e:
                        print(f"GPSデコードエラー: {e}")
                
                if not current_location:
                    print("[WARN] GPS位置情報を取得できません。リトライします...")
                    self.driver.motor_stop_brake()
                    time.sleep(1)
                    continue
    
                heading = self.bno.getVector(BNO055.VECTOR_EULER)[0]
                if heading is None:
                    print("[WARN] BNO055から方位角を取得できません。リトライします...")
                    self.driver.motor_stop_brake()
                    time.sleep(1)
                    continue
    
                # 2. 計算
                dist_to_goal = self.get_distance_to_goal(current_location, self.GOAL_LOCATION)
                bearing_to_goal = self.get_bearing_to_goal(current_location, self.GOAL_LOCATION)
                angle_error = (bearing_to_goal - heading + 360) % 360
    
                print(f"[INFO] 距離:{dist_to_goal: >6.1f}m | 目標方位:{bearing_to_goal: >5.1f}° | 現在方位:{heading: >5.1f}°")
    
                # 3. ゴール判定
                if dist_to_goal <= self.GOAL_THRESHOLD_M:
                    print(f"[GOAL] 目標地点に到達しました！ (距離: {dist_to_goal:.2f}m)")
                    self.driver.motor_stop_free()
                    break
    
                # 4. 方向調整フェーズ
                if angle_error > self.ANGLE_THRESHOLD_DEG and angle_error < (360 - self.ANGLE_THRESHOLD_DEG):
            
                    if angle_error > 180: # 反時計回り（左）に回る方が近い
                        print(f"[TURN] 左に回頭します ")
                        self.driver.petit_left(0, self.turn_speed) 
                        self.driver.motor_stop_free()
                        
                    else: # 時計回り（右）に回る方が近い
                        print(f"[TURN] 右に回頭します")
                        self.driver.petit_right(0, self.turn_speed) 
                        self.driver.motor_stop_free()
                    
                    self.driver.motor_stop_free() # 確実な停止
                    time.sleep(0.5) # 回転後の安定待ち
                    continue # 方向調整が終わったら、次のループで再度GPSと方位を確認
    
    
                # 5. 前進フェーズ (PD制御による直進維持)
                print(f"[MOVE] 方向OK。PD制御で前進します。")
                following.follow_forward(self.driver, self.bno, 70, 8)
    
        except KeyboardInterrupt:
            print("\n[STOP] 手動で停止されました。")
        except Exception as e:
            print(f"\n[FATAL] 予期せぬエラーが発生しました: {e}")
        finally:
            print("クリーンアップ処理を実行します。")
            self.driver.cleanup()
            self.pi.bb_serial_read_close(self.RX_PIN)
            self.pi.stop()
            print("プログラムを終了しました。")
        
