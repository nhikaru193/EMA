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
import cv2
import numpy as np
from picamera2 import Picamera2
import camera
from collections import deque
from C_excellent_GPS import GPS

class PA:
    def __init__(self, bno: BNO055, goal_location: list):
        self.driver = MotorDriver(
            PWMA=12, AIN1=23, AIN2=18,
            PWMB=19, BIN1=16, BIN2=26,
            STBY=21
        )
        self.picam2 = Picamera2()
        config = self.picam2.create_still_configuration(main={"size": (320, 480)})
        self.picam2.configure(config)
        try:
            self.picam2.start()
        except Exception as e:
            self.camera_check = False
            print("カメラの起動に失敗しました→例外処理を行います")
        else:
            self.camera_check = True
            print("カメラの起動に成功しました")

        if not self.camera_check:
            print("フラッグへのgps誘導を行います")
            e_GPS_StoF = GPS(bno, goal_location = [35.925, 139.91])
            e_GPS_StoF.run()
            print("フラッグへの誘導を完了しました。ゴールへの誘導を開始します")
            e_GPS_FtoG = GPS(bno, goal_location = [35.925, 139.91])
            e_GPS_FtoG.run()
            #ここにコードの実行を打ち切るようなコードを入れる
        self.bno = bno
        self.goal_location = goal_location
        self.ANGLE_THRESHOLD_DEG = 10.0
        #------赤色マスクの作成------#
        self.lower_red1 = np.array([0, 100, 100])
        self.upper_red1 = np.array([10, 255, 255])
        self.lower_red2 = np.array([160, 100, 100])
        self.upper_red2 = np.array([180, 255, 255])
        
        self.RX_PIN = 17
        self.BAUD = 9600
        self.pi = pigpio.pi()
        if not self.pi.connected:
            raise RuntimeError("pigpio デーモンに接続できません。sudo pigpiod を起動してください。")
        err = self.pi.bb_serial_read_open(self.RX_PIN, self.BAUD, 8)
        if err != 0:
            self.pi.stop()
            raise RuntimeError(f"ソフトUART RX 設定失敗: GPIO={self.RX_PIN}, {self.BAUD}bps")
        
    #引数:画像フレーム　返り値:画像の赤色面積の全体のピクセル数に対する割合
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
        
    #返り値:赤色面積が10%以上Trueそれ以外False
    def detective_red(self):
        frame = self.picam2.capture_array()
        percentage = self.get_percentage(frame)
        if percentage > 10:
            return True
            print("この方向にパラシュートを検知しました")
        else:
            return False
            print("この方向にパラシュートは検知できませんでした")

    #左n度回頭はdegree負の値、右はその逆
    def degree_rotation(self, degree, threshold_deg = 5, sleeping = 0.01):
        before_heading = self.bno.getVector(BNO055.VECTOR_EULER)[0]
        target_heading = (before_heading + degree) % 360
        while True:
            current_heading = self.bno.getVector(BNO055.VECTOR_EULER)[0]
            delta_heading = ((target_heading - current_heading + 180) % 360) - 180
            if abs(delta_heading) <= threshold_deg:
                break
            elif delta_heading < -1 * threshold_deg:
                self.driver.petit_left(0, 90)
                time.sleep(sleeping)
                time.sleep(0.05)
                self.driver.motor_stop_brake()
                time.sleep(0.5)
            elif delta_heading > threshold_deg:
                self.driver.petit_right(0, 99)
                time.sleep(sleeping)
                time.sleep(0.05)
                self.driver.motor_stop_brake()
                time.sleep(0.5)
                
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

    def run(self):
        try:
            print("収納ホルダから脱出します")
            while True:
                if self.detective_red() is False:
                    following.follow_forward(self.driver, self.bno, 90, 3)
                    print("脱出を終了します")
                    break
                else:
                    print("前方にパラシュートが検知できたので、回頭を行います")
                    self.driver.petit_left(0, 90) 
                    time.sleep(0.3)
                    self.driver.motor_stop_brake()

            #過去方位データ蓄積用
            heading_list = deque(maxlen=5)

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
                heading_list.append(heading) #listの末尾にタプル形式でデータ蓄積　最終項を呼び出すときは[-1]
                if heading is None:
                    print("[WARN] BNO055から方位角を取得できません。リトライします...")
                    self.driver.motor_stop_brake()
                    time.sleep(1)
                    continue

                if len(heading_list) == 5:
                    print("スタック判定を行います")
                    a = abs((heading_list[2] - heading_list[3] + 180) % 360 - 180)
                    b = abs((heading_list[3] - heading_list[4] + 180) % 360 - 180)
                    c = abs((heading_list[1] - heading_list[2] + 180) % 360 - 180)
                    if a < 3 and b < 3 and c < 3:
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
                        time.sleep(1)
                        self.driver.changing_forward(90, 0)
                        time.sleep(0.5)
                        print("スタック離脱を終了します")
                        
                # 2. 計算
                bearing_to_goal = self.get_bearing_to_goal(current_location, self.goal_location)
                angle_error = (bearing_to_goal - heading + 360) % 360
    
                print(f"目標方位:{bearing_to_goal: >5.1f}° | 現在方位:{heading: >5.1f}°")
    
                # 4. 方向調整フェーズ
                if angle_error > self.ANGLE_THRESHOLD_DEG and angle_error < (360 - self.ANGLE_THRESHOLD_DEG):
            
                    if angle_error > 180: # 反時計回り（左）に回る方が近い
                        print(f"[TURN] 左に回頭します ")
                        self.driver.petit_left(0, 90) 
                        time.sleep(0.025)
                        self.driver.motor_stop_brake()
                        
                    else: # 時計回り（右）に回る方が近い
                        print(f"[TURN] 右に回頭します")
                        self.driver.petit_right(0, 90) 
                        time.sleep(0.025)
                        self.driver.motor_stop_brake()
                
                    time.sleep(0.5) # 回転後の安定待ち
                    continue # 方向調整が終わったら、次のループで再度GPSと方位を確認
    
                else:
                    break
            #正面
            print("パラシュートの検知を行います。")
            p_front = self.detective_red()
            time.sleep(1)
            #30度左回頭
            self.degree_rotation(-30, 5)
            time.sleep(0.5)
            p_left = self.detective_red()
            time.sleep(1)
            #右60度回頭
            self.degree_rotation(60, 5)
            time.sleep(0.5)
            p_right = self.detective_red()
            time.sleep(1)
            #正面を向く
            self.degree_rotation(-30, 5)
            if p_front and p_left and p_right:
                self.degree_rotation(175, 5)
                time.sleep(0.5)
                following.follow_forward(self.driver, self.bno, 90, 3)
                self.degree_rotation(-90, 5)
                time.sleep(0.5)
                following.follow_forward(self.driver, self.bno, 90, 8)
                self.degree_rotation(-90, 5)
                time.sleep(0.5)
                following.follow_forward(self.driver, self.bno, 90, 10)
                print("パラ回避終了。GPS誘導に移ります")

            elif p_front and p_left:
                self.degree_rotation(90, 5)
                time.sleep(0.5)
                following.follow_forward(self.driver, self.bno, 90, 4)
                self.degree_rotation(-90, 5)
                time.sleep(0.5)
                following.follow_forward(self.driver, self.bno, 90, 8)
                print("パラ回避終了。GPS誘導に移ります")

            elif p_front:
                self.degree_rotation(-90, 5)
                time.sleep(0.5)
                following.follow_forward(self.driver, self.bno, 90, 4)
                self.degree_rotation(90, 5)
                time.sleep(0.5)
                following.follow_forward(self.driver, self.bno, 90, 8)
                print("パラ回避終了。GPS誘導に移ります")

            else:
                print("正面にパラシュートは検知できません")
                print("前進します")
                following.follow_forward(self.driver, self.bno, 90, 8)
                
        except KeyboardInterrupt:
            print("回避行動を中断します")

        finally:
            self.driver.cleanup()
            self.picam2.close()
            self.pi.bb_serial_read_close(self.RX_PIN)
            self.pi.stop()

if __name__ == '__main__':
    # 許容誤差を調整したい場合は、ここで値を設定できます
    # 例: detector = FlagDetector(triangle_tolerance=0.8)
    PA(bno, goal_location = [35.9175612, 139.9087922])
