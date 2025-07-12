import math
import time
import serial # GPSデータ解析のため
import pigpio
import RPi.GPIO as GPIO # GPIOクリーンアップのため
from motor import MotorDriver
from BNO055 import BNO055
import following # PD制御による直進維持

class RoverGPSNavigator:
    """
    GPSとBNO055 IMUを使用して、指定された目標地点へローバーをナビゲートするクラス。
    方向調整と前進のフェーズを繰り返しながら目標に接近します。
    """

    # --- クラス定数 ---
    # BNO055のキャリブレーション閾値（このクラスでは参照のみ）
    BNO_CALIB_GYRO_THRESHOLD = 3
    BNO_CALIB_MAG_THRESHOLD = 3
    BNO_CALIB_ACCEL_THRESHOLD = 3

    def __init__(self, driver_instance, bno_instance, pi_instance, rx_pin, gps_baud,
                 goal_location, goal_threshold_m=5.0,
                 angle_adjust_threshold_deg=15.0, turn_speed=45, move_speed=80, move_duration_s=1.5):
        """
        RoverGPSNavigatorのコンストラクタです。

        Args:
            driver_instance (MotorDriver): 既に初期化されたMotorDriverのインスタンス。
            bno_instance (BNO055): 既に初期化されたBNO055のインスタンス。
            pi_instance (pigpio.pi): 既に初期化されたpigpioのインスタンス。
            rx_pin (int): pigpioソフトウェアUARTの受信ピン番号 (GPSモジュールから)。
            gps_baud (int): GPSモジュールのボーレート。
            goal_location (list): 目標地点の [緯度, 経度] (例: [35.9186248, 139.9081672])。
            goal_threshold_m (float): 目標地点とみなす距離の閾値 (メートル)。
            angle_adjust_threshold_deg (float): これ以上の角度誤差があれば回頭する閾値 (度)。
            turn_speed (int): 回頭時のモーター速度 (0-100)。
            move_speed (int): 前進時の基本速度 (0-100)。
            move_duration_s (float): 一回の前進時間 (秒)。
        """
        self.driver = driver_instance # 外部から渡されたインスタンスを使用
        self.bno = bno_instance       # 外部から渡されたインスタンスを使用
        self.pi = pi_instance         # 外部から渡されたインスタンスを使用
        self.RX_PIN = rx_pin          # 外部から渡されたGPS RXピン
        self.GPS_BAUD = gps_baud      # 外部から渡されたGPSボーレート

        # 目標地点と制御パラメータ (動的に変更可能)
        self.GOAL_LOCATION = goal_location
        self.GOAL_THRESHOLD_M = goal_threshold_m
        self.ANGLE_ADJUST_THRESHOLD_DEG = angle_adjust_threshold_deg
        self.TURN_SPEED = turn_speed
        self.MOVE_SPEED = move_speed
        self.MOVE_DURATION_S = move_duration_s

        # GPS受信用のソフトUARTを開く
        err = self.pi.bb_serial_read_open(self.RX_PIN, self.GPS_BAUD, 8)
        if err != 0:
            print(f"🔴 RoverGPSNavigator: ソフトUART RX の設定に失敗：GPIO={self.RX_PIN}, {self.GPS_BAUD}bps, エラーコード: {err}")
            raise IOError("RoverGPSNavigator: GPS UART open failed.")
        print(f"✅ RoverGPSNavigator: ソフトUART RX を開始：GPIO={self.RX_PIN}, {self.GPS_BAUD}bps")
        print("✅ RoverGPSNavigator: インスタンス作成完了。")

    def set_goal_location(self, new_goal):
        """目標地点を変更します。"""
        self.GOAL_LOCATION = new_goal
        print(f"RoverGPSNavigator: 目標地点を {self.GOAL_LOCATION} に設定しました。")

    def set_goal_threshold(self, new_threshold):
        """ゴールとみなす距離の閾値を変更します。"""
        self.GOAL_THRESHOLD_M = new_threshold
        print(f"RoverGPSNavigator: ゴール閾値を {self.GOAL_THRESHOLD_M}m に設定しました。")

    def set_angle_adjust_threshold(self, new_threshold):
        """角度誤差許容範囲を変更します。"""
        self.ANGLE_ADJUST_THRESHOLD_DEG = new_threshold
        print(f"RoverGPSNavigator: 角度調整閾値を {self.ANGLE_ADJUST_THRESHOLD_DEG}° に設定しました。")

    def set_turn_speed(self, new_speed):
        """回頭時のモーター速度を変更します。"""
        self.TURN_SPEED = new_speed
        print(f"RoverGPSNavigator: 回頭速度を {self.TURN_SPEED} に設定しました。")

    def set_move_speed(self, new_speed):
        """前進時の基本速度を変更します。"""
        self.MOVE_SPEED = new_speed
        print(f"RoverGPSNavigator: 前進速度を {self.MOVE_SPEED} に設定しました。")

    def set_move_duration(self, new_duration):
        """一回の前進時間を変更します。"""
        self.MOVE_DURATION_S = new_duration
        print(f"RoverGPSNavigator: 一回の前進時間を {self.MOVE_DURATION_S}s に設定しました。")

    def _convert_to_decimal(self, coord, direction):
        """度分（ddmm.mmmm）形式を10進数に変換します。"""
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

    def _get_current_gps_location(self):
        """GPSデータから現在の緯度と経度を取得します。タイムアウトした場合、Noneを返します。"""
        start_time = time.time()
        timeout_duration = 5 # GPSデータ取得のタイムアウト時間
        while (time.time() - start_time) < timeout_duration:
            (count, data) = self.pi.bb_serial_read(self.RX_PIN)
            if count and data:
                try:
                    text = data.decode("ascii", errors="ignore")
                    if "$GNRMC" in text:
                        lines = text.split("\n")
                        for line in lines:
                            if line.startswith("$GNRMC"):
                                parts = line.strip().split(",")
                                if len(parts) > 6 and parts[2] == "A": # "A"はデータが有効であることを示す
                                    lat = self._convert_to_decimal(parts[3], parts[4])
                                    lon = self._convert_to_decimal(parts[5], parts[6])
                                    return [lat, lon]
                except Exception as e:
                    print(f"警告: RoverGPSNavigator: GPSデコードエラー: {e}")
            time.sleep(0.01) # 短い待機でCPU負荷軽減
        print("[WARN] RoverGPSNavigator: GPS位置情報を取得できませんでした (タイムアウト)。")
        return None

    def _get_current_bno_heading(self):
        """BNO055から現在の方位角（ヘディング）を取得します。Noneが返される場合、短い時間待機して再試行します。"""
        heading = self.bno.getVector(BNO055.VECTOR_EULER)[0]
        if heading is None:
            wait_start_time = time.time()
            max_wait_time = 0.5 # 0.5秒まで待機
            while heading is None and (time.time() - wait_start_time < max_wait_time):
                time.sleep(0.01) # 10ミリ秒待機
                heading = self.bno.getVector(BNO055.VECTOR_EULER)[0]
        if heading is None:
            print("[WARN] RoverGPSNavigator: BNO055から方位角を取得できませんでした (タイムアウト)。")
            return None
        return heading

    def _get_bearing_to_goal(self, current, goal):
        """現在の位置から目標位置への方位（度）を計算します。"""
        if current is None or goal is None: return None
        lat1, lon1 = math.radians(current[0]), math.radians(current[1])
        lat2, lon2 = math.radians(goal[0]), math.radians(goal[1])
        delta_lon = lon2 - lon1
        y = math.sin(delta_lon) * math.cos(lat2)
        x = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(delta_lon)
        bearing_rad = math.atan2(y, x)
        return (math.degrees(bearing_rad) + 360) % 360

    def _get_distance_to_goal(self, current, goal):
        """現在の位置から目標位置までの距離（メートル）を計算します (Haversine公式)。"""
        if current is None or goal is None: return float('inf')
        lat1, lon1 = math.radians(current[0]), math.radians(current[1])
        lat2, lon2 = math.radians(goal[0]), math.radians(goal[1])
        radius = 6378137.0  # 地球の平均半径 (メートル)
        dlat = lat2 - lat1
        dlon = lon2 - lon1
        a = math.sin(dlat / 2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2)**2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
        dist = radius * c
        return dist

    def navigate_to_goal(self):
        """
        ローバーをGPS目標地点まで自律的にナビゲートするメインループです。
        """
        try:
            print(f"🚀 RoverGPSNavigator: ナビゲーション開始！目標: {self.GOAL_LOCATION} ({self.GOAL_THRESHOLD_M:.1f}m以内)")

            while True:
                # 1. 現在地の取得と状態把握
                current_location = self._get_current_gps_location()
                if current_location is None:
                    print("[WARN] RoverGPSNavigator: GPS位置情報を取得できません。停止してリトライします...")
                    self.driver.motor_stop_brake()
                    time.sleep(1)
                    continue

                current_heading = self._get_current_bno_heading()
                if current_heading is None:
                    print("[WARN] RoverGPSNavigator: BNO055から方位角を取得できません。停止してリトライします...")
                    self.driver.motor_stop_brake()
                    time.sleep(1)
                    continue

                # 2. 目標までの距離と方位を計算
                dist_to_goal = self._get_distance_to_goal(current_location, self.GOAL_LOCATION)
                bearing_to_goal = self._get_bearing_to_goal(current_location, self.GOAL_LOCATION)

                # 目標方位と現在方位の誤差を計算 (±180度の範囲)
                angle_error = (bearing_to_goal - current_heading + 180 + 360) % 360 - 180

                print(f"[INFO] RoverGPSNavigator: 距離:{dist_to_goal: >6.1f}m | 目標方位:{bearing_to_goal: >5.1f}° | 現在方位:{current_heading: >5.1f}° | 誤差:{angle_error: >5.1f}°")

                # 3. ゴール判定
                if dist_to_goal <= self.GOAL_THRESHOLD_M:
                    print(f"\n🎉 RoverGPSNavigator: 目標地点に到達しました！ (距離: {dist_to_goal:.2f}m)")
                    self.driver.motor_stop_free()
                    break # ループ終了

                # 4. 方向調整フェーズ (角度誤差が大きい場合のみ回頭)
                if abs(angle_error) > self.ANGLE_ADJUST_THRESHOLD_DEG:
                    turn_duration = 0.15 + (min(abs(angle_error), 360 - abs(angle_error)) / 180.0) * 0.2
                    
                    if angle_error < 0: # ターゲットが現在より小さい場合（左に回る方が近い）
                        print(f"[TURN] RoverGPSNavigator: 左に回頭します (誤差: {angle_error:.1f}°, 時間: {turn_duration:.2f}秒)")
                        self.driver.petit_left(0, self.TURN_SPEED)
                        self.driver.petit_left(self.TURN_SPEED, 0) # 2引数バージョン
                    else: # ターゲットが現在より大きい場合（右に回る方が近い）
                        print(f"[TURN] RoverGPSNavigator: 右に回頭します (誤差: {angle_error:.1f}°, 時間: {turn_duration:.2f}秒)")
                        self.driver.petit_right(0, self.TURN_SPEED)
                        self.driver.petit_right(self.TURN_SPEED, 0) # 2引数バージョン
                    
                    time.sleep(turn_duration)
                    self.driver.motor_stop_brake() # 確実な停止
                    time.sleep(0.5) # 回転後の安定待ち
                    continue # 方向調整が終わったら、次のループで再度GPSと方位を確認

                # 5. 前進フェーズ (PD制御による直進維持)
                print(f"[MOVE] RoverGPSNavigator: 方向OK。PD制御で前進します。")
                # `following.follow_forward` は外部モジュールなので、driverとbnoを渡す
                following.follow_forward(self.driver, self.bno, self.MOVE_SPEED, self.MOVE_DURATION_S)
                self.driver.motor_stop_brake() # 前進後確実に停止
                time.sleep(0.5) # 次のサイクルまでの待機

        except KeyboardInterrupt:
            print("\n[STOP] RoverGPSNavigator: 手動で停止されました。")
        except Exception as e:
            print(f"\n[FATAL] RoverGPSNavigator: 予期せぬエラーが発生しました: {e}")
        finally:
            self.driver.motor_stop_brake() # 念のため停止
            # GPSソフトUARTクローズはcleanup_all_resourcesで行われる

    def cleanup(self):
        """RoverGPSNavigator独自のクリーンアップ処理（現在はモーター停止のみ。UARTクローズは外部で管理）"""
        if self.driver:
            self.driver.motor_stop_brake()
        print("RoverGPSNavigator: クリーンアップ完了。")
