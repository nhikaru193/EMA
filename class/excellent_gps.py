import math
import time
import serial # IM920通信用ですが、このコードでは直接使われていないためコメントアウト
import pigpio
import RPi.GPIO as GPIO
from motor import MotorDriver      # ユーザーのMotorDriverクラスを使用
from BNO055 import BNO055
import smbus # BME280用ですが、このコードでは直接使われていないためコメントアウト
import struct # このコードでは直接使われていないためコメントアウト
import following # 別のファイルに定義された方向追従制御関数 (PD制御ロジックを内包)

class RoverGPSNavigator:
    """
    GPSとBNO055 IMUを使用して、指定された目標地点へローバーをナビゲートするクラス。
    方向調整と前進のフェーズを繰り返しながら目標に接近します。
    """

    # === 制御パラメータ (クラス定数として定義) ===
    # GPSピン設定
    RX_PIN = 17
    GPS_BAUD = 9600

    # モータードライバーピン
    PWMA = 12
    AIN1 = 23
    AIN2 = 18
    PWMB = 19
    BIN1 = 16
    BIN2 = 26
    STBY = 21

    # BNO055 IMU
    BNO055_ADDRESS = 0x28

    def __init__(self, goal_location, goal_threshold_m=5.0,
                 angle_adjust_threshold_deg=20.0, turn_speed=40, move_speed=70, move_duration_s=8,
                 kp=0.50, kd=0.15):
        """
        RoverGPSNavigatorのコンストラクタです。

        Args:
            goal_location (list): 目標地点の [緯度, 経度] (例: [35.9186248, 139.9081672])。
            goal_threshold_m (float): 目標地点とみなす距離の閾値 (メートル)。
            angle_adjust_threshold_deg (float): これ以上の角度誤差があれば回頭する閾値 (度)。
            turn_speed (int): 回頭時のモーター速度 (0-100)。
            move_speed (int): 前進時の基本速度 (0-100)。
            move_duration_s (float): 一回の前進時間 (秒)。
            kp (float): PD制御の比例ゲイン。
            kd (float): PD制御の微分ゲイン。
        """
        self.GOAL_LOCATION = goal_location
        self.GOAL_THRESHOLD_M = goal_threshold_m
        self.ANGLE_ADJUST_THRESHOLD_DEG = angle_adjust_threshold_deg # クラス外から変更できるように名前変更
        self.TURN_SPEED = turn_speed
        self.MOVE_SPEED = move_speed
        self.MOVE_DURATION_S = move_duration_s

        self.Kp = kp
        self.Kd = kd

        self.driver = None
        self.pi = None
        self.bno = None

        self._initialize_hardware()

    def _initialize_hardware(self):
        """
        必要なハードウェア (モータードライバー, pigpio, BNO055) を初期化します。
        """
        # GPIO設定
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)

        # モータードライバーの初期化
        self.driver = MotorDriver(
            PWMA=self.PWMA, AIN1=self.AIN1, AIN2=self.AIN2,
            PWMB=self.PWMB, BIN1=self.BIN1, BIN2=self.BIN2,
            STBY=self.STBY
        )

        # pigpio 初期化
        self.pi = pigpio.pi()
        if not self.pi.connected:
            print("🔴 pigpio デーモンに接続できません。'sudo pigpiod' を実行してください。")
            self.cleanup() # 失敗時はクリーンアップ
            exit(1)

        # ソフトUART RXの設定 (GPS用)
        err = self.pi.bb_serial_read_open(self.RX_PIN, self.GPS_BAUD, 8)
        if err != 0:
            print(f"🔴 ソフトUART RX の設定に失敗：GPIO={self.RX_PIN}, {self.GPS_BAUD}bps, エラーコード: {err}")
            self.cleanup() # 失敗時はクリーンアップ
            exit(1)
        print(f"▶ ソフトUART RX を開始：GPIO={self.RX_PIN}, {self.GPS_BAUD}bps")

        # BNO055 初期化
        self.bno = BNO055(address=self.BNO055_ADDRESS) # addressを明示的に指定
        if not self.bno.begin():
            print("🔴 BNO055の初期化に失敗しました。")
            self.cleanup() # 失敗時はクリーンアップ
            exit(1)
        time.sleep(1) # センサー安定化のための待機
        self.bno.setExternalCrystalUse(True)
        self.bno.setMode(BNO055.OPERATION_MODE_NDOF)
        time.sleep(1) # モード設定後の待機
        print("✅ センサー類の初期化完了。")

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
        """GPSデータから現在の緯度と経度を取得します。
        タイムアウトした場合、Noneを返します。
        """
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
                    print(f"警告: GPSデコードエラー: {e}")
            time.sleep(0.01) # 短い待機でCPU負荷軽減
        print("[WARN] GPS位置情報を取得できませんでした (タイムアウト)。")
        return None

    def _get_current_bno_heading(self):
        """BNO055から現在の方位角（ヘディング）を取得します。
        Noneが返される場合、短い時間待機して再試行します。
        """
        heading = self.bno.getVector(BNO055.VECTOR_EULER)[0]
        if heading is None:
            wait_start_time = time.time()
            max_wait_time = 0.5 # 0.5秒まで待機
            while heading is None and (time.time() - wait_start_time < max_wait_time):
                time.sleep(0.01) # 10ミリ秒待機
                heading = self.bno.getVector(BNO055.VECTOR_EULER)[0]
        if heading is None:
            print("[WARN] BNO055から方位角を取得できませんでした (タイムアウト)。")
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

    def _wait_for_bno055_calibration(self):
        """BNO055センサーの完全キャリブレーションを待機します。"""
        print("BNO055のキャリブレーション待機中...")
        calibration_start_time = time.time()
        while True:
            sys_cal, gyro_cal, accel_cal, mag_cal = self.bno.getCalibration()
            print(f"Calib → Sys:{sys_cal}, Gyro:{gyro_cal}, Acc:{accel_cal}, Mag:{mag_cal}", end='\r')
            # ジャイロ、地磁気がレベル3になればOKとする（または全て3にする）
            if gyro_cal == 3 and mag_cal == 3: # accel_cal == 3 も追加するとより厳密
                print("\n✅ キャリブレーション完了！ナビゲーションを開始します。")
                break
            time.sleep(0.5) # 0.5秒ごとに状態を確認
        print(f"キャリブレーションにかかった時間: {time.time() - calibration_start_time:.1f}秒\n")

    def navigate_to_goal(self):
        """
        ローバーをGPS目標地点まで自律的にナビゲートするメインループです。
        """
        try:
            # BNO055キャリブレーション待機
            self._wait_for_bno055_calibration()

            print(f"🚀 ナビゲーション開始！目標: {self.GOAL_LOCATION} ({self.GOAL_THRESHOLD_M:.1f}m以内)")

            while True:
                # 1. 現在地の取得と状態把握
                current_location = self._get_current_gps_location()
                if current_location is None:
                    self.driver.motor_stop_brake() # GPS取れない間は停止
                    time.sleep(1)
                    continue

                current_heading = self._get_current_bno_heading()
                if current_heading is None:
                    self.driver.motor_stop_brake() # BNO取れない間は停止
                    time.sleep(1)
                    continue

                # 2. 目標までの距離と方位を計算
                dist_to_goal = self._get_distance_to_goal(current_location, self.GOAL_LOCATION)
                bearing_to_goal = self._get_bearing_to_goal(current_location, self.GOAL_LOCATION)

                # 目標方位と現在方位の誤差を計算 (±180度の範囲)
                angle_error = (bearing_to_goal - current_heading + 180 + 360) % 360 - 180

                print(f"[INFO] 距離:{dist_to_goal: >6.1f}m | 目標方位:{bearing_to_goal: >5.1f}° | 現在方位:{current_heading: >5.1f}° | 誤差:{angle_error: >5.1f}°")

                # 3. ゴール判定
                if dist_to_goal <= self.GOAL_THRESHOLD_M:
                    print(f"\n🎉 目標地点に到達しました！ (距離: {dist_to_goal:.2f}m)")
                    self.driver.motor_stop_free()
                    break # ループ終了

                # 4. 方向調整フェーズ (角度誤差が大きい場合のみ回頭)
                # 誤差の絶対値が閾値より大きい場合に回頭
                if abs(angle_error) > self.ANGLE_ADJUST_THRESHOLD_DEG:
                    turn_duration = 0.15 + (min(abs(angle_error), 360 - abs(angle_error)) / 180.0) * 0.2
                    
                    if angle_error < 0: # ターゲットが現在より小さい場合（左に回る方が近い）
                        print(f"[TURN] 左に回頭します (誤差: {angle_error:.1f}°, 時間: {turn_duration:.2f}秒)")
                        self.driver.petit_left(0, self.TURN_SPEED)
                        self.driver.petit_left(self.TURN_SPEED, 0)
                    else: # ターゲットが現在より大きい場合（右に回る方が近い）
                        print(f"[TURN] 右に回頭します (誤差: {angle_error:.1f}°, 時間: {turn_duration:.2f}秒)")
                        self.driver.petit_right(0, self.TURN_SPEED)
                        self.driver.petit_right(self.TURN_SPEED, 0)
                    
                    time.sleep(turn_duration)
                    self.driver.motor_stop_brake() # 確実な停止
                    time.sleep(0.5) # 回転後の安定待ち
                    continue # 方向調整が終わったら、次のループで再度GPSと方位を確認

                # 5. 前進フェーズ (PD制御による直進維持)
                print(f"[MOVE] 方向OK。PD制御で前進します。")
                # `following.follow_forward` は外部モジュールなので、driverとbnoを渡す
                # Kp, Kdはfollowingモジュール内部で使われていると想定される
                following.follow_forward(self.driver, self.bno, self.MOVE_SPEED, self.MOVE_DURATION_S)
                self.driver.motor_stop_brake() # 前進後確実に停止
                time.sleep(0.5) # 次のサイクルまでの待機

        except KeyboardInterrupt:
            print("\n[STOP] 手動で停止されました。")
        except Exception as e:
            print(f"\n[FATAL] 予期せぬエラーが発生しました: {e}")
        finally:
            self.cleanup()

    def cleanup(self):
        """
        プログラム終了時に使用したリソースを解放します。
        """
        print("クリーンアップ処理を実行します。")
        if self.driver:
            self.driver.cleanup() # MotorDriverのクリーンアップメソッドを呼び出す
        if self.pi and self.pi.connected:
            self.pi.bb_serial_read_close(self.RX_PIN)
            self.pi.stop()
        # BNO055は明示的なクローズは不要な場合が多いですが、必要なら追加
        # if self.bno:
        #     self.bno.end() # BNO055ライブラリに終了メソッドがあれば
        GPIO.cleanup() # GPIOピンの状態をリセット
        print("プログラムを終了しました。")


# === プログラム実行エントリポイント ===
if __name__ == "__main__":
    # === 制御パラメータの設定 ===
    # 12号館前の座標例
    goal_coords = [35.9186248, 139.9081672]
    
    # RoverGPSNavigatorのインスタンスを作成
    # ここで全てのパラメータを調整できます
    navigator = RoverGPSNavigator(
        goal_location=goal_coords,
        goal_threshold_m=5.0,           # ゴールとみなす距離 (メートル)
        angle_adjust_threshold_deg=15.0, # これ以上の角度誤差があれば回頭する (度)
        turn_speed=45,                  # 回頭時のモーター速度 (0-100)
        move_speed=80,                  # 前進時の基本速度 (0-100)
        move_duration_s=1.5,            # 一回の前進時間 (秒)
        # PD制御のKp, Kdはこのクラスでは直接使わないが、コンストラクタで受け取って
        # `following.py` に渡す想定があるなら残す
        # Kp=0.50, # followingモジュール内で直接定義されている場合が多い
        # Kd=0.15  # followingモジュール内で直接定義されている場合が多い
    )

    # ナビゲーションを開始
    navigator.navigate_to_goal()
