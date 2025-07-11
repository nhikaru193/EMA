import RPi.GPIO as GPIO
import time
import pigpio
import board
import busio
import numpy as np
import cv2
from picamera2 import Picamera2
from libcamera import Transform # TransformはPicamera2のconfigureで使用

import sys
import os
import math

# カスタムモジュールのインポート (同じディレクトリにあることを想定)
from motor import MotorDriver
from BNO055 import BNO055 # BNO055センサーライブラリ
import following # 別のファイルに定義された方向追従制御関数

# --- BNO055用のラッパークラス (変更なし、そのまま利用) ---
class BNO055Wrapper:
    """
    BNO055センサーから方位データを取得するためのシンプルなラッパークラス。
    `adafruit_bno055.BNO055`オブジェクトをラップします。
    """
    def __init__(self, adafruit_bno055_sensor):
        self.sensor = adafruit_bno055_sensor

    def get_heading(self):
        """
        現在のヘディング（方位）を度単位で取得します。
        Noneが返される場合、短い時間待機して再試行します。
        """
        heading = self.sensor.euler[0]
        if heading is None:
            wait_start_time = time.time()
            max_wait_time = 0.5 # 0.5秒まで待機
            while heading is None and (time.time() - wait_start_time < max_wait_time):
                time.sleep(0.01) # 10ミリ秒待機
                heading = self.sensor.euler[0]
        if heading is None:
            # 最終的に取得できない場合、0.0を返す（状況により適切なデフォルト値は異なる）
            return 0.0
        return heading

class RoverNavigator:
    """
    自律型ローバーのナビゲーションと障害物回避を制御するメインクラス。
    GPS、IMU (BNO055)、カメラ (Picamera2) を統合し、モータードライバーを制御します。
    """

    # --- 定数設定 ---
    # GPS
    RX_PIN = 17 # GPSモジュールからのRXピン（pigpioのソフトUART用）
    GPS_BAUD = 9600

    # 目標座標 (Noda, Chiba, Japan)
    DESTINATION_LAT = 35.9248066
    DESTINATION_LON = 139.9112360

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

    # カメラ設定
    CAMERA_WIDTH = 640
    CAMERA_HEIGHT = 480
    CAMERA_FRAMERATE = 30
    CAMERA_ROTATION = 90 # Picamera2のTransformで画像を回転させる角度

    # 赤色検出設定
    SAVE_IMAGE_DIR = "/home/mark1/Pictures/"
    MIN_RED_PIXEL_RATIO_PER_CELL = 0.10 # グリッドセルあたりの最小赤色ピクセル比率

    # 旋回・回避設定
    ANGLE_GPS_ADJUST_THRESHOLD_DEG = 20.0 # GPS方位調整の許容誤差（度）
    ANGLE_RELATIVE_TURN_TOLERANCE_DEG = 20.0 # 相対旋回の許容誤差（度）
    TURN_SPEED = 90 # 旋回時のモーター速度
    TURN_RE_ALIGN_SPEED = 80 # 再調整時のモーター速度
    MAX_TURN_ATTEMPTS = 100 # 旋回調整の最大試行回数
    FORWARD_SPEED_DEFAULT = 90 # 通常の前進速度
    FORWARD_DURATION_DEFAULT = 5 # 通常の前進時間 (秒)

    def __init__(self):
        """
        ローバーナビゲーターのコンストラクタです。
        各種ハードウェアの初期設定を行います。
        """
        # GPIO設定（BCMモード）
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)

        # pigpioの初期化
        self.pi = pigpio.pi()
        if not self.pi.connected:
            print("🔴 pigpioデーモンに接続できません。'sudo pigpiod'で起動してください。")
            sys.exit(1) # 接続できない場合は終了

        # モータードライバーの初期化
        self.driver = MotorDriver(
            PWMA=self.PWMA, AIN1=self.AIN1, AIN2=self.AIN2,
            PWMB=self.PWMB, BIN1=self.BIN1, BIN2=self.BIN2,
            STBY=self.STBY
        )

        # BNO055 IMUの初期化
        self.bno_sensor_raw = BNO055(address=self.BNO055_ADDRESS)
        if not self.bno_sensor_raw.begin():
            print("🔴 BNO055センサーの初期化に失敗しました。終了します。")
            self.cleanup() # 失敗時はクリーンアップして終了
            sys.exit(1)
        self.bno_sensor_raw.setMode(BNO055.OPERATION_MODE_NDOF)
        self.bno_sensor_raw.setExternalCrystalUse(True)
        time.sleep(1) # センサー安定化のための待機
        self.bno_wrapper = BNO055Wrapper(self.bno_sensor_raw) # ラッパークラスでラップ

        # Picamera2の初期化
        self.picam2 = Picamera2()
        # configureでカメラ画像を90度回転させる
        self.picam2.configure(self.picam2.create_preview_configuration(
            main={"size": (self.CAMERA_WIDTH, self.CAMERA_HEIGHT)},
            controls={"FrameRate": self.CAMERA_FRAMERATE},
            transform=Transform(rotation=self.CAMERA_ROTATION) # ここで物理的なカメラの向きを補正
        ))
        self.picam2.start()
        time.sleep(2) # カメラ起動待機

        # pigpioソフトUARTの初期化 (GPS用)
        # BME280用のI2Cバスはここには直接関係ないので削除
        err = self.pi.bb_serial_read_open(self.RX_PIN, self.GPS_BAUD, 8)
        if err != 0:
            print(f"🔴 ソフトUART RX の設定に失敗：GPIO={self.RX_PIN}, {self.GPS_BAUD}bps, エラーコード: {err}")
            self.cleanup() # 失敗時はクリーンアップして終了
            sys.exit(1)
        print(f"▶ ソフトUART RX を開始：GPIO={self.RX_PIN}, {self.GPS_BAUD}bps")

        print("✅ ローバーシステム初期化完了。")

    def _convert_to_decimal(self, coord, direction):
        """NMEA形式のGPS座標を十進数に変換します。"""
        degrees = int(coord[:2]) if direction in ['N', 'S'] else int(coord[:3])
        minutes = float(coord[2:]) if direction in ['N', 'S'] else float(coord[3:])
        decimal = degrees + minutes / 60
        if direction in ['S', 'W']:
            decimal *= -1
        return decimal

    def _get_current_location(self):
        """GPSデータから現在の緯度と経度を取得します。
        タイムアウトした場合、None, Noneを返します。
        """
        timeout = time.time() + 5 # 5秒のタイムアウト
        while time.time() < timeout:
            (count, data) = self.pi.bb_serial_read(self.RX_PIN)
            if count and data:
                try:
                    text = data.decode("ascii", errors="ignore")
                    if "$GNRMC" in text:
                        for line in text.split("\n"):
                            if "$GNRMC" in line:
                                parts = line.strip().split(",")
                                if len(parts) > 6 and parts[2] == "A": # "A"はデータが有効であることを示す
                                    lat = self._convert_to_decimal(parts[3], parts[4])
                                    lon = self._convert_to_decimal(parts[5], parts[6])
                                    return lat, lon
                except Exception as e:
                    print(f"警告: GPSデータ解析エラー: {e}")
                    # 解析エラーはログに記録し、次のデータフレームを待つ
                # 短い待機でCPU負荷軽減
                time.sleep(0.01)
            time.sleep(0.1) # データがない場合も少し待機
        print("警告: GPSデータの取得に失敗しました (タイムアウト)。")
        return None, None

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
        """現在の位置から目標位置までの距離（メートル）を計算します。"""
        if current is None or goal is None: return float('inf')
        lat1, lon1 = math.radians(current[0]), math.radians(current[1])
        lat2, lon2 = math.radians(goal[0]), math.radians(goal[1])
        radius = 6378137.0 # 地球の平均半径（メートル）
        dlat = lat2 - lat1
        dlon = lon2 - lon1
        a = math.sin(dlat / 2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2)**2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
        dist = radius * c
        return dist

    def _save_image_for_debug(self, path):
        """現在のカメラフレームをBGR形式で指定されたパスに保存します。"""
        frame = self.picam2.capture_array() # Picamera2はRGB形式のNumPy配列を返す
        if frame is None:
            print("警告: 画像キャプチャ失敗：フレームがNoneです。")
            return None
        frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        cv2.imwrite(path, frame_bgr)
        print(f"デバッグ画像を保存しました: {path}")
        return frame_bgr

    def _detect_red_in_grid(self, save_filename="akairo_grid.jpg"):
        """
        カメラ画像を縦2x横3のグリッドに分割し、各セルでの赤色検出を行い、その位置情報を返します。
        画像はソフトウェア的に回転・反転処理されます。
        """
        save_path = os.path.join(self.SAVE_IMAGE_DIR, save_filename)
        
        try:
            # Picamera2はconfigureで回転を処理済みのため、ここでは追加の回転は不要
            # ただし、元のコードにあった左右反転はここで行う
            frame_rgb = self.picam2.capture_array() # Picamera2はデフォルトでRGB形式のNumPy配列を返す
            if frame_rgb is None:
                print("警告: 画像キャプチャ失敗: フレームがNoneです。")
                return 'error_in_processing'

            processed_frame_bgr = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)
            
            # ★★★ ここで画像を左右反転させる ★★★
            # Picamera2のconfigureで回転は処理済みなので、ここでは反転のみ
            processed_frame_bgr = cv2.flip(processed_frame_bgr, 1) # 1は水平フリップ (左右反転)
            
            # もし、回転後＆左右反転後にさらに上下が反転している場合は、次の行を有効にしてみてください
            # processed_frame_bgr = cv2.flip(processed_frame_bgr, 0) # 0は垂直フリップ (上下反転)

            height, width, _ = processed_frame_bgr.shape
            cell_height = height // 2
            cell_width = width // 3
            cells = {
                'top_left': (0, cell_height, 0, cell_width),
                'top_middle': (0, cell_height, cell_width, 2 * cell_width),
                'top_right': (0, cell_height, 2 * cell_width, width),
                'bottom_left': (cell_height, height, 0, cell_width),
                'bottom_middle': (cell_height, height, cell_width, 2 * cell_width),
                'bottom_right': (cell_height, height, 2 * cell_width, width),
            }
            red_counts = {key: 0 for key in cells}
            total_pixels_in_cell = {key: 0 for key in cells}

            # 赤色範囲のHSV閾値
            lower_red1 = np.array([0, 100, 100])
            upper_red1 = np.array([10, 255, 255])
            lower_red2 = np.array([160, 100, 100])
            upper_red2 = np.array([180, 255, 255])

            blurred_full_frame = cv2.GaussianBlur(processed_frame_bgr, (5, 5), 0)
            hsv_full = cv2.cvtColor(blurred_full_frame, cv2.COLOR_BGR2HSV)
            mask_full = cv2.bitwise_or(cv2.inRange(hsv_full, lower_red1, upper_red1),
                                     cv2.inRange(hsv_full, lower_red2, upper_red2))
            red_pixels_full = np.count_nonzero(mask_full)
            total_pixels_full = height * width
            red_percentage_full = red_pixels_full / total_pixels_full if total_pixels_full > 0 else 0.0

            if red_percentage_full >= 0.80:
                print(f"画像全体の赤色ピクセル割合: {red_percentage_full:.2%} (高割合) -> high_percentage_overall")
                cv2.imwrite(save_path, processed_frame_bgr) # グリッド描画なしで元のフレームを保存
                return 'high_percentage_overall'

            debug_frame = processed_frame_bgr.copy() # デバッグ用にコピー
            for cell_name, (y_start, y_end, x_start, x_end) in cells.items():
                cell_frame = processed_frame_bgr[y_start:y_end, x_start:x_end]
                blurred_cell_frame = cv2.GaussianBlur(cell_frame, (5, 5), 0)
                hsv_cell = cv2.cvtColor(blurred_cell_frame, cv2.COLOR_BGR2HSV)
                mask_cell = cv2.bitwise_or(cv2.inRange(hsv_cell, lower_red1, upper_red1),
                                           cv2.inRange(hsv_cell, lower_red2, upper_red2))
                red_counts[cell_name] = np.count_nonzero(mask_cell)
                total_pixels_in_cell[cell_name] = cell_frame.shape[0] * cell_frame.shape[1]
                
                # デバッグフレームにグリッドと検出状況を描画
                color = (255, 0, 0) # デフォルトは青
                thickness = 2
                if red_counts[cell_name] / total_pixels_in_cell[cell_name] >= self.MIN_RED_PIXEL_RATIO_PER_CELL:
                    color = (0, 0, 255) # 赤色検出で赤
                    thickness = 3
                cv2.rectangle(debug_frame, (x_start, y_start), (x_end, y_end), color, thickness)
                cv2.putText(debug_frame, f"{cell_name}: {(red_counts[cell_name] / total_pixels_in_cell[cell_name]):.2f}", 
                            (x_start + 5, y_start + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

            # デバッグ画像を保存
            directory = os.path.dirname(save_path)
            if not os.path.exists(directory):
                os.makedirs(directory)
            cv2.imwrite(save_path, debug_frame)
            print(f"グリッド検出画像を保存しました: {save_path}")

            # 下段セルの赤色検出割合を評価
            bottom_left_ratio = red_counts['bottom_left'] / total_pixels_in_cell['bottom_left']
            bottom_middle_ratio = red_counts['bottom_middle'] / total_pixels_in_cell['bottom_middle']
            bottom_right_ratio = red_counts['bottom_right'] / total_pixels_in_cell['bottom_right']

            detected_cells_bottom_row = []
            if bottom_left_ratio >= self.MIN_RED_PIXEL_RATIO_PER_CELL:
                detected_cells_bottom_row.append('bottom_left')
            if bottom_middle_ratio >= self.MIN_RED_PIXEL_RATIO_PER_CELL:
                detected_cells_bottom_row.append('bottom_middle')
            if bottom_right_ratio >= self.MIN_RED_PIXEL_RATIO_PER_CELL:
                detected_cells_bottom_row.append('bottom_right')

            # 検出結果に基づいて文字列を返す
            if len(detected_cells_bottom_row) == 0:
                print("赤色を検出しませんでした (下段)。")
                return 'none_detected'
            elif 'bottom_left' in detected_cells_bottom_row and 'bottom_right' not in detected_cells_bottom_row:
                print("赤色が左下に偏って検出されました。")
                return 'left_bottom'
            elif 'bottom_right' in detected_cells_bottom_row and 'bottom_left' not in detected_cells_bottom_row:
                print("赤色が右下に偏って検出されました。")
                return 'right_bottom'
            elif 'bottom_middle' in detected_cells_bottom_row: # 中央に検出されたら優先
                print("赤色が下段中央に検出されました。")
                return 'bottom_middle'
            else: # その他（左右両方だが中央なしなど）
                print("赤色が下段の特定の場所に検出されましたが、左右の偏りはありません。")
                return 'bottom_middle' # デフォルトとして中央とみなす

        except Exception as e:
            print(f"🔴 カメラ撮影・グリッド処理中にエラーが発生しました: {e}")
            return 'error_in_processing'

    def _turn_to_relative_angle(self, angle_offset_deg, turn_speed, angle_tolerance_deg, max_turn_attempts):
        """
        現在のBNO055の方位から、指定された角度だけ相対的に旋回します。
        """
        initial_heading = self.bno_wrapper.get_heading()
        if initial_heading is None:
            print("警告: turn_to_relative_angle: 初期方位が取得できませんでした。旋回を中止します。")
            return False
        
        # 目標絶対方位を計算 (0-360度の範囲に収める)
        target_heading = (initial_heading + angle_offset_deg + 360) % 360
        print(f"現在のBNO方位: {initial_heading:.2f}度, 相対目標角度: {angle_offset_deg:.2f}度 -> 絶対目標方位: {target_heading:.2f}度")

        loop_count = 0
        
        while loop_count < max_turn_attempts:
            current_heading = self.bno_wrapper.get_heading()
            if current_heading is None:
                print("警告: turn_to_relative_angle: 旋回中に方位が取得できませんでした。スキップします。")
                self.driver.motor_stop_brake()
                time.sleep(0.1)
                loop_count += 1
                continue

            # 角度誤差を-180度から+180度の範囲で計算
            angle_error = (target_heading - current_heading + 180 + 360) % 360 - 180

            if abs(angle_error) <= angle_tolerance_deg:
                print(f"[TURN] 相対回頭完了。最終誤差: {angle_error:.2f}度 (試行回数: {loop_count})")
                self.driver.motor_stop_brake()
                time.sleep(0.5)
                return True

            # 誤差に応じて旋回時間を動的に調整
            # 最小0.02秒 + 誤差が大きいほど長くする
            turn_duration_on = 0.02 + (abs(angle_error) / 180.0) * 0.2
            
            if angle_error < 0: # ターゲットが現在より小さい場合（左に回る必要がある）
                self.driver.petit_left(0, turn_speed) # 左旋回
                self.driver.petit_left(turn_speed, 0)
            else: # ターゲットが現在より大きい場合（右に回る必要がある）
                self.driver.petit_right(0, turn_speed) # 右旋回
                self.driver.petit_right(turn_speed, 0)
            
            time.sleep(turn_duration_on)
            self.driver.motor_stop_brake() # 短く回頭したら停止
            time.sleep(0.05) # 停止してセンサーが落ち着くのを待つ
            
            loop_count += 1
        
        print(f"警告: turn_to_relative_angle: 最大試行回数({max_turn_attempts}回)内に目標角度に到達できませんでした。最終誤差: {angle_error:.2f}度 (試行回数: {loop_count})")
        self.driver.motor_stop_brake()
        time.sleep(0.5)
        return False

    def _wait_for_bno055_calibration(self):
        """BNO055センサーの完全キャリブレーションを待機します。"""
        print("BNO055のキャリブレーション待機中...")
        calibration_start_time = time.time()
        while True:
            sys_cal, gyro_cal, accel_cal, mag_cal = self.bno_sensor_raw.getCalibration()
            print(f"Calib → Sys:{sys_cal}, Gyro:{gyro_cal}, Acc:{accel_cal}, Mag:{mag_cal} ", end='\r')
            sys.stdout.flush() # 出力を即座に反映

            # ジャイロ、地磁気だけでなく、加速度計もレベル3になるまで待機
            # 経験上、ジャイロと地磁気が3になれば加速度計もすぐに3になることが多い
            if gyro_cal == 3 and accel_cal == 3 and mag_cal == 3:
                print("\n✅ キャリブレーション完了！ナビゲーションを開始します。")
                break
            time.sleep(0.5) # 0.5秒ごとに状態を確認
        print(f"キャリブレーションにかかった時間: {time.time() - calibration_start_time:.1f}秒\n")

    def run_navigation_loop(self):
        """
        ローバーのメインナビゲーションおよび障害物回避ループを実行します。
        """
        try:
            # === BNO055キャリブレーション待機 ===
            self._wait_for_bno055_calibration()

            # メインの自律走行ループ
            while True:
                print("\n--- 新しい走行サイクル開始 ---")
                
                # STEP 2: GPS現在地取得し、目標方位計算
                print("\n=== ステップ2: GPS現在地取得と目標方位計算 ===")
                current_gps_coords = self._get_current_location()
                goal_gps_coords = (self.DESTINATION_LAT, self.DESTINATION_LON)

                if current_gps_coords[0] is None or current_gps_coords[1] is None:
                    print("警告: GPSデータが取得できませんでした。2秒待機してリトライします...")
                    time.sleep(2)
                    continue

                print(f"現在地：緯度={current_gps_coords[0]:.4f}, 経度={current_gps_coords[1]:.4f}")
                
                target_gps_heading = self._get_bearing_to_goal(current_gps_coords, goal_gps_coords)
                if target_gps_heading is None:
                    print("警告: 目標方位の計算に失敗しました。2秒待機してリトライします...")
                    time.sleep(2)
                    continue

                print(f"GPSに基づく目標方位：{target_gps_heading:.2f}度")
                
                distance_to_goal = self._get_distance_to_goal(current_gps_coords, goal_gps_coords)
                print(f"目的地までの距離：{distance_to_goal:.2f}メートル")

                # 目標距離が十分に近ければ終了
                if distance_to_goal < 1.0: # 例えば1メートル以内になったら完了とみなす
                    print("\n🎉 目的地に到達しました！ミッション完了！")
                    break # メインループを終了

                # STEP 3: その場で回頭 (動的調整)
                print("\n=== ステップ3: 目標方位への回頭 (動的調整) ===")
                turn_attempt_count = 0
                while turn_attempt_count < self.MAX_TURN_ATTEMPTS:
                    current_bno_heading = self.bno_wrapper.get_heading()
                    if current_bno_heading is None:
                        print("警告: 旋回中にBNO055方位が取得できませんでした。1秒待機してリトライします。")
                        self.driver.motor_stop_brake()
                        time.sleep(1)
                        turn_attempt_count += 1
                        continue

                    angle_error = (target_gps_heading - current_bno_heading + 180 + 360) % 360 - 180
                    
                    if abs(angle_error) <= self.ANGLE_GPS_ADJUST_THRESHOLD_DEG:
                        print(f"[TURN] 方位調整完了。最終誤差: {angle_error:.2f}度")
                        break

                    turn_duration = 0.02 + (abs(angle_error) / 180.0) * 0.2 # 誤差に応じて旋回時間を動的に調整
                    if angle_error < 0: # ターゲットが現在より小さい場合（左に回る必要がある）
                        print(f"[TURN] 左に回頭します (誤差: {angle_error:.2f}度, 時間: {turn_duration:.2f}秒)")
                        self.driver.petit_left(0, self.TURN_SPEED)
                        self.driver.petit_left(self.TURN_SPEED, 0)
                    else: # ターゲットが現在より大きい場合（右に回る必要がある）
                        print(f"[TURN] 右に回頭します (誤差: {angle_error:.2f}度, 時間: {turn_duration:.2f}秒)")
                        self.driver.petit_right(0, self.TURN_SPEED)
                        self.driver.petit_right(self.TURN_SPEED, 0)
                    
                    time.sleep(turn_duration)
                    self.driver.motor_stop_brake()
                    time.sleep(0.5) # 停止してセンサーが落ち着くのを待つ

                    turn_attempt_count += 1

                if turn_attempt_count >= self.MAX_TURN_ATTEMPTS and abs(angle_error) > self.ANGLE_GPS_ADJUST_THRESHOLD_DEG:
                    print(f"警告: 最大回頭試行回数に達しましたが、目標方位に到達できませんでした。最終誤差: {angle_error:.2f}度")
                
                self.driver.motor_stop_brake()
                time.sleep(0.5)

                # STEP 4 & 5: カメラ検知と前進
                print("\n=== ステップ4&5: カメラ検知と前進 ===")
                
                # パラシュート検出
                red_location_result = self._detect_red_in_grid(save_filename=f"detection_cycle_{int(time.time())}.jpg")

                if red_location_result == 'left_bottom':
                    print("赤色が左下に検出されました → 右に90度回頭して回避します。")
                    self._turn_to_relative_angle(90, self.TURN_SPEED, self.ANGLE_RELATIVE_TURN_TOLERANCE_DEG, self.MAX_TURN_ATTEMPTS)
                    print("回頭後、少し前進します。")
                    following.follow_forward(self.driver, self.bno_sensor_raw, base_speed=self.FORWARD_SPEED_DEFAULT, duration_time=self.FORWARD_DURATION_DEFAULT)
                elif red_location_result == 'right_bottom':
                    print("赤色が右下に検出されました → 左に90度回頭して回避します。")
                    self._turn_to_relative_angle(-90, self.TURN_SPEED, self.ANGLE_RELATIVE_TURN_TOLERANCE_DEG, self.MAX_TURN_ATTEMPTS)
                    print("回頭後、少し前進します。")
                    following.follow_forward(self.driver, self.bno_sensor_raw, base_speed=self.FORWARD_SPEED_DEFAULT, duration_time=self.FORWARD_DURATION_DEFAULT)
                elif red_location_result == 'bottom_middle':
                    print("赤色が下段中央に検出されました → 右に120度回頭して前進します。")
                    self._turn_to_relative_angle(120, self.TURN_SPEED, self.ANGLE_RELATIVE_TURN_TOLERANCE_DEG, self.MAX_TURN_ATTEMPTS)
                    print("120度回頭後、少し前進します (1回目)。")
                    following.follow_forward(self.driver, self.bno_sensor_raw, base_speed=self.FORWARD_SPEED_DEFAULT, duration_time=self.FORWARD_DURATION_DEFAULT)
                    self.driver.motor_stop_brake()
                    time.sleep(0.5)

                    print("さらに左に30度回頭し、前進します。")
                    self._turn_to_relative_angle(-30, self.TURN_SPEED, self.ANGLE_RELATIVE_TURN_TOLERANCE_DEG, self.MAX_TURN_ATTEMPTS) # 左に30度回頭
                    print("左30度回頭後、少し前進します (2回目)。")
                    following.follow_forward(self.driver, self.bno_sensor_raw, base_speed=self.FORWARD_SPEED_DEFAULT, duration_time=self.FORWARD_DURATION_DEFAULT)
                elif red_location_result == 'high_percentage_overall':
                    print("画像全体に高割合で赤色を検出 → パラシュートが覆いかぶさっている可能性。10秒待機して様子を見ます。")
                    time.sleep(10)
                    print("待機後、少し前進します。")
                    following.follow_forward(self.driver, self.bno_sensor_raw, base_speed=self.FORWARD_SPEED_DEFAULT, duration_time=3)
                elif red_location_result == 'none_detected':
                    print("赤色を検出しませんでした → GPS方向追従制御で前進します。(速度90, 5秒)")
                    following.follow_forward(self.driver, self.bno_sensor_raw, base_speed=self.FORWARD_SPEED_DEFAULT, duration_time=self.FORWARD_DURATION_DEFAULT)
                elif red_location_result == 'error_in_processing':
                    print("カメラ処理でエラーが発生しました。2秒待機してリトライします...")
                    time.sleep(2)
                    continue # メインループの先頭に戻る

                self.driver.motor_stop_brake() # 各アクション後に停止
                time.sleep(0.5)

                # ★★★ 回避後の再確認ロジック（3点スキャン） ★★★
                print("\n=== 回避後の周囲確認を開始します (3点スキャン) ===")
                avoidance_confirmed_clear = False

                # 1. ローバーを目的地のGPS方向へ再度向ける
                print("\n=== 回避後: 再度目的地の方位へ回頭 ===")
                turn_attempt_count_realign = 0
                while turn_attempt_count_realign < self.MAX_TURN_ATTEMPTS:
                    current_bno_heading = self.bno_wrapper.get_heading()
                    if current_bno_heading is None:
                        print("警告: 再調整中にBNO055方位が取得できませんでした。1秒待機してリトライします。")
                        self.driver.motor_stop_brake()
                        time.sleep(1)
                        turn_attempt_count_realign += 1
                        continue

                    angle_error = (target_gps_heading - current_bno_heading + 180 + 360) % 360 - 180
                    
                    if abs(angle_error) <= self.ANGLE_GPS_ADJUST_THRESHOLD_DEG:
                        print(f"[RE-ALIGN] GPS方向への再調整完了。最終誤差: {angle_error:.2f}度")
                        break

                    turn_duration = 0.02 + (abs(angle_error) / 180.0) * 0.2
                    if angle_error < 0:
                        print(f"[RE-ALIGN] 左に回頭します (誤差: {angle_error:.2f}度, 時間: {turn_duration:.2f}秒)")
                        self.driver.petit_left(0, self.TURN_RE_ALIGN_SPEED)
                        self.driver.petit_left(self.TURN_RE_ALIGN_SPEED, 0)
                    else:
                        print(f"[RE-ALIGN] 右に回頭します (誤差: {angle_error:.2f}度, 時間: {turn_duration:.2f}秒)")
                        self.driver.petit_right(0, self.TURN_RE_ALIGN_SPEED)
                        self.driver.petit_right(self.TURN_RE_ALIGN_SPEED, 0)
                    
                    time.sleep(turn_duration)
                    self.driver.motor_stop_brake()
                    time.sleep(0.5)
                    turn_attempt_count_realign += 1
                    
                if turn_attempt_count_realign >= self.MAX_TURN_ATTEMPTS and abs(angle_error) > self.ANGLE_GPS_ADJUST_THRESHOLD_DEG:
                    print(f"警告: 回避後の目的地方位への回頭が不十分です。最終誤差: {angle_error:.2f}度")
                self.driver.motor_stop_brake()
                time.sleep(0.5)

                # 2. 正面、左30度、右30度の3方向で赤色検知
                scan_results = {
                    'front': 'none_detected',
                    'left_30': 'none_detected',
                    'right_30': 'none_detected'
                }
                
                # 正面
                print("→ 正面方向の赤色を確認します...")
                scan_results['front'] = self._detect_red_in_grid(save_filename=f"confirm_front_{int(time.time())}.jpg")

                # 左30度
                print("→ 左に30度回頭し、赤色を確認します...")
                self._turn_to_relative_angle(-30, self.TURN_SPEED, self.ANGLE_RELATIVE_TURN_TOLERANCE_DEG, self.MAX_TURN_ATTEMPTS)
                scan_results['left_30'] = self._detect_red_in_grid(save_filename=f"confirm_left_{int(time.time())}.jpg")
                print("→ 左30度から正面に戻します...")
                self._turn_to_relative_angle(30, self.TURN_SPEED, self.ANGLE_RELATIVE_TURN_TOLERANCE_DEG, self.MAX_TURN_ATTEMPTS) # 右30度で戻す

                # 右30度
                print("→ 右に30度回頭し、赤色を確認します...")
                self._turn_to_relative_angle(30, self.TURN_SPEED, self.ANGLE_RELATIVE_TURN_TOLERANCE_DEG, self.MAX_TURN_ATTEMPTS)
                scan_results['right_30'] = self._detect_red_in_grid(save_filename=f"confirm_right_{int(time.time())}.jpg")
                print("→ 右30度から正面に戻します...")
                self._turn_to_relative_angle(-30, self.TURN_SPEED, self.ANGLE_RELATIVE_TURN_TOLERANCE_DEG, self.MAX_TURN_ATTEMPTS) # 左30度で戻す

                # 3方向の結果を評価
                is_front_clear = (scan_results['front'] == 'none_detected')
                is_left_clear = (scan_results['left_30'] == 'none_detected')
                is_right_clear = (scan_results['right_30'] == 'none_detected')

                if is_front_clear and is_left_clear and is_right_clear:
                    print("\n=== 3点スキャン結果: 全ての方向でパラシュートは検出されませんでした。回避成功、次のGPSポイントへ！ ===")
                    avoidance_confirmed_clear = True
                    # continue # 回避は成功したので、次のGPS計算サイクルへ進む
                else:
                    print("\n=== 3点スキャン結果: まだパラシュートが検出されました。再回避を試みます。 ===")
                    print(f"検出詳細: 正面: {scan_results['front']}, 左30: {scan_results['left_30']}, 右30: {scan_results['right_30']}")
                    
                    # 検出された方向に基づいて再回避行動を選択
                    if scan_results['left_30'] != 'none_detected': # 左30度で検出されたら右90度
                        print("左30度で検出されたため、右90度回頭して回避します。")
                        self._turn_to_relative_angle(90, self.TURN_SPEED, self.ANGLE_RELATIVE_TURN_TOLERANCE_DEG, self.MAX_TURN_ATTEMPTS)
                    elif scan_results['right_30'] != 'none_detected': # 右30度で検出されたら左90度
                        print("右30度で検出されたため、左90度回頭して回避します。")
                        self._turn_to_relative_angle(-90, self.TURN_SPEED, self.ANGLE_RELATIVE_TURN_TOLERANCE_DEG, self.MAX_TURN_ATTEMPTS)
                    elif scan_results['front'] != 'none_detected': # 正面で検出されたら右120度
                        print("正面で検出されたため、右120度回頭して回避します。")
                        self._turn_to_relative_angle(120, self.TURN_SPEED, self.ANGLE_RELATIVE_TURN_TOLERANCE_DEG, self.MAX_TURN_ATTEMPTS)
                        self.driver.motor_stop_brake()
                        time.sleep(0.5)

                        print("さらに左に30度回頭し、前進します。")
                        self._turn_to_relative_angle(-30, self.TURN_SPEED, self.ANGLE_RELATIVE_TURN_TOLERANCE_DEG, self.MAX_TURN_ATTEMPTS) # 左に30度回頭
                        print("左30度回頭後、少し前進します (2回目)。")
                        following.follow_forward(self.driver, self.bno_sensor_raw, base_speed=self.FORWARD_SPEED_DEFAULT, duration_time=self.FORWARD_DURATION_DEFAULT)
                    else: # その他の場合 (例えばエラーで検出された場合など、念のため)
                        print("詳細不明な検出のため、右120度回頭して回避します。")
                        self._turn_to_relative_angle(120, self.TURN_SPEED, self.ANGLE_RELATIVE_TURN_TOLERANCE_DEG, self.MAX_TURN_ATTEMPTS)
                    
                    following.follow_forward(self.driver, self.bno_sensor_raw, base_speed=self.FORWARD_SPEED_DEFAULT, duration_time=self.FORWARD_DURATION_DEFAULT) # 少し前進
                    self.driver.motor_stop_brake()
                    time.sleep(1) # 再回避後のクールダウン
                    
                    continue # メインループの先頭に戻り、GPS取得から再開

        except KeyboardInterrupt:
            print("\nユーザー割り込みで終了します。")
        except Exception as e:
            print(f"メイン処理中に予期せぬエラーが発生しました: {e}")
            self.driver.motor_stop_brake()
        finally:
            self.cleanup()

    def cleanup(self):
        """
        プログラム終了時に使用したリソースを解放します。
        """
        if self.driver:
            self.driver.cleanup()
        if self.pi and self.pi.connected:
            self.pi.bb_serial_read_close(self.RX_PIN)
            self.pi.stop()
            print("pigpioリソースをクリーンアップしました。")
        if self.picam2:
            self.picam2.close()
            print("Picamera2をクローズしました。")
        GPIO.cleanup()
        print("=== ローバー制御システムを終了しました。 ===")

# --- メイン実行ブロック ---
if __name__ == "__main__":
    # ローバーナビゲーターのインスタンスを作成
    # ここで設定値を調整することもできますが、クラス内でデフォルト値を定義しています。
    # 例: navigator = RoverNavigator(destination_lat=35.1234, destination_lon=139.5678)
    navigator = RoverNavigator()

    # ナビゲーションループを開始
    navigator.run_navigation_loop()
