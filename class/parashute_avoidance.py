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

# --- BNO055Wrapper クラスは削除される前提 ---

class RoverNavigator:
    """
    自律型ローバーのナビゲーションと障害物回避を制御するメインクラス。
    GPS、IMU (BNO055)、カメラ (Picamera2) を統合し、モータードライバーを制御します。
    """

    # --- 定数設定（変更なし） ---
    # GPS
    RX_PIN = 17
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
    CAMERA_ROTATION = 90

    # 赤色検出設定
    SAVE_IMAGE_DIR = "/home/mark1/Pictures/"
    MIN_RED_PIXEL_RATIO_PER_CELL = 0.10

    # 旋回・回避設定
    ANGLE_GPS_ADJUST_THRESHOLD_DEG = 10.0
    ANGLE_RELATIVE_TURN_TOLERANCE_DEG = 10.0
    TURN_SPEED = 90
    TURN_RE_ALIGN_SPEED = 90
    MAX_TURN_ATTEMPTS = 100
    FORWARD_SPEED_DEFAULT = 100
    FORWARD_DURATION_DEFAULT = 5

    def __init__(self, bno_sensor: BNO055): # bno_sensorを引数で受け取る形はそのまま
        """
        ローバーナビゲーターのコンストラクタです。
        各種ハードウェアの初期設定を行います。
        """
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)

        self.pi = pigpio.pi()
        if not self.pi.connected:
            print("🔴 pigpioデーモンに接続できません。'sudo pigpiod'で起動してください。")
            sys.exit(1)

        self.driver = MotorDriver(
            PWMA=self.PWMA, AIN1=self.AIN1, AIN2=self.AIN2,
            PWMB=self.PWMB, BIN1=self.BIN1, BIN2=self.BIN2,
            STBY=self.STBY
        )

        # BNO055 IMUのセットアップ（変更なし、引数で受け取る形）
        self.bno = bno_sensor # 引数で受け取ったBNO055インスタンスをself.bnoに格納
        # begin()は呼び出し側で行われているはずですが、念のためチェック
        if not self.bno.begin(): # .begin() は BNO055 のメソッド
            print("🔴 BNO055センサーの初期化に失敗しました。終了します。")
            self.cleanup()
            sys.exit(1)
        self.bno.setMode(BNO055.OPERATION_MODE_NDOF)
        self.bno.setExternalCrystalUse(True)
        time.sleep(1) # センサー安定化のための待機
        # self.bno_wrapper は不要になるため、削除

        self.picam2 = Picamera2()
        self.picam2.configure(self.picam2.create_preview_configuration(
            main={"size": (self.CAMERA_WIDTH, self.CAMERA_HEIGHT)},
            controls={"FrameRate": self.CAMERA_FRAMERATE},
            transform=Transform(rotation=self.CAMERA_ROTATION)
        ))
        self.picam2.start()
        time.sleep(2)

        err = self.pi.bb_serial_read_open(self.RX_PIN, self.GPS_BAUD, 8)
        if err != 0:
            print(f"🔴 ソフトUART RX の設定に失敗：GPIO={self.RX_PIN}, {self.GPS_BAUD}bps, エラーコード: {err}")
            self.cleanup()
            sys.exit(1)
        print(f"▶ ソフトUART RX を開始：GPIO={self.RX_PIN}, {self.GPS_BAUD}bps")

        print("✅ ローバーシステム初期化完了。")

    def _convert_to_decimal(self, coord, direction):
        # 変更なし
        degrees = int(coord[:2]) if direction in ['N', 'S'] else int(coord[:3])
        minutes = float(coord[2:]) if direction in ['N', 'S'] else float(coord[3:])
        decimal = degrees + minutes / 60
        if direction in ['S', 'W']:
            decimal *= -1
        return decimal

    def _get_current_location(self):
        # 変更なし
        timeout = time.time() + 5
        while time.time() < timeout:
            (count, data) = self.pi.bb_serial_read(self.RX_PIN)
            if count and data:
                try:
                    text = data.decode("ascii", errors="ignore")
                    if "$GNRMC" in text:
                        for line in text.split("\n"):
                            if "$GNRMC" in line:
                                parts = line.strip().split(",")
                                if len(parts) > 6 and parts[2] == "A":
                                    lat = self._convert_to_decimal(parts[3], parts[4])
                                    lon = self._convert_to_decimal(parts[5], parts[6])
                                    return lat, lon
                except Exception as e:
                    print(f"警告: GPSデータ解析エラー: {e}")
                time.sleep(0.01)
            time.sleep(0.1)
        print("警告: GPSデータの取得に失敗しました (タイムアウト)。")
        return None, None

    def _get_bearing_to_goal(self, current, goal):
        # 変更なし
        if current is None or goal is None: return None
        lat1, lon1 = math.radians(current[0]), math.radians(current[1])
        lat2, lon2 = math.radians(goal[0]), math.radians(goal[1])
        delta_lon = lon2 - lon1
        y = math.sin(delta_lon) * math.cos(lat2)
        x = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(delta_lon)
        bearing_rad = math.atan2(y, x)
        return (math.degrees(bearing_rad) + 360) % 360

    def _get_distance_to_goal(self, current, goal):
        # 変更なし
        if current is None or goal is None: return float('inf')
        lat1, lon1 = math.radians(current[0]), math.radians(current[1])
        lat2, lon2 = math.radians(goal[0]), math.radians(goal[1])
        radius = 6378137.0
        dlat = lat2 - lat1
        dlon = lon2 - lon1
        a = math.sin(dlat / 2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2)**2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
        dist = radius * c
        return dist

    def _save_image_for_debug(self, path):
        # 変更なし
        frame = self.picam2.capture_array()
        if frame is None:
            print("警告: 画像キャプチャ失敗：フレームがNoneです。")
            return None
        frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        cv2.imwrite(path, frame_bgr)
        print(f"デバッグ画像を保存しました: {path}")
        return frame_bgr

    def _detect_red_in_grid(self, save_filename="akairo_grid.jpg"):
        # 変更なし
        save_path = os.path.join(self.SAVE_IMAGE_DIR, save_filename)
        
        try:
            frame_rgb = self.picam2.capture_array()
            if frame_rgb is None:
                print("警告: 画像キャプチャ失敗: フレームがNoneです。")
                return 'error_in_processing'

            processed_frame_bgr = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)
            processed_frame_bgr = cv2.flip(processed_frame_bgr, 1)
            
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
                cv2.imwrite(save_path, processed_frame_bgr)
                return 'high_percentage_overall'

            debug_frame = processed_frame_bgr.copy()
            for cell_name, (y_start, y_end, x_start, x_end) in cells.items():
                cell_frame = processed_frame_bgr[y_start:y_end, x_start:x_end]
                blurred_cell_frame = cv2.GaussianBlur(cell_frame, (5, 5), 0)
                hsv_cell = cv2.cvtColor(blurred_cell_frame, cv2.COLOR_BGR2HSV)
                mask_cell = cv2.bitwise_or(cv2.inRange(hsv_cell, lower_red1, upper_red1),
                                             cv2.inRange(hsv_cell, lower_red2, upper_red2))
                red_counts[cell_name] = np.count_nonzero(mask_cell)
                total_pixels_in_cell[cell_name] = cell_frame.shape[0] * cell_frame.shape[1]
                
                color = (255, 0, 0)
                thickness = 2
                if red_counts[cell_name] / total_pixels_in_cell[cell_name] >= self.MIN_RED_PIXEL_RATIO_PER_CELL:
                    color = (0, 0, 255)
                    thickness = 3
                cv2.rectangle(debug_frame, (x_start, y_start), (x_end, y_end), color, thickness)
                cv2.putText(debug_frame, f"{cell_name}: {(red_counts[cell_name] / total_pixels_in_cell[cell_name]):.2f}", 
                                (x_start + 5, y_start + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

            directory = os.path.dirname(save_path)
            if not os.path.exists(directory):
                os.makedirs(directory)
            cv2.imwrite(save_path, debug_frame)
            print(f"グリッド検出画像を保存しました: {save_path}")

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

            if len(detected_cells_bottom_row) == 0:
                print("赤色を検出しませんでした (下段)。")
                return 'none_detected'
            elif 'bottom_left' in detected_cells_bottom_row and 'bottom_right' not in detected_cells_bottom_row:
                print("赤色が左下に偏って検出されました。")
                return 'left_bottom'
            elif 'bottom_right' in detected_cells_bottom_row and 'bottom_left' not in detected_cells_bottom_row:
                print("赤色が右下に偏って検出されました。")
                return 'right_bottom'
            elif 'bottom_middle' in detected_cells_bottom_row:
                print("赤色が下段中央に検出されました。")
                return 'bottom_middle'
            else:
                print("赤色が下段の特定の場所に検出されましたが、左右の偏りはありません。")
                return 'bottom_middle'

        except Exception as e:
            print(f"🔴 カメラ撮影・グリッド処理中にエラーが発生しました: {e}")
            return 'error_in_processing'

    def _get_bno_heading_robust(self):
        """
        BNO055から方位を堅牢に取得するヘルパーメソッド。
        BNO055Wrapperの機能を移設。
        """
        # BNO055ライブラリの生のeuler[0]を使用
        heading = self.bno.euler[0]
        if heading is None:
            wait_start_time = time.time()
            max_wait_time = 0.5 # 0.5秒まで待機
            while heading is None and (time.time() - wait_start_time < max_wait_time):
                time.sleep(0.01) # 10ミリ秒待機
                heading = self.bno.euler[0] # 再試行
        if heading is None:
            return 0.0 # 最終的に取得できない場合、0.0を返す
        return heading

    def _turn_to_relative_angle(self, angle_offset_deg, turn_speed, angle_tolerance_deg, max_turn_attempts):
        """
        現在のBNO055の方位から、指定された角度だけ相対的に旋回します。
        """
        # BNO055Wrapperの代わりに直接BNO055のデータを使用
        initial_heading = self._get_bno_heading_robust()
        if initial_heading is None: # _get_bno_heading_robust()はNoneを返さないはずですが、念のため
            print("警告: turn_to_relative_angle: 初期方位が取得できませんでした。旋回を中止します。")
            return False
        
        target_heading = (initial_heading + angle_offset_deg + 360) % 360
        print(f"現在のBNO方位: {initial_heading:.2f}度, 相対目標角度: {angle_offset_deg:.2f}度 -> 絶対目標方位: {target_heading:.2f}度")

        loop_count = 0
        
        while loop_count < max_turn_attempts:
            current_heading = self._get_bno_heading_robust() # ここもヘルパーメソッドを使用
            # _get_bno_heading_robust()はNoneを返さないので、Noneチェックは不要

            angle_error = (target_heading - current_heading + 180 + 360) % 360 - 180

            if abs(angle_error) <= angle_tolerance_deg:
                print(f"[TURN] 相対回頭完了。最終誤差: {angle_error:.2f}度 (試行回数: {loop_count})")
                self.driver.motor_stop_brake()
                time.sleep(0.5)
                return True

            turn_duration_on = 0.02 + (abs(angle_error) / 180.0) * 0.2
            
            if angle_error < 0:
                self.driver.petit_left(0, turn_speed)
                self.driver.petit_left(turn_speed, 0)
            else:
                self.driver.petit_right(0, turn_speed)
                self.driver.petit_right(turn_speed, 0)
            
            time.sleep(turn_duration_on)
            self.driver.motor_stop_brake()
            time.sleep(0.05)
            
            loop_count += 1
        
        print(f"警告: turn_to_relative_angle: 最大試行回数({max_turn_attempts}回)内に目標角度に到達できませんでした。最終誤差: {angle_error:.2f}度 (試行回数: {loop_count})")
        self.driver.motor_stop_brake()
        time.sleep(0.5)
        return False

    def _wait_for_bno055_calibration(self):
        """BNO055センサーの完全キャリブレーションを待機します。"""
        # BNO055Wrapperは直接関係ないので変更なし
        print("BNO055のキャリブレーション待機中...")
        calibration_start_time = time.time()
        while True:
            sys_cal, gyro_cal, accel_cal, mag_cal = self.bno.getCalibration() # self.bnoを使用
            print(f"Calib → Sys:{sys_cal}, Gyro:{gyro_cal}, Acc:{accel_cal}, Mag:{mag_cal} ", end='\r')
            sys.stdout.flush()
            if gyro_cal == 3:
                print("\n✅ キャリブレーション完了！ナビゲーションを開始します。")
                break
            time.sleep(0.5)
        print(f"キャリブレーションにかかった時間: {time.time() - calibration_start_time:.1f}秒\n")

    def run_navigation_loop(self):
        """
        ローバーのメインナビゲーションおよび障害物回避ループを実行します。
        """
        try:
            self._wait_for_bno055_calibration()

            while True:
                print("\n--- 新しい走行サイクル開始 ---")
                
                print("\n=== 初期動作: 1秒間前進します ===")
                # following.pyのfollow_forward関数もBNO055Wrapperに依存している場合、
                # その関数もBNO055のインスタンスを直接受け取るように修正するか、
                # ここでBNO055Wrapperの役割を代替する処理を記述する必要があります。
                # 現状のfollowing.pyのfollow_forwardの定義によりますが、
                # 例えば BNO055Wrapper(self.bno) のようにインスタンスを作って渡すことも可能です。
                # ここではfollowing.pyのfollow_forwardが生のBNO055インスタンスを受け取ると仮定します。
                # following.follow_forward(self.driver, self.bno_wrapper, base_speed=self.FORWARD_SPEED_DEFAULT, duration_time=1)
                following.follow_forward(self.driver, self.bno, base_speed=self.FORWARD_SPEED_DEFAULT, duration_time=1) # self.bno に変更
                self.driver.motor_stop_brake()
                time.sleep(0.5)
                print("1秒前進が完了しました。")

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

                if distance_to_goal < 1.0:
                    print("\n🎉 目的地に到達しました！ミッション完了！")
                    break

                print("\n=== ステップ3: 目標方位への回頭 (動的調整) ===")
                turn_attempt_count = 0
                while turn_attempt_count < self.MAX_TURN_ATTEMPTS:
                    current_bno_heading = self._get_bno_heading_robust() # ここをヘルパーメソッドに置き換え
                    # _get_bno_heading_robust()はNoneを返さないので、Noneチェックは不要

                    angle_error = (target_gps_heading - current_bno_heading + 180 + 360) % 360 - 180
                    
                    if abs(angle_error) <= self.ANGLE_GPS_ADJUST_THRESHOLD_DEG:
                        print(f"[TURN] 方位調整完了。最終誤差: {angle_error:.2f}度")
                        break

                    turn_duration = 0.02 + (abs(angle_error) / 180.0) * 0.2
                    if angle_error < 0:
                        print(f"[TURN] 左に回頭します (誤差: {angle_error:.2f}度, 時間: {turn_duration:.2f}秒)")
                        self.driver.petit_left(0, self.TURN_SPEED)
                        self.driver.petit_left(self.TURN_SPEED, 0)
                    else:
                        print(f"[TURN] 右に回頭します (誤差: {angle_error:.2f}度, 時間: {turn_duration:.2f}秒)")
                        self.driver.petit_right(0, self.TURN_SPEED)
                        self.driver.petit_right(self.TURN_SPEED, 0)
                    
                    time.sleep(turn_duration)
                    self.driver.motor_stop_brake()
                    time.sleep(0.5)

                    turn_attempt_count += 1

                if turn_attempt_count >= self.MAX_TURN_ATTEMPTS and abs(angle_error) > self.ANGLE_GPS_ADJUST_THRESHOLD_DEG:
                    print(f"警告: 最大回頭試行回数に達しましたが、目標方位に到達できませんでした。最終誤差: {angle_error:.2f}度")
                
                self.driver.motor_stop_brake()
                time.sleep(0.5)

                print("\n=== ステップ4&5: カメラ検知と前進 ===")
                
                red_location_result = self._detect_red_in_grid(save_filename=f"detection_cycle_{int(time.time())}.jpg")

                if red_location_result == 'left_bottom':
                    print("赤色が左下に検出されました → 右に90度回頭して回避します。")
                    self._turn_to_relative_angle(90, self.TURN_SPEED, self.ANGLE_RELATIVE_TURN_TOLERANCE_DEG, self.MAX_TURN_ATTEMPTS)
                    print("回頭後、少し前進します。")
                    following.follow_forward(self.driver, self.bno, base_speed=self.FORWARD_SPEED_DEFAULT, duration_time=self.FORWARD_DURATION_DEFAULT) # self.bno に変更
                elif red_location_result == 'right_bottom':
                    print("赤色が右下に検出されました → 左に90度回頭して回避します。")
                    self._turn_to_relative_angle(-90, self.TURN_SPEED, self.ANGLE_RELATIVE_TURN_TOLERANCE_DEG, self.MAX_TURN_ATTEMPTS)
                    print("回頭後、少し前進します。")
                    following.follow_forward(self.driver, self.bno, base_speed=self.FORWARD_SPEED_DEFAULT, duration_time=self.FORWARD_DURATION_DEFAULT) # self.bno に変更
                elif red_location_result == 'bottom_middle':
                    print("赤色が下段中央に検出されました → 右に120度回頭して前進します。")
                    self._turn_to_relative_angle(120, self.TURN_SPEED, self.ANGLE_RELATIVE_TURN_TOLERANCE_DEG, self.MAX_TURN_ATTEMPTS)
                    print("120度回頭後、少し前進します (1回目)。")
                    following.follow_forward(self.driver, self.bno, base_speed=self.FORWARD_SPEED_DEFAULT, duration_time=self.FORWARD_DURATION_DEFAULT) # self.bno に変更
                    self.driver.motor_stop_brake()
                    time.sleep(0.5)

                    print("さらに左に30度回頭し、前進します。")
                    self._turn_to_relative_angle(-30, self.TURN_SPEED, self.ANGLE_RELATIVE_TURN_TOLERANCE_DEG, self.MAX_TURN_ATTEMPTS)
                    print("左30度回頭後、少し前進します (2回目)。")
                    following.follow_forward(self.driver, self.bno, base_speed=self.FORWARD_SPEED_DEFAULT, duration_time=self.FORWARD_DURATION_DEFAULT) # self.bno に変更
                elif red_location_result == 'high_percentage_overall':
                    print("画像全体に高割合で赤色を検出 → パラシュートが覆いかぶさっている可能性。10秒待機して様子を見ます。")
                    time.sleep(10)
                    print("待機後、少し前進します。")
                    following.follow_forward(self.driver, self.bno, base_speed=self.FORWARD_SPEED_DEFAULT, duration_time=3) # self.bno に変更
                elif red_location_result == 'none_detected':
                    print("赤色を検出しませんでした → GPS方向追従制御で前進します。(速度90, 5秒)")
                    following.follow_forward(self.driver, self.bno, base_speed=self.FORWARD_SPEED_DEFAULT, duration_time=self.FORWARD_DURATION_DEFAULT) # self.bno に変更
                elif red_location_result == 'error_in_processing':
                    print("カメラ処理でエラーが発生しました。2秒待機してリトライします...")
                    time.sleep(2)
                    continue

                self.driver.motor_stop_brake()
                time.sleep(0.5)

                print("\n=== 回避後の周囲確認を開始します (3点スキャン) ===")
                avoidance_confirmed_clear = False

                print("\n=== 回避後: 再度目的地の方位へ回頭 ===")
                turn_attempt_count_realign = 0
                while turn_attempt_count_realign < self.MAX_TURN_ATTEMPTS:
                    current_bno_heading = self._get_bno_heading_robust() # ここをヘルパーメソッドに置き換え
                    # _get_bno_heading_robust()はNoneを返さないので、Noneチェックは不要

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

                print("→ 正面方向の赤色を確認します...")
                scan_results['front'] = self._detect_red_in_grid(save_filename=f"confirm_front_{int(time.time())}.jpg")

                print("→ 左に30度回頭し、赤色を確認します...")
                self._turn_to_relative_angle(-30, self.TURN_SPEED, self.ANGLE_RELATIVE_TURN_TOLERANCE_DEG, self.MAX_TURN_ATTEMPTS)
                scan_results['left_30'] = self._detect_red_in_grid(save_filename=f"confirm_left_{int(time.time())}.jpg")
                print("→ 左30度から正面に戻します...")
                self._turn_to_relative_angle(30, self.TURN_SPEED, self.ANGLE_RELATIVE_TURN_TOLERANCE_DEG, self.MAX_TURN_ATTEMPTS)

                print("→ 右に30度回頭し、赤色を確認します...")
                self._turn_to_relative_angle(30, self.TURN_SPEED, self.ANGLE_RELATIVE_TURN_TOLERANCE_DEG, self.MAX_TURN_ATTEMPTS)
                scan_results['right_30'] = self._detect_red_in_grid(save_filename=f"confirm_right_{int(time.time())}.jpg")
                print("→ 右30度から正面に戻します...")
                self._turn_to_relative_angle(-30, self.TURN_SPEED, self.ANGLE_RELATIVE_TURN_TOLERANCE_DEG, self.MAX_TURN_ATTEMPTS)

                is_front_clear = (scan_results['front'] == 'none_detected')
                is_left_clear = (scan_results['left_30'] == 'none_detected')
                is_right_clear = (scan_results['right_30'] == 'none_detected')

                if is_front_clear and is_left_clear and is_right_clear:
                    print("\n=== 3点スキャン結果: 全ての方向でパラシュートは検出されませんでした。回避成功、次のGPSポイントへ！ ===")
                    avoidance_confirmed_clear = True
                else:
                    print("\n=== 3点スキャン結果: まだパラシュートが検出されました。再回避を試みます。 ===")
                    print(f"検出詳細: 正面: {scan_results['front']}, 左30: {scan_results['left_30']}, 右30: {scan_results['right_30']}")
                    
                    if scan_results['left_30'] != 'none_detected':
                        print("左30度で検出されたため、右90度回頭して回避します。")
                        self._turn_to_relative_angle(90, self.TURN_SPEED, self.ANGLE_RELATIVE_TURN_TOLERANCE_DEG, self.MAX_TURN_ATTEMPTS)
                    elif scan_results['right_30'] != 'none_detected':
                        print("右30度で検出されたため、左90度回頭して回避します。")
                        self._turn_to_relative_angle(-90, self.TURN_SPEED, self.ANGLE_RELATIVE_TURN_TOLERANCE_DEG, self.MAX_TURN_ATTEMPTS)
                    elif scan_results['front'] != 'none_detected':
                        print("正面で検出されたため、右120度回頭して回避します。")
                        self._turn_to_relative_angle(120, self.TURN_SPEED, self.ANGLE_RELATIVE_TURN_TOLERANCE_DEG, self.MAX_TURN_ATTEMPTS)
                        self.driver.motor_stop_brake()
                        time.sleep(0.5)

                        print("さらに左に30度回頭し、前進します。")
                        self._turn_to_relative_angle(-30, self.TURN_SPEED, self.ANGLE_RELATIVE_TURN_TOLERANCE_DEG, self.MAX_TURN_ATTEMPTS)
                        print("左30度回頭後、少し前進します (2回目)。")
                        following.follow_forward(self.driver, self.bno, base_speed=self.FORWARD_SPEED_DEFAULT, duration_time=self.FORWARD_DURATION_DEFAULT) # self.bno に変更
                    else:
                        print("詳細不明な検出のため、右120度回頭して回避します。")
                        self._turn_to_relative_angle(120, self.TURN_SPEED, self.ANGLE_RELATIVE_TURN_TOLERANCE_DEG, self.MAX_TURN_ATTEMPTS)
                    
                    following.follow_forward(self.driver, self.bno, base_speed=self.FORWARD_SPEED_DEFAULT, duration_time=self.FORWARD_DURATION_DEFAULT) # self.bno に変更
                    self.driver.motor_stop_brake()
                    time.sleep(1)
                    
                    continue

        except KeyboardInterrupt:
            print("\nユーザー割り込みで終了します。")
        except Exception as e:
            print(f"メイン処理中に予期せぬエラーが発生しました: {e}")
            self.driver.motor_stop_brake()
        finally:
            self.cleanup()

    def cleanup(self):
        # 変更なし
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
    try:
        # BNO055センサーの初期化はここで明示的に行います
        # BNO055_ADDRESSはRoverNavigatorクラスの定数なので、RoverNavigator.BNO055_ADDRESSでアクセス
        bno_instance = BNO055(address=RoverNavigator.BNO055_ADDRESS)
        if not bno_instance.begin():
            print("🔴 メインスクリプトでBNO055センサーの初期化に失敗しました。終了します。")
            sys.exit(1)

        navigator = RoverNavigator(bno_sensor=bno_instance)
        navigator.run_navigation_loop()

    except KeyboardInterrupt:
        print("\nユーザー割り込みで終了します。")
    except Exception as e:
        print(f"メイン処理中に予期せぬエラーが発生しました: {e}")
        # エラー発生時のモーター停止など、追加のクリーンアップが必要な場合
        # navigatorオブジェクトが作成済みなら、そのcleanupを呼び出す
        if 'navigator' in locals() and navigator is not None:
            navigator.cleanup()
        else:
            # navigatorが作成される前にエラーが発生した場合の最低限のクリーンアップ
            GPIO.cleanup()
            if 'bno_instance' in locals() and bno_instance is not None:
                # BNO055ライブラリに終了処理があれば呼び出す
                pass # 現状のBNO055ライブラリには明示的なclose()などがないようです
            print("=== ローバー制御システムを終了しました (早期終了)。 ===")
    finally:
        # 正常終了時も cleanup を呼ぶ
        if 'navigator' in locals() and navigator is not None:
            navigator.cleanup()
