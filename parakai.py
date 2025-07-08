# main_rover_control.py (回避後の追加確認ロジック)

import RPi.GPIO as GPIO
import time
import pigpio
import board
import busio
import numpy as np
import cv2
from picamera2 import Picamera2
from libcamera import Transform
import sys
import os
import math

# カスタムモジュールのインポート
from motor import MotorDriver
from BNO055 import BNO055
import following

# --- BNO055用のラッパークラス (変更なし) ---
class BNO055Wrapper:
    def __init__(self, adafruit_bno055_sensor):
        self.sensor = adafruit_bno055_sensor

    def get_heading(self):
        heading = self.sensor.euler[0]
        if heading is None:
            wait_start_time = time.time()
            max_wait_time = 0.5
            while heading is None and (time.time() - wait_start_time < max_wait_time):
                time.sleep(0.01)
                heading = self.sensor.euler[0]
        if heading is None:
            return 0.0
        return heading

# --- 定数設定 (変更なし) ---
destination_lat = 35.9185366
destination_lon = 139.9085042
RX_PIN = 17

# --- 関数定義 (省略 - 変更なし) ---
def convert_to_decimal(coord, direction):
    degrees = int(coord[:2]) if direction in ['N', 'S'] else int(coord[:3])
    minutes = float(coord[2:]) if direction in ['N', 'S'] else float(coord[3:])
    decimal = degrees + minutes / 60
    if direction in ['S', 'W']:
        decimal *= -1
    return decimal

def get_current_location(pi_instance, rx_pin):
    timeout = time.time() + 5
    while time.time() < timeout:
        (count, data) = pi_instance.bb_serial_read(rx_pin)
        if count and data:
            try:
                text = data.decode("ascii", errors="ignore")
                if "$GNRMC" in text:
                    for line in text.split("\n"):
                        if "$GNRMC" in line:
                            parts = line.strip().split(",")
                            if len(parts) > 6 and parts[2] == "A":
                                lat = convert_to_decimal(parts[3], parts[4])
                                lon = convert_to_decimal(parts[5], parts[6])
                                return lat, lon
            except Exception as e:
                print(f"GPSデータ解析エラー: {e}")
                continue
        time.sleep(0.1)
    print("GPSデータの取得に失敗しました (タイムアウト)。")
    return None, None

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
    radius = 6378137.0
    dlat = lat2 - lat1
    dlon = lon2 - lon1
    a = math.sin(dlat / 2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2)**2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    dist = radius * c
    return dist

def save_image_for_debug(picam2_instance, path="/home/mark1/Pictures/paravo_image.jpg"):
    frame = picam2_instance.capture_array()
    if frame is None:
        print("画像キャプチャ失敗：フレームがNoneです。")
        return None
    frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
    cv2.imwrite(path, frame_bgr)
    print(f"画像保存成功: {path}")
    return frame

def detect_red_in_grid(picam2_instance, save_path="/home/mark1/Pictures/akairo_grid.jpg", min_red_pixel_ratio_per_cell=0.10):
    """
    カメラ画像を縦2x横3のグリッドに分割し、各セルでの赤色検出を行い、その位置情報を返します。
    """
    try:
        frame_rgb = picam2_instance.capture_array()
        if frame_rgb is None:
            print("画像キャプチャ失敗: フレームがNoneです。")
            return 'error_in_processing'

        frame_bgr = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)
        rotated_frame_bgr = cv2.rotate(frame_bgr, cv2.ROTATE_90_COUNTERCLOCKWISE)
        
        height, width, _ = rotated_frame_bgr.shape
        cell_height = height // 2 ; cell_width = width // 3
        cells = {
            'top_left': (0, cell_height, 0, cell_width), 'top_middle': (0, cell_height, cell_width, 2 * cell_width),
            'top_right': (0, cell_height, 2 * cell_width, width),
            'bottom_left': (cell_height, height, 0, cell_width), 'bottom_middle': (cell_height, height, cell_width, 2 * cell_width),
            'bottom_right': (cell_height, height, 2 * cell_width, width),
        }
        red_counts = {key: 0 for key in cells} ; total_pixels_in_cell = {key: 0 for key in cells}

        lower_red1 = np.array([0, 100, 100]) ; upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([160, 100, 100]) ; upper_red2 = np.array([180, 255, 255])

        blurred_full_frame = cv2.GaussianBlur(rotated_frame_bgr, (5, 5), 0)
        hsv_full = cv2.cvtColor(blurred_full_frame, cv2.COLOR_BGR2HSV)
        mask_full = cv2.bitwise_or(cv2.inRange(hsv_full, lower_red1, upper_red1),
                                   cv2.inRange(hsv_full, lower_red2, upper_red2))
        red_pixels_full = np.count_nonzero(mask_full) ; total_pixels_full = height * width
        red_percentage_full = red_pixels_full / total_pixels_full if total_pixels_full > 0 else 0.0

        if red_percentage_full >= 0.80:
            print(f"画像全体の赤色ピクセル割合: {red_percentage_full:.2%} (高割合) -> high_percentage_overall")
            cv2.imwrite(save_path, rotated_frame_bgr)
            return 'high_percentage_overall'

        debug_frame = rotated_frame_bgr.copy()
        for cell_name, (y_start, y_end, x_start, x_end) in cells.items():
            cell_frame = rotated_frame_bgr[y_start:y_end, x_start:x_end]
            blurred_cell_frame = cv2.GaussianBlur(cell_frame, (5, 5), 0)
            hsv_cell = cv2.cvtColor(blurred_cell_frame, cv2.COLOR_BGR2HSV)
            mask_cell = cv2.bitwise_or(cv2.inRange(hsv_cell, lower_red1, upper_red1),
                                       cv2.inRange(hsv_cell, lower_red2, upper_red2))
            red_counts[cell_name] = np.count_nonzero(mask_cell)
            total_pixels_in_cell[cell_name] = cell_frame.shape[0] * cell_frame.shape[1]
            
            color = (255, 0, 0) ; thickness = 2
            if red_counts[cell_name] / total_pixels_in_cell[cell_name] >= min_red_pixel_ratio_per_cell:
                color = (0, 0, 255) ; thickness = 3
            cv2.rectangle(debug_frame, (x_start, y_start), (x_end, y_end), color, thickness)
            cv2.putText(debug_frame, f"{cell_name}: {(red_counts[cell_name] / total_pixels_in_cell[cell_name]):.2f}", 
                        (x_start + 5, y_start + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

        directory = os.path.dirname(save_path)
        if not os.path.exists(directory): os.makedirs(directory)
        cv2.imwrite(save_path, debug_frame)
        print(f"グリッド検出画像を保存しました: {save_path}")

        bottom_left_ratio = red_counts['bottom_left'] / total_pixels_in_cell['bottom_left']
        bottom_middle_ratio = red_counts['bottom_middle'] / total_pixels_in_cell['bottom_middle']
        bottom_right_ratio = red_counts['bottom_right'] / total_pixels_in_cell['bottom_right']

        detected_cells = []
        if bottom_left_ratio >= min_red_pixel_ratio_per_cell: detected_cells.append('bottom_left')
        if bottom_middle_ratio >= min_red_pixel_ratio_per_cell: detected_cells.append('bottom_middle')
        if bottom_right_ratio >= min_red_pixel_ratio_per_cell: detected_cells.append('bottom_right')

        if len(detected_cells) == 0:
            print("赤色を検出しませんでした (下段)")
            return 'none_detected'
        elif 'bottom_left' in detected_cells and 'bottom_right' not in detected_cells:
            print("赤色が左下に偏って検出されました")
            return 'left_bottom'
        elif 'bottom_right' in detected_cells and 'bottom_left' not in detected_cells:
            print("赤色が右下に偏って検出されました")
            return 'right_bottom'
        elif 'bottom_left' in detected_cells and 'bottom_middle' in detected_cells and 'bottom_right' in detected_cells:
            print("赤色が下段全体に広く検出されました")
            return 'bottom_middle'
        elif 'bottom_middle' in detected_cells:
            print("赤色が下段中央に検出されました")
            return 'bottom_middle'
        else:
            print("赤色が下段の特定の場所に検出されましたが、左右の偏りはありません")
            return 'bottom_middle'

    except Exception as e:
        print(f"カメラ撮影・グリッド処理中にエラーが発生しました: {e}")
        return 'error_in_processing'

# --- 新しい関数: 指定角度へ回頭するヘルパー関数 ---
def turn_to_relative_angle(driver, bno_sensor_instance, angle_offset_deg, turn_speed=40, angle_tolerance_deg=3.0, max_turn_time=10.0):
    """
    現在のBNO055の方位から、指定された角度だけ相対的に旋回します。
    Args:
        driver: MotorDriverのインスタンス。
        bno_sensor_instance: BNO055センサーのインスタンス（get_heading()を持つ）。
        angle_offset_deg (float): 目標とする相対角度（例: 90度右旋回なら90, 90度左旋回なら-90）。
        turn_speed (int): 旋回速度 (0-100)。
        angle_tolerance_deg (float): 許容する最終角度誤差（度）。
        max_turn_time (float): 最大旋回時間（秒）。
    Returns:
        bool: 成功した場合はTrue、タイムアウトやエラーの場合はFalse。
    """
    initial_heading = bno_sensor_instance.get_heading()
    if initial_heading is None:
        print("警告: turn_to_relative_angle: 初期方位が取得できませんでした。")
        return False
    
    target_heading = (initial_heading + angle_offset_deg + 360) % 360
    print(f"現在のBNO方位: {initial_heading:.2f}度, 相対目標角度: {angle_offset_deg:.2f}度 -> 絶対目標方位: {target_heading:.2f}度")

    start_time = time.time()
    
    while time.time() - start_time < max_turn_time:
        current_heading = bno_sensor_instance.get_heading()
        if current_heading is None:
            print("警告: turn_to_relative_angle: 旋回中に方位が取得できませんでした。スキップします。")
            driver.motor_stop_brake()
            time.sleep(0.1)
            continue

        angle_error = (target_heading - current_heading + 180 + 360) % 360 - 180

        if abs(angle_error) <= angle_tolerance_deg:
            print(f"[TURN] 相対回頭完了。最終誤差: {angle_error:.2f}度")
            driver.motor_stop_brake()
            time.sleep(0.5)
            return True

        if angle_error < 0: # 現在向いている方向が目標より左 (反時計回り)
            driver.changing_left(0, turn_speed)
        else: # 現在向いている方向が目標より右 (時計回り)
            driver.changing_right(0, turn_speed)
        
        time.sleep(0.05) # 短いポーリング間隔
    
    print(f"警告: turn_to_relative_angle: 最大旋回時間({max_turn_time}秒)内に目標角度に到達できませんでした。最終誤差: {angle_error:.2f}度")
    driver.motor_stop_brake()
    time.sleep(0.5)
    return False


# --- メインシーケンス ---
if __name__ == "__main__":
    # GPIO設定
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)

    # デバイス初期化
    driver = MotorDriver(
        PWMA=12, AIN1=23, AIN2=18,
        PWMB=19, BIN1=16, BIN2=26,
        STBY=21
    )
    pi_instance = pigpio.pi()
    if not pi_instance.connected:
        print("pigpioデーモンに接続できません。終了します。")
        exit()
    pi_instance.bb_serial_read_open(RX_PIN, 9600, 8)

    # BNO055.py の BNO055 クラスを使用
    bno_sensor = BNO055(address=0x28) # BNO055.py の BNO055 クラスのインスタンスを作成
    if not bno_sensor.begin(): # BNO055センサーの初期化
        print("BNO055センサーの初期化に失敗しました。終了します。")
        exit()
    bno_sensor.setMode(BNO055.OPERATION_MODE_NDOF) # NDOFモードに設定
    bno_sensor.setExternalCrystalUse(True) # 外部クリスタルを使用する場合
    time.sleep(1) # センサー安定待ち
    
    picam2_instance = Picamera2()
    picam2_instance.configure(picam2_instance.create_preview_configuration(
        main={"size": (640, 480)},
        controls={"FrameRate": 30}
    ))
    picam2_instance.start()
    time.sleep(2)

    try:
        # === BNO055キャリブレーション待機 ===
        print("BNO055のキャリブレーション待機中...")
        while True:
            sys_cal, gyro_cal, accel_cal, mag_cal = bno_sensor.getCalibration() # bno_sensor.getCalibration() を使用
            print(f"Calib → Sys:{sys_cal}, Gyro:{gyro_cal}, Acc:{accel_cal}, Mag:{mag_cal}", end='\r')
            sys.stdout.flush()
            if gyro_cal == 3 and mag_cal == 3:
                print("\nキャリブレーション完了！ナビゲーションを開始します。")
                break
            time.sleep(0.5)

        # メインの自律走行ループ
        while True:
            print("\n--- 新しい走行サイクル開始 ---")
            
            # STEP 2: GPS現在地取得し、目標方位計算
            print("\n=== ステップ2: GPS現在地取得と目標方位計算 ===")
            current_gps_coords = get_current_location(pi_instance, RX_PIN)
            goal_gps_coords = (destination_lat, destination_lon)

            if current_gps_coords[0] is None or current_gps_coords[1] is None:
                print("GPSデータが取得できませんでした。リトライします...")
                time.sleep(2)
                continue

            print(f"現在地：緯度={current_gps_coords[0]:.4f}, 経度={current_gps_coords[1]:.4f}")
            
            target_gps_heading = get_bearing_to_goal(current_gps_coords, goal_gps_coords)
            if target_gps_heading is None:
                print("警告: 目標方位の計算に失敗しました。リトライします...")
                time.sleep(2)
                continue

            print(f"GPSに基づく目標方位：{target_gps_heading:.2f}度")
            
            distance_to_goal = get_distance_to_goal(current_gps_coords, goal_gps_coords)
            print(f"目的地までの距離：{distance_to_goal:.2f}メートル")

            # STEP 3: その場で回頭 (動的調整)
            print("\n=== ステップ3: 目標方位への回頭 (動的調整) ===")
            ANGLE_THRESHOLD_DEG = 20.0 # 許容誤差を5度に設定
            turn_speed = 40
            max_turn_attempts = 100
            turn_attempt_count = 0

            # このブロックは、後述の再確認ループでも再利用するため、関数化することも検討できますが、
            # 現状の複雑さを考えると、一旦このままにしておきます。
            while turn_attempt_count < max_turn_attempts:
                current_bno_heading = bno_sensor.get_heading() # bno_sensor.get_heading() を使用
                if current_bno_heading is None:
                    print("警告: 旋回中にBNO055方位が取得できませんでした。リトライします。")
                    driver.motor_stop_brake()
                    time.sleep(1)
                    turn_attempt_count += 1
                    continue

                angle_error = (target_gps_heading - current_bno_heading + 180 + 360) % 360 - 180
                
                if abs(angle_error) <= ANGLE_THRESHOLD_DEG:
                    print(f"[TURN] 方位調整完了。最終誤差: {angle_error:.2f}度")
                    break

                turn_duration = 0.15 + (abs(angle_error) / 180.0) * 0.2
                if angle_error < 0:
                    print(f"[TURN] 左に回頭します (誤差: {angle_error:.2f}度, 時間: {turn_duration:.2f}秒)")
                    driver.changing_left(0, turn_speed)
                else:
                    print(f"[TURN] 右に回頭します (誤差: {angle_error:.2f}度, 時間: {turn_duration:.2f}秒)")
                    driver.changing_right(0, turn_speed)
                
                time.sleep(turn_duration)
                driver.motor_stop_brake()
                time.sleep(0.5)

                turn_attempt_count += 1

            if turn_attempt_count >= max_turn_attempts and abs(angle_error) > ANGLE_THRESHOLD_DEG:
                print(f"警告: 最大回頭試行回数に達しましたが、目標方位に到達できませんでした。最終誤差: {angle_error:.2f}度")
            
            driver.motor_stop_brake()
            time.sleep(0.5)

            # STEP 4 & 5: カメラ検知と前進
            print("\n=== ステップ4&5: カメラ検知と前進 ===")
            
            red_location_result = detect_red_in_grid(picam2_instance, save_path="/home/mark1/Pictures/akairo_grid.jpg", min_red_pixel_ratio_per_cell=0.10)

            if red_location_result == 'left_bottom':
                print("赤色が左下に検出されました → 右に回頭します")
                turn_to_relative_angle(driver, bno_sensor, 90, turn_speed=40, angle_tolerance_deg=30.0) # 右90度
                print("回頭後、少し前進します")
                following.follow_forward(driver, bno_sensor, base_speed=60, duration_time=2)
            elif red_location_result == 'right_bottom':
                print("赤色が右下に検出されました → 左に回頭します")
                turn_to_relative_angle(driver, bno_sensor, -90, turn_speed=40, angle_tolerance_deg=30.0) # 左90度
                print("回頭後、少し前進します")
                following.follow_forward(driver, bno_sensor, base_speed=60, duration_time=2)
            elif red_location_result == 'bottom_middle':
                print("赤色が下段中央に検出されました → 右に120度回頭して前進します")
                turn_to_relative_angle(driver, bno_sensor, 120, turn_speed=40, angle_tolerance_deg=30.0) # 右120度
                print("120度回頭後、少し前進します")
                following.follow_forward(driver, bno_sensor, base_speed=70, duration_time=3)
            elif red_location_result == 'high_percentage_overall':
                print("画像全体に高割合で赤色を検出 → パラシュートが覆いかぶさっている可能性。長く待機して様子を見ます")
                time.sleep(10) # より長く待機
                print("待機後、少し前進します")
                following.follow_forward(driver, bno_sensor, base_speed=50, duration_time=3)
            elif red_location_result == 'none_detected':
                print("赤色を検出しませんでした → 方向追従制御で前進します。(速度80, 5秒)")
                following.follow_forward(driver, bno_sensor, base_speed=80, duration_time=5)
            elif red_location_result == 'error_in_processing':
                print("カメラ処理でエラーが発生しました。少し待機します...")
                time.sleep(2)

            driver.motor_stop_brake()

            # ★★★ 回避後の再確認ループ ★★★
            print("\n=== 回避後の周囲確認を開始します ===")
            confirmation_attempts = 3 # 最大3回確認を試みる
            avoidance_confirmed = False

            for i in range(confirmation_attempts):
                print(f"--- 回避確認試行 {i+1}/{confirmation_attempts} ---")
                
                # 1. 目的地のGPS方向を向く
                print("ローバーを目的地のGPS方向へ向けます...")
                current_bno_heading = bno_sensor.get_heading()
                if current_bno_heading is None:
                    print("警告: BNO055方位が取得できませんでした。GPS方向への回頭をスキップします。")
                    driver.motor_stop_brake()
                    time.sleep(1)
                else:
                    # STEP 3 と同じ回頭ロジックを再利用
                    turn_attempt_count_realign = 0
                    while turn_attempt_count_realign < max_turn_attempts: # max_turn_attempts はメインループのものを再利用
                        current_bno_heading = bno_sensor.get_heading()
                        if current_bno_heading is None:
                            print("警告: 再調整中にBNO055方位が取得できませんでした。リトライします。")
                            driver.motor_stop_brake()
                            time.sleep(1)
                            turn_attempt_count_realign += 1
                            continue

                        angle_error = (target_gps_heading - current_bno_heading + 180 + 360) % 360 - 180
                        
                        if abs(angle_error) <= ANGLE_THRESHOLD_DEG:
                            print(f"[RE-ALIGN] GPS方向への再調整完了。最終誤差: {angle_error:.2f}度")
                            break

                        turn_duration = 0.15 + (abs(angle_error) / 180.0) * 0.2
                        if angle_error < 0:
                            print(f"[RE-ALIGN] 左に回頭します (誤差: {angle_error:.2f}度, 時間: {turn_duration:.2f}秒)")
                            driver.changing_left(0, turn_speed)
                        else:
                            print(f"[RE-ALIGN] 右に回頭します (誤差: {angle_error:.2f}度, 時間: {turn_duration:.2f}秒)")
                            driver.changing_right(0, turn_speed)
                        
                        time.sleep(turn_duration)
                        driver.motor_stop_brake()
                        time.sleep(0.5)
                        turn_attempt_count_realign += 1
                    
                    if turn_attempt_count_realign >= max_turn_attempts and abs(angle_error) > ANGLE_THRESHOLD_DEG:
                        print(f"警告: GPS方向への再調整が不十分でした。最終誤差: {angle_error:.2f}度")
                    driver.motor_stop_brake()
                    time.sleep(0.5)


                # 2. 正面、左30度、右30度の3方向で赤色検知
                all_directions_clear = True
                
                # 正面
                print("→ 正面方向の赤色を確認します...")
                result_front = detect_red_in_grid(picam2_instance, save_path=f"/home/mark1/Pictures/confirm_front_{i+1}.jpg", min_red_pixel_ratio_per_cell=0.10)
                if result_front != 'none_detected':
                    print(f"正面で赤色を検出しました: {result_front}")
                    all_directions_clear = False

                # 左30度
                if all_directions_clear: # 正面がクリアなら次へ
                    print("→ 左に30度回頭し、赤色を確認します...")
                    turn_to_relative_angle(driver, bno_sensor, -30, turn_speed=40, angle_tolerance_deg=5.0) # 左30度
                    result_left = detect_red_in_grid(picam2_instance, save_path=f"/home/mark1/Pictures/confirm_left_{i+1}.jpg", min_red_pixel_ratio_per_cell=0.10)
                    if result_left != 'none_detected':
                        print(f"左30度で赤色を検出しました: {result_left}")
                        all_directions_clear = False
                    
                    # 元の方向に戻す (重要)
                    print("→ 左30度から正面に戻します...")
                    turn_to_relative_angle(driver, bno_sensor, 30, turn_speed=40, angle_tolerance_deg=5.0) # 右30度で戻す

                # 右30度
                if all_directions_clear: # 左30度もクリアなら次へ
                    print("→ 右に30度回頭し、赤色を確認します...")
                    turn_to_relative_angle(driver, bno_sensor, 30, turn_speed=40, angle_tolerance_deg=5.0) # 右30度
                    result_right = detect_red_in_grid(picam2_instance, save_path=f"/home/mark1/Pictures/confirm_right_{i+1}.jpg", min_red_pixel_ratio_per_cell=0.10)
                    if result_right != 'none_detected':
                        print(f"右30度で赤色を検出しました: {result_right}")
                        all_directions_clear = False
                    
                    # 元の方向に戻す (重要)
                    print("→ 右30度から正面に戻します...")
                    turn_to_relative_angle(driver, bno_sensor, -30, turn_speed=40, angle_tolerance_deg=5.0) # 左30度で戻す

                if all_directions_clear:
                    print("全ての方向でパラシュートは検出されませんでした。回避成功！")
                    avoidance_confirmed = True
                    break # 確認成功したらループを抜ける
                else:
                    print(f"警告: まだパラシュートらしきものを検出しました。再回避を試みます。")
                    # 再回避のロジック (下段中央の回避ロジックを再利用)
                    turn_to_relative_angle(driver, bno_sensor, 120, turn_speed=40, angle_tolerance_deg=5.0) # 右に120度旋回
                    following.follow_forward(driver, bno_sensor, base_speed=70, duration_time=3) # 少し前進
                    driver.motor_stop_brake()
                    time.sleep(1) # 再回避後のクールダウン

            if avoidance_confirmed:
                print("\n=== 前進処理が完了しました。プログラムを終了します。 ===")
                break # メインループを終了
            else:
                print("\n警告: 回避後の周囲確認でパラシュートが解消されませんでした。次の走行サイクルに進みます。")
                # ここで break せずに次の走行サイクル (メインwhile Trueの先頭) に戻るか、終了するかを選択
                # 今回は、無限ループのbreakをそのままにしておくため、このelseブロックには到達しません。
                # もしこのまま終了させたくないなら、上記 'break' を削除し、continue に変更。
                break # 確認が取れなければ終了

    except Exception as e:
        print(f"メイン処理中に予期せぬエラーが発生しました: {e}")
        driver.motor_stop_brake()

    finally:
        if 'driver' in locals():
            driver.cleanup()
        if 'pi_instance' in locals() and pi_instance.connected:
            pi_instance.bb_serial_read_close(RX_PIN)
            pi_instance.stop()
        if 'picam2_instance' in locals():
            picam2_instance.close()
        GPIO.cleanup()
        print("=== 処理を終了しました。 ===")
