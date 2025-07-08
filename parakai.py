# main_rover_control.py (縦2分割、横3分割の赤色検出)

import RPi.GPIO as GPIO
import time
import pigpio
import board
import busio
import adafruit_bno055
import numpy as np
import cv2
from picamera2 import Picamera2
from libcamera import Transform
import sys
import os

# カスタムモジュールのインポート
from motor import MotorDriver
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

# --- 関数定義 (既存のものを保持) ---
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

def calculate_heading(current_lat, current_lon, dest_lat, dest_lon):
    import math
    delta_lon = math.radians(dest_lon - current_lon)
    y = math.sin(delta_lon) * math.cos(math.radians(dest_lat))
    x = math.cos(math.radians(current_lat)) * math.sin(math.radians(dest_lat)) - \
        math.sin(math.radians(current_lat)) * math.cos(math.radians(dest_lat)) * math.cos(delta_lon)
    bearing = math.degrees(math.atan2(y, x))
    return (bearing + 360) % 360

def save_image_for_debug(picam2_instance, path="/home/mark1/Pictures/paravo_image.jpg"):
    frame = picam2_instance.capture_array()
    if frame is None:
        print("画像キャプチャ失敗：フレームがNoneです。")
        return None
    frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
    cv2.imwrite(path, frame_bgr)
    print(f"画像保存成功: {path}")
    return frame

# --- 新しいカメラ撮影・赤色検出関数 ---
def detect_red_in_grid(picam2_instance, save_path="/home/mark1/Pictures/akairo_grid.jpg", min_red_pixel_ratio_per_cell=0.10):
    """
    カメラ画像を縦2x横3のグリッドに分割し、各セルでの赤色検出を行い、その位置情報を返します。
    パラシュートは下3つに検出される可能性が高いという前提を維持します。

    Args:
        picam2_instance: Picamera2のインスタンス。
        save_path (str): デバッグ用にグリッドと検出結果が描画された画像を保存するフルパス。
        min_red_pixel_ratio_per_cell (float): セルごとの赤色と判断する最小ピクセル割合 (0.0〜1.0)。

    Returns:
        str: 'left_bottom', 'middle_bottom', 'right_bottom',
             'high_percentage_overall', 'none_detected', 'error_in_processing'
    """
    try:
        frame_rgb = picam2_instance.capture_array()
        if frame_rgb is None:
            print("画像キャプチャ失敗: フレームがNoneです。")
            return 'error_in_processing'

        frame_bgr = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)
        # 画像を反時計回りに90度回転 (カメラが時計回りに90度傾いている場合)
        rotated_frame_bgr = cv2.rotate(frame_bgr, cv2.ROTATE_90_COUNTERCLOCKWISE)
        
        height, width, _ = rotated_frame_bgr.shape

        # グリッドの定義 (縦2, 横3)
        # 各セルの高さと幅を計算
        cell_height = height // 2
        cell_width = width // 3

        # 各セルの座標範囲 (y_start:y_end, x_start:x_end)
        # 0 | 1 | 2
        # --+---+--
        # 3 | 4 | 5
        cells = {
            # 上段
            'top_left': (0, cell_height, 0, cell_width),
            'top_middle': (0, cell_height, cell_width, 2 * cell_width),
            'top_right': (0, cell_height, 2 * cell_width, width),
            # 下段
            'bottom_left': (cell_height, height, 0, cell_width),
            'bottom_middle': (cell_height, height, cell_width, 2 * cell_width),
            'bottom_right': (cell_height, height, 2 * cell_width, width),
        }

        red_counts = {key: 0 for key in cells}
        total_pixels_in_cell = {key: 0 for key in cells}

        # 赤色のHSV範囲 (Hが0-10と160-180の一般的な赤色範囲)
        lower_red1 = np.array([0, 100, 100])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([160, 100, 100])
        upper_red2 = np.array([180, 255, 255])

        # 全体的な赤色検出 (高割合検出用)
        blurred_full_frame = cv2.GaussianBlur(rotated_frame_bgr, (5, 5), 0)
        hsv_full = cv2.cvtColor(blurred_full_frame, cv2.COLOR_BGR2HSV)
        mask_full = cv2.bitwise_or(cv2.inRange(hsv_full, lower_red1, upper_red1),
                                   cv2.inRange(hsv_full, lower_red2, upper_red2))
        red_pixels_full = np.count_nonzero(mask_full)
        total_pixels_full = height * width
        red_percentage_full = red_pixels_full / total_pixels_full if total_pixels_full > 0 else 0.0

        if red_percentage_full >= 0.80:
            print(f"画像全体の赤色ピクセル割合: {red_percentage_full:.2%} (高割合) -> high_percentage_overall")
            cv2.imwrite(save_path, rotated_frame_bgr) # デバッグ用
            return 'high_percentage_overall'

        # 各セルで赤色を検出
        debug_frame = rotated_frame_bgr.copy() # デバッグ表示用にコピー
        for cell_name, (y_start, y_end, x_start, x_end) in cells.items():
            cell_frame = rotated_frame_bgr[y_start:y_end, x_start:x_end]
            
            # ガウシアンブラーを適用
            blurred_cell_frame = cv2.GaussianBlur(cell_frame, (5, 5), 0)
            hsv_cell = cv2.cvtColor(blurred_cell_frame, cv2.COLOR_BGR2HSV)
            
            mask_cell = cv2.bitwise_or(cv2.inRange(hsv_cell, lower_red1, upper_red1),
                                       cv2.inRange(hsv_cell, lower_red2, upper_red2))
            
            red_counts[cell_name] = np.count_nonzero(mask_cell)
            total_pixels_in_cell[cell_name] = cell_frame.shape[0] * cell_frame.shape[1]
            
            # デバッグ用にグリッドと検出状況を描画
            color = (255, 0, 0) # 青
            thickness = 2
            if red_counts[cell_name] / total_pixels_in_cell[cell_name] >= min_red_pixel_ratio_per_cell:
                color = (0, 0, 255) # 赤 (検出されたら)
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

        # 下段のセルで赤色検出の可能性が高い場合を優先して判定
        # パラシュートは下三つに検出される可能性が高い、という前提
        bottom_left_ratio = red_counts['bottom_left'] / total_pixels_in_cell['bottom_left']
        bottom_middle_ratio = red_counts['bottom_middle'] / total_pixels_in_cell['bottom_middle']
        bottom_right_ratio = red_counts['bottom_right'] / total_pixels_in_cell['bottom_right']

        detected_cells = []
        if bottom_left_ratio >= min_red_pixel_ratio_per_cell:
            detected_cells.append('bottom_left')
        if bottom_middle_ratio >= min_red_pixel_ratio_per_cell:
            detected_cells.append('bottom_middle')
        if bottom_right_ratio >= min_red_pixel_ratio_per_cell:
            detected_cells.append('bottom_right')

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
            return 'bottom_middle' # 下段全体の場合は中央の動きで調整
        elif 'bottom_middle' in detected_cells:
            print("赤色が下段中央に検出されました")
            return 'bottom_middle'
        else:
            print("赤色が下段の特定の場所に検出されましたが、左右の偏りはありません")
            return 'bottom_middle' # それ以外のケースは中央として扱う

    except Exception as e:
        print(f"カメラ撮影・グリッド処理中にエラーが発生しました: {e}")
        return 'error_in_processing'

# --- メインシーケンス ---
if __name__ == "__main__":
    # (初期化処理は変更なし)
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)

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

    i2c_bus = busio.I2C(board.SCL, board.SDA)
    original_bno_sensor = adafruit_bno055.BNO055_I2C(i2c_bus)
    bno_sensor_for_following = BNO055Wrapper(original_bno_sensor)

    picam2_instance = Picamera2()
    # Picamera2のconfigureからtransformを削除 (ソフトウェア回転に任せるため)
    picam2_instance.configure(picam2_instance.create_preview_configuration(
        main={"size": (640, 480)},
        controls={"FrameRate": 30}
    ))
    picam2_instance.start()
    time.sleep(2)

    try:
        # STEP 1: BNO055 キャリブレーション (変更なし)
        print("\n=== ステップ1: BNO055キャリブレーション開始 ===")
        print("センサーを動かしてキャリブレーションを進めてください...")
        while True:
            sys_cal, gyro_cal, accel_cal, mag_cal = original_bno_sensor.calibration_status
            print(f"Calib → Sys:{sys_cal}, Gyro:{gyro_cal}, Acc:{accel_cal}, Mag:{mag_cal}", end='\r')
            sys.stdout.flush()
            
            if gyro_cal == 3 and mag_cal == 3:
                print("\nキャリブレーション完了！")
                break
            time.sleep(0.1)

        while True:
            # STEP 2: GPS現在地取得し、目標方位計算 (変更なし)
            print("\n--- 新しい走行サイクル開始 ---")
            print("\n=== ステップ2: GPS現在地取得と目標方位計算 ===")
            current_lat, current_lon = get_current_location(pi_instance, RX_PIN)
            if current_lat is None or current_lon is None:
                print("GPSデータが取得できませんでした。リトライします...")
                time.sleep(2)
                continue
            print(f"現在地：緯度={current_lat:.4f}, 経度={current_lon:.4f}")
            target_gps_heading = calculate_heading(current_lat, current_lon, destination_lat, destination_lon)
            print(f"GPSに基づく目標方位：{target_gps_heading:.2f}度")

            # STEP 3: その場で回頭 (動的調整) (変更なし)
            print("\n=== ステップ3: 目標方位への回頭 (動的調整) ===")
            ANGLE_THRESHOLD_DEG = 5.0
            turn_speed = 40
            max_turn_attempts = 100
            turn_attempt_count = 0
            while turn_attempt_count < max_turn_attempts:
                current_bno_heading = original_bno_sensor.euler[0]
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
            
            # 新しい detect_red_location 関数で赤色検出
            red_location_result = detect_red_location(picam2_instance, min_red_pixel_ratio_per_cell=0.10)

            if red_location_result == 'left_bottom':
                print("赤色が左下に検出されました → 右に回頭します")
                driver.changing_right(0, 40)
                time.sleep(0.8) # 回避のための旋回時間
                driver.motor_stop_brake()
                print("回頭後、少し前進します")
                following.follow_forward(driver, bno_sensor_for_following, base_speed=60, duration_time=2)
            elif red_location_result == 'right_bottom':
                print("赤色が右下に検出されました → 左に回頭します")
                driver.changing_left(0, 40)
                time.sleep(0.8) # 回避のための旋回時間
                driver.motor_stop_brake()
                print("回頭後、少し前進します")
                following.follow_forward(driver, bno_sensor_for_following, base_speed=60, duration_time=2)
            elif red_location_result == 'bottom_middle':
                print("赤色が下段中央に検出されました → その場で少し待機して様子を見ます")
                time.sleep(5) # 少し長めに待機
                print("待機後、少し前進します")
                following.follow_forward(driver, bno_sensor_for_following, base_speed=70, duration_time=3)
            elif red_location_result == 'high_percentage_overall':
                print("画像全体に高割合で赤色を検出 → パラシュートが覆いかぶさっている可能性。長く待機して様子を見ます")
                time.sleep(10) # より長く待機
                print("待機後、少し前進します")
                following.follow_forward(driver, bno_sensor_for_following, base_speed=50, duration_time=3)
            elif red_location_result == 'none_detected':
                print("赤色を検出しませんでした → 方向追従制御で前進します。(速度80, 5秒)")
                following.follow_forward(driver, bno_sensor_for_following, base_speed=80, duration_time=5)
            elif red_location_result == 'error_in_processing':
                print("カメラ処理でエラーが発生しました。少し待機します...")
                time.sleep(2)

            driver.motor_stop_brake() # 念のため停止

            print("\n=== 前進処理が完了しました。プログラムを終了します。 ===")
            break

    except Exception as e:
        print(f"メイン処理中に予期せぬエラーが発生しました: {e}")
        driver.motor_stop_brake()

    finally:
        # (終了処理は変更なし)
        if 'driver' in locals():
            driver.cleanup()
        if 'pi_instance' in locals() and pi_instance.connected:
            pi_instance.bb_serial_read_close(RX_PIN)
            pi_instance.stop()
        if 'picam2_instance' in locals():
            picam2_instance.close()
        GPIO.cleanup()
        print("=== 処理を終了しました。 ===")
