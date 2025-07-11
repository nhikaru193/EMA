# main_control.py

import time
import threading
import sys
import smbus
import RPi.GPIO as GPIO
import math
import numpy as np
import cv2
import pigpio
from picamera2 import Picamera2
from libcamera import Transform # TransformはPicamera2のバージョンによっては不要かもしれません

# カスタムモジュールのインポート
# これらのファイルがmain_control.pyと同じディレクトリにあることを確認してください。
from BNO055 import BNO055
from GPS_communication import EmGpsDatalink
from motor import MotorDriver # MotorDriverクラスが含まれるファイル
import following # following.py に含まれる following.follow_forward など
from Flag_Detector2 import FlagDetector # Flag_Detector2.py に含まれる FlagDetector クラス

# --- グローバル変数と共通設定 ---
# BME280関連
t_fine = 0.0
digT = []
digP = []
digH = []
i2c = None # SMBusインスタンスはmain_sequence内で初期化
bme280_address = 0x76 # BME280のアドレス

# GPS関連
GPS_RX_PIN = 17 # GPSデータ受信用のpigpioソフトUARTピン
GPS_BAUD = 9600

# IM920SL関連 (em_gps_datalink.pyでも使用)
IM920_TX_PIN = 27 # IM920SLの送信ピン (pigpioソフトUART用だが、今回はem_gps_datalink内で処理)
IM920_BAUD = 19200 # IM920SLのボーレート
WIRELESS_GROUND_PIN = 22 # ワイヤレスグラウンド制御用のGPIOピン番号

# 目標座標（例：ゴール地点）
# これはゴールまでのGPS誘導、およびパラシュート回避後のGPS再調整で使用
GOAL_LOCATION = [35.9186248, 139.9081672] # 12号館前

# --- BME280 初期化と補正関数群 ---
# これらの関数は、BME280センサーから正確な温度・気圧データを読み取るために必要です。
def init_bme280_sensors():
    """BME280センサーを初期設定します。"""
    global i2c
    try:
        i2c = smbus.SMBus(1) # Raspberry PiのI2Cバス1を使用
        i2c.write_byte_data(bme280_address, 0xF2, 0x01) # 湿度オーバーサンプリングをx1に設定
        i2c.write_byte_data(bme280_address, 0xF4, 0x27) # 温度x1, 気圧x1オーバーサンプリング, ノーマルモード
        i2c.write_byte_data(bme280_address, 0xF5, 0xA0) # スタンバイ時間1000ms, フィルターオフ
        print("[BME280] センサー初期化完了。")
    except Exception as e:
        print(f"ERROR: [BME280] センサー初期化失敗: {e}")
        raise IOError("BME280 sensor initialization failed.")

def read_bme280_compensate_params():
    """BME280の補正係数を読み込みます。センサーごとに異なるため、一度読み込む必要があります。"""
    global digT, digP, digH
    try:
        # 温度補正係数を読み込み
        dat_t = i2c.read_i2c_block_data(bme280_address, 0x88, 6)
        digT = [(dat_t[1] << 8) | dat_t[0], (dat_t[3] << 8) | dat_t[2], (dat_t[5] << 8) | dat_t[4]]
        for i in range(1, 2): # 符号付き16bit整数として扱う
            if digT[i] >= 32768:
                digT[i] -= 65536
        # 気圧補正係数を読み込み
        dat_p = i2c.read_i2c_block_data(bme280_address, 0x8E, 18)
        digP = [(dat_p[i+1] << 8) | dat_p[i] for i in range(0, 18, 2)]
        for i in range(1, 8): # 符号付き16bit整数として扱う
            if digP[i] >= 32768:
                digP[i] -= 65536
        # 湿度補正係数を読み込み
        dh = i2c.read_byte_data(bme280_address, 0xA1)
        dat_h = i2c.read_i2c_block_data(bme280_address, 0xE1, 8)
        digH = [dh, (dat_h[1] << 8) | dat_h[0], dat_h[2],
                (dat_h[3] << 4) | (0x0F & dat_h[4]),
                (dat_h[5] << 4) | ((dat_h[4] >> 4) & 0x0F),
                dat_h[6]]
        if digH[1] >= 32768: # 符号付き16bit整数として扱う
            digH[1] -= 65536
        for i in range(3, 4): # 符号付き16bit整数として扱う
            if digH[i] >= 32768:
                digH[i] -= 65536
        if digH[5] >= 128: # 符号付き8bit整数として扱う
            digH[5] -= 256
        print("[BME280] 補正係数読み込み完了。")
    except Exception as e:
        print(f"ERROR: [BME280] 補正係数読み込み失敗: {e}")
        raise IOError("BME280 compensation parameter reading failed.")


def bme280_compensate_t(adc_T):
    """生の温度ADC値を補正し、実際の温度を返します。"""
    global t_fine
    var1 = (adc_T / 8.0 - digT[0] * 2.0) * digT[1] / 2048.0
    var2 = ((adc_T / 16.0 - digT[0]) ** 2) * digT[2] / 16384.0
    t_fine = var1 + var2
    t = (t_fine * 5 + 128) / 256 / 100
    return t

def bme280_compensate_p(adc_P):
    """生の気圧ADC値を補正し、実際の気圧を返します。"""
    global t_fine
    p = 0.0
    var1 = t_fine - 128000.0
    var2 = var1 * var1 * digP[5]
    var2 += (var1 * digP[4]) * 131072.0
    var2 += digP[3] * 3.435973837e10
    var1 = (var1 * var1 * digP[2]) / 256.0 + (var1 * digP[1]) * 4096
    var1 = (1.407374884e14 + var1) * (digP[0] / 8589934592.0)
    if var1 == 0:
        return 0
    p = (1048576.0 - adc_P) * 2147483648.0 - var2
    p = (p * 3125) / var1
    var1 = digP[8] * (p / 8192.0)**2 / 33554432.0
    var2 = digP[7] * p / 524288.0
    p = (p + var1 + var2) / 256 + digP[6] * 16.0
    return p / 256 / 100 # hPa (ヘクトパスカル) 単位で返すため100で割る

def get_pressure_and_temperature():
    """BME280から気圧と温度を読み込み、補正して返します。"""
    try:
        dat = i2c.read_i2c_block_data(bme280_address, 0xF7, 8)
        adc_p = (dat[0] << 16 | dat[1] << 8 | dat[2]) >> 4 # 気圧の生データ
        adc_t = (dat[3] << 16 | dat[4] << 8 | dat[5]) >> 4 # 温度の生データ
        
        temperature = bme280_compensate_t(adc_t)
        pressure = bme280_compensate_p(adc_p)
        return pressure, temperature
    except Exception as e:
        print(f"ERROR: [BME280] データ取得失敗: {e}")
        return None, None

# --- GPS関連のヘルパー関数 ---
def convert_gps_to_decimal(coord, direction):
    """NMEA形式のGPS座標を十進数に変換します。"""
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
    """現在の位置から目標位置への方位角を計算します (真北基準、度)。"""
    if current is None or goal is None: return None
    lat1, lon1 = math.radians(current[0]), math.radians(current[1])
    lat2, lon2 = math.radians(goal[0]), math.radians(goal[1])
    delta_lon = lon2 - lon1
    y = math.sin(delta_lon) * math.cos(lat2)
    x = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(delta_lon)
    bearing_rad = math.atan2(y, x)
    return (math.degrees(bearing_rad) + 360) % 360

def get_distance_to_goal(current, goal):
    """2点間の距離をHaversine公式で計算します (メートル)。"""
    if current is None or goal is None: return float('inf')
    lat1, lon1 = math.radians(current[0]), math.radians(current[1])
    lat2, lon2 = math.radians(goal[0]), math.radians(goal[1])
    radius = 6378137.0 # 地球の半径 (メートル)
    dlat = lat2 - lat1
    dlon = lon2 - lon1
    a = math.sin(dlat / 2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2)**2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    dist = radius * c
    return dist

# --- カメラ関連のヘルパー関数 ---
# 赤色検出のためのHSV範囲
lower_red1 = np.array([0, 100, 100])
upper_red1 = np.array([10, 255, 255])
lower_red2 = np.array([160, 100, 100])
upper_red2 = np.array([180, 255, 255])

def save_image_for_debug(picam2_instance, path="/home/mark1/Pictures/debug_image.jpg"):
    """デバッグ用に画像を保存します。"""
    try:
        frame = picam2_instance.capture_array()
        if frame is None:
            print("警告: [Camera] 画像キャプチャ失敗：フレームがNoneです。")
            return None
        frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        directory = os.path.dirname(path)
        if not os.path.exists(directory):
            os.makedirs(directory)
        cv2.imwrite(path, frame_bgr)
        print(f"[Camera] 画像保存成功: {path}")
        return frame
    except Exception as e:
        print(f"ERROR: [Camera] 画像保存中にエラーが発生しました: {e}")
        return None

def detect_red_in_grid(picam2_instance, save_path="/home/mark1/Pictures/akairo_grid.jpg", min_red_pixel_ratio_per_cell=0.05):
    """
    カメラ画像をグリッドに分割し、赤色検出を行います。
    カメラの物理的な傾きを補正するための回転・反転処理を含みます。
    """
    try:
        frame_rgb = picam2_instance.capture_array()
        if frame_rgb is None:
            print("ERROR: [Camera] 画像キャプチャ失敗: フレームがNoneです。")
            return 'error_in_processing'

        frame_bgr = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)
        
        # Picamera2のconfigureでTransform(rotation=90)を使っている場合、
        # ここでのcv2.rotate(ROTATE_90_COUNTERCLOCKWISE)は不要かもしれません。
        # 必要に応じて調整してください。今回は二重回転にならないよう注意。
        processed_frame_bgr = frame_bgr
        # processed_frame_bgr = cv2.rotate(frame_bgr, cv2.ROTATE_90_COUNTERCLOCKWISE) # 必要に応じて有効化
        
        # 左右反転 (水平フリップ)
        processed_frame_bgr = cv2.flip(processed_frame_bgr, 1) # 1は水平フリップ (左右反転)
        
        height, width, _ = processed_frame_bgr.shape
        cell_height = height // 2 ; cell_width = width // 3
        cells = {
            'top_left': (0, cell_height, 0, cell_width), 'top_middle': (0, cell_height, cell_width, 2 * cell_width),
            'top_right': (0, cell_height, 2 * cell_width, width),
            'bottom_left': (cell_height, height, 0, cell_width), 'bottom_middle': (cell_height, height, cell_width, 2 * cell_width),
            'bottom_right': (cell_height, height, 2 * cell_width, width),
        }
        red_counts = {key: 0 for key in cells} ; total_pixels_in_cell = {key: 0 for key in cells}

        blurred_full_frame = cv2.GaussianBlur(processed_frame_bgr, (5, 5), 0)
        hsv_full = cv2.cvtColor(blurred_full_frame, cv2.COLOR_BGR2HSV)
        mask_full = cv2.bitwise_or(cv2.inRange(hsv_full, lower_red1, upper_red1),
                                   cv2.inRange(hsv_full, lower_red2, upper_red2))
        red_pixels_full = np.count_nonzero(mask_full) ; total_pixels_full = height * width
        red_percentage_full = red_pixels_full / total_pixels_full if total_pixels_full > 0 else 0.0

        if red_percentage_full >= 0.80:
            print(f"[Camera] 画像全体の赤色ピクセル割合: {red_percentage_full:.2%} (高割合) -> high_percentage_overall")
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
            
            color = (255, 0, 0) ; thickness = 2
            if red_counts[cell_name] / total_pixels_in_cell[cell_name] >= min_red_pixel_ratio_per_cell:
                color = (0, 0, 255) ; thickness = 3
            cv2.rectangle(debug_frame, (x_start, y_start), (x_end, y_end), color, thickness)
            cv2.putText(debug_frame, f"{cell_name}: {(red_counts[cell_name] / total_pixels_in_cell[cell_name]):.2f}", 
                        (x_start + 5, y_start + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

        directory = os.path.dirname(save_path)
        if not os.path.exists(directory): os.makedirs(directory)
        cv2.imwrite(save_path, debug_frame)
        print(f"[Camera] グリッド検出画像を保存しました: {save_path}")

        bottom_left_ratio = red_counts['bottom_left'] / total_pixels_in_cell['bottom_left']
        bottom_middle_ratio = red_counts['bottom_middle'] / total_pixels_in_cell['bottom_middle']
        bottom_right_ratio = red_counts['bottom_right'] / total_pixels_in_cell['bottom_right']

        detected_cells = []
        if bottom_left_ratio >= min_red_pixel_ratio_per_cell: detected_cells.append('bottom_left')
        if bottom_middle_ratio >= min_red_pixel_ratio_per_cell: detected_cells.append('bottom_middle')
        if bottom_right_ratio >= min_red_pixel_ratio_per_cell: detected_cells.append('bottom_right')

        if len(detected_cells) == 0:
            print("[Camera] 赤色を検出しませんでした (下段)")
            return 'none_detected'
        elif 'bottom_left' in detected_cells and 'bottom_right' not in detected_cells:
            print("[Camera] 赤色が左下に偏って検出されました")
            return 'left_bottom'
        elif 'bottom_right' in detected_cells and 'bottom_left' not in detected_cells:
            print("[Camera] 赤色が右下に偏って検出されました")
            return 'right_bottom'
        elif 'bottom_left' in detected_cells and 'bottom_middle' in detected_cells and 'bottom_right' in detected_cells:
            print("[Camera] 赤色が下段全体に広く検出されました")
            return 'bottom_middle'
        elif 'bottom_middle' in detected_cells:
            print("[Camera] 赤色が下段中央に検出されました")
            return 'bottom_middle'
        else:
            print("[Camera] 赤色が下段の特定の場所に検出されましたが、左右の偏りはありません")
            return 'bottom_middle'

    except Exception as e:
        print(f"ERROR: [Camera] 撮影・グリッド処理中にエラーが発生しました: {e}")
        return 'error_in_processing'

# --- 共通の回頭関数 ---
def turn_to_relative_angle(driver_instance, bno_sensor_instance, angle_offset_deg, turn_speed=40, angle_tolerance_deg=3.0, max_turn_attempts=100):
    """
    現在のBNO055の方位から、指定された角度だけ相対的に旋回します。
    """
    initial_heading = bno_sensor_instance.getVector(BNO055.VECTOR_EULER)[0]
    if initial_heading is None:
        print("警告: [Turn] 初期方位が取得できませんでした。")
        return False
    
    target_heading = (initial_heading + angle_offset_deg + 360) % 360
    print(f"[Turn] 現在のBNO方位: {initial_heading:.2f}度, 相対目標角度: {angle_offset_deg:.2f}度 -> 絶対目標方位: {target_heading:.2f}度")

    loop_count = 0
    
    while loop_count < max_turn_attempts:
        current_heading = bno_sensor_instance.getVector(BNO055.VECTOR_EULER)[0]
        if current_heading is None:
            print("警告: [Turn] 旋回中に方位が取得できませんでした。スキップします。")
            driver_instance.motor_stop_brake()
            time.sleep(0.1)
            loop_count += 1
            continue

        angle_error = (target_heading - current_heading + 180 + 360) % 360 - 180

        if abs(angle_error) <= angle_tolerance_deg:
            print(f"[Turn] 相対回頭完了。最終誤差: {angle_error:.2f}度 (試行回数: {loop_count})")
            driver_instance.motor_stop_brake()
            time.sleep(0.5)
            return True

        turn_duration_on = 0.02 + (abs(angle_error) / 180.0) * 0.2
        if angle_error < 0: # 左旋回
            driver_instance.petit_left(0, turn_speed)
            driver_instance.petit_left(turn_speed, 0)
        else: # 右旋回
            driver_instance.petit_right(0, turn_speed)
            driver_instance.petit_right(turn_speed, 0)
            
        time.sleep(turn_duration_on)
        driver_instance.motor_stop_brake()
        time.sleep(0.05)
        
        loop_count += 1
    
    print(f"警告: [Turn] 最大試行回数({max_turn_attempts}回)内に目標角度に到達できませんでした。最終誤差: {angle_error:.2f}度 (試行回数: {loop_count})")
    driver_instance.motor_stop_brake()
    time.sleep(0.5)
    return False


# --- 各フェーズの関数定義 ---

def getEM_release():
    """放出判定を行います。この段階ではまだGPS通信は開始されません。"""
    print("放出判定中...")
    time.sleep(1) # 処理に時間がかかることをシミュレート
    print("放出判定完了。")

def getEM_land(bno_sensor_instance):
    """
    着地判定を行います。BME280とBNO055センサーを使用します。
    Args:
        bno_sensor_instance (BNO055): 初期化済みのBNO055センサーインスタンス。
    Returns:
        bool: 着地が成功したか（タイムアウト含む）どうか。
    """
    # BME280センサーの初期化と補正係数読み込み
    try:
        init_bme280_sensors()
        read_bme280_compensate_params()
    except IOError as e:
        print(f"ERROR: [着地判定] BME280初期化エラー: {e}")
        return False

    # BNO055の初期化はメインシーケンスで行われている前提
    # ここではBNO055のキャリブレーション待機を行う（もし必要なら）
    print("\n⚙️ BNO055 キャリブレーション中... (着地判定用)")
    calibration_start_time = time.time()
    # 実際にはBNO055のキャリブレーションは、起動後継続的に行われ、
    # 特定のキャリブレーションレベルに達するまで待機することが多いです。
    # ここでは簡易的にジャイロと地磁気のレベル3を待ちます。
    while True:
        sys_cal, gyro_cal, accel_cal, mag_cal = bno_sensor_instance.getCalibration()
        print(f"  現在のキャリブレーション状態 → システム:{sys_cal}, ジャイロ:{gyro_cal}, 加速度:{accel_cal}, 地磁気:{mag_cal} ", end='\r')
        sys.stdout.flush() # 出力バッファをフラッシュしてすぐに表示
        if gyro_cal == 3 and mag_cal == 3: # ジャイロと地磁気が完全キャリブレーション (レベル3)
            print("\n✅ BNO055 キャリブレーション完了！")
            break
        if time.time() - calibration_start_time > 30: # 30秒でタイムアウト
            print("\n⚠️ BNO055 キャリブレーションにタイムアウトしました。精度が低下する可能性があります。")
            break
        time.sleep(0.5) # 0.5秒ごとに状態を確認

    # 着地判定パラメータ
    pressure_change_threshold = 0.1 # 気圧の変化量閾値 (hPa)。この値以下になったら条件成立。
    acc_threshold_abs = 0.5         # 線形加速度の絶対値閾値 (m/s²)。
    gyro_threshold_abs = 0.5        # 角速度の絶対値閾値 (°/s)。
    consecutive_checks = 3          # 着地判定が連続して成立する必要のある回数。
    timeout = 60                    # 判定を打ち切るタイムアウト時間 (秒)。

    print("\n🛬 着地判定開始...")
    print(f"    気圧変化量閾値: < {pressure_change_threshold:.2f} hPa")
    print(f"    加速度絶対値閾値: < {acc_threshold_abs:.2f} m/s² (X, Y, Z軸)")
    print(f"    角速度絶対値閾値: < {gyro_threshold_abs:.2f} °/s (X, Y, Z軸)")
    print(f"    連続成立回数: {consecutive_checks}回")
    print(f"    タイムアウト: {timeout}秒\n")

    landing_count = 0 # 連続成立回数
    start_time = time.time()
    last_check_time = time.time() # 前回のチェック時刻
    previous_pressure = None # 気圧変化量を追跡するための変数

    try:
        # ヘッダーを一度だけ出力
        print(f"{'Timestamp(s)':<15}{'Elapsed(s)':<12}{'Pressure(hPa)':<15}{'P_Chg(hPa)':<18}{'Acc_X':<8}{'Acc_Y':<8}{'Acc_Z':<8}{'Gyro_X':<8}{'Gyro_Y':<8}{'Gyro_Z':<8}")
        print("-" * 120)

        while True:
            current_time = time.time()
            elapsed_total = current_time - start_time

            # タイムアウト判定
            if elapsed_total > timeout:
                print(f"\n⏰ タイムアウト ({timeout}秒経過)。条件成立回数 {landing_count} 回でしたが、強制的に着地判定を成功とします。")
                return True
            
            # データ取得と表示は一定間隔で行う
            if (current_time - last_check_time) < 0.2: # 約0.2秒間隔でデータ取得と表示
                time.sleep(0.01) # 短いスリープでCPU負荷軽減
                continue
            
            last_check_time = current_time

            # センサーデータの取得
            current_pressure, _ = get_pressure_and_temperature()
            if current_pressure is None:
                print("警告: [着地判定] BME280データ取得失敗。スキップします。")
                time.sleep(0.1)
                continue
                
            acc_x, acc_y, acc_z = bno_sensor_instance.getVector(BNO055.VECTOR_LINEARACCEL)
            gyro_x, gyro_y, gyro_z = bno_sensor_instance.getVector(BNO055.VECTOR_GYROSCOPE)

            # 気圧変化量の計算
            pressure_delta = float('inf') # 初回は非常に大きな値にして条件を満たさないようにする
            if previous_pressure is not None:
                pressure_delta = abs(current_pressure - previous_pressure)
            
            # データをコンソールに整形して出力
            print(f"{current_time:<15.3f}{elapsed_total:<12.1f}{current_pressure:<15.2f}{pressure_delta:<18.2f}{acc_x:<8.2f}{acc_y:<8.2f}{acc_z:<8.2f}{gyro_x:<8.2f}{gyro_y:<8.2f}{gyro_z:<8.2f}", end='\r')

            # 着地条件の判定
            is_landing_condition_met = (
                pressure_delta <= pressure_change_threshold and
                abs(acc_x) < acc_threshold_abs and
                abs(acc_y) < acc_threshold_abs and
                abs(acc_z) < acc_threshold_abs and
                abs(gyro_x) < gyro_threshold_abs and
                abs(gyro_y) < gyro_threshold_abs and
                abs(gyro_z) < gyro_threshold_abs
            )

            # 次のループのために現在の気圧を保存
            previous_pressure = current_pressure

            if is_landing_condition_met:
                landing_count += 1
                print(f"\n💡 条件成立！連続判定中: {landing_count}/{consecutive_checks} 回")
            else:
                if landing_count > 0:
                    print(f"\n--- 条件不成立。カウントリセット ({landing_count} -> 0) ---")
                landing_count = 0

            # 連続成立回数の確認
            if landing_count >= consecutive_checks:
                print(f"\n🎉 着地判定成功！連続 {consecutive_checks} 回条件成立！")
                return True

    except KeyboardInterrupt:
        print("\n\nプログラムがユーザーによって中断されました。")
        return False
    except Exception as e:
        print(f"\n\n🚨 着地判定中にエラーが発生しました: {e}")
        return False
    finally:
        print("\n--- 着地判定処理終了 ---")


def getparakai(driver_instance, bno_sensor_instance, picam2_instance):
    """
    パラシュート回避行動を行います。カメラとBNO055センサーを使用します。
    Args:
        driver_instance (MotorDriver): モータードライバーインスタンス。
        bno_sensor_instance (BNO055): BNO055センサーインスタンス。
        picam2_instance (Picamera2): Picamera2インスタンス。
    Returns:
        bool: 回避行動が成功したと判断された場合。
    """
    print("パラシュート回避を開始します")
    # ここにメインの自律走行ループから持ってきたパラシュート回避のロジックを組み込む

    # === BNO055キャリブレーション待機 ===
    # メインシーケンスで既にキャリブレーションしているはずだが、念のため簡易確認
    print("[回避] BNO055のキャリブレーション状態確認中...")
    sys_cal, gyro_cal, accel_cal, mag_cal = bno_sensor_instance.getCalibration()
    print(f"[回避] Calib → Sys:{sys_cal}, Gyro:{gyro_cal}, Acc:{accel_cal}, Mag:{mag_cal}")
    if gyro_cal != 3 or mag_cal != 3:
        print("警告: [回避] BNO055のキャリブレーションが不十分です。方位制御の精度が落ちる可能性があります。")
        
    avoidance_confirmed_clear = False
    
    # 旗を検出した後のカメラとモーター制御のループ
    # この部分のロジックは main_rover_control.py の「メインの自律走行ループ」から持ってきたものです。
    # 完全に同じロジックを繰り返すか、調整が必要です。
    # 今回は簡潔にするため、主要な回避ロジックのみを抽出します。
    # もし目標方位を維持しつつ回避が必要なら、GPS目標方位を再度取得する必要があります。
    
    # 仮の目標GPS方位（回避動作後に再調整するため）
    # この値は実際のミッション開始時の目標方位と同じか、状況に応じて再計算が必要です。
    # 今回はGOAL_LOCATIONへの方位を仮に設定します。
    # get_current_locationがem_gps_datalinkに移動したので、そちらから取得するか、
    # main_sequenceからgps_datalinkインスタンスを受け取る必要があります。
    
    # 仮のGPS現在地と目標地。実際にはループ内で更新が必要。
    current_gps_coords = None 
    # GPSデータをEmGpsDatalinkから取得
    gps_data_from_thread = gps_datalink_instance.get_current_gps()
    if gps_data_from_thread:
        current_gps_coords = (gps_data_from_thread['latitude'], gps_data_from_thread['longitude'])
    else:
        print("警告: [回避] GPSデータがまだ利用できません。方位調整が正確に行えない可能性があります。")
    
    target_gps_heading = get_bearing_to_goal(current_gps_coords, GOAL_LOCATION) if current_gps_coords else bno_sensor_instance.getVector(BNO055.VECTOR_EULER)[0] # GPSがなければ現在のBNO方位を維持

    try:
        # STEP 4 & 5: カメラ検知と前進 (パラシュート回避の主要ロジック)
        print("\n=== パラシュート回避: カメラ検知と回避行動 ===")
        
        red_location_result = detect_red_in_grid(picam2_instance, save_path="/home/mark1/Pictures/akairo_grid.jpg", min_red_pixel_ratio_per_cell=0.10)

        if red_location_result == 'left_bottom':
            print("赤色が左下に検出されました → 右に回頭します")
            turn_to_relative_angle(driver_instance, bno_sensor_instance, 90, turn_speed=90, angle_tolerance_deg=20) # 右90度
            print("回頭後、少し前進します")
            following.follow_forward(driver_instance, bno_sensor_instance, base_speed=100, duration_time=5)
        elif red_location_result == 'right_bottom':
            print("赤色が右下に検出されました → 左に回頭します")
            turn_to_relative_angle(driver_instance, bno_sensor_instance, -90, turn_speed=90, angle_tolerance_deg=20) # 左90度
            print("回頭後、少し前進します")
            following.follow_forward(driver_instance, bno_sensor_instance, base_speed=100, duration_time=5)
        elif red_location_result == 'bottom_middle':
            print("赤色が下段中央に検出されました → 右に120度回頭して前進します")
            turn_to_relative_angle(driver_instance, bno_sensor_instance, 120, turn_speed=90, angle_tolerance_deg=20) # 右120度
            driver_instance.motor_stop_brake()
            time.sleep(0.5)

            print("さらに左に30度回頭し、前進します。")
            turn_to_relative_angle(driver_instance, bno_sensor_instance, -30, turn_speed=90, angle_tolerance_deg=20) # 左に30度回頭
            following.follow_forward(driver_instance, bno_sensor_instance, base_speed=100, duration_time=5)
        elif red_location_result == 'high_percentage_overall':
            print("画像全体に高割合で赤色を検出 → パラシュートが覆いかぶさっている可能性。長く待機して様子を見ます")
            time.sleep(10)
            print("待機後、少し前進します")
            following.follow_forward(driver_instance, bno_sensor_instance, base_speed=90, duration_time=3)
        elif red_location_result == 'none_detected':
            print("赤色を検出しませんでした → 方向追従制御で前進します。(速度80, 5秒)")
            following.follow_forward(driver_instance, bno_sensor_instance, base_speed=90, duration_time=5)
        elif red_location_result == 'error_in_processing':
            print("カメラ処理でエラーが発生しました。少し待機します...")
            time.sleep(2)

        driver_instance.motor_stop_brake()

        # ★★★ 回避後の再確認ロジック（3点スキャン） ★★★
        print("\n=== 回避後の周囲確認を開始します (3点スキャン) ===")
        avoidance_confirmed_clear = False
        
        # 3回まで再回避試行を許可するループ
        for _ in range(3): # 最大3回再試行
            # 1. ローバーを目的地のGPS方向へ再度向ける
            # このフェーズではGPSデータリンクが生きているので、最新のGPS方位を取得
            current_gps_coords = None
            gps_data_from_thread = gps_datalink_instance.get_current_gps() # グローバル変数としてアクセス
            if gps_data_from_thread:
                current_gps_coords = (gps_data_from_thread['latitude'], gps_data_from_thread['longitude'])
                target_gps_heading = get_bearing_to_goal(current_gps_coords, GOAL_LOCATION)
            else:
                print("警告: [回避] GPSデータが利用できません。目的地方位への再調整はスキップされます。")
                target_gps_heading = bno_sensor_instance.getVector(BNO055.VECTOR_EULER)[0] # GPSなしでBNOの方位を使用
            
            if target_gps_heading is not None:
                print("\n=== 回避後: 再度目的地の方位へ回頭 ===")
                turn_to_relative_angle(driver_instance, bno_sensor_instance, target_gps_heading - bno_sensor_instance.getVector(BNO055.VECTOR_EULER)[0], turn_speed=80, angle_tolerance_deg=20)
            
            # 2. 正面、左30度、右30度の3方向で赤色検知
            scan_results = {
                'front': 'none_detected',
                'left_30': 'none_detected',
                'right_30': 'none_detected'
            }
            
            print("→ 正面方向の赤色を確認します...")
            scan_results['front'] = detect_red_in_grid(picam2_instance, save_path="/home/mark1/Pictures/confirm_front.jpg", min_red_pixel_ratio_per_cell=0.10)

            print("→ 左に30度回頭し、赤色を確認します...")
            turn_to_relative_angle(driver_instance, bno_sensor_instance, -30, turn_speed=90, angle_tolerance_deg=20) # 左30度
            scan_results['left_30'] = detect_red_in_grid(picam2_instance, save_path="/home/mark1/Pictures/confirm_left.jpg", min_red_pixel_ratio_per_cell=0.10)
            print("→ 左30度から正面に戻します...")
            turn_to_relative_angle(driver_instance, bno_sensor_instance, 30, turn_speed=90, angle_tolerance_deg=20) # 右30度で戻す

            print("→ 右に30度回頭し、赤色を確認します...")
            turn_to_relative_angle(driver_instance, bno_sensor_instance, 30, turn_speed=90, angle_tolerance_deg=20) # 右30度
            scan_results['right_30'] = detect_red_in_grid(picam2_instance, save_path="/home/mark1/Pictures/confirm_right.jpg", min_red_pixel_ratio_per_cell=0.10)
            print("→ 右30度から正面に戻します...")
            turn_to_relative_angle(driver_instance, bno_sensor_instance, -30, turn_speed=90, angle_tolerance_deg=20) # 左30度で戻す

            # 3方向の結果を評価
            is_front_clear = (scan_results['front'] == 'none_detected')
            is_left_clear = (scan_results['left_30'] == 'none_detected')
            is_right_clear = (scan_results['right_30'] == 'none_detected')

            if is_front_clear and is_left_clear and is_right_clear:
                print("\n=== 3点スキャン結果: 全ての方向でパラシュートは検出されませんでした。回避成功！ ===")
                avoidance_confirmed_clear = True
                break # 再回避ループを終了
            else:
                print("\n=== 3点スキャン結果: まだパラシュートが検出されました。再回避を試みます。 ===")
                print(f"検出詳細: 正面: {scan_results['front']}, 左30: {scan_results['left_30']}, 右30: {scan_results['right_30']}")
                
                # 検出された方向に基づいて再回避行動を選択
                if scan_results['left_30'] != 'none_detected': # 左30度で検出されたら右90度
                    print("[回避] 左30度で検出されたため、右90度回頭して回避します。")
                    turn_to_relative_angle(driver_instance, bno_sensor_instance, 90, turn_speed=90, angle_tolerance_deg=20)
                elif scan_results['right_30'] != 'none_detected': # 右30度で検出されたら左90度
                    print("[回避] 右30度で検出されたため、左90度回頭して回避します。")
                    turn_to_relative_angle(driver_instance, bno_sensor_instance, -90, turn_speed=90, angle_tolerance_deg=20)
                elif scan_results['front'] != 'none_detected': # 正面で検出されたら右120度
                    print("[回避] 正面で検出されたため、右120度回頭して回避します。")
                    turn_to_relative_angle(driver_instance, bno_sensor_instance, 120, turn_speed=90, angle_tolerance_deg=20)
                    driver_instance.motor_stop_brake()
                    time.sleep(0.5)

                    print("[回避] さらに左に30度回頭し、前進します。")
                    turn_to_relative_angle(driver_instance, bno_sensor_instance, -30, turn_speed=90, angle_tolerance_deg=20) # 左に30度回頭
                    following.follow_forward(driver_instance, bno_sensor_instance, base_speed=100, duration_time=5)
                else: # その他の場合 (例えばエラーで検出された場合など、念のため)
                    print("[回避] 詳細不明な検出のため、右120度回頭して回避します。")
                    turn_to_relative_angle(driver_instance, bno_sensor_instance, 120, turn_speed=90, angle_tolerance_deg=20.0)
                
                following.follow_forward(driver_instance, bno_sensor_instance, base_speed=90, duration_time=5) # 少し前進
                driver_instance.motor_stop_brake()
                time.sleep(1) # 再回避後のクールダウン
                
                # 再回避ループの先頭に戻り、再度3点スキャンを試みる

        if not avoidance_confirmed_clear:
            print("警告: [回避] 複数回の回避試行後もパラシュートのクリアを確認できませんでした。")
            return False # 回避失敗と判断

    except Exception as e:
        print(f"ERROR: [パラシュート回避] 処理中に予期せぬエラーが発生しました: {e}")
        driver_instance.motor_stop_brake()
        return False
    
    print("パラシュート回避完了。")
    return True


def getEM_excellent_gps(driver_instance, bno_sensor_instance, gps_datalink_instance):
    """
    GPS誘導で第1フラッグまで移動します。
    Args:
        driver_instance (MotorDriver): モータードライバーインスタンス。
        bno_sensor_instance (BNO055): BNO055センサーインスタンス。
        gps_datalink_instance (EmGpsDatalink): EmGpsDatalinkインスタンス。
    Returns:
        bool: ナビゲーションが成功したかどうか。
    """
    print("GPS誘導中（第1フラッグまで）...")
    
    # === 制御パラメータ (チューニング用) ===
    # GOAL_LOCATION はこの関数の外で定義されているものを参照します
    GOAL_THRESHOLD_M = 5.0      # ゴールとみなす距離 (メートル)
    ANGLE_THRESHOLD_DEG = 15.0  # これ以上の角度誤差があれば回頭する (度)
    TURN_SPEED = 45             # 回頭時のモーター速度 (0-100)
    MOVE_SPEED = 80             # 前進時の基本速度 (0-100)
    MOVE_DURATION_S = 1.5       # 一回の前進時間 (秒)

    # === PD制御パラメータ ===
    Kp = 0.50   # 比例ゲイン: 誤差に対する反応の強さ
    Kd = 0.15   # 微分ゲイン: 揺り戻しを抑制し、動きを滑らかにする

    try:
        # BNO055キャリブレーション待機
        # メインシーケンスで既にキャリブレーションしているはずだが、念のため簡易確認
        print("[GPS誘導] BNO055のキャリブレーション状態確認中...")
        sys_cal, gyro_cal, accel_cal, mag_cal = bno_sensor_instance.getCalibration()
        print(f"[GPS誘導] Calib → Sys:{sys_cal}, Gyro:{gyro_cal}, Acc:{accel_cal}, Mag:{mag_cal}")
        if gyro_cal != 3 or mag_cal != 3:
            print("警告: [GPS誘導] BNO055のキャリブレーションが不十分です。ナビゲーションの精度が落ちる可能性があります。")

        while True:
            # 1. 状態把握
            current_location = None
            gps_data_from_thread = gps_datalink_instance.get_current_gps()
            if gps_data_from_thread:
                current_location = [gps_data_from_thread['latitude'], gps_data_from_thread['longitude']]
            
            if not current_location:
                print("[WARN] [GPS誘導] GPS位置情報を取得できません。リトライします...")
                driver_instance.motor_stop_brake()
                time.sleep(1)
                continue

            heading = bno_sensor_instance.getVector(BNO055.VECTOR_EULER)[0]
            if heading is None:
                print("[WARN] [GPS誘導] BNO055から方位角を取得できません。リトライします...")
                driver_instance.motor_stop_brake()
                time.sleep(1)
                continue

            # 2. 計算
            dist_to_goal = get_distance_to_goal(current_location, GOAL_LOCATION)
            bearing_to_goal = get_bearing_to_goal(current_location, GOAL_LOCATION)
            angle_error = (bearing_to_goal - heading + 360) % 360

            print(f"[INFO] [GPS誘導] 距離:{dist_to_goal: >6.1f}m | 目標方位:{bearing_to_goal: >5.1f}° | 現在方位:{heading: >5.1f}°")

            # 3. ゴール判定
            if dist_to_goal <= GOAL_THRESHOLD_M:
                print(f"[GOAL] 第1フラッグ（目標地点）に到達しました！ (距離: {dist_to_goal:.2f}m)")
                driver_instance.motor_stop_brake()
                return True

            # 4. 方向調整フェーズ
            if angle_error > ANGLE_THRESHOLD_DEG and angle_error < (360 - ANGLE_THRESHOLD_DEG):
                turn_speed = 40 # 回転速度は固定 (0-100)
                turn_duration = 0.15 + (min(angle_error, 360 - angle_error) / 180.0) * 0.2

                if angle_error > 180: # 反時計回り（左）に回る方が近い
                    print(f"[TURN] [GPS誘導] 左に回頭します ({turn_duration:.2f}秒)")
                    driver_instance.changing_left(0, turn_speed)
                    driver_instance.changing_left(turn_speed, 0)
                    time.sleep(turn_duration)
                else: # 時計回り（右）に回る方が近い
                    print(f"[TURN] [GPS誘導] 右に回頭します ({turn_duration:.2f}秒)")
                    driver_instance.changing_right(0, turn_speed)
                    driver_instance.changing_right(turn_speed, 0)
                    time.sleep(turn_duration)
                
                driver_instance.motor_stop_brake()
                time.sleep(0.5) # 回転後の安定待ち
                continue # 方向調整が終わったら、次のループで再度GPSと方位を確認

            # 5. 前進フェーズ (PD制御による直進維持)
            print(f"[MOVE] [GPS誘導] 方向OK。PD制御で前進します。")
            # `following.follow_forward`は内部でBNOセンサーを使って直進を維持
            following.follow_forward(driver_instance, bno_sensor_instance, 70, 8) # 速度70, 8秒前進

    except KeyboardInterrupt:
        print("\n[STOP] [GPS誘導] 手動で停止されました。")
        driver_instance.motor_stop_brake()
        return False
    except Exception as e:
        print(f"\n[FATAL] [GPS誘導] 予期せぬエラーが発生しました: {e}")
        driver_instance.motor_stop_brake()
        return False
    finally:
        print("[GPS誘導] 処理終了。")

    print("第1フラッグ到達。")
    return True


def getEM_Flag_Navigate(driver_instance, picam2_instance, bno_sensor_instance):
    """
    フラッグ誘導（カメラによる目標追跡）を行います。
    Args:
        driver_instance (MotorDriver): モータードライバーインスタンス。
        picam2_instance (Picamera2): Picamera2インスタンス。
        bno_sensor_instance (BNO055): BNO055センサーインスタンス。
    Returns:
        bool: 目標フラッグへの接近が成功したかどうか。
    """
    print("フラッグ誘導中...")
    
    # フラッグ検出器の初期化
    detector = FlagDetector(picam2_instance=picam2_instance) # Picamera2インスタンスを渡す
    screen_area = detector.width * detector.height
    
    # フラッグ追跡設定
    TARGET_SHAPES = ["三角形", "長方形"] # 追跡する図形のリスト
    AREA_THRESHOLD_PERCENT = 20.0 # 画面占有率がこの値を超えたら接近完了とみなす

    try:
        # --- 全てのターゲットに対してループ ---
        for target_name in TARGET_SHAPES:
            print(f"\n---====== 新しい目標: [{target_name}] の探索を開始します ======---")
            
            task_completed = False
            while not task_completed:
                
                # --- 探索 ---
                print(f"[{target_name}] を探しています...")
                detected_data = detector.detect() # Picamera2はdetector内部で処理される
                target_flag = None
                for flag in detected_data: # find_target_flagのロジックをここに展開
                    for shape in flag['shapes']:
                        if shape['name'] == target_name:
                            target_flag = flag
                            break
                    if target_flag: break # 見つかったらループを抜ける

                # 見つからない場合は回転して探索
                if target_flag is None:
                    print(f"[{target_name}] が見つかりません。探索行動を開始します。")
                    search_count = 0
                    while target_flag is None and search_count < 40: # タイムアウト設定 (約8秒)
                        
                        # 探索のための動き: 少し前進しつつ、その場で全方位スキャン
                        print("[フラッグ誘導] 探索中: 少し前進し、その場で回転してスキャン")
                        
                        # 短く前進
                        following.follow_forward(driver_instance, bno_sensor_instance, base_speed=60, duration_time=1)
                        driver_instance.motor_stop_brake()
                        time.sleep(0.5)

                        # 全方位スキャン（左右に回頭しつつ検出を試みる）
                        initial_heading_scan = bno_sensor_instance.getVector(BNO055.VECTOR_EULER)[0]
                        if initial_heading_scan is None:
                            print("警告: [フラッグ誘導] 探索中にBNO方位が取得できません。スキャンをスキップ。")
                            break
                        
                        scan_angle_step = 30 # 30度ずつスキャン
                        num_scans = 12 # 360度をカバーするために12回 (360/30)
                        
                        for i in range(num_scans):
                            detected_data = detector.detect()
                            target_flag = None
                            for flag in detected_data:
                                for shape in flag['shapes']:
                                    if shape['name'] == target_name:
                                        target_flag = flag
                                        break
                                if target_flag: break
                            
                            if target_flag:
                                print(f"[フラッグ誘導] [{target_name}] 探索中に発見しました！")
                                break # 探索スキャンループを抜ける

                            if i < num_scans - 1:
                                # 少しずつ回頭
                                turn_to_relative_angle(driver_instance, bno_sensor_instance, scan_angle_step, turn_speed=50, angle_tolerance_deg=10, max_turn_attempts=10)
                            
                            search_count += 1
                            if search_count >= 40: # 全体の探索タイムアウト
                                break

                        if target_flag:
                            break # 見つかったら探索フェーズ全体を抜ける
                        
                        if search_count >= 40:
                            print(f"[フラッグ誘導] 探索しましたが [{target_name}] は見つかりませんでした。次の目標に移ります。")
                            return False # このターゲットの追跡を諦める

                # --- 追跡（中央寄せ＆接近）---
                if target_flag:
                    print(f"[{target_name}] を発見！追跡を開始します。")
                    while target_flag:
                        # --- 中央寄せ ---
                        if target_flag['location'] != '中央':
                            print(f"[フラッグ誘導] 位置を調整中... (現在位置: {target_flag['location']})")
                            if target_flag['location'] == '左':
                                turn_to_relative_angle(driver_instance, bno_sensor_instance, -15, turn_speed=60, angle_tolerance_deg=5) # 左に少し回頭
                            elif target_flag['location'] == '右':
                                turn_to_relative_angle(driver_instance, bno_sensor_instance, 15, turn_speed=60, angle_tolerance_deg=5) # 右に少し回頭
                            
                            # 動かした直後に再検出
                            print("  [フラッグ誘導] 再検出中...")
                            detected_data = detector.detect()
                            target_flag = None
                            for flag in detected_data:
                                for shape in flag['shapes']:
                                    if shape['name'] == target_name:
                                        target_flag = flag
                                        break
                                if target_flag: break # 見つかったら抜ける
                            
                            if not target_flag:
                                print(f"[フラッグ誘導] 調整中に [{target_name}] を見失いました。")
                                break # 追跡ループを抜ける
                            
                            continue # 位置を再評価するため、ループの最初に戻る
                            
                        # --- 接近 ---
                        else: # 中央にいる場合
                            flag_area = cv2.contourArea(target_flag['flag_contour'])
                            area_percent = (flag_area / screen_area) * 100
                            print(f"[フラッグ誘導] 中央に補足。接近中... (画面占有率: {area_percent:.1f}%)")

                            # 面積の比較
                            if area_percent >= AREA_THRESHOLD_PERCENT:
                                print(f"[フラッグ誘導] [{target_name}] に接近完了！")
                                task_completed = True
                                time.sleep(1)
                                break # 追跡ループを抜ける
                            else:
                                # しきい値未満なら、前進
                                following.follow_forward(driver_instance, bno_sensor_instance, base_speed=40, duration_time=1) # ゆっくり前進
                                driver_instance.motor_stop_brake()
                                time.sleep(0.1)
                                
                            # 動作後に再検出
                            print("  [フラッグ誘導] 再検出中...")
                            detected_data = detector.detect()
                            target_flag = None
                            for flag in detected_data:
                                for shape in flag['shapes']:
                                    if shape['name'] == target_name:
                                        target_flag = flag
                                        break
                                if target_flag: break # 見つかったら抜ける
                            
                            if not target_flag:
                                print(f"[フラッグ誘導] 追跡中に [{target_name}] を見失いました。再探索します。")
                                break # 追跡ループ(while target_flag)を抜ける
                else: # target_flag が None の場合 (探索で何も見つからなかった場合)
                    break # task_completed == False のまま外側のループを抜ける

            if task_completed:
                print(f"[{target_name}] のフラッグ誘導が成功しました。")
            else:
                print(f"警告: [{target_name}] のフラッグ誘導が完了しませんでした。次の目標に進みます。")
                # ここでFalseを返すと、ミッション全体が停止する可能性があります。
                # 次のフラッグに進むのが自然な動作なら、Falseを返さずループ続行。
                # 今回は全てのターゲットを順に処理する想定なので、Falseを返してミッション失敗とすることも可能。
                # ここではミッションを続けるため、何も返さない。

        print("\n---====== 全ての目標の探索が完了しました ======---")
        return True # 全てのフラッグ誘導が完了したとみなす
    
    except Exception as e:
        print(f"ERROR: [フラッグ誘導] 処理中に予期せぬエラーが発生しました: {e}")
        driver_instance.motor_stop_brake()
        return False
    finally:
        print("[フラッグ誘導] 処理終了。")


def getcamera():
    """
    物資設置を行います（サーボモーター操作）。
    Args:
        なし（サーボ制御は直接行う）
    Returns:
        bool: 物資設置が成功したかどうか。
    """
    print("物資設置中（カメラ操作を伴う）...")
    
    # サーボモーターのGPIOピン設定
    SERVO_PIN = 13
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(SERVO_PIN, GPIO.OUT)

    pwm = None
    try:
        # 50Hz の PWM波形（サーボ用）
        pwm = GPIO.PWM(SERVO_PIN, 50)
        pwm.start(0)

        def set_servo_duty(duty):
            pwm.ChangeDutyCycle(duty)
            time.sleep(0.5)

        def changing_servo_reverse(before, after):
            # この関数は線形にデューティサイクルを変化させるように見えますが、
            # 実際のサーボモーター制御では単純なsleepでは滑らかな動きになりにくいです。
            # また、speedというグローバル変数は定義されていません。
            # ここではシンプルに最終的なデューティサイクルを設定する形にします。
            # もしこのアニメーション的な動きが必要なら、改めて検討が必要です。
            num_steps = 100
            for i in range(num_steps):
                current_duty = before + (after - before) * i / (num_steps - 1)
                set_servo_duty(current_duty)
                time.sleep(0.01) # より短いsleepで滑らかにする

        # 物資投下動作のシミュレーション（サーボを動かす）
        print("[物資設置] サーボを逆回転（物資投下）させます。")
        set_servo_duty(4.0) # 逆回転のデューティサイクルに設定
        time.sleep(5) # 10秒は長すぎるため5秒に短縮
        
        print("[物資設置] サーボを停止します。")
        set_servo_duty(7.5) # 停止位置 (中央) に戻す
        time.sleep(1)

        print("物資設置完了。")
        return True

    except Exception as e:
        print(f"ERROR: [物資設置] 処理中にエラーが発生しました: {e}")
        return False
    finally:
        if pwm:
            pwm.stop()
        GPIO.cleanup(SERVO_PIN) # サーボピンのみクリーンアップ
        print("[物資設置] サーボクリーンアップ完了。")


def getmotor(driver_instance, bno_sensor_instance, gps_datalink_instance):
    """
    ゴールまでGPS誘導を行います（モーター制御）。
    Args:
        driver_instance (MotorDriver): モータードライバーインスタンス。
        bno_sensor_instance (BNO055): BNO055センサーインスタンス。
        gps_datalink_instance (EmGpsDatalink): EmGpsDatalinkインスタンス。
    Returns:
        bool: ゴールに到達したかどうか。
    """
    print("ゴールまでGPS誘導中（モーター制御を伴う）...")
    
    # === 制御パラメータ (チューニング用) ===
    # GOAL_LOCATION はこの関数の外で定義されているものを参照します
    GOAL_THRESHOLD_M = 1.0 # 最終的なゴール判定はより厳しくする（例：1メートル以内）
    ANGLE_THRESHOLD_DEG = 10.0 # 最終的な調整はより厳しくする
    TURN_SPEED = 40
    MOVE_SPEED = 60
    MOVE_DURATION_S = 1.0

    try:
        # BNO055キャリブレーション待機
        print("[ゴールGPS誘導] BNO055のキャリブレーション状態確認中...")
        sys_cal, gyro_cal, accel_cal, mag_cal = bno_sensor_instance.getCalibration()
        print(f"[ゴールGPS誘導] Calib → Sys:{sys_cal}, Gyro:{gyro_cal}, Acc:{accel_cal}, Mag:{mag_cal}")
        if gyro_cal != 3 or mag_cal != 3:
            print("警告: [ゴールGPS誘導] BNO055のキャリブレーションが不十分です。ナビゲーションの精度が落ちる可能性があります。")

        while True:
            # 1. 状態把握 (GPSデータはEmGpsDatalinkスレッドから取得)
            current_location = None
            gps_data_from_thread = gps_datalink_instance.get_current_gps()
            if gps_data_from_thread:
                current_location = [gps_data_from_thread['latitude'], gps_data_from_thread['longitude']]
            
            if not current_location:
                print("[WARN] [ゴールGPS誘導] GPS位置情報を取得できません。リトライします...")
                driver_instance.motor_stop_brake()
                time.sleep(1)
                continue

            heading = bno_sensor_instance.getVector(BNO055.VECTOR_EULER)[0]
            if heading is None:
                print("[WARN] [ゴールGPS誘導] BNO055から方位角を取得できません。リトライします...")
                driver_instance.motor_stop_brake()
                time.sleep(1)
                continue

            # 2. 計算
            dist_to_goal = get_distance_to_goal(current_location, GOAL_LOCATION)
            bearing_to_goal = get_bearing_to_goal(current_location, GOAL_LOCATION)
            angle_error = (bearing_to_goal - heading + 360) % 360

            print(f"[INFO] [ゴールGPS誘導] 距離:{dist_to_goal: >6.1f}m | 目標方位:{bearing_to_goal: >5.1f}° | 現在方位:{heading: >5.1f}°")

            # 3. ゴール判定
            if dist_to_goal <= GOAL_THRESHOLD_M:
                print(f"[GOAL] ゴール地点に到達しました！ (距離: {dist_to_goal:.2f}m)")
                driver_instance.motor_stop_brake()
                return True

            # 4. 方向調整フェーズ
            if angle_error > ANGLE_THRESHOLD_DEG and angle_error < (360 - ANGLE_THRESHOLD_DEG):
                turn_speed_actual = TURN_SPEED
                turn_duration_actual = 0.15 + (min(angle_error, 360 - angle_error) / 180.0) * 0.2

                if angle_error > 180: # 反時計回り（左）に回る方が近い
                    print(f"[TURN] [ゴールGPS誘導] 左に回頭します ({turn_duration_actual:.2f}秒)")
                    driver_instance.changing_left(0, turn_speed_actual)
                    driver_instance.changing_left(turn_speed_actual, 0)
                    time.sleep(turn_duration_actual)
                else: # 時計回り（右）に回る方が近い
                    print(f"[TURN] [ゴールGPS誘導] 右に回頭します ({turn_duration_actual:.2f}秒)")
                    driver_instance.changing_right(0, turn_speed_actual)
                    driver_instance.changing_right(turn_speed_actual, 0)
                    time.sleep(turn_duration_actual)
                
                driver_instance.motor_stop_brake()
                time.sleep(0.5) # 回転後の安定待ち
                continue

            # 5. 前進フェーズ (PD制御による直進維持)
            print(f"[MOVE] [ゴールGPS誘導] 方向OK。PD制御で前進します。")
            following.follow_forward(driver_instance, bno_sensor_instance, MOVE_SPEED, MOVE_DURATION_S)

    except KeyboardInterrupt:
        print("\n[STOP] [ゴールGPS誘導] 手動で停止されました。")
        driver_instance.motor_stop_brake()
        return False
    except Exception as e:
        print(f"\n[FATAL] [ゴールGPS誘導] 予期せぬエラーが発生しました: {e}")
        driver_instance.motor_stop_brake()
        return False
    finally:
        print("[ゴールGPS誘導] 処理終了。")

    print("ゴール付近に到達。")
    return True


def getEM_Goal_Detective_NOSHIRO(driver_instance, picam2_instance, bno_sensor_instance):
    """
    ゴール検知（赤色コーンのカメラ検出による最終ゴール判定）を行います。
    Args:
        driver_instance (MotorDriver): モータードライバーインスタンス。
        picam2_instance (Picamera2): Picamera2インスタンス。
        bno_sensor_instance (BNO055): BNO055センサーインスタンス。
    Returns:
        bool: ゴールが検知されたかどうか。
    """
    print("ゴール検知中...")

    # 赤色検出のためのHSV範囲（グローバル変数を参照）
    # lower_red1, upper_red1, lower_red2, upper_red2 は既に定義済み

    # 探索時のモータ回転数
    left_a = 90
    right_a = 80

    # counterの最大値 (赤コーンが見つからない場合の探索試行回数)
    counter_max = 5

    # --- ヘルパー関数群（ゴール検知専用）---
    def get_percentage(frame_raw):
        """フレーム中の赤色ピクセルの割合を計算します。"""
        # Picamera2のTransformで回転されている場合、ここではcv2.rotateは不要
        frame = cv2.cvtColor(frame_raw, cv2.COLOR_RGB2BGR)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask = cv2.bitwise_or(mask1, mask2)
        
        red_area = np.count_nonzero(mask)
        total_area = frame.shape[0] * frame.shape[1]
        percentage = (red_area / total_area) * 100
        print(f"[ゴール検知] 検知割合は{percentage:.2f}%です")
        return percentage

    def get_block_number_by_density(frame_raw):
        """画像を5分割し、最も赤の密度が高いブロックの番号（1〜5）を返します。"""
        frame = cv2.cvtColor(frame_raw, cv2.COLOR_RGB2BGR)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask = cv2.bitwise_or(mask1, mask2)
        
        height, width = mask.shape
        block_width = width // 5
        red_ratios = []
        for i in range(5):
            x_start = i * block_width
            x_end = (i + 1) * block_width if i < 4 else width
            block_mask = mask[:, x_start:x_end]
            red_count = np.count_nonzero(block_mask)
            total_count = block_mask.size
            ratio = red_count / total_count
            red_ratios.append(ratio)

        # デバッグ出力（オプション）
        for i, r in enumerate(red_ratios):
            print(f"[ゴール検知 DEBUG] ブロック{i+1}の赤密度: {r:.2%}")

        max_ratio = max(red_ratios)
        if max_ratio < 0.08: # 閾値は調整が必要かもしれません
            print("[ゴール検知] ❌ 赤色が検出されません（全ブロックで密度低）")
            return None
        else:
            block_num = red_ratios.index(max_ratio) + 1
            print(f"[ゴール検知] 一番密度の高いブロックは{block_num}です")
            return block_num

    try:
        counter = counter_max
        print("[ゴール検知] ゴール誘導を開始します")
        
        while True:
            if counter <= 0:
                print("[ゴール検知] 赤コーンが近くにありません。探索を行います")
                counter = counter_max # カウンターをリセットして再探索開始
                search_attempt_count = 0
                max_search_attempts = 10 # 探索時の最大試行回数
                
                while True: # 探索ループ
                    if search_attempt_count >= max_search_attempts:
                        print("[ゴール検知] 探索試行回数を超過しました。ゴール検知失敗。")
                        return False # ゴール検知失敗

                    print("[ゴール検知] 探索中: 少し前進し、その場で回転してスキャン")
                    # 照度条件が悪いかコーンが近くにないため、少し移動する。螺旋移動の一部をイメージ
                    
                    # 短く前進
                    following.follow_forward(driver_instance, bno_sensor_instance, base_speed=left_a, duration_time=2) # left_a, right_a は速度として使用
                    driver_instance.motor_stop_brake()
                    time.sleep(0.5)

                    # 全方位スキャン（左右に回頭しつつ検出を試みる）
                    initial_heading_scan = bno_sensor_instance.getVector(BNO055.VECTOR_EULER)[0]
                    if initial_heading_scan is None:
                        print("警告: [ゴール検知] 探索中にBNO方位が取得できません。スキャンをスキップ。")
                        break # 探索ループを抜ける
                    
                    scan_angle_step = 30 # 30度ずつスキャン
                    num_scans = 12 # 360度をカバーするために12回 (360/30)
                    
                    found_cone_in_scan = False
                    for i in range(num_scans):
                        frame = picam2_instance.capture_array()
                        if frame is None:
                            print("警告: [ゴール検知] カメラフレーム取得失敗。")
                            time.sleep(0.5)
                            continue

                        percentage = get_percentage(frame)
                        if percentage > 15: # コーンを検出したとみなすしきい値
                            print("[ゴール検知] 赤コーンの探索に成功しました")
                            found_cone_in_scan = True
                            break # スキャンループを抜ける

                        if i < num_scans - 1:
                            # 少しずつ回頭
                            turn_to_relative_angle(driver_instance, bno_sensor_instance, scan_angle_step, turn_speed=50, angle_tolerance_deg=10, max_turn_attempts=10)
                        
                    if found_cone_in_scan:
                        break # 外側の探索ループを抜ける
                    else:
                        print("[ゴール検知] 付近にはコーンを検知できなかったため、再度探索を行います")
                        search_attempt_count += 1
                        time.sleep(1) # 次の探索サイクルまで待機

            # メインのゴール追跡ロジック
            frame = picam2_instance.capture_array()
            if frame is None:
                print("警告: [ゴール検知] カメラフレーム取得失敗。リトライします。")
                time.sleep(1)
                continue

            percentage = get_percentage(frame)
            number = get_block_number_by_density(frame)
            
            print(f"[ゴール検知] 赤割合: {percentage:.2f}% ----- 画面場所:{number}です ")

            if percentage >= 90: # 十分に接近したと判断
                print("[ゴール検知] percentageでのゴール判定 - ゴール検知成功！")
                driver_instance.motor_stop_brake()
                return True
            elif number == 3: # 中央にいる場合
                if percentage > 40:
                    print("[ゴール検知] 中央、高割合: 小さく前進します (1回)")
                    following.follow_forward(driver_instance, bno_sensor_instance, base_speed=60, duration_time=0.5) # 短く前進
                elif percentage > 20:
                    print("[ゴール検知] 中央、中割合: 小さく前進します (3回分)")
                    following.follow_forward(driver_instance, bno_sensor_instance, base_speed=60, duration_time=1.5) # やや前進
                elif percentage > 10:
                    print("[ゴール検知] 中央、低割合: 小さく前進します (5回分)")
                    following.follow_forward(driver_instance, bno_sensor_instance, base_speed=60, duration_time=2.5) # そこそこ前進
                else:
                    print("[ゴール検知] 距離が遠いため、前進を行います")
                    following.follow_forward(driver_instance, bno_sensor_instance, base_speed=70, duration_time=2) # 通常前進
                driver_instance.motor_stop_brake()
                time.sleep(0.5)
                counter = counter_max # 正常に追跡できているのでカウンターをリセット
                
            elif number == 1 or number == 2: # 左に偏っている場合
                turn_angle = 10 if number == 2 else 20 # 2ブロックなら10度、1ブロックなら20度
                print(f"[ゴール検知] 左にコーンを検知 ({number}番ブロック) → 右に回頭します ({turn_angle}度)")
                turn_to_relative_angle(driver_instance, bno_sensor_instance, turn_angle, turn_speed=90, angle_tolerance_deg=5)
                driver_instance.motor_stop_brake()
                time.sleep(0.5)
                if percentage < 50: # 回頭後、割合が低ければ少し前進
                    print("[ゴール検知] 正面にとらえることができませんでしたが、検知割合が低いため、接近します")
                    following.follow_forward(driver_instance, bno_sensor_instance, base_speed=70, duration_time=1)
                counter = counter_max # 正常に追跡できているのでカウンターをリセット
            
            elif number == 4 or number == 5: # 右に偏っている場合
                turn_angle = -10 if number == 4 else -20 # 4ブロックなら-10度、5ブロックなら-20度
                print(f"[ゴール検知] 右にコーンを検知 ({number}番ブロック) → 左に回頭します ({turn_angle}度)")
                turn_to_relative_angle(driver_instance, bno_sensor_instance, turn_angle, turn_speed=90, angle_tolerance_deg=5)
                driver_instance.motor_stop_brake()
                time.sleep(0.5)
                if percentage < 50: # 回頭後、割合が低ければ少し前進
                    print("[ゴール検知] 正面にとらえることができませんでしたが、検知割合が低いため、接近します")
                    following.follow_forward(driver_instance, bno_sensor_instance, base_speed=70, duration_time=1)
                counter = counter_max # 正常に追跡できているのでカウンターをリセット
            
            elif number is None: # 赤色が見つからない場合
                print("[ゴール検知] 視野内に赤色コーンが見つかりません。探索のため少し回頭します。")
                turn_to_relative_angle(driver_instance, bno_sensor_instance, 15, turn_speed=80, angle_tolerance_deg=10) # 右に15度回頭して探索
                driver_instance.motor_stop_brake()
                time.sleep(0.5)
                counter -= 1 # カウンターを減らす

    except KeyboardInterrupt:
        print("\n[ゴール検知] 手動で停止されました。")
        driver_instance.motor_stop_brake()
        return False
    except Exception as e:
        print(f"ERROR: [ゴール検知] 予期せぬエラーが発生しました: {e}")
        driver_instance.motor_stop_brake()
        return False
    finally:
        print("[ゴール検知] 処理終了。")
    return False # ループを抜けてもゴールに到達しなかった場合

# --- メインシーケンス ---
# すべてのデバイスの初期化と、各フェーズ関数の呼び出しを制御します。
def main_sequence():
    print("--- 制御シーケンス開始 ---")

    # === デバイスインスタンスの宣言と初期化 ===
    driver = None
    pi_instance = None
    bno_sensor = None
    picam2_instance = None
    gps_datalink_instance = None # EmGpsDatalinkのインスタンス

    try:
        # --- 共通デバイスの初期化 ---
        print("共通デバイスの初期化を開始します...")
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)

        driver = MotorDriver(
            PWMA=12, AIN1=23, AIN2=18,
            PWMB=19, BIN1=16, BIN2=26,
            STBY=21
        )
        print("[メイン] モータードライバー初期化完了。")

        pi_instance = pigpio.pi()
        if not pi_instance.connected:
            raise ConnectionRefusedError("pigpioデーモンに接続できません。sudo pigpiod を実行してください。")
        print("[メイン] pigpio接続完了。")
        
        # BNO055センサーの初期化
        bno_sensor = BNO055(address=0x28) # BNO055のI2Cアドレスを指定
        if not bno_sensor.begin():
            raise IOError("BNO055センサーの初期化に失敗しました。")
        bno_sensor.setExternalCrystalUse(True)
        bno_sensor.setMode(BNO055.OPERATION_MODE_NDOF)
        time.sleep(1) # センサー安定化のための待機
        print("[メイン] BNO055センサー初期化完了。")

        # Picamera2の初期化
        picam2_instance = Picamera2()
        # カメラ画像を90度回転させる設定
        picam2_instance.configure(picam2_instance.create_preview_configuration(
            main={"size": (640, 480)}, # カメラ解像度
            controls={"FrameRate": 30},
            transform=Transform(rotation=90) # ハードウェア回転
        ))
        picam2_instance.start()
        time.sleep(2) # カメラ起動待機
        print("[メイン] Picamera2初期化完了。")
        
        print("共通デバイスの初期化完了。")

        # === BNO055キャリブレーション待機（メインシーケンスの開始時に一度行う） ===
        print("BNO055のキャリブレーション待機中...")
        calibration_start_time = time.time()
        while True:
            sys_cal, gyro_cal, accel_cal, mag_cal = bno_sensor.getCalibration()
            print(f"Calib → Sys:{sys_cal}, Gyro:{gyro_cal}, Acc:{accel_cal}, Mag:{mag_cal}", end='\r')
            sys.stdout.flush()
            if gyro_cal == 3 and mag_cal == 3: # 最低限、ジャイロと地磁気のキャリブレーションを待つ
                print("\n✅ メインBNO055キャリブレーション完了！")
                break
            if time.time() - calibration_start_time > 60: # キャリブレーションタイムアウト (例: 60秒)
                print("\n⚠️ BNO055キャリブレーションにタイムアウトしました。ナビゲーション精度が低下する可能性があります。")
                break
            time.sleep(0.5)

        # --- シーケンス実行 ---
        
        # 1. 放出判定
        print("\n--- シーケンス1: 放出判定 ---")
        getEM_release()
        
        # 2. GPS通信の開始 (放出判定後)
        print("\n--- シーケンス2: GPS通信の準備と開始 ---")
        print("放出判定完了。GPS通信の準備を開始します。")
        gps_datalink_instance = EmGpsDatalink(
            rx_pin=GPS_RX_PIN,
            tx_pin=IM920_TX_PIN,
            baud_soft_uart=GPS_BAUD,
            baud_im920=IM920_BAUD,
            wireless_pin=WIRELESS_GROUND_PIN
        )
        gps_datalink_instance.start() # GPSデータリンクのスレッドを起動
        print("GPS通信の準備が完了し、通信を開始しました。")

        # GPS通信開始後、すぐに最新のGPSデータを取得して表示
        time.sleep(1.0) # GPSスレッドがデータを取得するまで少し待機 (最低1秒は必要)
        current_gps_data = gps_datalink_instance.get_current_gps()
        if current_gps_data:
            print(f"  (メイン: GPS通信開始直後のデータ: 緯度={current_gps_data['latitude']:.6f}, 経度={current_gps_data['longitude']:.6f})")
        else:
            print("  (メイン: GPS通信開始直後のデータはまだ取得されていません)")

        # 3. 着地判定
        print("\n--- シーケンス3: 着地判定 ---")
        is_landed = getEM_land(bno_sensor) # BNO055インスタンスを渡す
        if is_landed:
            print("\n=== 着地を確認しました！次のシーケンスへ進みます。 ===")
        else:
            print("\n=== 着地が確認できませんでした。シーケンスを続行します。 ===")

        # 4. パラシュート回避
        print("\n--- シーケンス4: パラシュート回避 ---")
        # gps_datalink_instanceを渡すように変更
        avoidance_successful = getparakai(driver, bno_sensor, picam2_instance, gps_datalink_instance) 
        if avoidance_successful:
            print("パラシュート回避シーケンス成功。")
        else:
            print("パラシュート回避シーケンス失敗。") # エラー処理に応じて継続/停止を判断

        # 5. 第1フラッグまでGPS誘導 (GPSを頼りに大まかに移動)
        print("\n--- シーケンス5: 第1フラッグまでGPS誘導 ---")
        # gps_datalink_instanceを渡すように変更
        gps_nav_successful = getEM_excellent_gps(driver, bno_sensor, gps_datalink_instance)
        if gps_nav_successful:
            print("第1フラッグまでのGPS誘導成功。")
        else:
            print("第1フラッグまでのGPS誘導失敗。")

        # 6. フラッグ誘導 (カメラでフラッグを検知し追跡)
        print("\n--- シーケンス6: フラッグ誘導 ---")
        flag_nav_successful = getEM_Flag_Navigate(driver, picam2_instance, bno_sensor)
        if flag_nav_successful:
            print("フラッグ誘導成功。")
        else:
            print("フラッグ誘導失敗。")

        # 7. 物資設置
        print("\n--- シーケンス7: 物資設置 ---")
        payload_successful = getcamera() # サーボ制御は直接行うためインスタンス不要
        if payload_successful:
            print("物資設置成功。")
        else:
            print("物資設置失敗。")

        # 8. ゴールまでGPS誘導 (最終目的地へ)
        print("\n--- シーケンス8: ゴールまでGPS誘導 ---")
        # gps_datalink_instanceを渡すように変更
        final_gps_nav_successful = getmotor(driver, bno_sensor, gps_datalink_instance)
        if final_gps_nav_successful:
            print("ゴールまでのGPS誘導成功。")
        else:
            print("ゴールまでのGPS誘導失敗。")

        # 9. ゴール検知 (赤色コーンによる最終検知)
        print("\n--- シーケンス9: ゴール検知 ---")
        goal_detected_successful = getEM_Goal_Detective_NOSHIRO(driver, picam2_instance, bno_sensor)
        if goal_detected_successful:
            print("\n=== ゴール検知成功！ミッション完了！ ===")
        else:
            print("\n=== ゴール検知失敗。ミッション完了できませんでした。 ===")

    except ConnectionRefusedError as e:
        print(f"致命的エラー: pigpioデーモン接続失敗: {e}")
        sys.exit(1)
    except IOError as e:
        print(f"致命的エラー: ハードウェア初期化失敗: {e}")
        sys.exit(1)
    except Exception as e:
        print(f"メインシーケンスで予期せぬエラーが発生しました: {e}")
        # 例外が発生した場合も、クリーンアップのためにfinallyブロックが実行されます。
    finally:
        # === 全デバイスのクリーンアップ ===
        print("\n--- 全デバイスのクリーンアップを開始します ---")
        if gps_datalink_instance:
            try:
                gps_datalink_instance.stop() # GPSスレッドの停止とクリーンアップ
            except Exception as e:
                print(f"GPSデータリンクの停止中にエラー: {e}")
        
        # モータードライバーのクリーンアップ
        if driver:
            try:
                driver.cleanup()
            except Exception as e:
                print(f"モータードライバークリーンアップエラー: {e}")
        
        # Picamera2のクリーンアップ
        if picam2_instance:
            try:
                picam2_instance.stop()
                picam2_instance.close()
            except Exception as e:
                print(f"Picamera2クリーンアップエラー: {e}")

        # pigpioのクリーンアップ (pigpioデーモンへの接続を閉じる)
        # EmGpsDatalink内でpi.stop()を呼んでいる場合、ここで再度呼ぶとエラーになる可能性があります。
        # 共有リソースの管理には注意が必要です。
        # EmGpsDatalinkのcleanup_on_error/cleanup内でpi.stop()を呼び出しているため、ここでは呼び出しません。
        # もし他の場所でpigpioインスタンスを直接扱っているなら、適切にclose()してください。
        # if pi_instance and pi_instance.connected:
        #     pi_instance.stop() # これはEmGpsDatalinkのcleanupで呼ばれているべき

        # GPIO全体のクリーンアップ（サーボやモーターのGPIO設定をリセット）
        # 物資設置で`GPIO.cleanup(SERVO_PIN)`をしているため、全体を`GPIO.cleanup()`する前に
        # 全てのピンが適切に停止しているか確認が必要です。
        # ここでは最終的なGPIOの状態をリセットします。
        GPIO.cleanup() 
        print("--- すべてのシーケンスが完了し、リソースが解放されました。---")

if __name__ == "__main__":
    main_sequence()
