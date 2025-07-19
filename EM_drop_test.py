import cv2
import numpy as np
import time
from picamera2 import Picamera2
from motor import MotorDriver
import following
from BNO055 import BNO055
import smbus
import RPi.GPIO as GPIO # RPi.GPIOはモータードライバやその他のcleanupで使うため残す
import os
import sys
import math
import pigpio # pigpioのインポートを追加

# --- 共通のBME280グローバル変数と関数 ---
t_fine = 0.0
digT = []
digP = []
digH = []

i2c = smbus.SMBus(1)
BME280_address = 0x76

def init_bme280():
    """BME280センサーを初期化します。"""
    i2c.write_byte_data(BME280_address, 0xF2, 0x01)
    i2c.write_byte_data(BME280_address, 0xF4, 0x27)
    i2c.write_byte_data(BME280_address, 0xF5, 0xA0)

def read_compensate():
    """BME280の補正パラメータを読み込みます。"""
    global digT, digP, digH
    dat_t = i2c.read_i2c_block_data(BME280_address, 0x88, 6)
    digT = [(dat_t[1] << 8) | dat_t[0], (dat_t[3] << 8) | dat_t[2], (dat_t[5] << 8) | dat_t[4]]
    for i in range(1, 2):
        if digT[i] >= 32768:
            digT[i] -= 65536
    dat_p = i2c.read_i2c_block_data(BME280_address, 0x8E, 18)
    digP = [(dat_p[i+1] << 8) | dat_p[i] for i in range(0, 18, 2)]
    for i in range(1, 8):
        if digP[i] >= 32768:
            digP[i] -= 65536
    dh = i2c.read_byte_data(BME280_address, 0xA1)
    dat_h = i2c.read_i2c_block_data(BME280_address, 0xE1, 8)
    digH = [dh, (dat_h[1] << 8) | dat_h[0], dat_h[2],
            (dat_h[3] << 4) | (0x0F & dat_h[4]),
            (dat_h[5] << 4) | ((dat_h[4] >> 4) & 0x0F),
            dat_h[6]]
    if digH[1] >= 32768:
        digH[1] -= 65536
    for i in range(3, 4):
        if digH[i] >= 32768:
            digH[i] -= 65536
    if digH[5] >= 128:
        digH[5] -= 256

def bme280_compensate_t(adc_T):
    """BME280の温度値を補正します。"""
    global t_fine
    var1 = (adc_T / 8.0 - digT[0] * 2.0) * digT[1] / 2048.0
    var2 = ((adc_T / 16.0 - digT[0]) ** 2) * digT[2] / 16384.0
    t_fine = var1 + var2
    t = (t_fine * 5 + 128) / 256 / 100
    return t

def bme280_compensate_p(adc_P):
    """BME280の気圧値を補正します。"""
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
    return p / 256 / 100

def get_pressure_and_temperature():
    """BME280から気圧と温度を読み込み、補正して返します。"""
    dat = i2c.read_i2c_block_data(BME280_address, 0xF7, 8)
    adc_p = (dat[0] << 16 | dat[1] << 8 | dat[2]) >> 4
    adc_t = (dat[3] << 16 | dat[4] << 8 | dat[5]) >> 4

    temperature = bme280_compensate_t(adc_t)
    pressure = bme280_compensate_p(adc_p)
    return pressure, temperature

# --- 1. 放出判定用の関数 ---

def check_release(bno_sensor_instance, pressure_change_threshold=0.3, acc_z_threshold_abs=4.0, consecutive_checks=3, timeout=60):
    """
    放出判定を行う関数。BME280の気圧変化とBNO055のZ軸加速度を監視します。
    タイムアウト時は強制的に放出成功とみなしてTrueを返します。
    """
    init_bme280()
    read_compensate()

    if not bno_sensor_instance.begin():
        print("🔴 BNO055 初期化失敗。放出判定を中止します。")
        # BNO055が使えない場合でも次のフェーズに進む
        print("⚠️ BNO055 初期化失敗のため、タイムアウトを待たずに次のフェーズへ移行します。")
        return True # 強制的に成功とみなす

    bno_sensor_instance.setExternalCrystalUse(True)
    bno_sensor_instance.setMode(BNO055.OPERATION_MODE_NDOF)
    print("\n⚠️ BNO055 キャリブレーションはスキップされました。線形加速度の精度が低下する可能性があります。")

    print("\n🚀 放出判定開始...")
    print(f"    初期気圧からの変化量閾値: >= {pressure_change_threshold:.2f} hPa")
    print(f"    Z軸加速度絶対値閾値: > {acc_z_threshold_abs:.2f} m/s²")
    print(f"    連続成立回数: {consecutive_checks}回")
    print(f"    タイムアウト: {timeout}秒\n")

    release_count = 0
    start_time = time.time()
    last_check_time = time.time()
    initial_pressure = None

    try:
        print(f"{'Timestamp(s)':<15}{'Elapsed(s)':<12}{'Current_P(hPa)':<15}{'Initial_P(hPa)':<15}{'P_Chg(hPa)':<15}{'Acc_Z(m/s2)':<12}")
        print("-" * 100)

        while True:
            current_time = time.time()
            elapsed_total = current_time - start_time

            if elapsed_total > timeout:
                print(f"\n⏰ タイムアウト ({timeout}秒経過)。放出判定は成功とみなされ、次のフェーズへ移行します。")
                return True # タイムアウトでも成功とみなして次へ

            if (current_time - last_check_time) < 0.2:
                time.sleep(0.01)
                continue

            last_check_time = current_time

            current_pressure, _ = get_pressure_and_temperature()
            _, _, acc_z = bno_sensor_instance.getVector(BNO055.VECTOR_LINEARACCEL)

            if initial_pressure is None:
                initial_pressure = current_pressure
                print(f"{current_time:<15.3f}{elapsed_total:<12.1f}{current_pressure:<15.2f}{initial_pressure:<15.2f}{'-':<15}{acc_z:<12.2f}")
                print("\n--- 初期気圧設定完了。放出条件監視中... ---")
                continue

            pressure_delta_from_initial = abs(current_pressure - initial_pressure)

            print(f"{current_time:<15.3f}{elapsed_total:<12.1f}{current_pressure:<15.2f}{initial_pressure:<15.2f}{pressure_delta_from_initial:<15.2f}{acc_z:<12.2f}")

            is_release_condition_met = (
                pressure_delta_from_initial >= pressure_change_threshold and
                abs(acc_z) > acc_z_threshold_abs
            )

            if is_release_condition_met:
                release_count += 1
                print(f"\n💡 条件成立！連続判定中: {release_count}/{consecutive_checks} 回")
            else:
                if release_count > 0:
                    print(f"\n--- 条件不成立。カウントリセット ({release_count} -> 0) ---")
                release_count = 0

            if release_count >= consecutive_checks:
                print(f"\n🎉 放出判定成功！連続 {consecutive_checks} 回条件成立！")
                return True

    except KeyboardInterrupt:
        print(f"\n{current_time:<15.3f}{elapsed_total:<12.1f}{current_pressure:<15.2f}{initial_pressure:<15.2f}{pressure_delta_from_initial:<15.2f}{acc_z:<12.2f}")
        print("\n\nプログラムがユーザーによって中断されました。放出判定は成功とみなされ、次のフェーズへ移行します。")
        return True # ユーザー中断でも成功とみなして次へ
    except Exception as e:
        print(f"\n{current_time:<15.3f}{elapsed_total:<12.1f}{current_pressure:<15.2f}{initial_pressure:<15.2f}{pressure_delta_from_initial:<15.2f}{acc_z:<12.2f}")
        print(f"\n\n🚨 エラーが発生しました: {e}。放出判定は成功とみなされ、次のフェーズへ移行します。")
        return True # エラーでも成功とみなして次へ
    finally:
        print("\n--- 放出判定処理終了 ---")


# --- 2. 着地判定用の関数 ---

def check_landing(bno_sensor_instance, driver_instance, pressure_change_threshold=0.1, acc_threshold_abs=0.5, gyro_threshold_abs=0.5, consecutive_checks=3, timeout=120, calibrate_bno055=True): # driver_instanceを追加
    """
    着地判定を行う関数。気圧の変化量、加速度、角速度が閾値内に収まる状態を監視します。
    タイムアウト時は強制的に着地成功とみなしてTrueを返します。
    """
    init_bme280()
    read_compensate()

    if not bno_sensor_instance.begin():
        print("🔴 BNO055 初期化失敗。着地判定を中止します。")
        # BNO055が使えない場合でも次のフェーズに進む
        print("⚠️ BNO055 初期化失敗のため、タイムアウトを待たずに次のフェーズへ移行します。")
        return True # 強制的に成功とみなす

    bno_sensor_instance.setExternalCrystalUse(True)
    bno_sensor_instance.setMode(BNO055.OPERATION_MODE_NDOF)

    if calibrate_bno055:
        print("\n⚙️ BNO055 キャリブレーション中... センサーをいろんな向きにゆっくり回してください。")
        print("    (ジャイロが完全キャリブレーション(レベル3)になるのを待ちます)")

        print("機体回転前に3秒間待機します...")
        time.sleep(3)
        print("機体回転を開始します。")

        calibration_start_time = time.time()
        rotation_start_time = time.time()
        CALIBRATION_TURN_SPEED = 90
        TURN_DURATION = 0.5
        STOP_DURATION = 0.2

        while True:
            calibration_data = bno_sensor_instance.getCalibration()
            if calibration_data is not None and len(calibration_data) == 4:
                sys_cal, gyro_cal, accel_cal, mag_cal = calibration_data
            else:
                print("⚠️ BNO055キャリブレーションデータ取得失敗。リトライ中...", end='\r')
                time.sleep(0.5)
                continue

            print(f"    現在のキャリブレーション状態 → システム:{sys_cal}, ジャイロ:{gyro_cal}, 加速度:{accel_cal}, 地磁気:{mag_cal} ", end='\r')

            if gyro_cal == 3:
                print("\n✅ BNO055 キャリブレーション完了！")
                driver_instance.motor_stop_brake()
                break

            # タイムアウトチェックを追加 (キャリブレーションが長すぎないように)
            if (time.time() - calibration_start_time) > 60: # 例: 1分でタイムアウト
                print("\n⏰ BNO055 キャリブレーションがタイムアウトしました。未完了のまま着地判定に進みます。")
                driver_instance.motor_stop_brake()
                break

            if (time.time() - rotation_start_time) < TURN_DURATION:
                driver_instance.changing_right(0, CALIBRATION_TURN_SPEED)
            elif (time.time() - rotation_start_time) < (TURN_DURATION + STOP_DURATION):
                driver_instance.motor_stop_brake()
            else:
                rotation_start_time = time.time()

            time.sleep(0.1)

        print(f"    キャリブレーションにかかった時間: {time.time() - calibration_start_time:.1f}秒\n")
    else:
        print("\n⚠️ BNO055 キャリブレーション待機はスキップされました。")
        driver_instance.motor_stop_brake()

    print("🛬 着地判定開始...")
    print(f"    気圧変化量閾値: < {pressure_change_threshold:.2f} hPa")
    print(f"    加速度絶対値閾値: < {acc_threshold_abs:.2f} m/s² (X, Y, Z軸)")
    print(f"    角速度絶対値閾値: < {gyro_threshold_abs:.2f} °/s (X, Y, Z軸)")
    print(f"    連続成立回数: {consecutive_checks}回")
    print(f"    タイムアウト: {timeout}秒\n")

    landing_count = 0
    start_time = time.time()
    last_check_time = time.time()
    previous_pressure = None

    try:
        print(f"{'Timestamp(s)':<15}{'Elapsed(s)':<12}{'Pressure(hPa)':<15}{'Pressure_Chg(hPa)':<18}{'Acc_X':<8}{'Acc_Y':<8}{'Acc_Z':<8}{'Gyro_X':<8}{'Gyro_Y':<8}{'Gyro_Z':<8}")
        print("-" * 120)

        while True:
            current_time = time.time()
            elapsed_total = current_time - start_time

            if elapsed_total > timeout:
                print(f"\n⏰ タイムアウト ({timeout}秒経過)。条件成立回数 {landing_count} 回でしたが、着地判定を成功とみなし、次のフェーズへ移行します。")
                return True # タイムアウトでも成功とみなして次へ

            if (current_time - last_check_time) < 0.2:
                time.sleep(0.01)
                continue

            last_check_time = current_time

            current_pressure, _ = get_pressure_and_temperature()
            acc_x, acc_y, acc_z = bno_sensor_instance.getVector(BNO055.VECTOR_LINEARACCEL)
            gyro_x, gyro_y, gyro_z = bno_sensor_instance.getVector(BNO055.VECTOR_GYROSCOPE)

            pressure_delta = float('inf')
            if previous_pressure is not None:
                pressure_delta = abs(current_pressure - previous_pressure)

            print(f"{current_time:<15.3f}{elapsed_total:<12.1f}{current_pressure:<15.2f}{pressure_delta:<18.2f}{acc_x:<8.2f}{acc_y:<8.2f}{acc_z:<8.2f}{gyro_x:<8.2f}{gyro_y:<8.2f}{gyro_z:<8.2f}", end='\r')

            is_landing_condition_met = (
                pressure_delta <= pressure_change_threshold and
                abs(acc_x) < acc_threshold_abs and
                abs(acc_y) < acc_threshold_abs and
                abs(acc_z) < acc_threshold_abs and
                abs(gyro_x) < gyro_threshold_abs and
                abs(gyro_y) < gyro_threshold_abs and
                abs(gyro_z) < gyro_threshold_abs
            )

            previous_pressure = current_pressure

            if is_landing_condition_met:
                landing_count += 1
                print(f"\n💡 条件成立！連続判定中: {landing_count}/{consecutive_checks} 回")
            else:
                if landing_count > 0:
                    print(f"\n--- 条件不成立。カウントリセット ({landing_count} -> 0) ---")
                landing_count = 0

            if landing_count >= consecutive_checks:
                print(f"\n🎉 着地判定成功！連続 {consecutive_checks} 回条件成立！")
                return True

    except KeyboardInterrupt:
        print("\n\nプログラムがユーザーによって中断されました。着地判定は成功とみなされ、次のフェーズへ移行します。")
        return True # ユーザー中断でも成功とみなして次へ
    except Exception as e:
        print(f"\n\n🚨 エラーが発生しました: {e}。着地判定は成功とみなされ、次のフェーズへ移行します。")
        return True # エラーでも成功とみなして次へ
    finally:
        print("\n--- 判定処理終了 ---")


# BNO055用のラッパークラス
class BNO055Wrapper:
    def __init__(self, bno055_sensor_instance):
        self.sensor = bno055_sensor_instance

    def get_heading(self):
        euler_angles = self.sensor.getVector(BNO055.VECTOR_EULER)
        if euler_angles is None or len(euler_angles) < 3 or euler_angles[0] is None:
            wait_start_time = time.time()
            max_wait_time = 0.1
            while (euler_angles is None or len(euler_angles) < 3 or euler_angles[0] is None) and (time.time() - wait_start_time < max_wait_time):
                time.sleep(0.005)
                euler_angles = self.sensor.getVector(BNO055.VECTOR_EULER)

        if euler_angles is None or len(euler_angles) < 3 or euler_angles[0] is None:
            return None

        heading = euler_angles[0]
        return heading

def save_image_for_debug(picam2_instance, path="/home/mark1/1_Pictures/paravo_image.jpg"):
    """デバッグ用に画像を保存します。"""
    frame_rgb = picam2_instance.capture_array()
    if frame_rgb is None:
        print("画像キャプチャ失敗：フレームがNoneです。")
        return None

    frame_bgr = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)
    rotated_frame_bgr = cv2.rotate(frame_bgr, cv2.ROTATE_90_COUNTERCLOCKWISE)
    processed_frame_bgr = cv2.flip(rotated_frame_bgr, 1) # 水平フリップ

    directory = os.path.dirname(path)
    if not os.path.exists(directory): os.makedirs(directory)
    cv2.imwrite(path, processed_frame_bgr)
    print(f"画像保存成功: {path}")
    return processed_frame_bgr

def detect_red_in_grid(picam2_instance, save_path="/home/mark1/1_Pictures/akairo_grid.jpg", min_red_pixel_ratio_per_cell=0.05):
    """
    カメラ画像を縦2x横3のグリッドに分割し、各セルでの赤色検出を行い、その位置情報を返します。
    キャプチャした画像を反時計回りに90度回転させてから左右反転させて処理します。
    """
    try:
        frame_rgb = picam2_instance.capture_array()
        if frame_rgb is None:
            print("画像キャプション失敗: フレームがNoneです。")
            return 'error_in_processing'

        frame_bgr = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)

        rotated_frame_bgr = cv2.rotate(frame_bgr, cv2.ROTATE_90_COUNTERCLOCKWISE)

        processed_frame_bgr = cv2.flip(rotated_frame_bgr, 1)

        height, width, _ = processed_frame_bgr.shape
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

        lower_orange = np.array([5, 150, 150])  # オレンジ色の下限
        upper_orange = np.array([30, 255, 255]) # オレンジ色の上限

        blurred_full_frame = cv2.GaussianBlur(processed_frame_bgr, (5, 5), 0)
        hsv_full = cv2.cvtColor(blurred_full_frame, cv2.COLOR_BGR2HSV)
        mask_full_red = cv2.bitwise_or(cv2.inRange(hsv_full, lower_red1, upper_red1),
                                     cv2.inRange(hsv_full, lower_red2, upper_red2))
        mask_full_orange = cv2.inRange(hsv_full, lower_orange, upper_orange)
        mask_full = cv2.bitwise_or(mask_full_red, mask_full_orange)
        red_pixels_full = np.count_nonzero(mask_full) ; total_pixels_full = height * width
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
            mask_cell_red = cv2.bitwise_or(cv2.inRange(hsv_cell, lower_red1, upper_red1),
                                         cv2.inRange(hsv_cell, lower_red2, upper_red2))
            mask_cell_orange = cv2.inRange(hsv_cell, lower_orange, upper_orange)
            mask_cell = cv2.bitwise_or(mask_cell_red, mask_cell_orange)
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

def turn_to_relative_angle(driver, bno_sensor_wrapper_instance, angle_offset_deg, turn_speed=90, angle_tolerance_deg=15.0, max_turn_attempts=100):
    """
    現在のBNO055の方位から、指定された角度だけ相対的に旋回します。
    """
    initial_heading = bno_sensor_wrapper_instance.get_heading()
    if initial_heading is None:
        print("警告: turn_to_relative_angle: 初期方位が取得できませんでした。")
        return False

    target_heading = (initial_heading + angle_offset_deg + 360) % 360
    print(f"現在のBNO方位: {initial_heading:.2f}度, 相対目標角度: {angle_offset_deg:.2f}度 -> 絶対目標方位: {target_heading:.2f}度")

    loop_count = 0

    while loop_count < max_turn_attempts:
        current_heading = bno_sensor_wrapper_instance.get_heading()
        if current_heading is None:
            print("警告: turn_to_relative_angle: 旋回中に方位が取得できませんでした。スキップします。")
            driver.motor_stop_brake()
            time.sleep(0.1)
            loop_count += 1
            continue

        angle_error = (target_heading - current_heading + 180 + 360) % 360 - 180

        if abs(angle_error) <= angle_tolerance_deg:
            print(f"[TURN] 相対回頭完了。最終誤差: {angle_error:.2f}度 (試行回数: {loop_count})")
            driver.motor_stop_brake()
            time.sleep(0.5)
            return True

        turn_duration_on = 0.02 + (abs(angle_error) / 180.0) * 0.2

        if angle_error < 0:
            driver.petit_left(0, turn_speed)
            driver.petit_left(turn_speed, 0)
        else:
            driver.petit_right(0, turn_speed)
            driver.petit_right(turn_speed, 0)

        time.sleep(turn_duration_on)
        driver.motor_stop_brake()
        time.sleep(0.05)

        loop_count += 1

    print(f"警告: turn_to_relative_angle: 最大試行回数({max_turn_attempts}回)内に目標角度に到達できませんでした。最終誤差: {angle_error:.2f}度 (試行回数: {loop_count})")
    driver.motor_stop_brake()
    time.sleep(0.5)
    return False


# --- ニクロム線溶断関数 ---
NICHROME_PIN = 25
HEATING_DURATION_SECONDS = 3.0

def activate_nichrome_wire(t_melt = 4):
    """
    ニクロム線を指定された時間だけオンにして溶断シーケンスを実行します。
    """
    print("\n--- ニクロム線溶断シーケンスを開始します。 ---")
    pi = None # piオブジェクトを初期化
    try:
        pi = pigpio.pi() # pigpioのインスタンスを生成
        if not pi.connected:
            raise Exception("pigpioデーモンに接続できませんでした。")

        meltPin = NICHROME_PIN

        # ピンモードを設定（ここではpigpioで設定）
        pi.set_mode(meltPin, pigpio.OUTPUT)
        # 初期状態をLOWに設定
        pi.write(meltPin, 0)
        time.sleep(1) # 安定時間

        print(f"GPIO {meltPin} をHIGHに設定し、ニクロム線をオンにします。")
        pi.write(meltPin, 1) # HIGHに設定
        time.sleep(t_melt)
        print(f"{t_melt}秒間、加熱しました。")

        print(f"GPIO {meltPin} をLOWに設定し、ニクロム線をオフにします。")
        pi.write(meltPin, 0) # LOWに設定
        time.sleep(1) # オフ後の安定時間
        print("ニクロム線溶断シーケンスが正常に完了しました。")

    except Exception as e:
        print(f"🚨 ニクロム線溶断中にエラーが発生しました: {e}")
        if pi and pi.connected:
            pi.write(NICHROME_PIN, 0) # エラー時も安全のためオフ
    finally:
        if pi and pi.connected:
            pi.stop() # pigpioの接続を停止
    print("--- ニクロム線溶断シーケンス終了。 ---")


# --- メイン実行ブロック ---
if __name__ == "__main__":
    # RPi.GPIOはモータードライバやBNO055（I2C経由だがcleanupでGPIOを扱う場合）の
    # cleanupのために残す場合があるため、setmodeは継続
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)

    # ニクロム線ピンの初期設定はpigpioで行うため、RPi.GPIOでの設定は削除
    # GPIO.setup(NICHROME_PIN, GPIO.OUT, initial=GPIO.LOW) は削除

    # BNO055センサーの生インスタンス（放出判定と着地判定で直接使用）
    bno_raw_sensor = BNO055(address=0x28)

    # --- ステージ0: 放出判定 ---
    print("\n--- ステージ0: 放出判定を開始します ---")
    is_released = check_release(
        bno_raw_sensor,
        pressure_change_threshold=0.3,
        acc_z_threshold_abs=4.0,
        consecutive_checks=3,
        timeout=30
    )

    if is_released:
        print("\n=== ローバーの放出判定が成功したか、タイムアウトにより次のフェーズへ移行します。 ===")
    else:
        # このパスは、is_releasedがFalseを返す、すなわち何らかの致命的エラーで失敗した場合のみ実行される（タイムアウト時はTrueを返すため）。
        print("\n=== ローバーの放出判定が致命的なエラーにより失敗しました。しかし、プログラムは続行されます。 ===")
        pass # 現状では何もしないが、必要に応じてエラー通知などを追加

    # 放出が確認されたか、タイムアウトで移行する場合にデバイスを初期化
    driver = MotorDriver(
        PWMA=12, AIN1=23, AIN2=18,
        PWMB=19, BIN1=16, BIN2=26,
        STBY=21
    )

    # BNO055Wrapperインスタンス
    bno_sensor_wrapper = BNO055Wrapper(bno_raw_sensor)

    picam2 = Picamera2()
    picam2.configure(picam2.create_still_configuration(
        main={"size": (320, 240)}
    ))
    picam2.start()
    time.sleep(1)


    try:
        # --- ステージ1: 着地判定 ---
        print("\n--- ステージ1: 着地判定を開始します ---")
        is_landed = check_landing(
            bno_raw_sensor,
            driver,
            pressure_change_threshold=0.1,
            acc_threshold_abs=0.5,
            gyro_threshold_abs=0.5,
            consecutive_checks=3,
            timeout=30,
            calibrate_bno055=True
        )

        if is_landed:
            print("\n=== ローバーの着地判定が成功したか、タイムアウトにより次のフェーズへ移行します。 ===")
        else:
            # このパスは、is_landedがFalseを返す、すなわち何らかの致命的エラーで失敗した場合のみ実行される。
            print("\n=== ローバーの着地判定が致命的なエラーにより失敗しました。しかし、プログラムは続行されます。 ===")
            driver.motor_stop_brake()
            time.sleep(1)

        driver.motor_stop_brake()
        time.sleep(1)

        # --- ステージ1.5: ニクロム線溶断シーケンス ---
        activate_nichrome_wire(t_melt = 4)
        time.sleep(2)

        # --- ステージ2: パラシュート即時回避と最終確認 ---
        print("\n--- ステージ2: 着地後のパラシュート即時回避と最終確認を開始します ---")

        # 360度スキャンを開始する前に一度だけ前進
        print(f"\n→ 360度スキャン開始前に1秒前進します...")
        following.follow_forward(driver, bno_raw_sensor, base_speed=60, duration_time=1)
        driver.motor_stop_brake()
        time.sleep(0.5) # 前進後の安定時間


        # 回避と最終確認のループ
        while True:
            print("\n🔍 360度パラシュートスキャンを開始...")
            detected_during_scan_cycle = False

            scan_angles_offsets = [0, 45, 45, 45, 45, 45, 45, 45] # 45度ずつに修正

            for i, angle_offset in enumerate(scan_angles_offsets):
                if i > 0:
                    print(f"→ {angle_offset}度旋回してスキャンします...")
                    turn_to_relative_angle(driver, bno_sensor_wrapper, angle_offset, turn_speed=90, angle_tolerance_deg=15)
                    time.sleep(0.5)
                    driver.motor_stop_brake()

                current_direction_str = ""
                total_angle_turned = sum(scan_angles_offsets[:i+1]) % 360
                if total_angle_turned == 0: current_direction_str = "正面(0度)"
                elif total_angle_turned == 45: current_direction_str = "右45度"
                elif total_angle_turned == 90: current_direction_str = "右90度"
                elif total_angle_turned == 135: current_direction_str = "右135度"
                elif total_angle_turned == 180: current_direction_str = "後方(180度)"
                elif total_angle_turned == 225: current_direction_str = "左135度"
                elif total_angle_turned == 270: current_direction_str = "左90度"
                elif total_angle_turned == 315: current_direction_str = "左45度"
                else: current_direction_str = f"方向不明({total_angle_turned}度)"


                print(f"--- スキャン方向: {current_direction_str} ---")
                # ファイル名に日本語が含まれないように修正
                safe_direction_str = current_direction_str.replace("正面(0度)", "front_0deg").replace("右45度", "right_45deg").replace("右90度", "right_90deg").replace("右135度", "right_135deg").replace("後方(180度)", "rear_180deg").replace("左135度", "left_135deg").replace("左90度", "left_90deg").replace("左45度", "left_45deg").replace("方向不明", "unknown_direction")

                scan_result = detect_red_in_grid(picam2, save_path=f"/home/mark1/1_Pictures/initial_scan_{safe_direction_str}.jpg", min_red_pixel_ratio_per_cell=0.10)

                if scan_result != 'none_detected' and scan_result != 'error_in_processing':
                    print(f"🚩 {current_direction_str}でパラシュートを検知しました！")
                    detected_during_scan_cycle = True

                    print(f"検出されたため、回避行動に移ります。")

                    if total_angle_turned <= 45 or total_angle_turned >= 315:
                        print("正面付近で検出されたため、右90度回頭して回避します。")
                        turn_to_relative_angle(driver, bno_sensor_wrapper, 90, turn_speed=90, angle_tolerance_deg=10)
                    elif total_angle_turned > 45 and total_angle_turned < 180:
                        print("右側で検出されたため、左90度回頭して回避します。")
                        turn_to_relative_angle(driver, bno_sensor_wrapper, -90, turn_speed=90, angle_tolerance_deg=10)
                    elif total_angle_turned > 180 and total_angle_turned < 315:
                        print("左側で検出されたため、右90度回頭して回避します。")
                        turn_to_relative_angle(driver, bno_sensor_wrapper, 90, turn_speed=90, angle_tolerance_deg=10)
                    else:
                        print("後方または不明な方向で検出されたため、右90度回頭して回避します。")
                        turn_to_relative_angle(driver, bno_sensor_wrapper, 90, turn_speed=90, angle_tolerance_deg=10)

                    print("回避のため少し前進します。(速度80, 3秒)")
                    following.follow_forward(driver, bno_raw_sensor, base_speed=80, duration_time=3)
                    driver.motor_stop_brake()
                    time.sleep(1)
                    break

                driver.motor_stop_brake()
                time.sleep(0.5)

            if not detected_during_scan_cycle:
                print("\n✅ 360度スキャンしましたが、パラシュートは検知されませんでした。初期回避フェーズ完了。")

                print("\n→ 少し前進します。(速度70, 5秒)")
                following.follow_forward(driver, bno_raw_sensor, base_speed=70, duration_time=5)
                driver.motor_stop_brake()
                time.sleep(1)

                final_scan_results = {
                    'front': 'none_detected',
                    'left_30': 'none_detected',
                    'right_30': 'none_detected'
                }

                print("\n=== 最終確認スキャンを開始します (正面、左30度、右30度) ===")

                print("→ 正面方向の赤色を確認します...")
                final_scan_results['front'] = detect_red_in_grid(picam2, save_path="/home/mark1/1_Pictures/final_confirm_front.jpg", min_red_pixel_ratio_per_cell=0.10)

                print("→ 左に30度回頭し、赤色を確認します...")
                turn_to_relative_angle(driver, bno_sensor_wrapper, -30, turn_speed=90, angle_tolerance_deg=10)
                final_scan_results['left_30'] = detect_red_in_grid(picam2, save_path="/home/mark1/1_Pictures/final_confirm_left.jpg", min_red_pixel_ratio_per_cell=0.10)
                print("→ 左30度から正面に戻します...")
                turn_to_relative_angle(driver, bno_sensor_wrapper, 30, turn_speed=90, angle_tolerance_deg=10)

                print("→ 右に30度回頭し、赤色を確認します...")
                turn_to_relative_angle(driver, bno_sensor_wrapper, 30, turn_speed=90, angle_tolerance_deg=10)
                final_scan_results['right_30'] = detect_red_in_grid(picam2, save_path="/home/mark1/1_Pictures/final_confirm_right.jpg", min_red_pixel_ratio_per_cell=0.10)
                print("→ 右30度から正面に戻します...")
                turn_to_relative_angle(driver, bno_sensor_wrapper, -30, turn_speed=90, angle_tolerance_deg=10)

                is_final_clear = (
                    final_scan_results['front'] == 'none_detected' and
                    final_scan_results['left_30'] == 'none_detected' and
                    final_scan_results['right_30'] == 'none_detected'
                )

                if is_final_clear:
                    print("\n🎉 最終確認スキャン結果: 全ての方向でパラシュートは検知されませんでした。ミッション完了！")
                    break
                else:
                    print("\n⚠️ 最終確認スキャン結果: パラシュートが検知されました。再度回避を試みます。")
                    continue

            continue


    except SystemExit as e:
        print(f"\nプログラムが強制終了されました: {e}")
    except Exception as e:
        print(f"\nメイン処理中に予期せぬエラーが発生しました: {e}")
    finally:
        # メインのfinallyブロックで全てのGPIOをクリーンアップ
        if 'driver' in locals():
            driver.cleanup() # モータードライバーのGPIOをクリーンアップ
        if 'picam2' in locals():
            picam2.close() # Picamera2を閉じる

        # RPi.GPIOのcleanupは、RPi.GPIOでセットアップされたピンのみをクリーンアップします。
        # pigpioで制御されたピン（NICHROME_PIN）は、activate_nichrome_wire()内のfinallyブロックで
        # pi.stop()によって適切に停止されます。
        GPIO.cleanup()
        print("=== すべてのクリーンアップが終了しました。プログラムを終了します。 ===")
