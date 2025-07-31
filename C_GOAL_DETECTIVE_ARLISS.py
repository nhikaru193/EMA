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
from BNO055 import BNO055 # GDAクラス内で使用するため必要

# --- 定数設定 (変更なし) ---
RX_PIN = 17

# --- ヘルパー関数 (クラス外に保持するか、必要に応じてstaticmethod化) ---
# これらの関数はデバイスインスタンスを直接操作しないため、独立していても良い
def save_image_for_debug(picam2_instance, path="/home/mark1/Pictures/paravo_image.jpg"):
    frame = picam2_instance.capture_array()
    if frame is None:
        print("画像キャプチャ失敗：フレームがNoneです。")
        return None
    frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
    cv2.imwrite(path, frame_bgr)
    print(f"画像保存成功: {path}")
    return frame

def detect_red_percentage(picam2_instance, save_path="/home/mark1/Pictures/red_detection_overall.jpg"):
    """
    カメラ画像をキャプチャし、画像全体における赤色ピクセルの割合を返します。
    ここではソフトウェア的に回転・反転を行います。
    エラー時は-1.0を返します。
    保存される画像は、回転後の通常のカラー画像です。
    """
    try:
        frame_rgb = picam2_instance.capture_array()
        if frame_rgb is None:
            print("画像キャプチャ失敗: フレームがNoneです。")
            return -1.0 # エラー値として-1.0を返す

        frame_bgr = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)
        rotated_frame_bgr = cv2.rotate(frame_bgr, cv2.ROTATE_90_COUNTERCLOCKWISE)

        directory = os.path.dirname(save_path)
        if not os.path.exists(directory):
            os.makedirs(directory)
        cv2.imwrite(save_path, rotated_frame_bgr) 
        print(f"通常の画像を保存しました: {save_path}")

        height, width, _ = rotated_frame_bgr.shape
        total_pixels = height * width

        hsv = cv2.cvtColor(rotated_frame_bgr, cv2.COLOR_BGR2HSV)

        lower_red1 = np.array([0, 100, 100])
        upper_red1 = np.array([10, 255, 255])

        lower_red2 = np.array([170, 100, 100])
        upper_red2 = np.array([180, 255, 255])

        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask = cv2.add(mask1, mask2)

        red_pixels = cv2.countNonZero(mask)
        red_percentage = (red_pixels / total_pixels) * 100 if total_pixels > 0 else 0.0

        print(f"検出結果: 画像全体の赤色割合: {red_percentage:.2f}%")
        return red_percentage / 100.0 # 割合は0-1の範囲で返すように調整

    except Exception as e:
        print(f"カメラ撮影・処理中にエラーが発生しました: {e}")
        return -1.0

def calculate_angle_average(angles_deg):
    """
    複数の角度（度数法）の平均値を計算します。角度の連続性を考慮し、ベクトル平均を使用します。
    例: [350, 10] の平均は 0 に近い値になります。
    """
    if not angles_deg:
        return None

    x_coords = [math.cos(math.radians(angle)) for angle in angles_deg]
    y_coords = [math.sin(math.radians(angle)) for angle in angles_deg]

    sum_x = sum(x_coords)
    sum_y = sum(y_coords)

    average_angle_rad = math.atan2(sum_y, sum_x)
    average_angle_deg = math.degrees(average_angle_rad)

    return (average_angle_deg + 360) % 360

# --- ロボット制御クラス ---
class GDA:
    # __init__ メソッドの引数に bno_sensor_instance を追加
    def __init__(self, bno_sensor_instance):
        # GPIO設定
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)

        # デバイス初期化
        self.driver = MotorDriver(
            PWMA=12, AIN1=23, AIN2=18,
            PWMB=19, BIN1=16, BIN2=26,
            STBY=21
        )
        
        self.pi_instance = pigpio.pi()
        if not self.pi_instance.connected:
            print("pigpioデーモンに接続できません。終了します。")
            sys.exit(1) # エラーコードで終了

        # main.py から渡された BNO055 インスタンスを使用
        self.bno_sensor = bno_sensor_instance 
        
        # BNO055 の初期化は main.py で行われるため、GDA.__init__ からは削除
        # if not self.bno_sensor.begin():
        #     print("BNO055センサーの初期化に失敗しました。終了します。")
        #     sys.exit(1)
        # self.bno_sensor.setMode(BNO055.OPERATION_MODE_NDOF)
        # self.bno_sensor.setExternalCrystalUse(True)
        # time.sleep(1)
        
        self.picam2 = Picamera2()
        self.picam2.configure(self.picam2.create_preview_configuration(
            main={"size": (640, 480)},
            controls={"FrameRate": 30},
            transform=Transform(rotation=90)
        ))
        self.picam2.start()
        time.sleep(2)
        print("GDA: すべてのデバイスが初期化されました。")

    def get_bno_heading(self):
        """
        BNO055から現在の方位を取得します。Noneが返される場合は待機して再試行します。
        """
        heading = None # 初期値をNoneに設定

        wait_start_time = time.time()
        max_wait_time = 0.5
        
        while heading is None and (time.time() - wait_start_time < max_wait_time):
            # bno_sensor.euler[0] の代わりに getVector(BNO055.VECTOR_EULER) を使用
            # getVectorは(heading, roll, pitch)のタプルを返すので、最初の要素がheadingです
            euler_angles = self.bno_sensor.getVector(BNO055.VECTOR_EULER)
            
            if euler_angles is not None:
                heading = euler_angles[0] # タプルの最初の要素がHeading
            else:
                heading = None # 取得できなかった場合はNoneを維持

            if heading is None: # Noneが返ってきたら少し待って再試行
                time.sleep(0.01) # 短時間待機

        return heading if heading is not None else 0.0 # 最終的にNoneなら0.0を返す

    def turn_to_relative_angle(self, angle_offset_deg, turn_speed=90, angle_tolerance_deg=15.0, max_turn_attempts=100):
        """
        現在のBNO055の方位から、指定された角度だけ相対的に旋回します。
        """
        initial_heading = self.get_bno_heading()
        if initial_heading is None: 
            print("警告: turn_to_relative_angle: 初期方位が取得できませんでした。")
            return False
        
        target_heading = (initial_heading + angle_offset_deg + 360) % 360
        print(f"現在のBNO方位: {initial_heading:.2f}度, 相対目標角度: {angle_offset_deg:.2f}度 -> 絶対目標方位: {target_heading:.2f}度")

        loop_count = 0
        
        while loop_count < max_turn_attempts:
            current_heading = self.get_bno_heading()
            if current_heading is None: 
                print("警告: turn_to_relative_angle: 旋回中に方位が取得できませんでした。スキップします。")
                self.driver.motor_stop_brake()
                time.sleep(0.1)
                loop_count += 1
                continue

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

    def perform_final_scan_and_terminate(self, turn_angle_step=20, final_threshold=0.15, min_red_detections_to_terminate=4, high_red_threshold=0.40):
        """
        ローバーを20度ずつ360度回転させ、設定された閾値以上の赤色を検知した方向を記録します。
        - high_red_threshold 以上の赤色を検知した場合、即座に180度回転して前進し、Trueを返します（最終確認スキャンを再開）。
        - スキャン終了後、final_threshold 以上の赤色をmin_red_detections_to_terminate個以上検知した場合、Falseを返します（プログラム終了指示）。
        - どちらの条件も満たさない場合はNoneを返します（次の走行サイクルへ）。
        """
        print(f"\n=== 最終確認スキャンを開始します ({final_threshold:.0%}閾値、{min_red_detections_to_terminate}ヶ所検知、{high_red_threshold:.0%}高閾値で即座に180度回転前進) ===")
        
        initial_heading = self.get_bno_heading()
        if initial_heading is None:
            print("警告: perform_final_scan_and_terminate: 初期方位が取得できませんでした。")
            return None

        final_scan_detected_angles = []
        
        print(f"  初回回転: {turn_angle_step}度...")
        self.turn_to_relative_angle(turn_angle_step, turn_speed=90, angle_tolerance_deg=15)
        
        for i in range(360 // turn_angle_step): 
            if i > 0:
                print(f"  --> スキャン中: さらに{turn_angle_step}度回転...")
                self.turn_to_relative_angle(turn_angle_step, turn_speed=90, angle_tolerance_deg=15)
                self.driver.motor_stop_brake()
                time.sleep(0.5)
            
            current_scan_heading = self.get_bno_heading()
            if current_scan_heading is None:
                print("警告: perform_final_scan_and_terminate: 旋回中に方位が取得できませんでした。スキップします。")
                self.driver.motor_stop_brake()
                time.sleep(0.1)
                continue

            print(f"--- 最終確認スキャン中: 現在の方向: {current_scan_heading:.2f}度 ---")

            overall_red_ratio = detect_red_percentage(
                self.picam2, 
                save_path=f"/home/mark1/Pictures/final_scan_{i*turn_angle_step + turn_angle_step:03d}.jpg"
            )

            if overall_red_ratio == -1.0:
                print("カメラ処理エラーのため、現在のスキャンステップをスキップします。")
                continue

            print(f"検出結果: 画像全体の赤色割合: {overall_red_ratio:.2%}")

            if overall_red_ratio >= high_red_threshold:
                print(f"\n  --> **高い赤色割合 ({high_red_threshold:.0%}) を検出しました！即座に180度回転して3秒間前進します。**")
                self.turn_to_relative_angle(180, turn_speed=90, angle_tolerance_deg=15)
                self.driver.motor_stop_brake()
                time.sleep(0.5)
                
                self.driver.petit_petit(4) # 前進速度を調整
                time.sleep(3) # 3秒間前進
                self.driver.motor_stop_brake()
                time.sleep(0.5)
                print("  --> 180度回転して3秒前進が完了しました。最終確認スキャンを最初から再開します。")
                return True

            if overall_red_ratio >= final_threshold:
                print(f"  --> 赤色を{final_threshold:.0%}以上検出！方向を記録します。")
                final_scan_detected_angles.append(current_scan_heading)
            
            self.driver.motor_stop_brake()
            time.sleep(0.5)

        if len(final_scan_detected_angles) >= min_red_detections_to_terminate:
            print(f"\n  --> 最終確認スキャンで{min_red_detections_to_terminate}ヶ所以上の赤色を検出しました ({len(final_scan_detected_angles)}ヶ所)。")
            
            target_center_angle = calculate_angle_average(final_scan_detected_angles)
            if target_center_angle is not None:
                print(f"  --> 検出された赤色の中心 ({target_center_angle:.2f}度) へ向きを調整します。")
                current_heading_at_end = self.get_bno_heading()
                if current_heading_at_end is not None:
                    angle_to_turn = (target_center_angle - current_heading_at_end + 180 + 360) % 360 - 180
                    self.turn_to_relative_angle(angle_to_turn, turn_speed=90, angle_tolerance_deg=15)
                    self.driver.motor_stop_brake()
                    time.sleep(1.0)
                    print("  --> 中心方向への向き調整が完了しました。")
                else:
                    print("警告: 最終スキャン後の中心回頭時に方位が取得できませんでした。")
            
            print("  --> 4ヶ所検知の条件（ただし40%未満）を満たしたため、ミッションを終了します。")
            return False
        else:
            print(f"\n=== 最終確認スキャンが完了しました。条件を満たしませんでした (検出箇所: {len(final_scan_detected_angles)}ヶ所)。 ===")
            return None

    def perform_initial_alignment_scan(self, turn_angle_step=20, alignment_threshold=0.10):
        """
        ローバーを20度ずつ270度回転させ、alignment_threshold 以上の赤色を検知したらその方向で回転を停止し、向きを合わせます。
        alignment_threshold 以上の赤色が検知されなかった場合は、最も多くの赤が検知された方向に向きを合わせてからTrueを返します。
        戻り値: (aligned_successfully:bool, aligned_heading:float, detected_red_angles:list)
        """
        print("\n=== 初期赤色アライメントスキャンを開始します (20%閾値) ===")
        
        initial_heading_at_start = self.get_bno_heading()
        if initial_heading_at_start is None:
            print("警告: perform_initial_alignment_scan: 初期方位が取得できません。")
            return False, None, []

        aligned = False
        max_red_ratio = -1.0
        best_heading_for_red = initial_heading_at_start
        
        detected_red_angles = []

        initial_turn_angle = 270 
        print(f"  初回回転: {initial_turn_angle}度...")
        self.turn_to_relative_angle(initial_turn_angle, turn_speed=90, angle_tolerance_deg=15)
        
        for i in range(270 // turn_angle_step):
            current_relative_angle_from_start_of_scan = (i + 1) * turn_angle_step
            
            if i > 0:
                print(f"  回転: {turn_angle_step}度...")
                self.turn_to_relative_angle(turn_angle_step, turn_speed=90, angle_tolerance_deg=15)
            
            current_scan_heading = self.get_bno_heading()
            if current_scan_heading is None:
                print("警告: perform_initial_alignment_scan: 旋回中に方位が取得できませんでした。スキップします。")
                self.driver.motor_stop_brake()
                time.sleep(0.1)
                continue

            print(f"\n--- 初期アライメントスキャン中: 現在の方向: {current_scan_heading:.2f}度 ---")
            
            overall_red_ratio = detect_red_percentage(
                self.picam2, 
                save_path=f"/home/mark1/Pictures/initial_alignment_scan_{current_relative_angle_from_start_of_scan:03d}.jpg"
            )

            if overall_red_ratio == -1.0:
                print("初期アライメントスキャン中にカメラ処理エラー。スキップします。")
                self.driver.motor_stop_brake()
                time.sleep(0.5)
                continue

            print(f"検出結果: 画像全体の赤色割合: {overall_red_ratio:.2f}%")

            if overall_red_ratio > max_red_ratio:
                max_red_ratio = overall_red_ratio
                best_heading_for_red = current_scan_heading 
            
            if overall_red_ratio * 100.0 >= alignment_threshold * 100.0:
                detected_red_angles.append(current_scan_heading)
                print(f"  --> 赤色を{alignment_threshold*100.0:.0f}%以上検出！方向を記録しました。")

            self.driver.motor_stop_brake()
            time.sleep(0.5)

            if i < (270 // turn_angle_step) - 1:
                print(f"  回転: {turn_angle_step}度...")
                self.turn_to_relative_angle(turn_angle_step, turn_speed=90, angle_tolerance_deg=15)

        if not detected_red_angles:
            print(f"初期アライメントスキャンで{alignment_threshold*100.0:.0f}%以上の赤色は検出されませんでした。")
            if max_red_ratio > -1.0:
                current_heading_at_end_of_scan = self.get_bno_heading()
                
                if current_heading_at_end_of_scan is not None:
                    angle_to_turn_to_best_red = (best_heading_for_red - current_heading_at_end_of_scan + 180 + 360) % 360 - 180
                    
                    print(f"  --> {alignment_threshold*100.0:.0f}%"
                          f"以上は検出されませんでしたが、最も多くの赤 ({max_red_ratio:.2f}%) が検出された方向 ({best_heading_for_red:.2f}度) へアライメントします (相対回転: {angle_to_turn_to_best_red:.2f}度)。")
                    self.turn_to_relative_angle(angle_to_turn_to_best_red, turn_speed=90, angle_tolerance_deg=15)
                    self.driver.motor_stop_brake()
                    time.sleep(0.5)
                    aligned = True
                    detected_red_angles.append(best_heading_for_red)
                else:
                    print("警告: スキャン終了時に方位が取得できず、最大赤色方向へのアライメントができませんでした。")
            else:
                print("初期アライメントスキャンで赤色は全く検出されませんでした。")
        else:
            aligned = True

        print("=== 初期赤色アライメントスキャンが完了しました。 ===")
        return aligned, best_heading_for_red, detected_red_angles

    def HAT_TRICK(self): # run_autonomous_mode を HAT_TRICK に名称変更
        """
        ロボットのメインの自律走行ループを実行します。
        """
        # BNO055のキャリブレーション待機は main.py で行われているため、ここではスキップ
        print("BNO055のキャリブレーション待機はスキップされました。自動操縦を開始します。")

        while True:
            print("\n--- 新しい走行サイクル開始 ---")
            
            print("\n=== 現在方位確認 ===")
            current_bno_heading_for_info = self.get_bno_heading()
            if current_bno_heading_for_info is None:
                print("警告: 現在方位が取得できませんでした。")
                time.sleep(2)
                continue
            print(f"現在のBNO方位: {current_bno_heading_for_info:.2f}度")
            self.driver.motor_stop_brake()
            time.sleep(0.5)

            # --- 初期アライメントスキャンを実行 ---
            skip_forward_scan_phase = False 
            aligned_in_initial_scan, initial_aligned_heading, initial_scan_detected_angles = self.perform_initial_alignment_scan()

            # --- 初期アライメントで4つ以上の赤色を検知した場合、最終処理へスキップ ---
            if len(initial_scan_detected_angles) >= 4:
                print(f"\n=== 初期アライメントスキャンで{len(initial_scan_detected_angles)}ヶ所の赤色を検知しました。最終確認スキャンへスキップします。 ===")
                while True:
                    final_scan_result = self.perform_final_scan_and_terminate(final_threshold=0.07, min_red_detections_to_terminate=4, high_red_threshold=0.40)
                    if final_scan_result is True:
                        print("最終確認スキャン条件達成（40%検出で即座に180度回転前進）。最終確認スキャンを再実行します。")
                        continue
                    elif final_scan_result is False:
                        print("最終確認スキャンが完了し、プログラム終了条件を満たしました。ミッションを終了します。")
                        sys.exit(0)
                    else: # final_scan_result is None
                        print("最終確認スキャンは条件を満たしませんでした。メインループの通常フローに戻ります。")
                        break
                continue

            if aligned_in_initial_scan:
                print(f"初期アライメントスキャンによって、何らかの赤色へアライメントされました。方位: {initial_aligned_heading:.2f}度")
            else:
                print("初期アライメントスキャンで赤色は全く検出されませんでした。")
            
            if len(initial_scan_detected_angles) >= 2:
                target_center_angle = calculate_angle_average(initial_scan_detected_angles)
                if target_center_angle is not None:
                    print(f"\n=== 初期アライメントスキャンで複数赤色検知地点の中心 ({target_center_angle:.2f}度) へ向きを調整します ===")
                    current_heading = self.get_bno_heading()
                    if current_heading is not None:
                        angle_to_turn = (target_center_angle - current_heading + 180 + 360) % 360 - 180
                        self.turn_to_relative_angle(angle_to_turn, turn_speed=90, angle_tolerance_deg=15)
                        self.driver.motor_stop_brake()
                        time.sleep(0.5)
                        print("中心方向への向き調整が完了しました。")
                        
                        print("  --> 中心方向へ向いた後、1秒間前進します。")
                        self.driver.petit_petit(9)
                        time.sleep(1)
                        self.driver.motor_stop_brake()
                        time.sleep(0.5)
                        print("  --> 1秒前進を完了しました。次の「周囲確認（最終確認スキャン）」へ移行します。")
                        skip_forward_scan_phase = True
                    else:
                        print("警告: 複数赤色検知後の回頭時に現在方位が取得できませんでした。")
                        skip_forward_scan_phase = False
                else:
                    print("警告: 検出された角度からの中心角度計算に失敗しました。")
                    skip_forward_scan_phase = False
            elif len(initial_scan_detected_angles) == 1:
                print(f"\n=== 赤色を1ヶ所のみ検出 ({initial_scan_detected_angles[0]::.2f}度) しました。その方向へ向きを調整済みです。")
                skip_forward_scan_phase = False
            else:
                print("\n=== 赤色検知がなかったため、次の行動に移ります。 ===")
                skip_forward_scan_phase = False

            if skip_forward_scan_phase:
                print("--- 「アライメント後、360度スキャンで赤色を探索し前進判断」フェーズをスキップします。 ---")
                pass
            else:
                print("\n=== アライメント後、360度スキャンで赤色を探索し前進判断 ===")
                any_red_detected_and_moved_this_scan = False  
                scan_detections = []

                print("  --> 前進判断のため、20度回転します...")
                self.turn_to_relative_angle(20, turn_speed=90, angle_tolerance_deg=15)
                self.driver.motor_stop_brake()
                time.sleep(0.5)

                for i in range(360 // 20): 
                    current_scan_heading_for_forward = self.get_bno_heading()
                    if current_scan_heading_for_forward is None:
                        print("警告: スキャン中に方位が取得できませんでした。スキップします。")
                        continue

                    print(f"--- 360度スキャン中: 現在の方向: {current_scan_heading_for_forward:.2f}度 ---")

                    current_red_percentage_scan = detect_red_percentage(
                        self.picam2, 
                        save_path=f"/home/mark1/Pictures/forward_scan_{i*20 + 20:03d}.jpg"
                    )

                    if current_red_percentage_scan == -1.0:
                        print("カメラ処理でエラーが発生しました。現在のスキャンステップをスキップします。")
                        continue
                    
                    scan_detections.append({'percentage': current_red_percentage_scan, 'heading': current_scan_heading_for_forward})

                    if current_red_percentage_scan >= 0.05:
                        print(f"  --> 赤色を{0.05:.0%}以上検出！この方向に1秒前進します。")
                        self.driver.petit_petit(4)
                        time.sleep(1)
                        self.driver.motor_stop_brake()
                        time.sleep(0.5)
                        any_red_detected_and_moved_this_scan = True  
                        break

                    if i < (360 // 20) - 1: 
                        print(f"  --> スキャン中: さらに20度回転...")
                        self.turn_to_relative_angle(20, turn_speed=90, angle_tolerance_deg=15)
                        self.driver.motor_stop_brake()
                        time.sleep(0.5)

                if not any_red_detected_and_moved_this_scan and len(initial_scan_detected_angles) == 1:
                    print("\n=== 初期アライメントスキャンで赤色を1ヶ所のみ検知。360度スキャンで2番目に赤の割合が大きかった方向へ前進します。 ===")
                    
                    if scan_detections:
                        sorted_detections = sorted(scan_detections, key=lambda x: x['percentage'], reverse=True)
                        
                        best_red_after_initial_alignment = None
                        if len(sorted_detections) >= 2:
                            for det in sorted_detections:
                                angle_diff = (det['heading'] - initial_aligned_heading + 180 + 360) % 360 - 180
                                if abs(angle_diff) > 20:
                                    best_red_after_initial_alignment = det
                                    break

                            if best_red_after_initial_alignment:
                                target_heading_for_second_best = best_red_after_initial_alignment['heading']
                                print(f"  --> 2番目に赤の割合が大きかった方向 ({target_heading_for_second_best:.2f}度, 割合: {best_red_after_initial_alignment['percentage']:.2%}) へ向きを調整し、1秒間前進します。")
                                current_heading_before_adjust = self.get_bno_heading()
                                if current_heading_before_adjust is not None:
                                    angle_to_turn = (target_heading_for_second_best - current_heading_before_adjust + 180 + 360) % 360 - 180
                                    self.turn_to_relative_angle(angle_to_turn, turn_speed=90, angle_tolerance_deg=15)
                                    self.driver.motor_stop_brake()
                                    time.sleep(0.5)
                                    print("向き調整が完了しました。")

                                    print("  --> 調整後、1秒間前進します。")
                                    self.driver.petit_petit(4)
                                    time.sleep(1)
                                    self.driver.motor_stop_brake()
                                    time.sleep(0.5)
                                    print("  --> 1秒前進を完了しました。")

                                    print("\n=== 1秒前進後、追加の360度スキャンと中心方向への調整を開始します ===")
                                    post_forward_scan_detected_angles = []
                                    print("  --> 追加スキャンのため、20度回転します...")
                                    self.turn_to_relative_angle(20, turn_speed=90, angle_tolerance_deg=15)
                                    self.driver.motor_stop_brake()
                                    time.sleep(0.5)

                                    for j in range(360 // 20):
                                        current_post_forward_heading = self.get_bno_heading()
                                        if current_post_forward_heading is None:
                                            print("警告: 追加スキャン中に方位が取得できませんでした。スキップします。")
                                            continue

                                        print(f"--- 追加スキャン中: 現在の方向: {current_post_forward_heading:.2f}度 ---")
                                        
                                        current_red_percentage_post_forward_scan = detect_red_percentage(
                                            self.picam2,
                                            save_path=f"/home/mark1/Pictures/post_forward_scan_{j*20 + 20:03d}.jpg"
                                        )

                                        if current_red_percentage_post_forward_scan == -1.0:
                                            print("カメラ処理エラーのため、現在の追加スキャンステップをスキップします。")
                                            continue

                                        if current_red_percentage_post_forward_scan >= 0.05:
                                            print(f"  --> 追加スキャンで赤色を{0.05:.0%}以上検出！方向を記録します。")
                                            post_forward_scan_detected_angles.append(current_post_forward_heading)
                                        
                                        if j < (360 // 20) - 1:
                                            print(f"  --> 追加スキャン中: さらに20度回転...")
                                            self.turn_to_relative_angle(20, turn_speed=90, angle_tolerance_deg=15)
                                            self.driver.motor_stop_brake()
                                            time.sleep(0.5)
                                        
                                    if len(post_forward_scan_detected_angles) >= 2:
                                        target_center_angle_post_forward = calculate_angle_average(post_forward_scan_detected_angles)
                                        if target_center_angle_post_forward is not None:
                                            print(f"\n=== 追加スキャンで複数赤色検知！中心 ({target_center_angle_post_forward:.2f}度) へ向きを調整します ===")
                                            current_heading_at_post_forward_end = self.get_bno_heading()
                                            if current_heading_at_post_forward_end is not None:
                                                angle_to_turn_post_forward = (target_center_angle_post_forward - current_heading_at_post_forward_end + 180 + 360) % 360 - 180
                                                self.turn_to_relative_angle(angle_to_turn_post_forward, turn_speed=90, angle_tolerance_deg=15)
                                                self.driver.motor_stop_brake()
                                                time.sleep(0.5)
                                                print("追加スキャン後の中心方向への向き調整が完了しました。")
                                            else:
                                                print("警告: 追加スキャン後の中心回頭時に方位が取得できませんでした。")
                                        else:
                                            print("警告: 追加スキャンで検出された角度からの中心角度計算に失敗しました。")
                                    else:
                                        print("\n=== 追加スキャンで赤色の複数検知はありませんでした。 ===")
                                else:
                                    print("警告: 2番目の方向への回頭時に現在方位が取得できませんでした。")
                            else:
                                print("警告: 2番目に適した赤色検出方向が見つかりませんでした。")
                        else:
                            print("警告: 2番目に適した赤色検出方向を特定するためのデータが不足しています。")


                if any_red_detected_and_moved_this_scan:
                    print("\n=== 赤色を検知し1秒前進しました。追加の2個検知スキャンを開始します ===")
                    
                    second_scan_detected_angles = []
                    
                    print("  --> 2回目スキャンのため、20度回転します...")
                    self.turn_to_relative_angle(20, turn_speed=90, angle_tolerance_deg=15)
                    self.driver.motor_stop_brake()
                    time.sleep(0.5)

                    for i in range(360 // 20):
                        current_scan_heading_for_second = self.get_bno_heading()
                        if current_scan_heading_for_second is None:
                            print("警告: 2回目スキャン中に方位が取得できませんでした。スキップします。")
                            continue

                        print(f"--- 2回目スキャン中: 現在の方向: {current_scan_heading_for_second:.2f}度 ---")

                        current_red_percentage_second_scan = detect_red_percentage(
                            self.picam2, 
                            save_path=f"/home/mark1/Pictures/second_scan_{i*20 + 20:03d}.jpg"
                        )

                        if current_red_percentage_second_scan == -1.0:
                            print("カメラ処理でエラーが発生しました。2回目スキャンステップをスキップします。")
                            continue
                        
                        if current_red_percentage_second_scan >= 0.05:
                            print(f"  --> 2回目スキャンで赤色を{0.05:.0%}以上検出！方向を記録します。")
                            second_scan_detected_angles.append(current_scan_heading_for_second)

                        if i < (360 // 20) - 1:
                            print(f"  --> 2回目スキャン中: さらに20度回転...")
                            self.turn_to_relative_angle(20, turn_speed=90, angle_tolerance_deg=15)
                            self.driver.motor_stop_brake()
                            time.sleep(0.5)

                    if len(second_scan_detected_angles) >= 2:
                        target_center_angle_second_scan = calculate_angle_average(second_scan_detected_angles)
                        if target_center_angle_second_scan is not None:
                            print(f"\n=== 2回目スキャンで複数赤色検知！中心 ({target_center_angle_second_scan:.2f}度) へ向きを調整し、1秒前進します ===")
                            current_heading_before_adjust_second = self.get_bno_heading()
                            if current_heading_before_adjust_second is not None:
                                angle_to_turn_second = (target_center_angle_second_scan - current_heading_before_adjust_second + 180 + 360) % 360 - 180
                                self.turn_to_relative_angle(angle_to_turn_second, turn_speed=90, angle_tolerance_deg=15)
                                self.driver.motor_stop_brake()
                                time.sleep(0.5)
                                print("中心方向への向き調整が完了しました。")
                                
                                print("  --> 中心方向へ向いた後、1秒間前進します。")
                                self.driver.petit_petit(8)
                                time.sleep(1)
                                self.driver.motor_stop_brake()
                                time.sleep(0.5)
                                print("  --> 1秒前進を完了しました。")
                            else:
                                print("警告: 2回目スキャン後の回頭時に現在方位が取得できませんでした。")
                        else:
                            print("警告: 2回目スキャンで検出された角度からの中心角度計算に失敗しました。")
                    else:
                        print("\n=== 2回目スキャンで赤色の複数検知はありませんでした。 ===")

            print("\n=== 周囲確認を開始します (360度スキャン - 最終確認用) ===")
            
            while True:
                final_scan_result = self.perform_final_scan_and_terminate(final_threshold=0.07, min_red_detections_to_terminate=4, high_red_threshold=0.40)
                if final_scan_result is True:
                    print("最終確認スキャン条件達成（40%検出で即座に180度回転前進）。最終確認スキャンを再実行します。")
                    continue
                elif final_scan_result is False:
                    print("最終確認スキャンが完了し、プログラム終了条件を満たしました。ミッションを終了します。")
                    sys.exit(0)
                else:
                    print("最終確認スキャンは条件を満たしませんでした。メインループの次の走行サイクルに進みます。")
                    break
            
            if not any_red_detected_and_moved_this_scan and len(initial_scan_detected_angles) == 0:
                print("  --> 赤色を検出しなかったため、3秒間前進し、再度アライメントから開始します。")
                self.driver.petit_petit(10)
                time.sleep(3)
                self.driver.motor_stop_brake()
                time.sleep(0.5)
            
            continue

    def cleanup(self):
        """
        GPIO、pigpio、カメラ、モータードライバをクリーンアップします。
        """
        print("=== クリーンアップを開始します ===")
        self.driver.cleanup()
        if self.pi_instance and self.pi_instance.connected:
            self.pi_instance.stop()
            print("pigpioデーモンが停止しました。")
        if self.picam2:
            self.picam2.close()
            print("Picamera2がクローズされました。")
        GPIO.cleanup()
        print("GPIOがクリーンアップされました。")
        print("=== 処理を終了しました。 ===")
