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
from motor import MotorDriver
from BNO055 import BNO055 # あなたのカスタムBNO055クラス

class GDA:
    def __init__(self, motor_pwma_pin=12, motor_ain1_pin=23, motor_ain2_pin=18,
                 motor_pwmb_pin=19, motor_bin1_pin=16, motor_bin2_pin=26,
                 motor_stby_pin=21, bno_sensor_instance=None, rx_pin=17):
        self.RX_PIN = rx_pin
        self.BAUD = 9600
        self.picam2_instance = Picamera2()
        config = self.picam2_instance.create_still_configuration(main={"size": (320, 480)})
        self.picam2_instance.configure(config)
        self.picam2_instance.start()
        self.driver = MotorDriver(
            PWMA=12, AIN1=23, AIN2=18,
            PWMB=19, BIN1=16, BIN2=26,
            STBY=21
        )
        self.bno_sensor = bno_sensor_instance # 渡されたBNO055インスタンスを保持
        self.pi_instance = pigpio.pi()
        if not self.pi_instance.connected:
            raise RuntimeError("pigpio デーモンに接続できません。sudo pigpiod を起動してください。")
        err = self.pi_instance.bb_serial_read_open(self.RX_PIN, self.BAUD, 8)
        if err != 0:
            self.pi_instance.stop()
            raise RuntimeError(f"ソフトUART RX 設定失敗: GPIO={self.RX_PIN}, {self.BAUD}bps")
        
    def _get_bno_heading(self):
        """
        BNO055センサーから現在の方位を取得します。
        Noneの場合のリトライロジックを含みます。
        """
        heading = self.bno_sensor.get_heading()
        if heading is None:
            wait_start_time = time.time()
            max_wait_time = 0.5
            while heading is None and (time.time() - wait_start_time < max_wait_time):
                time.sleep(0.01)
                heading = self.bno_sensor.get_heading()
        if heading is None:
            return 0.0 # 最終的にNoneなら0.0を返す
        return heading

    def _save_image_for_debug(self, path="/home/mark1/Pictures/paravo_image.jpg"):
        frame = self.picam2_instance.capture_array()
        if frame is None:
            print("画像キャプチャ失敗：フレームがNoneです。")
            return None
        frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        cv2.imwrite(path, frame_bgr)
        print(f"画像保存成功: {path}")
        return frame

    def _detect_red_percentage(self, save_path="/home/mark1/Pictures/red_detection_overall.jpg"):
        try:
            frame_rgb = self.picam2_instance.capture_array()
            if frame_rgb is None:
                print("画像キャプチャ失敗: フレームがNoneです。")
                return -1.0 

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
            return red_percentage / 100.0

        except Exception as e:
            print(f"カメラ撮影・処理中にエラーが発生しました: {e}")
            return -1.0

    def _turn_to_relative_angle(self, angle_offset_deg, turn_speed=90, angle_tolerance_deg=10.0, max_turn_attempts=100):
        """
        現在のBNO055の方位から、指定された角度だけ相対的に旋回します。
        BNO055Wrapperのget_heading()の代わりに、直接_get_bno_heading()を呼び出します。
        """
        initial_heading = self._get_bno_heading()
        if initial_heading is None:
            print("警告: turn_to_relative_angle: 初期方位が取得できませんでした。")
            return False
        
        target_heading = (initial_heading + angle_offset_deg + 360) % 360
        print(f"現在のBNO方位: {initial_heading:.2f}度, 相対目標角度: {angle_offset_deg:.2f}度 -> 絶対目標方位: {target_heading:.2f}度")

        loop_count = 0
        
        while loop_count < max_turn_attempts:
            current_heading = self._get_bno_heading()
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
                self.driver.petit_left(turn_speed, 0) # 左に旋回
            else:
                self.driver.petit_right(0, turn_speed)
                self.driver.petit_right(turn_speed, 0) # 右に旋回
            
            time.sleep(turn_duration_on)
            self.driver.motor_stop_brake()
            time.sleep(0.05)
            
            loop_count += 1
        
        print(f"警告: turn_to_relative_angle: 最大試行回数({max_turn_attempts}回)内に目標角度に到達できませんでした。最終誤差: {angle_error:.2f}度 (試行回数: {loop_count})")
        self.driver.motor_stop_brake()
        time.sleep(0.5)
        return False

    def _calculate_angle_average(self, angles_deg):
        if not angles_deg:
            return None

        x_coords = [math.cos(math.radians(angle)) for angle in angles_deg]
        y_coords = [math.sin(math.radians(angle)) for angle in angles_deg]

        sum_x = sum(x_coords)
        sum_y = sum(y_coords)

        average_angle_rad = math.atan2(sum_y, sum_x)
        average_angle_deg = math.degrees(average_angle_rad)

        return (average_angle_deg + 360) % 360

    def _perform_final_scan_and_terminate(self, turn_angle_step=20, final_threshold=0.15, min_red_detections_to_terminate=4, high_red_threshold=0.40):
        """
        BNO055Wrapperのget_heading()の代わりに、直接_get_bno_heading()を呼び出します。
        """
        print(f"\n=== 最終確認スキャンを開始します ({final_threshold:.0%}閾値、{min_red_detections_to_terminate}ヶ所検知、{high_red_threshold:.0%}高閾値で即座に180度回転前進) ===")
        initial_heading = self._get_bno_heading()
        if initial_heading is None:
            print("警告: 最終確認スキャン開始時に方位が取得できませんでした。")
            return None 

        final_scan_detected_angles = []
        
        # 初回回転 (スキャン開始位置の調整)
        print(f"  初回回転: {turn_angle_step}度...")
        self._turn_to_relative_angle(turn_angle_step, turn_speed=90, angle_tolerance_deg=20)
        
        for i in range(360 // turn_angle_step): # 360度をステップごとにスキャン
            if i > 0: # 2回目以降の回転
                print(f"  --> スキャン中: さらに{turn_angle_step}度回転...")
                self._turn_to_relative_angle(turn_angle_step, turn_speed=90, angle_tolerance_deg=20)
                self.driver.motor_stop_brake()
                time.sleep(0.5)
            
            current_scan_heading = self._get_bno_heading()
            if current_scan_heading is None:
                print("警告: 最終確認スキャン中に方位が取得できませんでした。スキップします。")
                self.driver.motor_stop_brake()
                time.sleep(0.1)
                continue

            print(f"--- 最終確認スキャン中: 現在の方向: {current_scan_heading:.2f}度 ---")

            overall_red_ratio = self._detect_red_percentage(
                save_path=f"/home/mark1/Pictures/final_scan_{i*turn_angle_step + turn_angle_step:03d}.jpg"
            )

            if overall_red_ratio == -1.0:
                print("カメラ処理エラーのため、現在のスキャンステップをスキップします。")
                continue

            print(f"検出結果: 画像全体の赤色割合: {overall_red_ratio:.2%}")

            if overall_red_ratio >= high_red_threshold:
                print(f"\n  --> **高い赤色割合 ({high_red_threshold:.0%}) を検出しました！即座に180度回転して前進します。**")
                self._turn_to_relative_angle(180, turn_speed=90, angle_tolerance_deg=20)
                self.driver.motor_stop_brake()
                time.sleep(0.5)
                
                self.driver.petit_petit(4) # 前進
                time.sleep(2)
                self.driver.motor_stop_brake()
                time.sleep(0.5)
                print("  --> 180度回転して前進が完了しました。最終確認スキャンを最初から再開します。")
                return True # 即座に再開を示す

            if overall_red_ratio >= final_threshold:
                print(f"  --> 赤色を{final_threshold:.0%}以上検出！方向を記録します。")
                final_scan_detected_angles.append(current_scan_heading)
            
            self.driver.motor_stop_brake()
            time.sleep(0.5)

        if len(final_scan_detected_angles) >= min_red_detections_to_terminate:
            print(f"\n  --> 最終確認スキャンで{min_red_detections_to_terminate}ヶ所以上の赤色を検出しました ({len(final_scan_detected_angles)}ヶ所)。")
            
            target_center_angle = self._calculate_angle_average(final_scan_detected_angles)
            if target_center_angle is not None:
                print(f"  --> 検出された赤色の中心 ({target_center_angle:.2f}度) へ向きを調整します。")
                current_heading_at_end = self._get_bno_heading()
                if current_heading_at_end is not None:
                    angle_to_turn = (target_center_angle - current_heading_at_end + 180 + 360) % 360 - 180
                    self._turn_to_relative_angle(angle_to_turn, turn_speed=90, angle_tolerance_deg=20)
                    self.driver.motor_stop_brake()
                    time.sleep(1.0)
                    print("  --> 中心方向への向き調整が完了しました。")
                else:
                    print("警告: 最終スキャン後の中心回頭時に方位が取得できませんでした。")
            
            print("  --> 4ヶ所検知の条件（ただし40%未満）を満たしたため、ミッションを終了します。")
            return False # 終了条件達成を示す
        else:
            print(f"\n=== 最終確認スキャンが完了しました。条件を満たしませんでした (検出箇所: {len(final_scan_detected_angles)}ヶ所)。 ===")
            return None # 条件未達を示す

    def _perform_initial_alignment_scan(self, turn_angle_step=20, alignment_threshold=0.10):
        """
        BNO055Wrapperのget_heading()の代わりに、直接_get_bno_heading()を呼び出します。
        """
        print("\n=== 初期赤色アライメントスキャンを開始します (20%閾値) ===")
        initial_heading_at_start = self._get_bno_heading()
        if initial_heading_at_start is None:
            print("警告: 初期アライメントスキャン開始時に方位が取得できません。")
            return False, None, []

        aligned = False
        max_red_ratio = -1.0
        best_heading_for_red = initial_heading_at_start
        
        detected_red_angles = []

        # スキャン開始時に指定された角度だけ回転します。
        # 初回の270度回転は、このループの前に一度だけ行うか、
        # あるいはループ内で最初のステップとして処理するかの選択が必要です。
        # 現在のコードの意図が270度回ってからスキャン開始のようなので、それを踏襲します。
        
        # 最初の270度回転
        print(f"  初回回転: 270度...")
        self._turn_to_relative_angle(270, turn_speed=90, angle_tolerance_deg=20)
        self.driver.motor_stop_brake()
        time.sleep(0.5)

        # 270度の範囲でスキャンするために、ループ回数を調整
        num_steps = 270 // turn_angle_step

        for i in range(num_steps):
            if i > 0: # 2回目以降の回転
                print(f"  回転: {turn_angle_step}度...")
                self._turn_to_relative_angle(turn_angle_step, turn_speed=90, angle_tolerance_deg=20)
                self.driver.motor_stop_brake()
                time.sleep(0.5)
            
            current_scan_heading = self._get_bno_heading()
            if current_scan_heading is None:
                print("警告: 初期アライメントスキャン中に方位が取得できませんでした。スキップします。")
                self.driver.motor_stop_brake()
                time.sleep(0.1)
                continue

            print(f"\n--- 初期アライメントスキャン中: 現在の方向: {current_scan_heading:.2f}度 ---")
            
            overall_red_ratio = self._detect_red_percentage(
                save_path=f"/home/mark1/Pictures/initial_alignment_scan_{i*turn_angle_step:03d}.jpg" # ログ名も調整
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
            
            # 各スキャンステップ後に停止
            self.driver.motor_stop_brake()
            time.sleep(0.5)

        # スキャン終了後のアライメントロジック
        if not detected_red_angles:
            print(f"初期アライメントスキャンで{alignment_threshold*100.0:.0f}%以上の赤色は検出されませんでした。")
            if max_red_ratio > -1.0:
                current_heading_at_end_of_scan = self._get_bno_heading()
                
                if current_heading_at_end_of_scan is not None:
                    angle_to_turn_to_best_red = (best_heading_for_red - current_heading_at_end_of_scan + 180 + 360) % 360 - 180
                    
                    print(f"  --> {alignment_threshold*100.0:.0f}%"
                          f"以上は検出されませんでしたが、最も多くの赤 ({max_red_ratio:.2f}%) が検出された方向 ({best_heading_for_red:.2f}度) へアライメントします (相対回転: {angle_to_turn_to_best_red:.2f}度)。")
                    self._turn_to_relative_angle(angle_to_turn_to_best_red, turn_speed=90, angle_tolerance_deg=20)
                    self.driver.motor_stop_brake()
                    time.sleep(0.5)
                    aligned = True
                    detected_red_angles.append(best_heading_for_red) # 最大赤色方向も検出角度リストに追加
                else:
                    print("警告: スキャン終了時に方位が取得できず、最大赤色方向へのアライメントができませんでした。")
            else:
                print("初期アライメントスキャンで赤色は全く検出されませんでした。")
        else:
            aligned = True

        print("=== 初期赤色アライメントスキャンが完了しました。 ===")
        return aligned, best_heading_for_red, detected_red_angles

    def HAT_TRICK(self):
        """メインの自律走行ループを開始します。"""
        try:
            print("BNO055のキャリブレーション待機はスキップされました。自動操縦を開始します。")

            while True:
                print("\n--- 新しい走行サイクル開始 ---")
                
                print("\n=== 現在方位確認 ===")
                current_bno_heading_for_info = self._get_bno_heading()
                if current_bno_heading_for_info is None:
                    print("警告: 現在方位が取得できませんでした。")
                    time.sleep(2)
                    continue
                print(f"現在のBNO方位: {current_bno_heading_for_info:.2f}度")
                self.driver.motor_stop_brake()
                time.sleep(0.5)

                skip_forward_scan_phase = False 
                aligned_in_initial_scan, initial_aligned_heading, initial_scan_detected_angles = self._perform_initial_alignment_scan() 

                if len(initial_scan_detected_angles) >= 4:
                    print(f"\n=== 初期アライメントスキャンで{len(initial_scan_detected_angles)}ヶ所の赤色を検知しました。最終確認スキャンへスキップします。 ===")
                    while True:
                        final_scan_result = self._perform_final_scan_and_terminate(final_threshold=0.07, min_red_detections_to_terminate=4, high_red_threshold=0.40)
                        if final_scan_result is True:
                            print("最終確認スキャン条件達成（40%検出で即座に180度回転前進）。最終確認スキャンを再実行します。")
                            continue
                        elif final_scan_result is False:
                            print("最終確認スキャンが完了し、プログラム終了条件を満たしました。ミッションを終了します。")
                            sys.exit(0)
                        else:
                            print("最終確認スキャンは条件を満たしませんでした。メインループの通常フローに戻ります。")
                            break
                    continue

                if aligned_in_initial_scan:
                    print(f"初期アライメントスキャンによって、何らかの赤色へアライメントされました。方位: {initial_aligned_heading:.2f}度")
                else:
                    print("初期アライメントスキャンで赤色は全く検出されませんでした。")
                
                if len(initial_scan_detected_angles) >= 2:
                    target_center_angle = self._calculate_angle_average(initial_scan_detected_angles)
                    if target_center_angle is not None:
                        print(f"\n=== 初期アライメントスキャンで複数赤色検知地点の中心 ({target_center_angle:.2f}度) へ向きを調整します ===")
                        current_heading = self._get_bno_heading()
                        if current_heading is not None:
                            angle_to_turn = (target_center_angle - current_heading + 180 + 360) % 360 - 180
                            self._turn_to_relative_angle(angle_to_turn, turn_speed=90, angle_tolerance_deg=20)
                            self.driver.motor_stop_brake()
                            time.sleep(0.5)
                            print("中心方向への向き調整が完了しました。")
                            
                            print("  --> 中心方向へ向いた後、1秒間前進します。")
                            self.driver.petit_petit(15)
                            time.sleep(1)
                            self.driver.motor_stop_brake()
                            time.sleep(0.5)
                            print("  --> 1秒前進を完了しました。次の「周囲確認（最終確認スキャン）」へ移行します。")
                            skip_forward_scan_phase = True
                        else:
                            print("警告: 複数赤色検知後の回頭時に現在方位が取得できませんでした。")
                            skip_forward_scan_phase = False # エラー時はスキップしない
                    else:
                        print("警告: 検出された角度からの中心角度計算に失敗しました。")
                        skip_forward_scan_phase = False # エラー時はスキップしない
                elif len(initial_scan_detected_angles) == 1:
                    print(f"\n=== 赤色を1ヶ所のみ検出 ({initial_scan_detected_angles[0]:.2f}度) しました。その方向へ向きを調整済みです。")
                    skip_forward_scan_phase = False
                else:
                    print("\n=== 赤色検知がなかったため、次の行動に移ります。 ===")
                    skip_forward_scan_phase = False

                if skip_forward_scan_phase:
                    print("--- 「アライメント後、360度スキャンで赤色を探索し前進判断」フェーズをスキップします。 ---")
                    pass # スキップする場合は何もしない
                else:
                    print("\n=== アライメント後、360度スキャンで赤色を探索し前進判断 ===")
                    
                    any_red_detected_and_moved_this_scan = False  
                    scan_detections = []

                    print("  --> 前進判断のため、20度回転します...")
                    self._turn_to_relative_angle(20, turn_speed=90, angle_tolerance_deg=20)
                    self.driver.motor_stop_brake()
                    time.sleep(0.5)

                    for i in range(360 // 20): 
                        current_scan_heading_for_forward = self._get_bno_heading()
                        if current_scan_heading_for_forward is None:
                            print("警告: スキャン中に方位が取得できませんでした。スキップします。")
                            continue

                        print(f"--- 360度スキャン中: 現在の方向: {current_scan_heading_for_forward:.2f}度 ---")

                        current_red_percentage_scan = self._detect_red_percentage(
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
                            break # 赤色を検出して前進したら、このスキャンを中断

                        if i < (360 // 20) - 1: # 最後のループでは回転しない
                            print(f"  --> スキャン中: さらに20度回転...")
                            self._turn_to_relative_angle(20, turn_speed=90, angle_tolerance_deg=20)
                            self.driver.motor_stop_brake()
                            time.sleep(0.5)

                    if not any_red_detected_and_moved_this_scan and len(initial_scan_detected_angles) == 1:
                        print("\n=== 初期アライメントスキャンで赤色を1ヶ所のみ検知。360度スキャンで2番目に赤の割合が大きかった方向へ前進します。 ===")
                        
                        if scan_detections:
                            sorted_detections = sorted(scan_detections, key=lambda x: x['percentage'], reverse=True)
                            
                            best_red_after_initial_alignment = None
                            if len(sorted_detections) >= 2:
                                for det in sorted_detections:
                                    # 元のアライメント方向と大きく異なる方向を選ぶための閾値
                                    angle_diff = (det['heading'] - initial_aligned_heading + 180 + 360) % 360 - 180
                                    if abs(angle_diff) > 20: # 20度以上離れている方向を探す
                                        best_red_after_initial_alignment = det
                                        break

                                if best_red_after_initial_alignment:
                                    target_heading_for_second_best = best_red_after_initial_alignment['heading']
                                    print(f"  --> 2番目に赤の割合が大きかった方向 ({target_heading_for_second_best:.2f}度, 割合: {best_red_after_initial_alignment['percentage']:.2%}) へ向きを調整し、1秒間前進します。")
                                    current_heading_before_adjust = self._get_bno_heading()
                                    if current_heading_before_adjust is not None:
                                        angle_to_turn = (target_heading_for_second_best - current_heading_before_adjust + 180 + 360) % 360 - 180
                                        self._turn_to_relative_angle(angle_to_turn, turn_speed=90, angle_tolerance_deg=20)
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
                                        self._turn_to_relative_angle(20, turn_speed=90, angle_tolerance_deg=20)
                                        self.driver.motor_stop_brake()
                                        time.sleep(0.5)

                                        for j in range(360 // 20):
                                            current_post_forward_heading = self._get_bno_heading()
                                            if current_post_forward_heading is None:
                                                print("警告: 追加スキャン中に方位が取得できませんでした。スキップします。")
                                                continue

                                            print(f"--- 追加スキャン中: 現在の方向: {current_post_forward_heading:.2f}度 ---")
                                            
                                            current_red_percentage_post_forward_scan = self._detect_red_percentage(
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
                                                self._turn_to_relative_angle(20, turn_speed=90, angle_tolerance_deg=20)
                                                self.driver.motor_stop_brake()
                                                time.sleep(0.5)
                                            
                                        if len(post_forward_scan_detected_angles) >= 2:
                                            target_center_angle_post_forward = self._calculate_angle_average(post_forward_scan_detected_angles)
                                            if target_center_angle_post_forward is not None:
                                                print(f"\n=== 追加スキャンで複数赤色検知！中心 ({target_center_angle_post_forward:.2f}度) へ向きを調整します ===")
                                                current_heading_at_post_forward_end = self._get_bno_heading()
                                                if current_heading_at_post_forward_end is not None:
                                                    angle_to_turn_post_forward = (target_center_angle_post_forward - current_heading_at_post_forward_end + 180 + 360) % 360 - 180
                                                    self._turn_to_relative_angle(angle_to_turn_post_forward, turn_speed=90, angle_tolerance_deg=20)
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
                        self._turn_to_relative_angle(20, turn_speed=90, angle_tolerance_deg=20)
                        self.driver.motor_stop_brake()
                        time.sleep(0.5)

                        for i in range(360 // 20):
                            current_scan_heading_for_second = self._get_bno_heading()
                            if current_scan_heading_for_second is None:
                                print("警告: 2回目スキャン中に方位が取得できませんでした。スキップします。")
                                continue

                            print(f"--- 2回目スキャン中: 現在の方向: {current_scan_heading_for_second:.2f}度 ---")

                            current_red_percentage_second_scan = self._detect_red_percentage(
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
                                self._turn_to_relative_angle(20, turn_speed=90, angle_tolerance_deg=20)
                                self.driver.motor_stop_brake()
                                time.sleep(0.5)

                        if len(second_scan_detected_angles) >= 2:
                            target_center_angle_second_scan = self._calculate_angle_average(second_scan_detected_angles)
                            if target_center_angle_second_scan is not None:
                                print(f"\n=== 2回目スキャンで複数赤色検知！中心 ({target_center_angle_second_scan:.2f}度) へ向きを調整し、1秒前進します ===")
                                current_heading_before_adjust_second = self._get_bno_heading()
                                if current_heading_before_adjust_second is not None:
                                    angle_to_turn_second = (target_center_angle_second_scan - current_heading_before_adjust_second + 180 + 360) % 360 - 180
                                    self._turn_to_relative_angle(angle_to_turn_second, turn_speed=90, angle_tolerance_deg=20)
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
                    final_scan_result = self._perform_final_scan_and_terminate(final_threshold=0.07, min_red_detections_to_terminate=4, high_red_threshold=0.40)
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
                
                continue # 次の走行サイクルへ

        except Exception as e:
            print(f"メイン処理中に予期せぬエラーが発生しました: {e}")
            self.driver.motor_stop_brake()

        finally:
            self.cleanup()

    def cleanup(self):
        if self.driver:
            self.driver.cleanup()
            print("MotorDriver cleaned up.")
        if self.pi_instance and self.pi_instance.connected:
            self.pi_instance.bb_serial_read_close(self.RX_PIN) # ソフトUARTのクローズを追加
            self.pi_instance.stop()
            print("pigpio stopped.")
        if self.picam2_instance:
            self.picam2_instance.close()
            print("Picamera2 closed.")
        GPIO.cleanup()
        print("GPIO cleaned up.")
        print("=== 処理を終了しました。 ===")


# --- メインシーケンス ---
if __name__ == "__main__":
    # BNO055センサーの初期化はGDAクラスの外部で行います
    bno_sensor_address = 0x28 # BNO055のアドレス
    bno_sensor = None # 初期化のためNoneで宣言

    try:
        bno_sensor = BNO055(address=bno_sensor_address)
        if not bno_sensor.begin():
            raise RuntimeError("BNO055センサーの初期化に失敗しました。終了します。")
        bno_sensor.setMode(BNO055.OPERATION_MODE_NDOF)
        bno_sensor.setExternalCrystalUse(True)
        time.sleep(1)
        print("BNO055 sensor initialized outside GDA class.")
        
        # GDAクラスのインスタンスを作成し、初期化したBNO055センサーを渡す
        rover = GDA(
            motor_pwma_pin=12, motor_ain1_pin=23, motor_ain2_pin=18,
            motor_pwmb_pin=19, motor_bin1_pin=16, motor_bin2_pin=26,
            motor_stby_pin=21, 
            bno_sensor_instance=bno_sensor, # ここでBNO055インスタンスを渡します
            rx_pin=17
        )
        rover.HAT_TRICK()

    except Exception as e:
        print(f"プログラム全体の初期化または実行中に予期せぬエラーが発生しました: {e}")
    finally:
        # GDAインスタンスが作成されなかった場合でもcleanupを試みる
        if 'rover' in locals():
            rover.cleanup()
        else:
            # GDAインスタンスが作成される前にエラーが発生した場合のクリーンアップ
            # BNO055には直接的なクリーンアップメソッドがない場合が多いので、
            # 必要であればここに追加します。
            # pigpioのbb_serial_read_openがGDA初期化時に呼ばれているため、
            # エラーでGDAの__init__が最後まで実行されなかった場合、bb_serial_read_closeが呼ばれない可能性があります。
            # pigpioのpiインスタンスもここで閉じるべきですが、bno_sensorが生成されていないとpi_instanceもないので、少し複雑です。
            # 安全のため、ここでは最低限のGPIOクリーンアップに留めます。
            GPIO.cleanup() 
        print("=== プログラムを終了しました。 ===")
