import cv2
import numpy as np
import time
from picamera2 import Picamera2
from motor import MotorDriver # Assuming motor.py contains MotorDriver
import following # Assuming following.py contains follow_forward
from BNO055 import BNO055 # Assuming BNO055.py contains BNO055
import RPi.GPIO as GPIO # RPi.GPIO is needed for MotorDriver and BNO055

class RedConeNavigator:
    """
    カメラで赤色のコーンを検出し、その位置と面積に基づいてローバーを誘導するクラス。
    コーンが見つからない場合は探索を行い、適切な位置にローバーを移動させます。
    """

    # --- クラス定数 (調整可能) ---
    # 赤色検出のHSV閾値
    LOWER_RED1 = np.array([0, 100, 100])
    UPPER_RED1 = np.array([10, 255, 255])
    LOWER_RED2 = np.array([160, 100, 100])
    UPPER_RED2 = np.array([180, 255, 255])
    
    # 探索時のモーター速度
    SEARCH_LEFT_MOTOR_SPEED = 90
    SEARCH_RIGHT_MOTOR_SPEED = 80
    
    # 連続してコーンを見失う許容回数
    CONE_LOST_MAX_COUNT = 5

    # 目標到達とみなす赤色面積の割合
    GOAL_PERCENTAGE_THRESHOLD = 90

    # 前進・旋回モーター速度
    MOVE_FORWARD_SPEED = 70
    SHORT_MOVE_DURATION = 1 # 短い前進の時間
    LONG_MOVE_DURATION = 2  # 長い前進の時間
    TURN_SPEED_HIGH = 100
    TURN_SPEED_MID = 90
    TURN_SPEED_LOW = 80
    
    # BNO055のキャリブレーション閾値
    BNO_CALIB_GYRO_THRESHOLD = 3
    BNO_CALIB_MAG_THRESHOLD = 3

    # カメラ設定
    CAMERA_WIDTH = 320
    CAMERA_HEIGHT = 240
    CAMERA_FRAMERATE = 30 # このスクリプトでは未使用だが、設定の明確化のため

    def __init__(self):
        """
        RedConeNavigatorのコンストラクタです。
        カメラ、モータードライバー、BNO055センサーを初期化します。
        """
        # モーターの初期化
        self.driver = MotorDriver(
            PWMA=12, AIN1=23, AIN2=18,
            PWMB=19, BIN1=16, BIN2=26,
            STBY=21
        )

        # BNO055の初期化
        self.bno = BNO055()
        if not self.bno.begin():
            print("🔴 BNO055の初期化に失敗しました。プログラムを終了します。")
            self.cleanup()
            exit(1)
        time.sleep(1)
        self.bno.setMode(BNO055.OPERATION_MODE_NDOF)
        time.sleep(1)
        self.bno.setExternalCrystalUse(True)
        print("✅ BNO055センサー初期化完了。")

        # カメラ初期化と設定
        self.picam2 = Picamera2()
        config = self.picam2.create_still_configuration(
            main={"size": (self.CAMERA_WIDTH, self.CAMERA_HEIGHT)}
        )
        self.picam2.configure(config)
        self.picam2.start()
        time.sleep(1)
        print(f"✅ カメラ初期化完了。解像度: {self.CAMERA_WIDTH}x{self.CAMERA_HEIGHT}")

        # BNO055キャリブレーション待機
        self._wait_for_bno055_calibration()

        self.cone_lost_counter = self.CONE_LOST_MAX_COUNT

    def _wait_for_bno055_calibration(self):
        """BNO055センサーのキャリブレーションを待機します。"""
        print("⚙️ BNO055のキャリブレーション待機中...")
        calibration_start_time = time.time()
        while True:
            sys_cal, gyro_cal, accel_cal, mag_cal = self.bno.getCalibration()
            print(f"Calib → Sys:{sys_cal}, Gyro:{gyro_cal}, Acc:{accel_cal}, Mag:{mag_cal}", end='\r')
            if gyro_cal >= self.BNO_CALIB_GYRO_THRESHOLD and mag_cal >= self.BNO_CALIB_MAG_THRESHOLD:
                print("\n✅ BNO055キャリブレーション完了！")
                break
            time.sleep(0.5)
        print(f"キャリブレーションにかかった時間: {time.time() - calibration_start_time:.1f}秒\n")

    def _preprocess_frame(self, frame):
        """フレームの共通前処理（回転、ぼかし、色空間変換）"""
        frame = cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)
        frame = cv2.GaussianBlur(frame, (5, 5), 0)
        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        return frame

    def get_red_percentage(self, frame):
        """画像中の赤色ピクセル割合を計算します。"""
        processed_frame = self._preprocess_frame(frame)
        hsv = cv2.cvtColor(processed_frame, cv2.COLOR_BGR2HSV)
        
        mask1 = cv2.inRange(hsv, self.LOWER_RED1, self.UPPER_RED1)
        mask2 = cv2.inRange(hsv, self.LOWER_RED2, self.UPPER_RED2)
        mask = cv2.bitwise_or(mask1, mask2)
        
        red_area = np.count_nonzero(mask)
        total_area = processed_frame.shape[0] * processed_frame.shape[1]
        percentage = (red_area / total_area) * 100
        print(f"検知割合は {percentage:.2f}% です")
        return percentage

    def get_red_block_by_density(self, frame):
        """
        画像を5分割し、最も赤色ピクセル密度の高いブロックの番号 (1〜5) を返します。
        赤色があまりにも少ない場合はNoneを返します。
        """
        processed_frame = self._preprocess_frame(frame)
        hsv = cv2.cvtColor(processed_frame, cv2.COLOR_BGR2HSV)
        
        mask1 = cv2.inRange(hsv, self.LOWER_RED1, self.UPPER_RED1)
        mask2 = cv2.inRange(hsv, self.LOWER_RED2, self.UPPER_RED2)
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

        for i, r in enumerate(red_ratios):
            print(f"[DEBUG] ブロック{i+1}の赤密度: {r:.2%}")

        max_ratio = max(red_ratios)
        # 閾値を調整して、ノイズをフィルタリング
        if max_ratio < 0.05: # 5%未満の場合は赤色がないとみなす
            print("❌ 赤色が検出されません（全ブロックで密度低）")
            return None
        else:
            block_number = red_ratios.index(max_ratio) + 1
            print(f"一番密度の高いブロックは {block_number} です")
            return block_number

    def search_for_cone(self):
        """
        赤色コーンを探索するロジックを実行します。
        コーンが見つかるまで前進と旋回を繰り返します。
        """
        print("🔄 赤コーンを探索中...")
        search_loop_limit = 10 # 探索ループの試行回数 (例: 10回前進+旋回を試す)

        for _ in range(search_loop_limit):
            # 少し前進して位置を変える
            print("探索のため少し前進します。")
            following.follow_forward(self.driver, self.bno, self.MOVE_FORWARD_SPEED, self.SHORT_MOVE_DURATION)
            self.driver.motor_stop_brake()
            time.sleep(0.5) # 停止して安定を待つ

            before_heading = self.bno.getVector(BNO055.VECTOR_EULER)[0]
            if before_heading is None:
                print("⚠️ 探索開始時の方位が取得できませんでした。")
                before_heading = 0 # フォールバック

            # 全方位を探索 (左回頭)
            print("この場所で全方位コーン探索を行います。")
            rotation_time_per_step = 0.2 # 1回の旋回時間
            max_rotation_steps = int(360 / (self.TURN_SPEED_MID * rotation_time_per_step * 0.5)) # 約360度回るためのステップ数 (概算)

            for step in range(max_rotation_steps):
                frame = self.picam2.capture_array()
                percentage = self.get_red_percentage(frame)
                
                if percentage > 15: # 探索中に十分な赤色を見つけたら終了
                    print("✅ 赤コーンの探索に成功しました。")
                    return True # コーンが見つかった
                
                print(f"視野角内にコーンを検知できませんでした。左回頭を行います (ステップ {step+1}/{max_rotation_steps})")
                self.driver.petit_left(0, self.TURN_SPEED_MID)
                self.driver.motor_stop_brake()
                time.sleep(rotation_time_per_step) # 短く回頭
                
                # BNOの方位をチェックして、ある程度回頭したかを確認するロジックも追加可能
                # after_heading = self.bno.getVector(BNO055.VECTOR_EULER)[0]
                # delta_heading = abs(after_heading - before_heading) # 簡単なデルタ計算

            print("❌ 現在の探索エリアではコーンを検知できませんでした。")
            # この探索エリアでコーンが見つからなかった場合、次の`search_loop_limit`試行へ
        
        print("⛔ 複数回の探索を試みましたが、コーンを検知できませんでした。")
        return False # コーンが見つからなかった

    def navigate_to_cone(self):
        """
        赤色コーンを見つけて追従し、最終的にコーンに到達するまでのメインループです。
        """
        print("🚀 ゴール誘導を開始します。")
        try:
            while True:
                frame = self.picam2.capture_array()
                time.sleep(0.1) # カメラキャプチャ後の短い待機

                percentage = self.get_red_percentage(frame)
                number = self.get_red_block_by_density(frame)
                
                print(f"現在の状態: 赤割合: {percentage:.2f}% | 画面場所: {number}")

                # 1. ゴール判定
                if percentage >= self.GOAL_PERCENTAGE_THRESHOLD:
                    print(f"🎉 ゴール判定！赤色面積が {self.GOAL_PERCENTAGE_THRESHOLD}% を超えました。")
                    break # ループ終了

                # 2. コーンの位置に基づく動作
                if number is None:
                    # コーンが見つからない場合、探索モードに移行
                    self.cone_lost_counter -= 1
                    print(f"⚠️ 赤コーンを見失いました。残りリトライ回数: {self.cone_lost_counter}")
                    if self.cone_lost_counter <= 0:
                        print("🚫 コーンを完全に喪失しました。探索モードに移行します。")
                        found_cone_in_search = self.search_for_cone()
                        if not found_cone_in_search:
                            print("❌ 探索でもコーンを見つけられませんでした。プログラムを終了します。")
                            break # 見つからなければ終了
                        else:
                            self.cone_lost_counter = self.CONE_LOST_MAX_COUNT # コーンが見つかったらリセット
                    else:
                        # 短い時間、その場で回頭して再探索
                        self.driver.petit_left(0, self.TURN_SPEED_LOW)
                        self.driver.petit_left(self.TURN_SPEED_LOW, 0)
                        self.driver.motor_stop_brake()
                        time.sleep(0.5) # 停止して再検出を待つ
                        continue # 再検出のためループの最初に戻る

                elif number == 3: # 中央にコーンがある場合
                    self.cone_lost_counter = self.CONE_LOST_MAX_COUNT # カウンターリセット
                    if percentage > 40:
                        print("✅ 中央にコーン、接近中（大）。短い前進。")
                        self.driver.petit_petit(1) # 短い前進 (MotorDriverに依存)
                        time.sleep(0.5)
                    elif percentage > 20:
                        print("✅ 中央にコーン、接近中（中）。中間の前進。")
                        self.driver.petit_petit(3) # 中間の前進 (MotorDriverに依存)
                        time.sleep(0.5)
                    elif percentage > 10:
                        print("✅ 中央にコーン、接近中（小）。長い前進。")
                        self.driver.petit_petit(5) # 長い前進 (MotorDriverに依存)
                        time.sleep(0.5)
                    else:
                        print("➡️ 距離が遠いため、目標追従で前進します。")
                        following.follow_forward(self.driver, self.bno, self.MOVE_FORWARD_SPEED, self.LONG_MOVE_DURATION)
                        self.driver.motor_stop_brake()
                        time.sleep(0.5)

                elif number == 1: # 左端にコーンがある場合
                    print("⬅️ コーンが左端にあります。右に旋回。")
                    self.driver.petit_right(0, self.TURN_SPEED_HIGH)
                    self.driver.petit_right(self.TURN_SPEED_HIGH, 0)
                    self.driver.motor_stop_brake()
                    time.sleep(0.5)
                    self.cone_lost_counter = self.CONE_LOST_MAX_COUNT # カウンターリセット

                elif number == 2: # 左中央にコーンがある場合
                    print("↙️ コーンが左中央にあります。右に旋回。")
                    self.driver.petit_right(0, self.TURN_SPEED_MID)
                    self.driver.petit_right(self.TURN_SPEED_MID, 0)
                    self.driver.motor_stop_brake()
                    time.sleep(0.5)
                    self.cone_lost_counter = self.CONE_LOST_MAX_COUNT # カウンターリセット
                    if percentage < 50: # まだ遠ければ少し前進して調整
                        print("正面にとらえることができませんでしたが、検知割合が低いため、接近します")
                        following.follow_forward(self.driver, self.bno, self.MOVE_FORWARD_SPEED, self.SHORT_MOVE_DURATION)
                        self.driver.motor_stop_brake()
                        time.sleep(0.5)

                elif number == 4: # 右中央にコーンがある場合
                    print("↘️ コーンが右中央にあります。左に旋回。")
                    self.driver.petit_left(0, self.TURN_SPEED_MID)
                    self.driver.petit_left(self.TURN_SPEED_MID, 0)
                    self.driver.motor_stop_brake()
                    time.sleep(0.5)
                    self.cone_lost_counter = self.CONE_LOST_MAX_COUNT # カウンターリセット
                    if percentage < 50: # まだ遠ければ少し前進して調整
                        print("正面にとらえることができませんでしたが、検知割合が低いため、接近します")
                        following.follow_forward(self.driver, self.bno, self.MOVE_FORWARD_SPEED, self.SHORT_MOVE_DURATION)
                        self.driver.motor_stop_brake()
                        time.sleep(0.5)

                elif number == 5: # 右端にコーンがある場合
                    print("➡️ コーンが右端にあります。左に旋回。")
                    self.driver.petit_left(0, self.TURN_SPEED_HIGH)
                    self.driver.petit_left(self.TURN_SPEED_HIGH, 0)
                    self.driver.motor_stop_brake()
                    time.sleep(0.5)
                    self.cone_lost_counter = self.CONE_LOST_MAX_COUNT # カウンターリセット
                
                time.sleep(0.1) # 各ループの最後に短い待機

        except KeyboardInterrupt:
            print("\n🚨 ユーザーによって中断されました。")
        except Exception as e:
            print(f"\n❌ エラーが発生しました: {e}")
        finally:
            self.cleanup()

    def cleanup(self):
        """
        プログラム終了時に使用したハードウェアリソースを解放します。
        """
        print("\n--- クリーンアップ処理を実行します ---")
        if self.picam2:
            self.picam2.close()
            print("カメラを閉じました。")
        if self.driver:
            self.driver.cleanup()
            print("モータードライバーをクリーンアップしました。")
        # BNO055の明示的なクローズは通常不要
        # pigpioのリソースはMotorDriver.cleanup()内で処理される場合もある
        GPIO.cleanup()
        print("✅ GPIOクリーンアップが終了しました。プログラムを終了します。")

# --- メイン実行ブロック ---
if __name__ == "__main__":
    # 赤コーンナビゲーターのインスタンスを作成
    navigator = RedConeNavigator()

    # ナビゲーションを開始
    navigator.navigate_to_cone()
