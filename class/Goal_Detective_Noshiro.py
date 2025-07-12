import cv2
import numpy as np
import time
from picamera2 import Picamera2
from motor import MotorDriver
import following # Assuming following.py contains follow_forward
from BNO055 import BNO055
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
    
    # BNO055のキャリブレーション閾値 (このクラスでは参照のみ)
    BNO_CALIB_GYRO_THRESHOLD = 3
    BNO_CALIB_MAG_THRESHOLD = 3
    BNO_CALIB_ACCEL_THRESHOLD = 3

    # カメラ設定 (このクラスでは参照のみ)
    CAMERA_WIDTH = 320
    CAMERA_HEIGHT = 240

    def __init__(self, driver_instance, bno_instance, picam2_instance,
                 cone_lost_max_count=None, goal_percentage_threshold=None):
        """
        RedConeNavigatorのコンストラクタです。
        """
        self.driver = driver_instance # 外部から渡されたインスタンスを使用
        self.bno = bno_instance       # 外部から渡されたインスタンスを使用
        self.picam2 = picam2_instance # 外部から渡されたインスタンスを使用

        # 設定値 (デフォルト値または引数で上書き)
        self.cone_lost_counter = cone_lost_max_count if cone_lost_max_count is not None else self.CONE_LOST_MAX_COUNT
        self.cone_lost_max_count = cone_lost_max_count if cone_lost_max_count is not None else self.CONE_LOST_MAX_COUNT
        self.goal_percentage_threshold = goal_percentage_threshold if goal_percentage_threshold is not None else self.GOAL_PERCENTAGE_THRESHOLD
        
        print("✅ RedConeNavigator: インスタンス作成完了。")

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
        print(f"RedConeNavigator: 検知割合は {percentage:.2f}% です")
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
            print(f"RedConeNavigator: [DEBUG] ブロック{i+1}の赤密度: {r:.2%}")

        max_ratio = max(red_ratios)
        # 閾値を調整して、ノイズをフィルタリング
        if max_ratio < 0.05: # 5%未満の場合は赤色がないとみなす
            print("RedConeNavigator: ❌ 赤色が検出されません（全ブロックで密度低）")
            return None
        else:
            block_number = red_ratios.index(max_ratio) + 1
            print(f"RedConeNavigator: 一番密度の高いブロックは {block_number} です")
            return block_number

    def search_for_cone(self):
        """
        赤色コーンを探索するロジックを実行します。
        コーンが見つかるまで前進と旋回を繰り返します。
        """
        print("RedConeNavigator: 🔄 赤コーンを探索中...")
        search_loop_limit = 10 # 探索ループの試行回数 (例: 10回前進+旋回を試す)

        for _ in range(search_loop_limit):
            # 少し前進して位置を変える
            print("RedConeNavigator: 探索のため少し前進します。")
            following.follow_forward(self.driver, self.bno, self.MOVE_FORWARD_SPEED, self.SHORT_MOVE_DURATION)
            self.driver.motor_stop_brake()
            time.sleep(0.5) # 停止して安定を待つ

            before_heading = self.bno.getVector(BNO055.VECTOR_EULER)[0]
            if before_heading is None:
                print("RedConeNavigator: ⚠️ 探索開始時の方位が取得できませんでした。")
                before_heading = 0 # フォールバック

            # 全方位を探索 (左回頭)
            print("RedConeNavigator: この場所で全方位コーン探索を行います。")
            rotation_time_per_step = 0.2 # 1回の旋回時間
            
            # 最大旋回ステップ数を調整
            # 経験的に、1回の petit_left(0, 70) で約5-10度回転すると仮定し、
            # 360度をカバーするために必要なステップ数を計算
            approx_angle_per_step = 7 # 1ステップあたりの概算角度 (要調整)
            max_rotation_steps = int(360 / approx_angle_per_step) + 5 # 余裕を持たせる

            for step in range(max_rotation_steps):
                frame = self.picam2.capture_array()
                percentage = self.get_red_percentage(frame)
                
                if percentage > 15: # 探索中に十分な赤色を見つけたら終了
                    print("RedConeNavigator: ✅ 赤コーンの探索に成功しました。")
                    return True # コーンが見つかった
                
                print(f"RedConeNavigator: 視野角内にコーンを検知できませんでした。左回頭を行います (ステップ {step+1}/{max_rotation_steps})")
                self.driver.petit_left(0, self.TURN_SPEED_MID)
                self.driver.petit_left(self.TURN_SPEED_MID, 0) # 2引数バージョン
                self.driver.motor_stop_brake()
                time.sleep(rotation_time_per_step) # 短く回頭
                time.sleep(0.2) # 停止してセンサー安定化
                
                # ここでBNOの方位をチェックして、ある程度回頭したかを確認するロジックも追加可能

            print("RedConeNavigator: ❌ 現在の探索エリアではコーンを検知できませんでした。")
        
        print("RedConeNavigator: ⛔ 複数回の探索を試みましたが、コーンを検知できませんでした。")
        return False # コーンが見つからなかった

    def navigate_to_cone(self):
        """
        赤色コーンを見つけて追従し、最終的にコーンに到達するまでのメインループです。
        """
        print("RedConeNavigator: 🚀 ゴール誘導を開始します。")
        try:
            while True:
                frame = self.picam2.capture_array()
                time.sleep(0.1) # カメラキャプチャ後の短い待機

                percentage = self.get_red_percentage(frame)
                block_number = self.get_red_block_by_density(frame) # block_number に変更
                
                print(f"RedConeNavigator: 現在の状態: 赤割合: {percentage:.2f}% | 画面場所:{block_number}")

                # 1. ゴール判定
                if percentage >= self.goal_percentage_threshold:
                    print(f"\n🎉 RedConeNavigator: ゴール判定！赤色面積が {self.goal_percentage_threshold}% を超えました。")
                    self.driver.motor_stop_brake() # 最終停止
                    break # ループ終了

                # 2. コーンの位置に基づく動作
                if block_number is None:
                    # コーンが見つからない場合、探索モードに移行
                    self.cone_lost_counter -= 1
                    print(f"RedConeNavigator: ⚠️ 赤コーンを見失いました。残りリトライ回数: {self.cone_lost_counter}")
                    if self.cone_lost_counter <= 0:
                        print("RedConeNavigator: 🚫 コーンを完全に喪失しました。探索モードに移行します。")
                        found_cone_in_search = self.search_for_cone()
                        if not found_cone_in_search:
                            print("RedConeNavigator: ❌ 探索でもコーンを見つけられませんでした。プログラムを終了します。")
                            self.driver.motor_stop_brake()
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

                elif block_number == 3: # 中央にコーンがある場合
                    self.cone_lost_counter = self.CONE_LOST_MAX_COUNT # カウンターリセット
                    if percentage > 40:
                        print("RedConeNavigator: ✅ 中央にコーン、接近中（大）。短い前進。")
                        self.driver.petit_petit(1) # 短い前進 (MotorDriverに依存)
                        time.sleep(0.5)
                    elif percentage > 20:
                        print("RedConeNavigator: ✅ 中央にコーン、接近中（中）。中間の前進。")
                        self.driver.petit_petit(3) # 中間の前進 (MotorDriverに依存)
                        time.sleep(0.5)
                    elif percentage > 10:
                        print("RedConeNavigator: ✅ 中央にコーン、接近中（小）。長い前進。")
                        self.driver.petit_petit(5) # 長い前進 (MotorDriverに依存)
                        time.sleep(0.5)
                    else:
                        print("RedConeNavigator: ➡️ 距離が遠いため、目標追従で前進します。")
                        following.follow_forward(self.driver, self.bno, self.MOVE_FORWARD_SPEED, self.LONG_MOVE_DURATION)
                        self.driver.motor_stop_brake()
                        time.sleep(0.5)

                elif block_number == 1: # 左端にコーンがある場合
                    print("RedConeNavigator: ⬅️ コーンが左端にあります。右に旋回。")
                    self.driver.petit_right(0, self.TURN_SPEED_HIGH)
                    self.driver.petit_right(self.TURN_SPEED_HIGH, 0)
                    self.driver.motor_stop_brake()
                    time.sleep(0.5)
                    self.cone_lost_counter = self.CONE_LOST_MAX_COUNT # カウンターリセット

                elif block_number == 2: # 左中央にコーンがある場合
                    print("RedConeNavigator: ↙️ コーンが左中央にあります。右に旋回。")
                    self.driver.petit_right(0, self.TURN_SPEED_MID)
                    self.driver.petit_right(self.TURN_SPEED_MID, 0)
                    self.driver.motor_stop_brake()
                    time.sleep(0.5)
                    self.cone_lost_counter = self.CONE_LOST_MAX_COUNT # カウンターリセット
                    if percentage < 50: # まだ遠ければ少し前進して調整
                        print("RedConeNavigator: 正面にとらえることができませんでしたが、検知割合が低いため、接近します")
                        following.follow_forward(self.driver, self.bno, self.MOVE_FORWARD_SPEED, self.SHORT_MOVE_DURATION)
                        self.driver.motor_stop_brake()
                        time.sleep(0.5)

                elif block_number == 4: # 右中央にコーンがある場合
                    print("RedConeNavigator: ↘️ コーンが右中央にあります。左に旋回。")
                    self.driver.petit_left(0, self.TURN_SPEED_MID)
                    self.driver.petit_left(self.TURN_SPEED_MID, 0)
                    self.driver.motor_stop_brake()
                    time.sleep(0.5)
                    self.cone_lost_counter = self.CONE_LOST_MAX_COUNT # カウンターリセット
                    if percentage < 50: # まだ遠ければ少し前進して調整
                        print("RedConeNavigator: 正面にとらえることができませんでしたが、検知割合が低いため、接近します")
                        following.follow_forward(self.driver, self.bno, self.MOVE_FORWARD_SPEED, self.SHORT_MOVE_DURATION)
                        self.driver.motor_stop_brake()
                        time.sleep(0.5)

                elif block_number == 5: # 右端にコーンがある場合
                    print("RedConeNavigator: ➡️ コーンが右端にあります。左に旋回。")
                    self.driver.petit_left(0, self.TURN_SPEED_HIGH)
                    self.driver.petit_left(self.TURN_SPEED_HIGH, 0)
                    self.driver.motor_stop_brake()
                    time.sleep(0.5)
                    self.cone_lost_counter = self.CONE_LOST_MAX_COUNT # カウンターリセット
                
                time.sleep(0.1) # 各ループの最後に短い待機

        except KeyboardInterrupt:
            print("\n🚨 RedConeNavigator: 手動で停止されました。")
        except Exception as e:
            print(f"\n[FATAL] RedConeNavigator: 予期せぬエラーが発生しました: {e}")
        finally:
            self.driver.motor_stop_brake() # 念のため停止

    def cleanup(self):
        """
        RedConeNavigator独自のクリーンアップ処理（現在はモーター停止のみ。Picamera2は外部で管理）
        """
        if self.driver:
            self.driver.motor_stop_brake()
        print("RedConeNavigator: クリーンアップ完了。")
