import cv2
import numpy as np
import time
from picamera2 import Picamera2
from motor import MotorDriver
import camera
import following
from BNO055 import BNO055
import math
from collections import deque
import pigpio

class GDA:
    def __init__(self, bno: BNO055, counter_max: int=50):
        self.driver = MotorDriver(
            PWMA=12, AIN1=23, AIN2=18,
            PWMB=19, BIN1=16, BIN2=26,
            STBY=21
        )
        self.bno = bno
        self.picam2 = Picamera2()
        config = self.picam2.create_still_configuration(main={"size": (320, 480)})
        self.picam2.configure(config)
        self.picam2.start()
        time.sleep(1)
        self.counter_max = counter_max
        self.lower_red1 = np.array([0, 100, 100])
        self.upper_red1 = np.array([10, 255, 255])
        self.lower_red2 = np.array([160, 100, 100])
        self.upper_red2 = np.array([180, 255, 255])
        self.pi = pigpio.pi()
        if not self.pi.connected:
            raise RuntimeError("pigpioデーモンに接続できません。`sudo pigpiod`を実行して確認してください。")
        
    def get_percentage(self, frame):
        frame = cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)
        frame = cv2.GaussianBlur(frame, (5, 5), 0)
        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask1 = cv2.inRange(hsv, self.lower_red1, self.upper_red1)
        mask2 = cv2.inRange(hsv, self.lower_red2, self.upper_red2)
        mask = cv2.bitwise_or(mask1, mask2)
        red_area = np.count_nonzero(mask)
        total_area = frame.shape[0] * frame.shape[1]
        percentage = (red_area / total_area) * 100
        print(f"検知割合は{percentage}%です")
        return percentage
    
    def run(self):
        left_a = 90
        right_a = 80
        counter = self.counter_max
        percentage = 0
        try:
            heading_list = deque(maxlen=5)
            print("ゴール誘導を開始します")
            
            # --- 探索モードの関数化 ---
            def perform_360_degree_search():
                nonlocal percentage # percentage変数を変更できるようにnonlocal宣言
                print("赤コーンが近くにありません。360度回転して最も良い方向を探索します。")
                
                best_percentage = 0.0
                best_heading = None
                initial_heading = self.bno.get_heading()
                
                # 360度回転しながら最も赤色の割合が高い方向を探す
                # 確実に360度回転させるために、少し多めに回転します (例: 380度)。
                target_rotation_angle = 380 # 目標回転角度 (deg)
                rotated_angle = 0 # 初期の回転量
                
                print("360度探索を開始...")
                # 最初のキャプチャと角度計算
                frame = self.picam2.capture_array()
                current_percentage = self.get_percentage(frame)
                current_heading = self.bno.get_heading()
                if current_percentage > best_percentage:
                    best_percentage = current_percentage
                    best_heading = current_heading
                    
                # 360度回転を制御するループ
                # ここでは、BNO055の絶対方位値が360度周期であることを利用して、開始方位を基準に約360度回るようにします。
                # 例: 最初の方向から360度進むまで、またはより堅牢な方法で
                # 簡単な実装として、一定回数少しずつ回転して計測します。
                # 厳密な360度回転が必要な場合は、IMUの角度変化を精密に追跡する必要があります。
                
                # ここをより堅牢な360度回転ロジックに置き換えることができます
                # 例: 初期方位から常に現在の角度を計算し、360度に達するまで回る
                # ただし、BNO055の heading は0-360なので、単純な引き算だとバグりやすい。
                # 以下は、360度回転を試みる簡易的なループです
                
                # 回転量の追跡を正確にするための初期設定
                start_heading_for_rotation = self.bno.get_heading()
                total_rotation_degrees = 0
                
                # 目標は360度以上回転すること。ここでは例えば10度ずつ回転し、36回繰り返す
                # より滑らかな回転と計測のために、小さいステップで回します
                num_steps = 36 # 10度ずつ回ると仮定して36ステップ
                angle_per_step = 10 # 1ステップあたりの目標回転角度
                
                for _ in range(num_steps + 4): # 余分に4ステップ回して確実に360度以上
                    self.driver.petit_right(0, 50) # ゆっくり右に回転
                    time.sleep(0.2) # 各回転ステップの待ち時間
                    self.driver.motor_stop_brake()

                    frame = self.picam2.capture_array()
                    current_percentage = self.get_percentage(frame)
                    current_heading = self.bno.get_heading()
                    
                    if current_percentage > best_percentage:
                        best_percentage = current_percentage
                        best_heading = current_heading
                        print(f"[探索中] 新しい最高の赤割合: {best_percentage:.2f}% @ 方位: {best_heading:.2f}°")
                    
                    # 実際の回転量の追跡（オプション、デバッグ用）
                    # delta_angle_step = self.bno.get_angle_change(start_heading_for_rotation, current_heading) # このようなメソッドがあれば
                    # total_rotation_degrees += delta_angle_step
                    # start_heading_for_rotation = current_heading # 次のステップのために更新
                    
                print(f"360度探索完了。最高赤割合: {best_percentage:.2f}% @ 方位: {best_heading:.2f}°")

                if best_heading is not None and best_percentage > 5: # 5%は最低限の検出閾値
                    print(f"最適な方向 ({best_heading:.2f}°)に調整します。")
                    # 最適な方位にロボットの向きを調整
                    # BNO055にturn_to_headingメソッドがあると仮定
                    self.bno.turn_to_heading(self.driver, best_heading, 70) 
                    self.driver.motor_stop_brake()
                    time.sleep(1.0)
                    
                    print("赤コーンの割合が15%になるまで前進します。")
                    # 赤色割合が15%になるまで前進
                    while True:
                        frame = self.picam2.capture_array()
                        current_percentage = self.get_percentage(frame)
                        print(f"前進中... 現在の赤割合: {current_percentage:.2f}%")
                        
                        if current_percentage >= 15:
                            print("赤割合が15%に達しました。前進を停止し、追従モードに戻ります。")
                            self.driver.motor_stop_brake()
                            time.sleep(0.5)
                            return True # 探索成功
                        
                        # --- ここに「見失った場合の停止＆再探索」ロジックを追加 ---
                        if current_percentage < 5: # 例: 5%未満になったら見失ったと判断
                            print("前進中に赤コーンを見失いました。停止し、再探索します。")
                            self.driver.motor_stop_brake()
                            time.sleep(0.5)
                            return False # 探索失敗（再探索が必要）
                        # --- 追加ロジックここまで ---

                        # 常に前進し続ける（速度は調整可能）
                        self.driver.petit_petit(2) # 短い時間前進を繰り返す
                        self.driver.motor_stop_brake()
                        time.sleep(0.2) # 各ステップの間に少し間隔を空ける
                else:
                    print("360度探索でもコーンを明確に検知できませんでした。")
                    return False # 探索失敗
            # --- 探索モードの関数化ここまで ---

            while True:
                if counter <= 0:
                    # 探索関数を呼び出し、成功した場合はそのまま続行
                    # 失敗した場合は、再度このifブロックに入り、探索が繰り返される
                    search_successful = perform_360_degree_search()
                    if not search_successful:
                        # 探索が成功しなかった場合（見つけられなかった、または見失った）
                        # カウンターを再度リセットして、次のループで再試行させる
                        counter = self.counter_max
                        continue # 現在のメインループのイテレーションをスキップして次へ
                    else:
                        # 探索と初期前進が成功した場合、カウンターをリセットし、
                        # 通常の追従ロジックに進む
                        counter = self.counter_max


                # --- 通常の追従ロジック ---
                frame = self.picam2.capture_array()
                time.sleep(0.2)
                percentage = self.get_percentage(frame)
                time.sleep(0.2)
                print(f"赤割合: {percentage:2f}%です ")

                if percentage >= 90:
                    print("percentageでのゴール判定")
                    break
                elif percentage > 15:
                    print("赤コーンを検知しました。接近します。")
                    if percentage > 40:
                        print("非常に近いので、ゆっくり前進します (petit_petit 2回)")
                        self.driver.petit_petit(2)
                    elif percentage > 20:
                        print("近いので、少し前進します (petit_petit 3回)")
                        self.driver.petit_petit(3)
                    else: # percentage > 15 かつ <= 20 の場合
                        print("遠いので、前進します (follow_forward)")
                        following.follow_forward(self.driver, self.bno, 70, 1)
                    counter = self.counter_max
                
                counter = counter - 1
                c_heading = self.bno.get_heading()
                heading_list.append(c_heading)
                if len(heading_list) == 5:
                    print("スタック判定を行います")
                    a = abs((heading_list[4] - heading_list[3] + 180) % 360 - 180)
                    b = abs((heading_list[3] - heading_list[2] + 180) % 360 - 180)
                    c = abs((heading_list[2] - heading_list[1] + 180) % 360 - 180)
                    if a < 1.5 and b < 1.5 and c < 1.5:
                        print("スタック判定です")
                        print("スタック離脱を行います")
                        self.driver.changing_right(0, 90)
                        time.sleep(3)
                        self.driver.changing_right(90, 0)
                        time.sleep(0.5)
                        self.driver.changing_left(0, 90)
                        time.sleep(3)
                        self.driver.changing_left(90, 0)
                        time.sleep(0.5)
                        self.driver.changing_forward(0, 90)
                        time.sleep(0.5)
                        self.driver.changing_forward(90, 0)
                        time.sleep(0.5)
                        print("スタック離脱を終了します")
                        heading_list.clear()
        finally:
            self.picam2.close()
            # self.pi.bb_serial_read_close(17) # pigpioのシリアル通信を使用していない場合、この行は不要かもしれません。
            print("カメラを閉じました。")
            print("ゴール判定")
            self.driver.cleanup()
            print("GPIOクリーンアップが終了しました。プログラムを終了します")
