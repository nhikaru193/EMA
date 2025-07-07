import math
import time
import serial
import pigpio
import RPi.GPIO as GPIO
from motor import MotorDriver      # ユーザーのMotorDriverクラスを使用
from BNO055 import BNO055
import smbus
import struct
import following

# === 制御パラメータ (チューニング用) ===
GOAL_LOCATION = [35.9186248, 139.9081672]  # 12号館前
GOAL_THRESHOLD_M = 5.0      # ゴールとみなす距離 (メートル)
ANGLE_THRESHOLD_DEG = 15.0  # これ以上の角度誤差があれば回頭する (度)
TURN_SPEED = 45             # 回頭時のモーター速度 (0-100)
MOVE_SPEED = 80             # 前進時の基本速度 (0-100)
MOVE_DURATION_S = 1.5       # 一回の前進時間 (秒)

# === PD制御パラメータ ===
Kp = 0.50   # 比例ゲイン: 誤差に対する反応の強さ
Kd = 0.15   # 微分ゲイン: 揺り戻しを抑制し、動きを滑らかにする

# === モーター初期化 ===
driver = MotorDriver(
    PWMA=12, AIN1=23, AIN2=18,    # 左モーター
    PWMB=19, BIN1=16, BIN2=26,    # 右モーター
    STBY=21
)

# === GPSピン設定 ===
RX_PIN = 17
BAUD = 9600

# === pigpio 初期化 ===
pi = pigpio.pi()
if not pi.connected:
    print("pigpio デーモンに接続できてないよ。sudo pigpiod を実行してください。")
    exit(1)

err = pi.bb_serial_read_open(RX_PIN, BAUD, 8)
if err != 0:
    print(f"ソフトUART RX の設定に失敗：GPIO={RX_PIN}, {BAUD}bps")
    pi.stop()
    exit(1)

print(f"▶ ソフトUART RX を開始：GPIO={RX_PIN}, {BAUD}bps")

# === BNO055 初期化 ===
bno = BNO055()
if not bno.begin():
    print("BNO055の初期化に失敗しました。")
    exit(1)
time.sleep(1)
bno.setExternalCrystalUse(True)
bno.setMode(BNO055.OPERATION_MODE_NDOF)
time.sleep(1)
print("センサー類の初期化完了。")

# === 度分→10進変換関数 ===
def convert_to_decimal(coord, direction):
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

# === BNO055キャリブレーション待機 ===
print("BNO055のキャリブレーション待機中...")
while True:
    sys, gyro, accel, mag = bno.getCalibration()
    print(f"Calib → Sys:{sys}, Gyro:{gyro}, Acc:{accel}, Mag:{mag}", end='\r')
    if gyro == 3 and mag == 3:
        print("\nキャリブレーション完了！ナビゲーションを開始します。")
        break
    time.sleep(0.5)

# === 2点間の方位角の計算 ===
def get_bearing_to_goal(current, goal):
    if current is None or goal is None: return None
    lat1, lon1 = math.radians(current[0]), math.radians(current[1])
    lat2, lon2 = math.radians(goal[0]), math.radians(goal[1])
    delta_lon = lon2 - lon1
    y = math.sin(delta_lon) * math.cos(lat2)
    x = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(delta_lon)
    bearing_rad = math.atan2(y, x)
    return (math.degrees(bearing_rad) + 360) % 360

# === 2点間の距離の計算 (Haversine公式) ===
def get_distance_to_goal(current, goal):
    if current is None or goal is None: return float('inf')
    lat1, lon1 = math.radians(current[0]), math.radians(current[1])
    lat2, lon2 = math.radians(goal[0]), math.radians(goal[1])
    radius = 6378137.0  # 地球の半径 (メートル)
    dlat = lat2 - lat1
    dlon = lon2 - lon1
    a = math.sin(dlat / 2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2)**2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    dist = radius * c
    return dist

# === ナビゲーション制御 ===
def navigate_to_goal():
    try:
        while True:
            # 1. 状態把握
            (count, data) = pi.bb_serial_read(RX_PIN)
            current_location = None
            
            if count and data:
                try:
                    text = data.decode("ascii", errors="ignore")
                    if "$GNRMC" in text:
                        lines = text.split("\n")
                        for line in lines:
                            if line.startswith("$GNRMC"):
                                parts = line.strip().split(",")
                                if len(parts) > 6 and parts[2] == "A":
                                    lat = convert_to_decimal(parts[3], parts[4])
                                    lon = convert_to_decimal(parts[5], parts[6])
                                    current_location = [lat, lon]
                                    break
                except Exception as e:
                    print(f"GPSデコードエラー: {e}")
            
            if not current_location:
                print("[WARN] GPS位置情報を取得できません。リトライします...")
                driver.motor_stop_brake()
                time.sleep(1)
                continue

            heading = bno.getVector(BNO055.VECTOR_EULER)[0]
            if heading is None:
                print("[WARN] BNO055から方位角を取得できません。リトライします...")
                driver.motor_stop_brake()
                time.sleep(1)
                continue

            # 2. 計算
            dist_to_goal = get_distance_to_goal(current_location, GOAL_LOCATION)
            bearing_to_goal = get_bearing_to_goal(current_location, GOAL_LOCATION)
            angle_error = (bearing_to_goal - heading + 360) % 360

            print(f"[INFO] 距離:{dist_to_goal: >6.1f}m | 目標方位:{bearing_to_goal: >5.1f}° | 現在方位:{heading: >5.1f}°")

            # 3. ゴール判定
            if dist_to_goal <= GOAL_THRESHOLD_M:
                print(f"[GOAL] 目標地点に到達しました！ (距離: {dist_to_goal:.2f}m)")
                driver.motor_stop_free()
                break

            # 4. 方向調整フェーズ
            ANGLE_THRESHOLD_DEG = 20.0 # 許容する角度誤差（度）
            if angle_error > ANGLE_THRESHOLD_DEG and angle_error < (360 - ANGLE_THRESHOLD_DEG):
                turn_speed = 40 # 回転速度は固定 (0-100)
                turn_duration = 0.15 + (min(angle_error, 360 - angle_error) / 180.0) * 0.2 #場所によって変える！！！

                if angle_error > 180: # 反時計回り（左）に回る方が近い
                    print(f"[TURN] 左に回頭します ({turn_duration:.2f}秒)")
                    driver.changing_left(0, turn_speed) 
                    driver.changing_left(turn_speed, 0) 
                    time.sleep(turn_duration)
                else: # 時計回り（右）に回る方が近い
                    print(f"[TURN] 右に回頭します ({turn_duration:.2f}秒)")
                    driver.changing_right(0, turn_speed) 
                    driver.changing_right(turn_speed, 0) 
                    time.sleep(turn_duration)
                
                driver.motor_stop_free() # 確実な停止
                time.sleep(0.5) # 回転後の安定待ち
                continue # 方向調整が終わったら、次のループで再度GPSと方位を確認


            # 5. 前進フェーズ (PD制御による直進維持)
            print(f"[MOVE] 方向OK。PD制御で前進します。")
            following.follow_forward(driver, bno, 70, 8)

    except KeyboardInterrupt:
        print("\n[STOP] 手動で停止されました。")
    except Exception as e:
        print(f"\n[FATAL] 予期せぬエラーが発生しました: {e}")
    finally:
        print("クリーンアップ処理を実行します。")
        driver.cleanup()
        pi.bb_serial_read_close(RX_PIN)
        pi.stop()
        print("プログラムを終了しました。")

# === プログラム実行 ===
if __name__ == "__main__":
    navigate_to_goal()
