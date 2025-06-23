import math
import time
import pigpio
import RPi.GPIO as GPIO
from motor import MotorDriver 
from BNO055 import BNO055

# インスタンス生成：GPIOピン番号を正しく指定
driver = MotorDriver(
    PWMA=12, AIN1=23, AIN2=18,   # 左モーター用（モータA）
    PWMB=19, BIN1=16, BIN2=26,   # 右モーター用（モータB）
    STBY=21                      # STBYピン
)

# === 目標地点設定（[緯度, 経度]）===
GOAL_LOCATION = [35.6586, 139.7454]  # 例：東京タワー

# === GPSデータ取得（仮の実装）===
TX_PIN = 17
RX_PIN = 27
BAUD = 9600

pi = pigpio.pi()
if not pi.connected:
    print("pigpio デーモンに接続できません。")
    exit(1)

err = pi.bb_serial_read_open(RX_PIN, BAUD, 8)
if err != 0:
    print(f"ソフトUART RX の設定に失敗：GPIO={RX_PIN}, {BAUD}bps")
    pi.stop()
    exit(1)

print(f"▶ ソフトUART RX を開始：GPIO={RX_PIN}, {BAUD}bps")

def convert_to_decimal(coord, direction):
    # 度分（ddmm.mmmm）形式を10進数に変換
    degrees = int(coord[:2]) if direction in ['N', 'S'] else int(coord[:3])
    minutes = float(coord[2:]) if direction in ['N', 'S'] else float(coord[3:])
    decimal = degrees + minutes / 60
    if direction in ['S', 'W']:
        decimal *= -1
    return decimal
  
# === 初期化変数 ===
last_lng = None
last_update_time = time.time()
in_stack = False
after_escape = False
stack_check_start_time = None

try:
    while True:
        (count, data) = pi.bb_serial_read(RX_PIN)
        if count and data:
            try:
                text = data.decode("ascii", errors="ignore")
                if "$GNRMC" in text:
                    lines = text.split("\n")
                    for line in lines:
                        if "$GNRMC" in line:
                            parts = line.strip().split(",")
                            if len(parts) > 6 and parts[2] == "A":
                                lat = convert_to_decimal(parts[3], parts[4])
                                lon = convert_to_decimal(parts[5], parts[6])
                                new_lng = [lat, lon]

                                if new_lng != last_lng:
                                    print("現在位置:", new_lng)
                                    last_lng = new_lng
                                    last_update_time = time.time()

                                    # ▼ 回避後に動いた → スタック離脱
                                    if after_escape:
                                        print("✅ スタックから離脱しました。通常走行へ復帰")
                                        after_escape = False
                                        in_stack = False
                                        stack_check_start_time = time.time()

            except Exception as e:
                print("デコードエラー:", e)

        now = time.time()

        # === 通常時スタック検出 ===
        if not in_stack and (now - last_update_time > 20):
            print("⚠ スタック検知：20秒間位置変化なし")

            # ▼ 回避動作：バック → 回転 → 再バック
            print("▶ 回避動作：バック→回転→再バック")
            driver.changing_retreat(0, 90)
            time.sleep(4)
            driver.changing_retreat(90, 0)
            driver.changing_right(0, 90)
            time.sleep(3)
            driver.changing_right(90, 0)
            driver.stop()

            in_stack = True
            after_escape = True
            stack_check_start_time = time.time()
            last_update_time = now  # タイマー更新

        # === 回避後、再スタック検出（動いていない） ===
        if after_escape and (now - stack_check_start_time > 20):
            print("⚠ 回避後も動きがないため再スタックと判定")
            # 再度回避動作
            driver.changing_retreat(25, 90)
            time.sleep(4)
            driver.changing_right(90, 90)
            time.sleep(3)
            driver.changing_retreat(90, 25)
            time.sleep(4)
            driver.stop()

            stack_check_start_time = time.time()  # 再監視開始

        time.sleep(0.1)

except KeyboardInterrupt:
    print("終了処理中...")

finally:
    pi.bb_serial_read_close(RX_PIN)
    pi.stop()
