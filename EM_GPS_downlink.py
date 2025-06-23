import serial
import time
import pigpio
import RPi.GPIO as GPIO
from motor import MotorDriver 

TX_PIN = 27
RX_PIN = 17
BAUD = 9600

pi = pigpio.pi()
if not pi.connected:
    print("pigpio デーモンに接続できません。")
    exit(1)

# インスタンス生成：GPIOピン番号を正しく指定
driver = MotorDriver(
    PWMA=12, AIN1=23, AIN2=18,   # 左モーター用（モータA）
    PWMB=19, BIN1=16, BIN2=26,   # 右モーター用（モータB）
    STBY=21                      # STBYピン
)

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

im920 = serial.Serial('/dev/serial0', 19200, timeout=1)

driver.changing_forward(0, 100)
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
                                #print("緯度と経度 (10進数):", [lat, lon])
                                data = f'{lat, lon}'
                                msg = f'TXDA 0003,{data}\r'
                                im920.write(msg.encode())
                                print(f"送信: {msg.strip()}")
                                time.sleep(2)
            except Exception as e:
                print("デコードエラー:", e)
        time.sleep(0.1)

except KeyboardInterrupt:
    print("\nユーザー割り込みで終了します。")

finally:
    pi.bb_serial_read_close(RX_PIN)
    pi.stop()
    driver.changing.forward(100, 0)
    print("終了しました。")
