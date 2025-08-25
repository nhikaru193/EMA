import serial
import RPi.GPIO as GPIO
import time
import pigpio

TX_PIN = 27
RX_PIN = 17
BAUD = 9600
im920 = serial.Serial('/dev/serial0', 19200, timeout=1)
wireless_PIN = 22

pi = pigpio.pi()
if not pi.connected:
    print("pigpio デーモンに接続できません。")
    exit(1)
err = pi.bb_serial_read_open(RX_PIN, BAUD, 8)
if err != 0:
    print(f"ソフトUART RX の設定に失敗：GPIO={RX_PIN}, {BAUD}bps")
    pi.stop()
    exit(1)

def convert_to_decimal(coord, direction):
    """
    度分（ddmm.mmmm）形式を10進数に変換します。
    """
    degrees = int(coord[:2]) if direction in ['N', 'S'] else int(coord[:3])
    minutes = float(coord[2:]) if direction in ['N', 'S'] else float(coord[3:])
    decimal = degrees + minutes / 60
    if direction in ['S', 'W']:
        decimal *= -1
    return decimal


print(f"▶ ソフトUART RX を開始：GPIO={RX_PIN}, {BAUD}bps")
GPIO.setmode(GPIO.BCM)
GPIO.setup(wireless_PIN, GPIO.OUT)
GPIO.setwarnings(False)

def send_unicast(node_id, payload):
    # ワイヤレスグラウンドON
    GPIO.output(wireless_PIN, GPIO.HIGH)
    print(f"GPIO{wireless_PIN} をHIGHに設定（ワイヤレスグラウンドON）")
    time.sleep(0.5)

    # TXDU送信
    cmd = f'TXDU {node_id},{payload}\r\n'
    im920.write(cmd.encode())
    print(f"送信: {cmd.strip()}")

    # 応答待ち
    time.sleep(1.0)
    if im920.in_waiting == 0:
        print("❌ 応答なし（in_waiting = 0）")
    else:
        while im920.in_waiting:
            res = im920.readline().decode(errors="ignore").strip()
            print("Response:", res)

send_unicast("0003", "HELLO")
