import serial
import RPi.GPIO as GPIO
import time

im920 = serial.Serial('/dev/serial0', 19200, timeout=1)
wireless_PIN = 22

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
