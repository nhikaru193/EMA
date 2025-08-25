import serial
import time
import pigpio

# --- 設定 ---
TX_PIN = 27
self.RX_PIN = 17
self.BAUD = 9600
WIRELESS_PIN = 22  # ワイヤレスグラウンド制御用のGPIOピン番号。適宜変更してください。

# --- pigpioの初期化 ---
self.pi = pigpio.pi()
if not self.pi.connected:
    raise RuntimeError("pigpio デーモンに接続できません。sudo pigpiod を起動してください。")
err = self.pi.bb_serial_read_open(self.RX_PIN, self.BAUD, 8)
if err != 0:
    self.pi.stop()
    raise RuntimeError(f"ソフトUART RX 設定失敗: GPIO={self.RX_PIN}, {self.BAUD}bps")

print(f"▶ ソフトUART RX を開始：GPIO={RX_PIN}, {BAUD}bps")

# WIRELESS_PINを出力に設定し、初期状態をLOW（ワイヤレスグラウンドOFF）にする
self.pi.set_mode(WIRELESS_PIN, pigpio.OUTPUT)
self.pi.write(WIRELESS_PIN, 0)
print(f"GPIO{WIRELESS_PIN} をOUTPUTに設定し、LOWに初期化しました。")


print(f"▶ ソフトUART RX を開始：GPIO={RX_PIN}, {BAUD}bps")

# --- 座標変換関数 ---
def convert_to_decimal(self, coord, direction):
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

# --- IM920シリアル通信の設定 ---
im920 = serial.Serial('/dev/serial0', 19200, timeout=1)

# --- IM920ユニキャスト送信関数（ワイヤレスグラウンド制御付き） ---
def send_TXDU(node_id, payload):
    # メッセージの準備と送信
    cmd = f'TXDU {node_id},{payload}\r\n'
    
    try:
        im920.write(cmd.encode())
        print(f"送信: {cmd.strip()}")
    except serial.SerialException as e:
        print(f"シリアル送信エラー: {e}")
    
    time.sleep(0.1)  # 送信後の短い遅延

    # ワイヤレスグラウンドOFF
    #pi.write(WIRELESS_PIN, 0)  # GPIOをLOWに設定
    #print(f"GPIO{WIRELESS_PIN} をLOWに設定（ワイヤレスグラウンドOFF）")
    #time.sleep(0.1)  # OFF後の短い遅延

# --- メインループ ---
try:
    pi.write(WIRELESS_PIN, 1)  # GPIOをHIGHに設定
    print(f"GPIO{WIRELESS_PIN} をHIGHに設定（ワイヤレスグラウンドON）")
    time.sleep(0.5)  # ワイヤレスグラウンドが安定するまで待機

    while True:
        (count, data) = self.pi.bb_serial_read(RX_PIN)
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
                                lat = self.convert_to_decimal(parts[3], parts[4])
                                lon = self.convert_to_decimal(parts[5], parts[6])
                                current_location = [lat, lon]
                                # GPSデータをユニキャストメッセージとして送信
                                gps_payload = f'{lat:.6f},{lon:.6f}'  # ペイロードのフォーマット
                                send_TXDU("0003", gps_payload)
                                
                                time.sleep(2)  # GPSデータ送信後の遅延
            except Exception as e:
                print(f"デコードエラー: {e}")
                
        if not current_location:
            print("[WARN] GPS位置情報を取得できません。リトライします...")
            time.sleep(1)
            continue
        

except KeyboardInterrupt:
    print("\nユーザー割り込みで終了します。")

finally:
    # --- クリーンアップ ---
    pi.bb_serial_read_close(RX_PIN)
    pi.write(WIRELESS_PIN, 0)  # 終了時にワイヤレスグラウンドがOFFになるようにする
    pi.set_mode(WIRELESS_PIN, pigpio.INPUT)  # ピンを安全のため入力に戻す
    pi.stop()
    im920.close()  # シリアルポートを閉じる
    print("終了しました。")
