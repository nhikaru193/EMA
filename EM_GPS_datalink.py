import serial
import time
import pigpio

# --- 設定 ---
TX_PIN = 27
RX_PIN = 17
BAUD = 9600
WIRELESS_PIN = 22  # ワイヤレスグラウンド制御用のGPIOピン番号。適宜変更してください。

# --- pigpioの初期化 ---
pi = pigpio.pi()
if not pi.connected:
    print("pigpio デーモンに接続できません。")
    exit(1)

# WIRELESS_PINを出力に設定し、初期状態をLOW（ワイヤレスグラウンドOFF）にする
pi.set_mode(WIRELESS_PIN, pigpio.OUTPUT)
pi.write(WIRELESS_PIN, 0)
print(f"GPIO{WIRELESS_PIN} をOUTPUTに設定し、LOWに初期化しました。")

# --- ソフトUART RXの設定 ---
err = pi.bb_serial_read_open(RX_PIN, BAUD, 8)
if err != 0:
    print(f"ソフトUART RX の設定に失敗：GPIO={RX_PIN}, {BAUD}bps")
    pi.stop()
    exit(1)

print(f"▶ ソフトUART RX を開始：GPIO={RX_PIN}, {BAUD}bps")

# --- 座標変換関数 ---
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

# --- IM920シリアル通信の設定 ---
im920 = serial.Serial('/dev/serial0', 19200, timeout=1)

# --- IM920ユニキャスト送信関数（ワイヤレスグラウンド制御付き） ---
def send_unicast(node_id, payload):
    """
    IM920SLを使用して指定されたノードIDにペイロードをユニキャスト送信します。
    送信前にワイヤレスグラウンドをONにし、送信後にOFFにします。
    """
    # ワイヤレスグラウンドON
    #pi.write(WIRELESS_PIN, 1)  # GPIOをHIGHに設定
    #print(f"GPIO{WIRELESS_PIN} をHIGHに設定（ワイヤレスグラウンドON）")
    #time.sleep(0.5)  # ワイヤレスグラウンドが安定するまで待機

    # メッセージの準備と送信
    # 'TXDA <Node ID>,<Payload>\r'形式でデータ送信
    # Node IDは4桁の16進数としてフォーマット
    node_id_str = f"{node_id:04X}"
    msg = f'TXDA {node_id_str},{payload}\r'
    
    try:
        im920.write(msg.encode())
        print(f"送信: {msg.strip()}")
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
        (count, data) = pi.bb_serial_read(RX_PIN)
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
                                
                                # GPSデータをユニキャストメッセージとして送信
                                # ターゲットのノードIDを定義してください
                                target_node_id = 0x0003  # 例のノードID。必要に応じて調整してください。
                                gps_payload = f'{lat:.6f},{lon:.6f}'  # ペイロードのフォーマット
                                send_unicast(target_node_id, gps_payload)
                                
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
