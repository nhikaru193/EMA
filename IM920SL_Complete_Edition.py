import serial
import time
import pigpio

# --- 設定 ---
self.TX_PIN = 27
self.RX_PIN = 17
self.BAUD = 9600
self.WIRELESS_PIN = 22# ワイヤレスグラウンド制御用のGPIOピン番号。適宜変更してください。

# --- pigpioの初期化 ---
self.pi = pigpio.pi()
if not self.pi.connected:
    raise RuntimeError("pigpio デーモンに接続できません。sudo pigpiod を起動してください。")

try:
    err = self.pi.bb_serial_read_open(self.RX_PIN, self.BAUD, 8)
    if err != 0:
        raise RuntimeError(f"ソフトUART RX 設定失敗: GPIO={self.RX_PIN}, {self.BAUD}bps")
    
    print(f"▶ ソフトUART RX を開始：GPIO={self.RX_PIN}, {self.BAUD}bps")
    
    # WIRELESS_PINの設定
    self.pi.set_mode(self.WIRELESS_PIN, pigpio.OUTPUT)
    self.pi.write(self.WIRELESS_PIN, 0)
    print(f"GPIO{self.WIRELESS_PIN} をOUTPUTに設定し、LOWに初期化しました。")
    
    # 初期化が成功した場合、ここで終了

except Exception as e:
    # 初期化中にエラーが発生した場合、pigpioを停止して再スロー
    self.pi.stop()
    raise e

    
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

def send_TXDU(self, node_id, payload):
    # メッセージの準備と送信
    cmd = f'TXDU {node_id},{payload}\r\n'
    
    try:
        self.im920.write(cmd.encode())
        print(f"送信: {cmd.strip()}")
    except serial.SerialException as e:
        print(f"シリアル送信エラー: {e}")
    
    time.sleep(0.1)  # 送信後の短い遅延

# --- メインループ ---
try:
    self.pi.write(self.WIRELESS_PIN, 1)  # GPIOをHIGHに設定
    print(f"GPIO{self.WIRELESS_PIN} をHIGHに設定（ワイヤレスグラウンドON）")
    time.sleep(0.5)  # ワイヤレスグラウンドが安定するまで待機


    while True:
        #------GPSデータ送信のコード(ARLISSで追加)ここから------#
        (count, data) = self.pi.bb_serial_read(self.RX_PIN)
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
                                self.send_TXDU("0003", gps_payload)
                                
                                time.sleep(2)  # GPSデータ送信後の遅延
                else:
                    print("GPS情報を取得できませんでした。リトライします")
                    
            except Exception as e:
                print("エラー！！")
            finally:
                print("gps情報の取得中")

finally:
    # --- クリーンアップ ---
    pi.bb_serial_read_close(RX_PIN)
    pi.write(WIRELESS_PIN, 0)  # 終了時にワイヤレスグラウンドがOFFになるようにする
    pi.set_mode(WIRELESS_PIN, pigpio.INPUT)  # ピンを安全のため入力に戻す
    pi.stop()
    im920.close()  # シリアルポートを閉じる
    print("終了しました。")
