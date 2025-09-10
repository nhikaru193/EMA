import time
import serial
import pigpio
import threading

class GPS:
    def __init__(self, pi):
        self.TX_PIN = 27
        self.RX_PIN = 17
        self.BAUD = 9600
        self.im920 = serial.Serial('/dev/serial0', 19200, timeout=5)
        self.is_running = True
        self.thread = None
        self.WIRELESS_PIN = 22
        self.pi.bb_serial_read_open(self.RX_PIN, 9600)
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
        cmd = f'TXDU {node_id},{payload}\r\n'
        try:
            self.im920.write(cmd.encode())
            print(f"送信: {cmd.strip()}")
        except serial.SerialException as e:
            print(f"シリアル送信エラー: {e}")
        time.sleep(0.1)

    def run(self):
        print("GPSデータ送信スレッドを開始します。")
        self.pi.write(self.WIRELESS_PIN, 1)
        print(f"GPIO{self.WIRELESS_PIN} をHIGHに設定（ワイヤレスグラウンドON）")
        time.sleep(0.5)

        try:
            while self.is_running:
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
                                        gps_payload = f'{lat:.6f},{lon:.6f}'
                                        self.send_TXDU("0003", gps_payload)
                                        time.sleep(2)
                                        break
                            else:
                                print("GPS情報を取得できませんでした。リトライします")
                        else:
                            print("GPS情報が見つかりませんでした。")
                    except Exception as e:
                        print(f"エラー！！: {e}")
                else:
                    print("データがありませんでした。")
                time.sleep(2)
        finally:
            self.pi.bb_serial_read_close(self.RX_PIN)
            self.pi.write(self.WIRELESS_PIN, 0)
            print("GPSデータ送信スレッドを終了します。")

    def start(self):
        self.thread = threading.Thread(target=self.run, daemon=True)
        self.thread.start()

    def stop(self):
        self.is_running = False
        if self.thread:
            self.thread.join()
