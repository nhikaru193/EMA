import serial
import time
import pigpio
import threading

class EmGpsDatalink:
    def __init__(self, rx_pin, tx_pin, baud_soft_uart, baud_im920, wireless_pin):
        # --- 設定 ---
        self.RX_PIN = rx_pin
        self.TX_PIN = tx_pin # 今回のGNRMC受信では使わないが、将来的な拡張のため保持
        self.BAUD_SOFT_UART = baud_soft_uart
        self.BAUD_IM920 = baud_im920
        self.WIRELESS_PIN = wireless_pin

        # --- pigpioの初期化 ---
        self.pi = pigpio.pi()
        if not self.pi.connected:
            print("ERROR: pigpio デーモンに接続できません。")
            raise ConnectionRefusedError("pigpio daemon not connected.")

        # WIRELESS_PINを出力に設定し、初期状態をLOW（ワイヤレスグラウンドOFF）にする
        self.pi.set_mode(self.WIRELESS_PIN, pigpio.OUTPUT)
        self.pi.write(self.WIRELESS_PIN, 0)
        print(f"[{self.__class__.__name__}] GPIO{self.WIRELESS_PIN} をOUTPUTに設定し、LOWに初期化しました。")

        # --- ソフトUART RXの設定 ---
        err = self.pi.bb_serial_read_open(self.RX_PIN, self.BAUD_SOFT_UART, 8)
        if err != 0:
            print(f"ERROR: ソフトUART RX の設定に失敗：GPIO={self.RX_PIN}, {self.BAUD_SOFT_UART}bps, エラーコード: {err}")
            self.pi.stop()
            raise IOError("Soft UART RX setup failed.")

        print(f"[{self.__class__.__name__}] ▶ ソフトUART RX を開始：GPIO={self.RX_PIN}, {self.BAUD_SOFT_UART}bps")

        # --- IM920シリアル通信の設定 ---
        try:
            self.im920 = serial.Serial('/dev/serial0', self.BAUD_IM920, timeout=1)
        except serial.SerialException as e:
            print(f"ERROR: IM920シリアルポートのオープンに失敗: {e}")
            self.cleanup()
            raise IOError("IM920 serial port open failed.")

        self.running = False
        self.gps_data_lock = threading.Lock()
        self.latest_gps_data = None # メインシーケンスから参照するための最新GPSデータ

    # --- 座標変換関数 ---
    def _convert_to_decimal(self, coord, direction):
        """
        度分（ddmm.mmmm）形式を10進数に変換します。
        """
        degrees = int(coord[:2]) if direction in ['N', 'S'] else int(coord[:3])
        minutes = float(coord[2:]) if direction in ['N', 'S'] else float(coord[3:])
        decimal = degrees + minutes / 60
        if direction in ['S', 'W']:
            decimal *= -1
        return decimal

    # --- IM920ユニキャスト送信関数（ワイヤレスグラウンド制御付き） ---
    def _send_unicast(self, node_id, payload):
        """
        IM920SLを使用して指定されたノードIDにペイロードをユニキャスト送信します。
        送信前にワイヤレスグラウンドをONにし、送信後にOFFにします。
        """
        # ワイヤレスグラウンドON
        self.pi.write(self.WIRELESS_PIN, 1)
        # print(f"[{self.__class__.__name__}] GPIO{self.WIRELESS_PIN} をHIGHに設定（ワイヤレスグラウンドON）")
        time.sleep(0.05) # ワイヤレスグラウンドが安定するまで待機 (元が0.5sと長すぎるため短縮)

        # メッセージの準備と送信
        node_id_str = f"{node_id:04X}"
        msg = f'TXDA {node_id_str},{payload}\r'
        
        try:
            self.im920.write(msg.encode())
            # print(f"[{self.__class__.__name__}] 送信: {msg.strip()}")
        except serial.SerialException as e:
            print(f"[{self.__class__.__name__}] シリアル送信エラー: {e}")
        
        time.sleep(0.05) # 送信後の短い遅延 (元が0.1sと長すぎるため短縮)

        # ワイヤレスグラウンドOFF
        self.pi.write(self.WIRELESS_PIN, 0)
        # print(f"[{self.__class__.__name__}] GPIO{self.WIRELESS_PIN} をLOWに設定（ワイヤレスグラウンドOFF）")
        time.sleep(0.05) # OFF後の短い遅延 (元が0.1sと長すぎるため短縮)

    def get_current_gps(self):
        """
        外部から最新のGPSデータを取得するためのメソッド。
        """
        with self.gps_data_lock:
            return self.latest_gps_data

    def _gps_read_loop(self):
        """
        GPSデータの読み取りと送信を行うメインループ。
        このメソッドが別スレッドで実行されます。
        """
        print(f"[{self.__class__.__name__}] GPS読み取り・送信スレッド開始。")
        while self.running:
            (count, data) = self.pi.bb_serial_read(self.RX_PIN)
            if count and data:
                try:
                    text = data.decode("ascii", errors="ignore")
                    if "$GNRMC" in text:
                        lines = text.split("\n")
                        for line in lines:
                            if "$GNRMC" in line:
                                parts = line.strip().split(",")
                                if len(parts) > 6 and parts[2] == "A": # 'A'はデータ有効
                                    lat = self._convert_to_decimal(parts[3], parts[4])
                                    lon = self._convert_to_decimal(parts[5], parts[6])
                                    
                                    gps_payload = f'{lat:.6f},{lon:.6f}' # ペイロードのフォーマット

                                    with self.gps_data_lock:
                                        self.latest_gps_data = {"latitude": lat, "longitude": lon, "raw": gps_payload}
                                    
                                    # GPSデータをユニキャストメッセージとして送信
                                    target_node_id = 0x0003 # 例のノードID。必要に応じて調整してください。
                                    self._send_unicast(target_node_id, gps_payload)
                                    
                                    time.sleep(1) # GPSデータ送信後の遅延 (元の2秒は少し長いため短縮)
                                    # このsleepはGPSデータ受信頻度と送信頻度の調整用。
                                    # 頻繁に送信しない場合は長くても良い。
                except Exception as e:
                    print(f"[{self.__class__.__name__}] データ処理/デコードエラー: {e}")
            time.sleep(0.05) # 受信ポーリング間隔。適宜調整してください。

        print(f"[{self.__class__.__name__}] GPS読み取り・送信スレッド終了。")


    def start(self):
        """
        GPSデータリンク処理を開始します。
        """
        if not self.running:
            self.running = True
            self.thread = threading.Thread(target=self._gps_read_loop)
            self.thread.start()
            print(f"[{self.__class__.__name__}] GPSデータリンクスレッドを起動しました。")

    def stop(self):
        """
        GPSデータリンク処理を停止します。
        """
        if self.running:
            print(f"[{self.__class__.__name__}] GPSデータリンクスレッド停止リクエスト送信中...")
            self.running = False
            self.thread.join() # スレッドが終了するのを待つ
            self.cleanup()
            print(f"[{self.__class__.__name__}] GPSデータリンク処理を停止しました。")

    def cleanup(self):
        """
        リソースを解放します。
        """
        if self.pi.connected:
            self.pi.bb_serial_read_close(self.RX_PIN)
            self.pi.write(self.WIRELESS_PIN, 0)
            self.pi.set_mode(self.WIRELESS_PIN, pigpio.INPUT)
            self.pi.stop()
        if self.im920.is_open:
            self.im920.close()
        print(f"[{self.__class__.__name__}] クリーンアップ完了。")

# このファイルを直接実行した場合のテストコード (オプション)
if __name__ == "__main__":
    # GPIOピン番号などの設定は、実際の環境に合わせてください
    # Raspberry Pi Zero Wなどでテストする場合は /dev/serial0 へのアクセス権限に注意
    # (例: sudo usermod -a -G dialout pi)
    
    # 実際にはメインのスクリプトからインスタンス化して使用します
    # ここは単体テスト用
    gps_link = None
    try:
        # この初期化パラメータはメインシーケンス側で定義されるものと合わせる必要があります
        gps_link = EmGpsDatalink(
            rx_pin=17, # RX_PIN
            tx_pin=27, # TX_PIN (実際には使わないが定義しておく)
            baud_soft_uart=9600,
            baud_im920=19200,
            wireless_pin=22
        )
        gps_link.start()
        print("5秒間GPSデータ受信・送信を待機します...")
        time.sleep(5)
        
        # 最新のGPSデータを取得してみる
        current_data = gps_link.get_current_gps()
        if current_data:
            print(f"テスト: 最新GPSデータ: {current_data}")
        else:
            print("テスト: GPSデータ未取得")

        print("さらに10秒間動作させます...")
        time.sleep(10)

    except (ConnectionRefusedError, IOError) as e:
        print(f"初期化エラーによりスクリプトを終了します: {e}")
    except KeyboardInterrupt:
        print("\nテストを中断します。")
    finally:
        if gps_link:
            gps_link.stop()
        print("テスト終了。")
