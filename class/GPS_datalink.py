import serial
import time
import pigpio
import math # mathモジュールが必要なのでインポート追加

class GpsIm920Communicator:
    """
    GPSモジュールからNMEA GNRMCセンテンスを読み取り、
    IM920無線モジュールを介して指定されたノードIDにGPSデータを送信するクラス。
    pigpioライブラリを使用してソフトウェアUART経由でGPSを受信し、
    GPIOを介してIM920SLのワイヤレスグラウンドを制御します。
    このバージョンは、スレッドで実行できるように停止メカニズムを備え、
    ハードウェアの初期化をactivate()メソッドに遅延させます。
    """

    def __init__(self, pi_instance, rx_pin=17, gps_baud=9600,
                 wireless_ctrl_pin=22, im920_port='/dev/serial0', im920_baud=19200,
                 target_node_id=0x0003):
        """
        GpsIm920Communicatorのコンストラクタです。
        ここではハードウェアは初期化せず、設定値を保存するだけです。
        """
        self.pi = pi_instance
        self.rx_pin = rx_pin
        self.gps_baud = gps_baud
        self.wireless_ctrl_pin = wireless_ctrl_pin
        self.im920_port = im920_port
        self.im920_baud = im920_baud
        self.target_node_id = target_node_id

        self.im920 = None # シリアルポートはactivate()で開く
        self._running = False # スレッドの実行状態を制御するフラグ
        self._activated = False # ハードウェアがアクティブ化されたかを示すフラグ

    def _convert_to_decimal(self, coord, direction):
        """
        度分（ddmm.mmmm）形式を10進数に変換します。
        """
        if not coord: return 0.0 # 空文字列の場合の対応を追加
        if direction in ['N', 'S']:
            degrees = int(coord[:2])
            minutes = float(coord[2:])
        else:
            degrees = int(coord[:3])
            minutes = float(coord[3:])
        decimal = degrees + minutes / 60
        if direction in ['S', 'W']:
            decimal *= -1
        return decimal

    # GPS_datalink.py の GpsIm920Communicator クラス内

    def _setup_gpio_and_uart(self):
        """GPIOピンとソフトウェアUARTを設定します。"""
        # wireless_ctrl_pin (GPIO22) は、システム起動時などに既にOUTPUTに設定されている可能性があるので、
        # set_modeは呼ばずに、直接writeでLOWにするだけにする。
        self.pi.write(self.wireless_ctrl_pin, 0) # GPIO22をLOW (OFF) に初期設定 (モードは既存を利用)
        print(f"GPIO{self.wireless_ctrl_pin} をLOWに初期化しました（モードは既存のまま）。")

        # --- ここから追加/修正 ---
        # rx_pin (GPIO17) を明示的に入力モードに設定してからソフトウェアUARTを開く
        # これにより、pigpioがピンの制御を確実に引き継ぐことを試みる
        self.pi.set_mode(self.rx_pin, pigpio.INPUT)
        print(f"GPIO{self.rx_pin} をINPUTモードに設定しました。")
        # --- ここまで追加/修正 ---

        err = self.pi.bb_serial_read_open(self.rx_pin, self.gps_baud, 8)
        if err != 0:
            raise IOError(f"ソフトUART RX の設定に失敗しました (エラーコード: {err})")
        print(f"▶ ソフトUART RX を開始：GPIO={self.rx_pin}, {self.gps_baud}bps")

    def _setup_im920_serial(self):
        """IM920シリアル通信を設定します。"""
        try:
            self.im920 = serial.Serial(self.im920_port, self.im920_baud, timeout=1)
            print(f"IM920 シリアルポートを開きました: {self.im920_port} @ {self.im920_baud}bps")
        except serial.SerialException as e:
            raise e

    def activate(self):
        """
        GPSソフトUARTとIM920シリアルを初期化し、ワイヤレスグラウンドをONにします。
        放出判定後に一度だけ呼び出されることを想定。
        """
        if self._activated:
            print("GpsIm920Communicatorは既にアクティブ化されています。")
            return
        
        print("✅ GpsIm920Communicatorをアクティブ化しています...")
        try:
            self._setup_gpio_and_uart()
            self._setup_im920_serial()
            self.turn_wireless_ground_on() # アクティブ化時に継続的にONにする
            self._activated = True
            print("✅ GpsIm920Communicatorアクティブ化完了。")
        except Exception as e:
            print(f"🔴 GpsIm920Communicatorのアクティブ化に失敗しました: {e}")
            self.cleanup() # アクティブ化失敗時はクリーンアップ
            raise # 例外を再発生させ、上位で処理できるようにする

    def turn_wireless_ground_on(self):
        """ワイヤレスグラウンドをONにします（継続的）。"""
        if self.pi:
            self.pi.write(self.wireless_ctrl_pin, 1)
            print(f"GPIO{self.wireless_ctrl_pin} をHIGHに設定（ワイヤレスグラウンドON）")
            time.sleep(0.5) # 安定化待機

    def turn_wireless_ground_off(self):
        """ワイヤレスグラウンドをOFFにします（継続的）。"""
        if self.pi:
            self.pi.write(self.wireless_ctrl_pin, 0)
            print(f"GPIO{self.wireless_ctrl_pin} をLOWに設定（ワイヤレスグラウンドOFF）")
            time.sleep(0.1) # 短い待機

    def get_current_gps_location(self):
        """
        GPSデータから現在の緯度と経度を一度だけ取得し返します。
        タイムアウトした場合、None, Noneを返します。
        """
        if not self._activated:
            print("警告: GpsIm920Communicatorがアクティブ化されていません。GPSデータ取得をスキップします。")
            return None, None

        start_time = time.time()
        timeout_duration = 2 # 短いタイムアウトで最新のデータを取得
        while (time.time() - start_time) < timeout_duration:
            (count, data) = self.pi.bb_serial_read(self.rx_pin)
            if count and data:
                try:
                    text = data.decode("ascii", errors="ignore")
                    if "$GNRMC" in text:
                        lines = text.split("\n")
                        for line in lines:
                            if line.startswith("$GNRMC"):
                                parts = line.strip().split(",")
                                if len(parts) > 6 and parts[2] == "A": # "A"はデータが有効であることを示す
                                    lat = self._convert_to_decimal(parts[3], parts[4])
                                    lon = self._convert_to_decimal(parts[5], parts[6])
                                    return lat, lon
                except Exception as e:
                    print(f"警告: GPSデータ解析エラー (get_current_gps_location): {e}")
            time.sleep(0.01) # 短い待機でCPU負荷軽減
        return None, None

    def send_unicast(self, payload):
        """
        IM920SLを使用してペイロードをユニキャスト送信します。
        ワイヤレスグラウンドは既にONになっている前提です。
        """
        if not self._activated or not self.im920 or not self.im920.is_open:
            print("警告: IM920通信がアクティブ化されていないか、シリアルポートが開いていません。送信をスキップします。")
            return

        node_id_str = f"{self.target_node_id:04X}"
        msg = f'TXDA {node_id_str},{payload}\r'
        
        try:
            self.im920.write(msg.encode())
            print(f"IM920送信: {msg.strip()}")
        except serial.SerialException as e:
            print(f"🔴 シリアル送信エラー: {e}")
        
        time.sleep(0.1) # 送信後の短い遅延

    def start_communication_loop(self):
        """
        GPSデータの受信とIM920を介した送信を開始するメインループです。
        _runningフラグがFalseになるまで実行されます。
        """
        if not self._activated:
            print("🔴 GpsIm920Communicatorがアクティブ化されていません。通信ループを開始できません。")
            return

        self._running = True
        print("✅ GPS受信とIM920送信を並行して開始します。")
        try:
            while self._running:
                lat, lon = self.get_current_gps_location() # タイムアウト付きで最新GPSを取得
                if lat is not None and lon is not None:
                    gps_payload = f'{lat:.6f},{lon:.6f}'
                    self.send_unicast(gps_payload)
                else:
                    print("警告: 通信ループ中にGPSデータが取得できませんでした。")
                time.sleep(5) # 例えば5秒ごとにGPSデータを送信

        except Exception as e:
            print(f"🔴 GPS/IM920通信ループ中にエラーが発生しました: {e}")
        finally:
            self._running = False # エラー終了時もフラグをFalseに
            print("GPS/IM920通信ループを終了します。")
            self._cleanup_internal_resources()

    def stop(self):
        """通信ループを停止するようフラグを設定します。"""
        self._running = False
        print("GPS/IM920通信ループ停止リクエストを受信しました。")

    def cleanup(self):
        """
        外部から呼び出されるクリーンアップメソッド。
        まずループを停止し、その後内部リソースをクリーンアップします。
        """
        self.stop() # まず通信ループを停止
        # スレッドが終了するのを待つ (メインスクリプト側でjoinするため、ここではフラグ設定のみ)
        self.turn_wireless_ground_off() # 明示的にOFFにする
        self._cleanup_internal_resources()

    def _cleanup_internal_resources(self):
        """このクラスが独自に初期化したリソースのみをクリーンアップします。"""
        if self._activated: # アクティブ化された場合のみクリーンアップを試みる
            if self.pi: 
                try:
                    self.pi.bb_serial_read_close(self.rx_pin)
                    self.pi.set_mode(self.wireless_ctrl_pin, pigpio.INPUT) # ピンを入力に戻す
                    print("GpsIm920Communicator: pigpio関連リソースをクリーンアップしました。")
                except Exception as e:
                    print(f"警告: GpsIm920Communicator: pigpioリソースクリーンアップ中にエラー: {e}")

            if self.im920 and self.im920.is_open:
                self.im920.close()
                print("GpsIm920Communicator: IM920シリアルポートを閉じました。")
            
            self._activated = False # クリーンアップされたことを示す
        print("GpsIm920Communicator: 内部リソースのクリーンアップ完了。")
