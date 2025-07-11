import serial
import time
import pigpio

class GpsIm920Communicator:
    """
    GPSモジュールからNMEA GNRMCセンテンスを読み取り、
    IM920無線モジュールを介して指定されたノードIDにGPSデータを送信するクラス。

    pigpioライブラリを使用してソフトウェアUART経由でGPSを受信し、
    GPIOを介してIM920SLのワイヤレスグラウンドを制御します。
    """

    def __init__(self, tx_pin=27, rx_pin=17, gps_baud=9600,
                 wireless_ctrl_pin=22, im920_port='/dev/serial0', im920_baud=19200,
                 target_node_id=0x0003):
        """
        GpsIm920Communicatorのコンストラクタです。

        Args:
            tx_pin (int): pigpioソフトウェアUARTの送信ピン番号。
            rx_pin (int): pigpioソフトウェアUARTの受信ピン番号 (GPSモジュールから)。
            gps_baud (int): GPSモジュールのボーレート。
            wireless_ctrl_pin (int): IM920SLのワイヤレスグラウンド制御用のGPIOピン番号。
            im920_port (str): IM920SLが接続されているシリアルポートのパス。
            im920_baud (int): IM920SLのボーレート。
            target_node_id (int): GPSデータをユニキャスト送信するIM920SLのターゲットノードID。
        """
        self.tx_pin = tx_pin
        self.rx_pin = rx_pin
        self.gps_baud = gps_baud
        self.wireless_ctrl_pin = wireless_ctrl_pin
        self.im920_port = im920_port
        self.im920_baud = im920_baud
        self.target_node_id = target_node_id

        self.pi = None
        self.im920 = None

    def _convert_to_decimal(self, coord, direction):
        """
        度分（ddmm.mmmm）形式を10進数に変換します。
        内部ヘルパーメソッド。
        """
        degrees = int(coord[:2]) if direction in ['N', 'S'] else int(coord[:3])
        minutes = float(coord[2:]) if direction in ['N', 'S'] else float(coord[3:])
        decimal = degrees + minutes / 60
        if direction in ['S', 'W']:
            decimal *= -1
        return decimal

    def _setup_gpio_and_uart(self):
        """
        pigpioを初期化し、GPIOピンとソフトウェアUARTを設定します。
        """
        self.pi = pigpio.pi()
        if not self.pi.connected:
            print("pigpio デーモンに接続できません。")
            raise ConnectionError("pigpioデーモンに接続できません。'sudo pigpiod'で起動してください。")

        # WIRELESS_PINを出力に設定し、初期状態をLOW（ワイヤレスグラウンドOFF）にする
        self.pi.set_mode(self.wireless_ctrl_pin, pigpio.OUTPUT)
        self.pi.write(self.wireless_ctrl_pin, 0)
        print(f"GPIO{self.wireless_ctrl_pin} をOUTPUTに設定し、LOWに初期化しました。")

        # ソフトUART RXの設定
        err = self.pi.bb_serial_read_open(self.rx_pin, self.gps_baud, 8)
        if err != 0:
            print(f"ソフトUART RX の設定に失敗：GPIO={self.rx_pin}, {self.gps_baud}bps, エラーコード: {err}")
            raise IOError(f"ソフトUART RX の設定に失敗しました (エラーコード: {err})")

        print(f"▶ ソフトUART RX を開始：GPIO={self.rx_pin}, {self.gps_baud}bps")

    def _setup_im920_serial(self):
        """
        IM920シリアル通信を設定します。
        """
        try:
            self.im920 = serial.Serial(self.im920_port, self.im920_baud, timeout=1)
            print(f"IM920 シリアルポートを開きました: {self.im920_port} @ {self.im920_baud}bps")
        except serial.SerialException as e:
            print(f"IM920 シリアルポートのオープンに失敗しました: {e}")
            raise e

    def send_unicast(self, payload):
        """
        IM920SLを使用して設定されたターゲットノードIDにペイロードをユニキャスト送信します。
        送信前にワイヤレスグラウンドをONにし、送信後にOFFにします。

        Args:
            payload (str): 送信する文字列データ。
        """
        if not self.pi or not self.im920:
            print("初期化が完了していません。最初に start() メソッドを呼び出してください。")
            return

        # ワイヤレスグラウンドON
        self.pi.write(self.wireless_ctrl_pin, 1)  # GPIOをHIGHに設定
        print(f"GPIO{self.wireless_ctrl_pin} をHIGHに設定（ワイヤレスグラウンドON）")
        time.sleep(0.5)  # ワイヤレスグラウンドが安定するまで待機

        # メッセージの準備と送信
        # 'TXDA <Node ID>,<Payload>\r'形式でデータ送信
        # Node IDは4桁の16進数としてフォーマット
        node_id_str = f"{self.target_node_id:04X}"
        msg = f'TXDA {node_id_str},{payload}\r'
        
        try:
            self.im920.write(msg.encode())
            print(f"送信: {msg.strip()}")
        except serial.SerialException as e:
            print(f"シリアル送信エラー: {e}")
        
        time.sleep(0.1)  # 送信後の短い遅延

        # ワイヤレスグラウンドOFF
        self.pi.write(self.wireless_ctrl_pin, 0)  # GPIOをLOWに設定
        print(f"GPIO{self.wireless_ctrl_pin} をLOWに設定（ワイヤレスグラウンドOFF）")
        time.sleep(0.1)  # OFF後の短い遅延

    def start_communication_loop(self):
        """
        GPSデータの受信とIM920を介した送信を開始するメインループです。
        Ctrl+Cで中断するまで実行されます。
        """
        try:
            self._setup_gpio_and_uart()
            self._setup_im920_serial()

            print("\nGPS受信とIM920送信を開始します。Ctrl+Cで終了してください。")
            while True:
                (count, data) = self.pi.bb_serial_read(self.rx_pin)
                if count and data:
                    try:
                        text = data.decode("ascii", errors="ignore")
                        if "$GNRMC" in text:
                            lines = text.split("\n")
                            for line in lines:
                                if "$GNRMC" in line:
                                    parts = line.strip().split(",")
                                    if len(parts) > 6 and parts[2] == "A": # "A"はデータが有効であることを示す
                                        lat = self._convert_to_decimal(parts[3], parts[4])
                                        lon = self._convert_to_decimal(parts[5], parts[6])
                                        
                                        # GPSデータをユニキャストメッセージとして送信
                                        gps_payload = f'{lat:.6f},{lon:.6f}' # ペイロードのフォーマット
                                        self.send_unicast(gps_payload)
                                        
                                        time.sleep(2) # GPSデータ送信後の遅延
                    except Exception as e:
                        print(f"データ処理エラー: {e}")
                time.sleep(0.1)

        except KeyboardInterrupt:
            print("\nユーザー割り込みで終了します。")
        except ConnectionError as e:
            print(f"初期化エラー: {e}")
        except Exception as e:
            print(f"予期せぬエラーが発生しました: {e}")
        finally:
            self._cleanup()

    def _cleanup(self):
        """
        リソースをクリーンアップし、GPIOを安全な状態に戻します。
        """
        if self.pi:
            self.pi.bb_serial_read_close(self.rx_pin)
            self.pi.write(self.wireless_ctrl_pin, 0)  # 終了時にワイヤレスグラウンドがOFFになるようにする
            self.pi.set_mode(self.wireless_ctrl_pin, pigpio.INPUT)  # ピンを安全のため入力に戻す
            self.pi.stop()
            print("pigpioリソースをクリーンアップしました。")
        if self.im920 and self.im920.is_open:
            self.im920.close()  # シリアルポートを閉じる
            print("IM920シリアルポートを閉じました。")
        print("通信モジュールを終了しました。")


# --- 実行例 ---
if __name__ == '__main__':
    # ご自身の環境に合わせて以下のパラメータを調整してください
    communicator = GpsIm920Communicator(
        tx_pin=27,                  # pigpioソフトUARTの送信ピン
        rx_pin=17,                  # pigpioソフトUARTの受信ピン (GPSモジュールから)
        gps_baud=9600,              # GPSモジュールのボーレート
        wireless_ctrl_pin=22,       # ワイヤレスグラウンド制御用のGPIOピン
        im920_port='/dev/serial0',  # IM920が接続されているシリアルポート
        im920_baud=19200,           # IM920のボーレート
        target_node_id=0x0003       # GPSデータを送信するターゲットのノードID (例: 0x0003)
    )

    # 通信ループを開始
    communicator.start_communication_loop()
