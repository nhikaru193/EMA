from BNO055 import BNO055
import BME280
import time
import fusing
import RPi.GPIO as GPIO
import struct
from motor import MotorDriver
import os
import csv
import math
import struct

#------GPSデータ送信(ARLISSで追加)ここから------#
import pigpio
import serial
#------GPSデータ送信(ARLISSで追加)ここまで------#

class LD:
    def __init__(self, bno: BNO055, p_counter = 3, h_counter = 3, timeout = 40, p_threshold = 0.50, h_threshold = 0.10):
        self.driver = MotorDriver(
            PWMA=12, AIN1=23, AIN2=18,    
            PWMB=19, BIN1=16, BIN2=26,    
            STBY=21                     
        )
        self.bno = bno
        self.TX_PIN = 27
        self.RX_PIN = 17
        self.BAUD = 9600
        self.WIRELESS_PIN = 22
        self.p_counter = p_counter
        self.h_counter = h_counter
        self.timeout = timeout
        self.p_threshold = p_threshold
        self.h_threshold = h_threshold
        self.start_time = time.time()
        self.im920 = serial.Serial('/dev/serial0', 19200, timeout=5)
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

        
    def run(self):
        try:
            print("着地判定を開始します")
            print("方位角変化量:第1シーケンス")
            self.start_time = time.time()
            max_counter =self.h_counter
            #heading着地判定
            current_time_str = time.strftime("%m%d-%H%M%S") #現在時刻をファイル名に含める
            filename = f"land_heading_data_{current_time_str}.csv"
            path_to = "/home/EMA/_csv"
            filename = os.path.join(path_to, filename)
            self.pi.write(self.WIRELESS_PIN, 1)  # GPIOをHIGHに設定
            print(f"GPIO{self.WIRELESS_PIN} をHIGHに設定（ワイヤレスグラウンドON）")
            time.sleep(0.5)  # ワイヤレスグラウンドが安定するまで待機

            with open(filename, "w", newline='') as f: # newline='' はCSV書き込みのベストプラクティス #withでファイルを安全に開く
                writer = csv.writer(f)
                writer.writerow(["heading", "delta_heading"])
                while True:
                    #------GPSデータ送信のコード(ARLISSで追加)ここから------#
                    print("GPSデータ送信シーケンスを開始します。GPS情報を5回送信します。")
                    for i in range(10):
                        print(f"GPSデータ送信中... ({i+1}/10回目)")
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

                        else:
                            print("データがありませんでした。")
                        
                        time.sleep(2) # 次の送信までの間隔
                        
                    self.pi.bb_serial_read_close(self.RX_PIN)
                    self.pi.write(self.WIRELESS_PIN, 0)  # 終了時にワイヤレスグラウンドがOFFになるようにする
                    self.pi.set_mode(self.WIRELESS_PIN, pigpio.INPUT)  # ピンを安全のため入力に戻す
                    self.im920.close()
                    print("GPSデータ送信シーケンスを終了しました。")
                    
                    #------GPSデータ送信のコード(ARLISSで追加)ここまで------#
                    current_time = time.time()
                    delta_time = current_time - self.start_time
                    before_heading = self.bno.getVector(BNO055.VECTOR_EULER)[0]
                    if before_heading is None:
                        print("BNO055の値が取得できませんでした")
                        time.sleep(1)
                        continue
                    print(f"t = {delta_time}||heading = {before_heading}")
                    time.sleep(1)
                    after_heading = self.bno.getVector(BNO055.VECTOR_EULER)[0]
                    delta_heading = min((after_heading -  before_heading) % 360, (before_heading -  after_heading) % 360)
                    writer.writerow([after_heading, delta_heading])
                    f.flush() # データをすぐにファイルに書き出す (バッファリングさせない)
                    if delta_heading < self.h_threshold:
                        self.h_counter = self.h_counter - 1
                        print(f"方位角着地判定{self.h_counter}回成功！")
                        if self.h_counter == 0:
                            print("方位角:変化量による着地判定")
                            break
                    else:
                        self.h_counter = max_counter
                        print("着地判定失敗。再度判定を行います")
                    if delta_time > self.timeout:
                        print("方位角:timeoutによる着地判定")
                        break
        
            #環境センサの初期設定
            BME280.init_bme280()
            BME280.read_compensate()
            print("気圧変化量:第2シーケンス")
            self.start_time = time.time()
            max_counter =self.p_counter

            current_time_str = time.strftime("%m%d-%H%M%S") #現在時刻をファイル名に含める
            filename = f"land_pressure_data_{current_time_str}.csv"
            path_to = "/home/EMA/_csv"
            filename = os.path.join(path_to, filename)
            
            #気圧着地判定
            with open(filename, "w", newline='') as f: # newline='' はCSV書き込みのベストプラクティス #withでファイルを安全に開く
                writer = csv.writer(f)
                writer.writerow(["pressure", "delta_pressure"])
                while True:
                    current_time = time.time()
                    delta_time = current_time - self.start_time
                    before_pressure = BME280.get_pressure()
                    print(f"t = {delta_time}||pressure = {before_pressure}")
                    time.sleep(5)
                    after_pressure = BME280.get_pressure()
                    # after_pressureがNoneの場合の考慮も必要ですが、元のコードの意図を尊重しここでは修正しません
                    delta_pressure = abs(after_pressure - before_pressure)
                    writer.writerow([after_pressure, delta_pressure])
                    if delta_pressure < self.p_threshold:
                        self.p_counter = self.p_counter - 1
                        print(f"気圧着地判定{self.p_counter}回成功！")
                        if self.p_counter == 0:
                            print("気圧:変化量による着地判定")
                            break
                    else:
                        self.p_counter = max_counter
                        print("着地判定失敗。再度判定を行います")
                    if delta_time > self.timeout:
                        print("気圧:timeoutによる着地判定")
                        break
        
            #溶断回路作動
            """
            print("着地判定正常終了。テグス溶断シーケンスに入ります")
            time.sleep(3)
            fusing.circuit()
            print("テグス溶断を完了しました。テグス溶断の確認を行います")
            before_heading = self.bno.getVector(BNO055.VECTOR_EULER)[0]
            self.driver.petit_left(0, 90)
            self.driver.petit_left(90, 0)
            after_heading = self.bno.getVector(BNO055.VECTOR_EULER)[0]
            delta_heading = min((after_heading -  before_heading) % 360, (before_heading -  after_heading) % 360)
            if delta_heading < 5:
                print("溶断の不良を確認しました。再度溶断シーケンスを行います")
                fusing.circuit()
                print("テグス溶断の再起動を終了しました")
                """
            
        except KeyboardInterrupt:
            print("割り込みにより、着地判定をスキップします")
            
        finally:
            """
            print("着地判定+溶断回路動作の終了です or 強制終了です")
            time.sleep(5)
            self.pi.bb_serial_read_close(self.RX_PIN)
            self.pi.write(self.WIRELESS_PIN, 0)  # 終了時にワイヤレスグラウンドがOFFになるようにする
            self.pi.set_mode(self.WIRELESS_PIN, pigpio.INPUT)  # ピンを安全のため入力に戻す
            self.im920.close()
            print("GPSデータ送信シーケンスを終了しました。")
            """
            self.driver.cleanup()
            self.pi.stop()
