import math
import time
import serial
import pigpio
from BNO055 import BNO055
import smbus
import struct
import os

RX_PIN = 17
BAUD = 9600
pi = pigpio.pi()
        if not pi.connected:
            raise RuntimeError("pigpio デーモンに接続できません。sudo pigpiod を起動してください。")
        err = pi.bb_serial_read_open(RX_PIN, BAUD, 8)
        if err != 0:
            pi.stop()
            raise RuntimeError(f"ソフトUART RX 設定失敗: GPIO={RX_PIN}, {BAUD}bps")
current_time_str = time.strftime("%m%d-%H%M%S")
filename = os.path.join("/home/EM/_csv", f"GPS_and_heading_data_{current_time_str}.csv")
current_location = [0, 0]
with open(filename, "w", newline='') as f:
    writer = csv.writer(f)
    writer.writerow(["latitude", "longitude"])
    while True:
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
                                break
            except Exception as e:
                print(f"GPSデコードエラー: {e}")
            except KeyboardInterrupt:
                print("GPS取得を中断します")
                break     
            finally:
                print("クリーンアップを実行します")
                pi.stop()
                pi.bb_serial_read_close(RX_PIN)
        
    
