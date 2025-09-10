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
        
        self.serial_is_open = False
        
        self.pi = pigpio.pi()
        if not self.pi.connected:
            raise RuntimeError("pigpio daemon is not connected. Please run 'sudo pigpiod'.")

        try:
            err = self.pi.bb_serial_read_open(self.RX_PIN, self.BAUD, 8)
            if err != 0:
                raise RuntimeError(f"Failed to set up software UART RX: GPIO={self.RX_PIN}, {self.BAUD}bps")
            
            self.serial_is_open = True
            
            print(f"▶ Started software UART RX: GPIO={self.RX_PIN}, {self.BAUD}bps")
            
            self.pi.set_mode(self.WIRELESS_PIN, pigpio.OUTPUT)
            self.pi.write(self.WIRELESS_PIN, 0)
            print(f"GPIO{self.WIRELESS_PIN} set to OUTPUT and initialized to LOW.")
            
        except Exception as e:
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
            print(f"Sent: {cmd.strip()}")
        except serial.SerialException as e:
            print(f"Serial transmission error: {e}")
        time.sleep(0.1)

        
    def run(self):
        try:
            print("Starting landing detection sequence.")
            print("Heading change sequence: 1st sequence")
            self.start_time = time.time()
            max_counter =self.h_counter
            current_time_str = time.strftime("%m%d-%H%M%S")
            filename = f"land_heading_data_{current_time_str}.csv"
            path_to = "/home/EMA/_csv"
            filename = os.path.join(path_to, filename)
            self.pi.write(self.WIRELESS_PIN, 1)
            print(f"GPIO{self.WIRELESS_PIN} set to HIGH (Wireless ground ON)")
            time.sleep(0.5)

            with open(filename, "w", newline='') as f:
                writer = csv.writer(f)
                writer.writerow(["heading", "delta_heading"])
                while True:
                    print("Starting GPS data transmission sequence. Transmitting 1 times.")
                    for i in range(1):
                        print(f"Transmitting GPS data... ({i+1}/1)")
                        (count, data) = self.pi.bb_serial_read(self.RX_PIN)
                        current_location = None
                        
                        # --- [修正箇所] ---
                        # try...except...else 構文を正しく修正
                        if count and data:
                            try:
                                text = data.decode("ascii", errors="ignore")
                                found_gps = False
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
                                                found_gps = True
                                                time.sleep(2)
                                                break # GPS情報が見つかったらループを抜ける
                                if not found_gps:
                                    print("Valid GPS data ($GNRMC) not found. Retrying.")
                            except Exception as e:
                                print(f"An error occurred while processing GPS data: {e}")
                        else:
                            print("No data received from GPS module.")
                    
                    self.pi.write(self.WIRELESS_PIN, 0)
                    self.pi.set_mode(self.WIRELESS_PIN, pigpio.INPUT)
                    self.im920.close()
                    print("Finished GPS data transmission sequence.")
                    # --------------------
                    
                    current_time = time.time()
                    delta_time = current_time - self.start_time
                    before_heading = self.bno.getVector(BNO055.VECTOR_EULER)[0]
                    if before_heading is None:
                        print("Failed to retrieve values from BNO055.")
                        time.sleep(1)
                        continue
                    print(f"t = {delta_time:.2f} || heading = {before_heading}")
                    time.sleep(1)
                    after_heading = self.bno.getVector(BNO055.VECTOR_EULER)[0]
                    delta_heading = min(abs(after_heading - before_heading), 360 - abs(after_heading - before_heading))
                    writer.writerow([after_heading, delta_heading])
                    f.flush()
                    if delta_heading < self.h_threshold:
                        self.h_counter -= 1
                        print(f"Heading landing check successful {max_counter - self.h_counter} time(s)!")
                        if self.h_counter == 0:
                            print("Landed based on heading change.")
                            break
                    else:
                        self.h_counter = max_counter
                        print("Landing check failed. Retrying.")
                    if delta_time > self.timeout:
                        print("Landed based on timeout.")
                        break
            
            BME280.init_bme280()
            BME280.read_compensate()
            print("Pressure change sequence: 2nd sequence")
            self.start_time = time.time()
            max_counter = self.p_counter

            current_time_str = time.strftime("%m%d-%H%M%S")
            filename = f"land_pressure_data_{current_time_str}.csv"
            filename = os.path.join(path_to, filename)
            
            with open(filename, "w", newline='') as f:
                writer = csv.writer(f)
                writer.writerow(["pressure", "delta_pressure"])
                while True:
                    current_time = time.time()
                    delta_time = current_time - self.start_time
                    before_pressure = BME280.get_pressure()
                    print(f"t = {delta_time:.2f} || pressure = {before_pressure}")
                    time.sleep(5)
                    after_pressure = BME280.get_pressure()
                    if after_pressure is not None and before_pressure is not None:
                        delta_pressure = abs(after_pressure - before_pressure)
                        writer.writerow([after_pressure, delta_pressure])
                        if delta_pressure < self.p_threshold:
                            self.p_counter -= 1
                            print(f"Pressure landing check successful {max_counter - self.p_counter} time(s)!")
                            if self.p_counter == 0:
                                print("Landed based on pressure change.")
                                break
                        else:
                            self.p_counter = max_counter
                            print("Landing check failed. Retrying.")
                    if delta_time > self.timeout:
                        print("Landed based on timeout.")
                        break
            
        except KeyboardInterrupt:
            print("Skipping landing detection due to interrupt.")
            
        finally:
            print("Finishing landing detection and fusing sequence, or forced exit.")
            time.sleep(5)
            self.driver.cleanup()
            
            if self.serial_is_open:
                self.pi.bb_serial_read_close(self.RX_PIN)
                print("Closed software UART RX.")
            
            self.pi.stop()
