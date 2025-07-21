import serial
import time
import pigpio
import csv

try:
    TX_PIN = 27
    RX_PIN = 17
    BAUD = 9600
    
    pi = pigpio.pi()
    
    def convert_to_decimal(coord, direction):
        # 度分（ddmm.mmmm）形式を10進数に変換
        degrees = int(coord[:2]) if direction in ['N', 'S'] else int(coord[:3])
        minutes = float(coord[2:]) if direction in ['N', 'S'] else float(coord[3:])
        decimal = degrees + minutes / 60
        if direction in ['S', 'W']:
            decimal *= -1
        return decimal
    
    lat = 0
    lon = 0
    
    filename = "GNSS_" + time.strftime("%m%d-%H%M%S") + ".csv"
    f = open(filename,"w")
    writer = csv.writer(f)
    start_time = time.time()
    while True:
        current_time = time.time()
        elapsed_time = current_time - start_time
        (count, data) = pi.bb_serial_read(RX_PIN)
        if count and data:
            try:
                text = data.decode("ascii", errors="ignore")
                if "$GNRMC" in text:
                    lines = text.split("\n")
                    for line in lines:
                        if "$GNRMC" in line:
                            parts = line.strip().split(",")
                            if len(parts) > 6 and parts[2] == "A":
                                lat = convert_to_decimal(parts[3], parts[4])
                                lon = convert_to_decimal(parts[5], parts[6])
                                print(f"t:{elapsed_time}, lat:{lat}, lon:{lon}")
                                writer.writerows([[elapsed_time, lat, lon]])
                                f.flush()
        if elapsed_time > 600:
            f.close()
            print("プログラムを終了します")
            
            break
        time.sleep(2)
finally:
    pi.stop()
