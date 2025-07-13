import smbus
import time
import struct
import math
import BME280
from BNO055 import BNO055
import pigpio
import serial

def convert_to_decimal(coord, direction):
    # 度分（ddmm.mmmm）形式を10進数に変換
    degrees = int(coord[:2]) if direction in ['N', 'S'] else int(coord[:3])
    minutes = float(coord[2:]) if direction in ['N', 'S'] else float(coord[3:])
    decimal = degrees + minutes / 60
    if direction in ['S', 'W']:
        decimal *= -1
    return decimal

# BNO055の初期設定
bno = BNO055()
time.sleep(0.5)
if not bno.begin():
    print("bnoが始まりませんでした")
    exit()
time.sleep(0.5)
bno.setMode(BNO055.OPERATION_MODE_NDOF)
time.sleep(0.5)

# 補正用
t_fine = 0.0

digT = []
digP = []
digH = []

# I2C設定
i2c = smbus.SMBus(1)
address = 0x76

BME280.init_bme280()
BME280.read_compensate()

# キャリブレーション
while True:
    sys, gyro, accel, mag = bno.getCalibration()
    print(f"Calib → Sys:{sys}, Gyro:{gyro}, Acc:{accel}, Mag:{mag}", end='\r\n')
    if gyro == 3 and accel == 3:
        print("キャリブレーション完了")
        break

# GPS
TX_PIN = 27
RX_PIN = 17
BAUD = 9600

pi = pigpio.pi()
if not pi.connected:
    print("pigpio デーモンに接続できません。")
    exit(1)

err = pi.bb_serial_read_open(RX_PIN, BAUD, 8)
if err != 0:
    print(f"ソフトUART RX の設定に失敗：GPIO={RX_PIN}, {BAUD}bps")
    pi.stop()
    exit(1)

print(f"▶ ソフトUART RX を開始：GPIO={RX_PIN}, {BAUD}bps")
lat = 0
lon = 0

#--- 変更点 1: 計測開始時刻を記録 ---
start_time = time.time()

# 加速度測定
while True:
    ax, ay, az = bno.getVector(BNO055.VECTOR_ACCELEROMETER)
    squ_a = ax ** 2 + ay ** 2 + az ** 2
    size_a = math.sqrt(squ_a)
    #print(f"総加速度の大きさ：{size_a}m/s^2")
    time.sleep(0.2)
    (count, data) = pi.bb_serial_read(RX_PIN)
    if count and data:
        text = data.decode("ascii", errors="ignore")
        if "$GNRMC" in text:
            lines = text.split("\n")
            for line in lines:
                if "$GNRMC" in line:
                    parts = line.strip().split(",")
                    if len(parts) > 6 and parts[2] == "A":
                        lat = convert_to_decimal(parts[3], parts[4])
                        lon = convert_to_decimal(parts[5], parts[6])
                        #print(f"緯度{lat}°, 経度{lon}°")
    s = bno.getVector(BNO055.VECTOR_EULER)
    s1 = s[0]
    s2 = s[1]
    s3 = s[2]
    q, w, e = BME280.read_data()
    if lat is None: # is None を使うのが一般的です
        lat = 0
    if lon is None: # is None を使うのが一般的です
        lon = 0

    #--- 変更点 2: 経過時間を計算 ---
    elapsed_time = time.time() - start_time

    #--- 変更点 3: print文に経過時間を追加 ---
    print(f"time:{elapsed_time: >5.1f}s, a:{str(size_a)[:4]}, t:{str(q)[:4]}, p:{str(w)[:4]}, h:{str(e)[:4]}, lat:{str(lat)[:4]}, lon:{str(lon)[:4]}, 9軸:{str(s1)[:3]}, {str(s2)[:3]}, {str(s3)[:3]}")

    time.sleep(0.2)
