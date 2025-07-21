import smbus
import time
import struct
import math
import BME280
from BNO055 import BNO055
from motor import MotorDriver
import RPi.GPIO as GPIO
import serial
import fusing
import csv
import pigpio

def convert_to_decimal(coord, direction):
    # 度分（ddmm.mmmm）形式を10進数に変換
    degrees = int(coord[:2]) if direction in ['N', 'S'] else int(coord[:3])
    minutes = float(coord[2:]) if direction in ['N', 'S'] else float(coord[3:])
    decimal = degrees + minutes / 60
    if direction in ['S', 'W']:
        decimal *= -1
    return decimal

try:
    #BNO055の初期設定
    bno = BNO055()
    time.sleep(0.5)
    if not bno.begin():
        print("bnoが始まりませんでした")
        exit()
    time.sleep(0.5)
    bno.setMode(BNO055.OPERATION_MODE_NDOF)
    time.sleep(0.5)
    
    #BNO055のキャリブレーション
    while True:
        sys, gyro, accel, mag = bno.getCalibration()
        print(f"Calib → Sys:{sys}, Gyro:{gyro}, Acc:{accel}, Mag:{mag}", end='\r\n')
        if gyro == 3 and accel == 3:
            print("キャリブレーション完了")
            break
    
    #BME280の初期設定
    t_fine = 0.0
    digT = []
    digP = []
    digH = []
    i2c = smbus.SMBus(1)
    address = 0x76
    BME280.init_bme280()
    BME280.read_compensate()
    
    #GPSモジュールの初期設定
    TX_PIN = 27
    RX_PIN = 17
    BAUD = 9600
    pi = pigpio.pi()
    err = pi.bb_serial_read_open(RX_PIN, BAUD, 8)
    lat = 0
    lon = 0
    
    start_time = time.time()
    
    filename = "vibration_test_" + time.strftime("%m%d-%H%M%S") + ".csv"
    f = open(filename,"w")
    writer = csv.writer(f)
    
    #測定開始
    (count, data) = pi.bb_serial_read(RX_PIN)
    while True:
        print("測定を開始します")
        current_time = time.time()
        elapsed_time = current_time - start_time
        ax, ay, az = bno.getVector(BNO055.VECTOR_ACCELEROMETER)
        prs, tmp, hum = BME280.get_data()
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
        print(f"t:{elapsed_time:.2f} | ax:{ax:.2f} | ay:{ay:.2f} | az:{az:.2f} | prs:{prs:.2f} | tmp:{tmp:.2f} | hum:{hum:.2f} | lat:{lat} | lon:{lon}")
        writer.writerows([[elapsed_time, ax, ay, az, prs, tmp, hum, lat, lon]])
        f.flush()
        time.sleep(0.2)
        if elapsed_time > 180:
            print("タイムアウト成功です。テグス溶断に移ります")
            f.close()
            break
    
    #テグス溶断
    fusing.circuit()
    
    #モータの起動
    print("モータの起動を行います")
    driver = MotorDriver(
        PWMA=12, AIN1=23, AIN2=18,
        PWMB=19, BIN1=16, BIN2=26,
        STBY=21
    )
    driver.changing_forward(0, 90)
    driver.changing_forward(90, 0)
    print("モータの起動を終了します")
    
    #無線機の起動
    print("無線通信を開始します")
    im920 = serial.Serial('/dev/serial1', 19200, timeout=1)
    time.sleep(1)
    for i in range(3):
        data = f'{lat, lon}'
        msg = f'TXDA 0003,{data}\r'
        im920.write(msg.encode())
    
    #カメラの起動
    picam2 = Picamera2()
    config = picam2.create_still_configuration(main={"size": (320, 480)})
    picam2.configure(config)
    picam2.start()
    time.sleep(1)
    frame = picam2.capture_array()
    frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
    image_path = "/home/mark1/_Pictures/vibration_test.jpg"
    cv2.imwrite(image_path, frame)

finally:
    driver.cleanup()
    pi.stop()
    err = pi.bb_serial_read_close(RX_PIN)
    picam2.close()
