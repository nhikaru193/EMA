import smbus
import time
import struct
import math
import BME280
from BNO055 import BNO055

#BNO055の初期設定
bno = BNO055()
time.sleep(0.5)
if not bno.begin():    
    print("bnoが始まりませんでした")
    exit()
time.sleep(0.5)
bno.setMode(BNO055.OPERATION_MODE_NDOF)
time.sleep(0.5)

#補正用
t_fine = 0.0

digT = []
digP = []
digH = []

#I2C設定
i2c = smbus.SMBus(1)
address = 0x76

BME280.init_bme280()

#キャリブレーション
while True:
    sys, gyro, accel, mag = bno.getCalibration()
    print(f"Calib → Sys:{sys}, Gyro:{gyro}, Acc:{accel}, Mag:{mag}", end='\r')
    if gyro == 3 and accel == 3:
        print("キャリブレーション完了")
        break

#加速度測定
while True:
    ax, ay, az = bno.getVector(BNO055.VECTOR_ACCELEROMETER)
    squ_a = ax ** 2 + ay ** 2 + az ** 2
    size_a = math.sqrt(squ_a)
    print(f"総加速度の大きさ：{size_a}m/s^2")
    time.sleep(0.2)
    BME280.read_data()
    time.sleep(0.5)



