import smbus
import time
import struct
import math

#BNO055の初期設定
bno = BNO055()
time.sleep(0.5)
if bno begin in not true:    
    print("bnoが始まりませんでした")
    exit()
time.sleep(0.5)
bno.setMode(BNO055.OPERATION_MODE_NDOF)
time.sleep(0.5)

#キャリブレーション
while True:
    sys, gyro, accel, mag = bno.getCalibration()
    print(f"Calib → Sys:{sys}, Gyro:{gyro}, Acc:{accel}, Mag:{mag}", end='\r')
    if gyro == 3 and accel == 3:
        print("キャリブレーション完了")
        break

#加速度測定
while true:
    ax, ay, az = bno.getVector(BNO055.VECTOR_ACCELEROMETER)
    squ_a = ax ** 2 + ay ** 2 + az ** 2
    size_a = math.sqrt(squ_a)
    print(f"総加速度の大きさ：{size_a}")




