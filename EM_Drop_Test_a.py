import time
import smbus
import struct
import BME280
from BNO055 import BNO055

#------BNO055------#
bno = BNO055()
bno.begin()
time.sleep(0.5)
bno.setMode(BNO055.OPERATION_MODE_NDOF)
time.sleep(1)
bno.setExternalCrystalUse(True)
#------------------#

#------BME280------#
t_fine = 0.0
digT = []
digP = []
digH = []
i2c = smbus.SMBus(1)
address = 0x76
BME280.init_bme280()
BME280.read_compensate()
#------------------#

#------BNO055のキャリブレーション------#
while True:
    sys, gyro, accel, mag = bno.getCalibration()
    print(f"sys;{sys}, gyro:{gyro}, accel:{accel}, mag:{mag}")
    if gyro == 3 and acc == 3:
        print("BNO055のキャリブレーション終了")
        break
#------------------------------------#

#------測定開始のコード------#
while True:
    start_time = time.time()
    time.sleep(0.2)
    current_time = time.time()
    delta_time = current_time - start_time
    pressure = BME280.get_pressure()
    ax, ay, az = bno.getVector(VECTOR_ACCELEROMETER)
    print(f"t:{delta_time} || p:{pressure} || ax:{ax} || ay:{ay} || az:{az}")
#---------------------------#
