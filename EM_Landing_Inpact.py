import smbus
import time
import struct
from BNO055 import BNO055
import math
import BME280
import BNO055

print("bme280の動作確認中")
time.sleep(1)
print("bme280の動作確認中")
time.sleep(1)
print("bme280の動作確認中")

#bme280の初期設定
# BME280関連のグローバル変数
t_fine = 0.0
digT = []
digP = []
digH = []
i2c = smbus.SMBus(1)
address = 0x76 
BME280.init_bme280()
BME280.read_compensate()

for i range (100):
    BME280.read_data()
	time.sleep(0.02)

print("bme280の動作終了中")
time.sleep(1)
print("bme280の動作終了中")
time.sleep(1)
print("bme280の動作終了中")


print("BNO055の動作確認中")
time.sleep(1)
print("BNO055の動作確認中")
time.sleep(1)
print("BNO055の動作確認中")

#BNO055の初期設定
bno = BNO055()
time.sleep(1)
bno.begin()
time.sleep(1)
bno.setmode(BNO055.OPERATION_MODE_NDOF)
bno.setExternalCrystalUse(True)
time.sleep(1)

