#デーモンで実行をする
#BNO-BME-motor-L76X-IM920sLの順
import smbus
import time
import struct
from BNO055 import BNO055
import math
import BME280
import BNO055
from motor import MotorDriver

#BNO055の初期設定
bno = BNO055()
time.sleep(1)
bno.begin()
time.sleep(1)
bno.setmode(BNO055.OPERATION_MODE_NDOF)
bno.setExternalCrystalUse(True)
time.sleep(1)

#BNO055のキャリブレーション
while True:
	a, b, gyro, d = bno.getCalibration()
	if gyro == 3
		print(BNO055センサのgyroキャリブレーションが終了しました)
		break

#高度調整の時間設け
print("1.3 mまで持ち上げ落下させてください")
time.sleep(1)

#BNO055の動作確認
print("BNO055の動作確認中")
time.sleep(1)
print("BNO055の動作確認中")

#counterの設定
BNO_counter = 3

#BNO055の測定開始
while True:
	before_heading = bno.get_heading()
	EULER = bno.getVector(BNO055.VECTOR_EULER)
	print(EULER)
	time.sleep(1)
	after_heading = bno.get_heading()
	d_heading = after_heading - before_heading
	delta_heading = abs(d_heading)

	if delta_heading < 5:
		BNO_counter = BNO_counter - 1
	else:
		BNO_counter = 3
		
	if BNO_counter == 0:
		print("－－－－－－－－着地判定－－－－－－－－－")
		break

#方位測定
print("BNO055の動作終了中")
time.sleep(1)
print("BNO055の動作終了中")

#BME280の動作確認
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

#測定データの出力
for i in range (100):
	BME280.read_data()
	time.sleep(0.02)

print("bme280の動作終了中")
time.sleep(1)
print("bme280の動作終了中")

#motor動作確認
print("モーターの動作確認中")
time.sleep(1)
print("モーターの動作確認中")

#モータの初期設定
driver = MotorDriver(
    PWMA=12, AIN1=23, AIN2=18,    # 左モーター
    PWMB=19, BIN1=16, BIN2=26,    # 右モーター
    STBY=21
)

for i in range(2):
	driver.changing_forward(0, 50)
	driver.changing_forward(50, 0)
	driver.changing_left(0, 40)
	driver.changing_left(40, 0)
	driver.changing_right(0, 40)
	driver.changing_right(40, 0)
	driver.changing_retreat(0, 100)
	driver.changing_retreat(100, 0)

print("モーターの動作終了中")
time.sleep(1)
print("モーターの動作終了中")


