#デーモンで実行をする
#BNO-BME-motor-L76X-IM920sLの順
import smbus
import time
import struct
from BNO055 import BNO055
import math
import BME280
from BNO055 import BNO055
from motor import MotorDriver
import serial
import pigpio
import RPi.GPIO as GPIO

def convert_to_decimal(coord, direction):
	# 度分（ddmm.mmmm）形式を10進数に変換
	degrees = int(coord[:2]) if direction in ['N', 'S'] else int(coord[:3])
	minutes = float(coord[2:]) if direction in ['N', 'S'] else float(coord[3:])
	decimal = degrees + minutes / 60
	if direction in ['S', 'W']:
		decimal *= -1
	return decimal

def fusing_circuit():
	try:
		NICHROME_PIN = 25
		HEATING_DURATION_SECONDS = 3.0
		GPIO.setmode(GPIO.BCM)
		GPIO.setup(NICHROME_PIN, GPIO.OUT, initial=GPIO.LOW)
		print("ニクロム線溶断シーケンスを開始します。")
		print(f"GPIO{NICHROME_PIN} をHIGHに設定し、ニクロム線をオンにします。")
		GPIO.output(NICHROME_PIN, GPIO.HIGH)
		print(f"{HEATING_DURATION_SECONDS}秒間、加熱します...")
		time.sleep(HEATING_DURATION_SECONDS)
		print(f"GPIO{NICHROME_PIN} をLOWに設定し、ニクロム線をオフにします。")
		GPIO.output(NICHROME_PIN, GPIO.LOW)
		print("シーケンスが正常に完了しました。")
	except KeyboardInterrupt:
		print("プログラムが中断されました。")
		GPIO.output(NICHROME_PIN, GPIO.LOW)
	finally:
		GPIO.cleanup()
		print("GPIOのクリーンアップを実行しました。")
	
#BNO055の初期設定
bno = BNO055()
time.sleep(1)
bno.begin()
time.sleep(1)
bno.setMode(BNO055.OPERATION_MODE_NDOF)
bno.setExternalCrystalUse(True)
time.sleep(1)

#BNO055のキャリブレーション
while True:
	a, gyro, c, d = bno.getCalibration()
	print(f"gyro = {gyro}")
	if gyro == 3:
		print("BNO055センサのgyroキャリブレーションが終了しました")
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

#テグス溶断
fusing_circuit()

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
for i in range (20):
	BME280.read_data()
	time.sleep(0.1)

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

#モータークリーンアップ
driver.motor_stop_brake()
driver.cleanup()

print("モーターの動作終了中")
time.sleep(1)
print("モーターの動作終了中")

#GPS動作確認
print("GPS取得の動作確認中")
time.sleep(1)
print("GPS取得の動作確認中")

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

for i in range(50):
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
						print(lat, lon)

pi.bb_serial_read_close(RX_PIN)
pi.stop()

print("GPS取得の動作終了中")
time.sleep(1)
print("GPS取得の動作終了中")

print("IM920sLの動作確認中")
time.sleep(1)
print("IM920sLの動作確認中")

port = serial.Serial("/dev/ttyS0", baudrate=9600, timeout=1)
for i in range (1, 10):
	msg = f"{i}\r\n"
	port.write(msg.encode("ascii"))
	print(f"{i}番目のデータを送信")

GPIO.cleanup()

print("IM920sLの動作終了中")
time.sleep(1)
print("IM920sLの動作終了中")

	






				


