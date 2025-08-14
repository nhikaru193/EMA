import RPi.GPIO as GPIO
import time
import struct
import pigpio

t_melt = 20

meltPin = 22
pi = pigpio.pi()
pi.write(meltPin, 0)
time.sleep(1)
print(f"ワイヤレスグラウンドをオフにします")
pi.write(meltPin, 1)
time.sleep(t_melt)
pi.write(meltPin, 0)
time.sleep(1)
print("終了します")

pi.stop()
