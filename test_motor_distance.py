import serial
import time
import pigpio
import 
import math
import RPi.GPIO as GPIO
from motor import MotorDriver  # ユーザーのMotorDriverクラスを使用
from BNO055 import BNO055
import smbus
import struct

pi = pigpio.pi()
if not pi.connected:
    print("pigpio デーモンに接続できません。")
    exit(1)

