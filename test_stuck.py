import stuck
import cv2
import numpy as np
import time
import camera
import smbus
from picamera2 import Picamera2
import struct
import RPi.GPIO as GPIO
import math
import pigpio

#作成ファイルのインポート
import fusing
import BME280
import following
from BNO055 import BNO055
from motor import MotorDriver
from Flag_B import Flag_B

goal_location = [40.1426175, 139.9876533]
stuck.GPS_navigate(goal_location, bno, driver=None, pi=None)
