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
from Flag_Detector2 import FlagDetector

#ミッション部分
from C_release import RD
from C_Landing_Detective import LD
from C_PARACHUTE_AVOIDANCE import PA
from C_Flag_Navi import FN
from C_excellent_GPS import GPS
from C_GOAL_DETECTIVE_NOSHIRO import GDN

RELEASE = RD(bno)
RELEASE.run()

LAND = LD(bno)
LAND.run()

FLAG = FN(bno)
FLAG.run()

GOAL = GDN(bno, 30)
GOAL.run()
