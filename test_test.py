import time
import serial
from motor import MotorDriver
import RPi.GPIO as GPIO
from picamera2 import Picamera2
import math
import numpy
from C_GOAL_DETECTIVE_NOSHIRO import GDN
import following
import cv2
import camera

bno = BNO055()
bno.begin()
time.sleep(1)
bno.setMode(BNO055.OPERATION_MODE_NDOF)
time.sleep(1)
bno.setExternalCrystalUse(True)

GDinN = GDN(bno)
GDinN.run()
