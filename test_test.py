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
from BNO055 import BNO055
from C_PARACHUTE_AVOIDANCE import PA
import smbus
import math
import pigpio
import struct
import following
import cv2
import camera

bno = BNO055()
bno.begin()
time.sleep(1)
bno.setMode(BNO055.OPERATION_MODE_NDOF)
time.sleep(1)
bno.setExternalCrystalUse(True)

while True:
    sys, gyro, accel, mag = bno.getCalibration()
    print(f"gyro:{gyro}")
    if gyro == 3:
        print("BNO055のキャリブレーション終了")
        break
      
AVOIDANCE = PA(bno, goal_location = [35.9189971, 139.9085032]) #ok
AVOIDANCE.run()

#完璧でしたGDN
"""
GDinN = GDN(bno)
GDinN.run()
"""
