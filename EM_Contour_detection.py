import time
import smbus
import struct
import following
import cv2
import math
import numpy as np
from picamera2 import Picamera2
from BNO055 import BNO055
from motor import MotorDriver
from Flag_Detection import FlagDetector
import RPi.GPIO as GPIO

# --- 設定値 ---
# 探索する図形の順番
TARGET_SHAPES = ["三角形", "長方形", "T字", "十字"] 
# この割合以上フラッグが画面を占めたら接近完了とする (%)
AREA_THRESHOLD_PERCENT = 20.0 

def find_target_flag(detected_data, target_name):
    """検出データから指定された図形(target_name)のフラッグを探して返す"""
    for flag in detected_data:
        for shape in flag['shapes']:
            if shape['name'] == target_name:
                return flag
    return None
  
  if __name__ == '__main__':
    detector = FlagDetector()
    driver = MotorDriver(
        PWMA=12, AIN1=23, AIN2=18,    # 左モーター
        PWMB=19, BIN1=16, BIN2=26,    # 右モーター
        STBY=21
    )

    screen_area = detector.width * detector.height
