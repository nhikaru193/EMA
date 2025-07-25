import cv2
import numpy as np
import time
from picamera2 import Picamera2
from motor import MotorDriver
from BNO055 import BNO055
import math
from collections import deque

lower_red1 = np.array([0, 100, 100])
upper_red1 = np.array([10, 255, 255])
lower_red2 = np.array([160, 100, 100])
upper_red2 = np.array([180, 255, 255])

def get_percentage(frame):
    frame = cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)
    frame = cv2.GaussianBlur(frame, (5, 5), 0)
    frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
    mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
    mask = cv2.bitwise_or(mask1, mask2)
    red_area = np.count_nonzero(mask)
    total_area = frame.shape[0] * frame.shape[1]
    percentage = (red_area / total_area) * 100
    print(f"検知割合は{percentage}%です")
    return percentage

picam2 = Picamera2()
config = picam2.create_still_configuration(main={"size": (320, 480)})
picam2.configure(config)
picam2.start()
time.sleep(1)

frame = picam2.capture_array()
per = get_percentage(frame)

picam2.close()
