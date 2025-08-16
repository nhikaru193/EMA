import cv2
import numpy as np
import time
from picamera2 import Picamera2
from motor import MotorDriver
import camera
import following
from BNO055 import BNO055
import math
from collections import deque
import pigpio


  
picam2 = Picamera2()
config = picam2.create_still_configuration(main={"size": (320, 480)})
picam2.configure(config)
picam2.start()
time.sleep(1)

lower_red1 = np.array([0, 100, 100])
upper_red1 = np.array([10, 255, 255])
lower_red2 = np.array([160, 100, 100])
upper_red2 = np.array([180, 255, 255])
        
        
def get_percentage(frame):
    # 画像の前処理
    frame = cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)
    frame = cv2.GaussianBlur(frame, (5, 5), 0)
    frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    # 赤色マスクの生成
    mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
    mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
    mask = cv2.bitwise_or(mask1, mask2)
    
    # 赤色部分の抽出
    masked_frame = cv2.bitwise_and(frame, frame, mask=mask)
    
    # マスクしたフレームの表示
    cv2.imshow("Red Masked Frame", masked_frame)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    
    # 既存の割合計算
    red_area = np.count_nonzero(mask)
    total_area = frame.shape[0] * frame.shape[1]
    percentage = (red_area / total_area) * 100
    print(f"検知割合は{percentage}%です")
    return percentage

def get_block_number_by_density(frame):
    frame = cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)
    frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
    frame = cv2.GaussianBlur(frame, (5, 5), 0)
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
    mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
    mask = cv2.bitwise_or(mask1, mask2)
    height, width = mask.shape
    block_width = width // 5
    red_ratios = []
    for i in range(5):
        x_start = i * block_width
        x_end = (i + 1) * block_width if i < 4 else width
        block_mask = mask[:, x_start:x_end]
        red_count = np.count_nonzero(block_mask)
        total_count = block_mask.size
        ratio = red_count / total_count
        red_ratios.append(ratio)
    for i, r in enumerate(red_ratios):
        print(f"[DEBUG] ブロック{i+1}の赤密度: {r:.2%}")
    max_ratio = max(red_ratios)
    if max_ratio < 0.08:
        print("❌ 赤色が検出されません（全ブロックで密度低）")
        return None  # 全体的に赤が少なすぎる場合
    else:
        print(f"一番密度の高いブロックは{red_ratios.index(max_ratio) + 1}です")
        return red_ratios.index(max_ratio) + 1
    
frame = picam2.capture_array()
get_percentage(frame)
time.sleep(0.1)
picam2.close()
print("カメラを閉じました。")
print("ゴール判定")
