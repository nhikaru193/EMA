import math
import time
import RPi.GPIO as GPIO
import cv2
import numpy as np
from picamera2 import Picamera2
import color

def goal_detective_color():
    # カメラ初期化と設定
    color.init_camera()
    
    try:
        while True:
            #関数定義
            percentage = color.get_percentage()
            
            # 判定出力
            print(f"🔴 赤割合: {percentage:.2f}% → ", end="")
    
            #画面場所検知
            number = color.get_block_number()
            
            if percentage >= 10.0:
                 print("非常に近い（終了）")
                 break
              
            elif percentage >= 5.0:
                 print("近い")
              
            elif percentage >= 2.0:
                 print("遠い")
    
            else: 
                print("範囲外")

                    #割合取得
            percentage = color.get_percentage()
                    
            if percentage >= 2.0:
               print("遠い")
               break               
                      
    finally:
        picam2.close()
        print("カメラを閉じました。プログラム終了。")
