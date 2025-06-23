import cv2
import numpy as np
import time
from picamera2 import Picamera2
from motor import MotorDriver
import color

#モータの初期化
driver = MotorDriver(
    PWMA=12, AIN1=23, AIN2=18,   # 左モーター用（モータA）
    PWMB=19, BIN1=16, BIN2=26,   # 右モーター用（モータB）
    STBY=21                      # STBYピン
)

# カメラ初期化と設定
color.init_camera()

#速度定義
Va = 0
Vb = 0

try:
    while True:
        #関数定義
        percentage = color.get_percentage()
        
        # 判定出力
        print(f"🔴 赤割合: {percentage:.2f}% → ", end="")

        #画面場所検知
        number = color.get_block_number()
        
        if percentage >= 10.0:
             Vb = 0
             print("非常に近い（終了）")
             driver.changing_forward(Va, Vb)
             driver.motor_stop_brake()
             break
          
        elif percentage >= 5.0:
             Vb = 50
             print("近い")
             driver.changing_forward(Va, Vb)
             time.sleep(0.1)
             Va = Vb
          
        elif percentage >= 2.0:
             Vb = 100
             print("遠い")
             driver.changing_forward(Va, Vb)
             time.sleep(0.1)
             Va = Vb

        else: 
            print("範囲外")
            while True:
                driver.changing_forward(Va, 0)
                driver.motor_stop_brake()

                if number == 1:
                    driver.changing_left(0, 15)
                    driver.changing_left(15, 0)

                elif number == 5:
                    driver.changing_right(0, 15)
                    driver.changing_right(15, 0)
                
                #割合取得
                percentage = color.get_percentage()
                
                if percentage >= 2.0:
                   Vb = 50
                   print("遠い")
                   driver.changing_forward(Va, Vb)
                   Va = Vb
                   break               
                  
finally:
    picam2.close()
    print("カメラを閉じました。プログラム終了。")
