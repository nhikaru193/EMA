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
    
    print("対象物を画面内に収める")
    #画面内に映っていない場合に探す
    while True:
        percentage = color.get_percentage()

        if percentage > 5:
            break

        elif:
            driver.changing_right(0, 60)
            driver.changing_right(60, 0)

    
    print("対象物を画面中央に収める")
    #画面中央(横に五分割した中央)に収める
    while true:
        number = color.get_block_number()
        
        if number == 1:
            driver.changing_left(0, 40)
            driver.changing_left(40, 0)

        elif number == 2:
            driver.changing_left(0, 25)
            driver.changing_left(25, 0)

        elif number == 3:
            break
        
        elif number == 4:
            driver.changing_right(0, 25)
            driver.changing_right(25, 0)
            
        else:
            driver.changing_right(0, 40)
            driver.changing_right(40, 0)
    

    #画面中央に写してからの誘導(画面外へ出ることはないと想定)
    while true:
        #画面割合、場所検知
        percentage = color.get_percentage()
        number = color.get_block_number()
        
        # 判定出力
        print(f"赤割合: {percentage:2f}%-----画面場所:{number}です ")

        if number == 3:
            if percentage > 40:
                break

            elif percentage > 20:
                Vb = 40
                driver.changing_forward(Va, Vb)
                Va = Vb

            elif percentage > 10:
                Vb = 60
                driver.changing_forward(Va, Vb)
                Va = Vb

            else:
                Vb = 80
                driver.changing_forward(Va, Vb)
                Va = Vb

        elif number == 1:
            driver.changing_left(0, 40)
            driver.changing_left(40, 0)

        elif number == 2:
            driver.changing_left(0, 25)
            driver.changing_left(25, 0)
        
        elif number == 4:
            driver.changing_right(0, 25)
            driver.changing_right(25, 0)
            
        elif number == 5:
            driver.changing_right(0, 40)
            driver.changing_right(40, 0)
                

finally:
    picam2.close()
    print("カメラを閉じました。")
    print("ゴール判定")
    driver.cleanup()
    print("GPIOクリーンアップが終了しました。プログラムを終了します")
