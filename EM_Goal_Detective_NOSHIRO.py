import cv2
import numpy as np
import time
from picamera2 import Picamera2
from motor import MotorDriver
import color
import following
from BNO055 import BNO055

def get_percentage():
    frame = picam2.capture_array()
    frame = cv2.cvtColor(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)
    frame = cv2.GaussianBlur(frame, (5, 5), 0)
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    lower_red1 = np.array([0, 30, 30])
    upper_red1 = np.array([20, 255, 255])
    lower_red2 = np.array([95, 30, 30])
    upper_red2 = np.array([130, 255, 255])
    mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
    mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
    mask = cv2.bitwise_or(mask1, mask2)
    red_area = np.count_nonzero(mask)
    total_area = frame.shape[0] * frame.shape[1]
    percentage = (red_area / total_area) * 100
    return percentage

#赤色面積の重心がどこにあたるか(画面を左から5分割:左から1→5)
def get_block_number():
    number = None
    frame = picam2.capture_array()
    frame = cv2.cvtColor(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)
    frame = cv2.GaussianBlur(frame, (5, 5), 0)
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    lower_red1 = np.array([0, 30, 30])
    upper_red1 = np.array([20, 255, 255])
    lower_red2 = np.array([95, 30, 30])
    upper_red2 = np.array([130, 255, 255])
    mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
    mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
    mask = cv2.bitwise_or(mask1, mask2)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if contours:
        largest = max(contours, key=cv2.contourArea)
        M = cv2.moments(largest)
        if M["m00"] != 0:
            cx = int(M["m10"] / M["m00"])  # x座標の重心
            width = frame.shape[1]
            w = width // 5  # 5分割幅
            if cx < w:
                number = 1
            elif cx < 2 * w:
                number = 2
            elif cx < 3 * w:
                number = 3
            elif cx < 4 * w:
                number = 4
            else:
                number = 5
        else:
            print("⚠️ 重心が計算できません")
    else:
        print("❌ 赤色物体が見つかりません")
    return number
    
#モータの初期化
driver = MotorDriver(
    PWMA=12, AIN1=23, AIN2=18,   # 左モーター用（モータA）
    PWMB=19, BIN1=16, BIN2=26,   # 右モーター用（モータB）
    STBY=21                      # STBYピン
)

#BNO055の初期設定
bno = BNO055()
bno.begin()
time.sleep(1)
bno.setMode(BNO055.OPERATION_MODE_NDOF)
time.sleep(1)
bno.setExternalCrystalUse(True)

# カメラ初期化と設定
#color.init_camera()
picam2 = Picamera2()
config = picam2.create_still_configuration(main={"size": (320, 240)})
picam2.configure(config)
picam2.start()
time.sleep(1)

#速度定義
Va = 0
Vb = 0
Va1 = 0
Va2 = 0
Vb1 = 0
Vb2 = 0

try:
    
    print("対象物を画面内に収める")
    #画面内に映っていない場合に探す
    while True:
        percentage = get_percentage()

        if percentage > 5:
            break

        else:
            driver.changing_right(0, 60)
            driver.changing_right(60, 0)

    
    print("対象物を画面中央に収める")
    #画面中央(横に五分割した中央)に収める
    while True:
        number = get_block_number()
        
        if number == 1:
            driver.changing_left(0, 60)
            driver.changing_left(60, 0)

        elif number == 2:
            driver.changing_left(0, 45)
            driver.changing_left(45, 0)

        elif number == 3:
            break
        
        elif number == 4:
            driver.changing_right(0, 45)
            driver.changing_right(45, 0)
            
        else:
            driver.changing_right(0, 60)
            driver.changing_right(60, 0)
    

    print("ゴール誘導を開始します")
    #画面中央に写してからの誘導(画面外へ出ることはないと想定)
    while True:
        #画面割合、場所検知
        percentage = get_percentage()
        number = get_block_number()
        
        # 判定出力
        print(f"赤割合: {percentage:2f}%-----画面場所:{number}です ")

        if number == 3:
            if percentage > 50:
                break

            elif percentage > 20:
                Va2 = 40
                Vb2 = 40
                following.follow_forward(driver, bno, 50, 3)
                Va1 = Va2
                Vb1 = Vb2
                
            elif percentage > 10:
                Va2 = 60
                Vb2 = 60
                following.follow_forward(driver, bno, 60, 3)
                Va1 = Va2
                Vb1 = Vb2

            else:
                Va2 = 80
                Vb2 = 80
                #driver.changing_forward(Vb1, Vb2)
                following.follow.forward(driver, bno, 80, 3)
                Va1 = Va2
                Vb1 = Vb2

        elif number == 1:
            Va2 = 0
            Vb2 = 70
            driver.changing_Lforward(Va1, Va2)
            driver.changing_Rforward(Vb1, Vb2)
            Va1 = Va2
            Vb1 = Vb2

        elif number == 2:
            Va2 = 0
            Vb2 = 50
            driver.changing_Lforward(Va1, Va2)
            driver.changing_Rforward(Vb1, Vb2)
            Va1 = Va2
            Vb1 = Vb2
        
        elif number == 4:
            Va2 = 50
            Vb2 = 0
            driver.changing_Lforward(Va1, Va2)
            driver.changing_Rforward(Vb1, Vb2)
            Va1 = Va2
            Vb1 = Vb2
            
        elif number == 5:
            Va2 = 70
            Vb2 = 0
            driver.changing_Lforward(Va1, Va2)
            driver.changing_Rforward(Vb1, Vb2)
            Va1 = Va2
            Vb1 = Vb2
                
finally:
    picam2.close()
    print("カメラを閉じました。")
    print("ゴール判定")
    driver.cleanup()
    print("GPIOクリーンアップが終了しました。プログラムを終了します")
