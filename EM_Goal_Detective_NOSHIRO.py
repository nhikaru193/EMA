import cv2
import numpy as np
import time
from picamera2 import Picamera2
from motor import MotorDriver
import camera
import following
from BNO055 import BNO055

def get_percentage(frame):
    frame = cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)
    frame = cv2.GaussianBlur(frame, (5, 5), 0)
    frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    #cv2.imwrite("/home/mark1/Pictures/noshiro.jpg", hsv)
    """
    lower_red1 = np.array([0, 100, 100])
    upper_red1 = np.array([10, 255, 255])
    lower_red2 = np.array([160, 100, 100])
    upper_red2 = np.array([180, 255, 255])
    """
    mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
    mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
    mask = cv2.bitwise_or(mask1, mask2)
    
    red_area = np.count_nonzero(mask)
    total_area = frame.shape[0] * frame.shape[1]
    percentage = (red_area / total_area) * 100
    print(f"検知割合は{percentage}%です")
    return percentage

#赤色面積の重心がどこにあたるか(画面を左から5分割:左から1→5)
def get_block_number(frame):
    number = None
    frame = cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)
    frame = cv2.GaussianBlur(frame, (5, 5), 0)
    frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    """
    lower_red1 = np.array([0, 100, 50])
    upper_red1 = np.array([10, 255, 255])
    lower_red2 = np.array([170, 100, 50])
    upper_red2 = np.array([180, 255, 255])
    """
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
            print(f"検知場所は{number}です")
        else:
            print("⚠️ 重心が計算できません")
            number = None
    else:
        print("❌ 赤色物体が見つかりません")
        number = None
    return number

def get_block_number_by_density(frame):
    # 前処理：回転・BGR変換・ぼかし
    frame = cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)
    frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
    frame = cv2.GaussianBlur(frame, (5, 5), 0)
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    """
    lower_red1 = np.array([0, 100, 100])
    upper_red1 = np.array([10, 255, 255])
    lower_red2 = np.array([160, 100, 100])
    upper_red2 = np.array([180, 255, 255])
    """
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

    # デバッグ出力（オプション）
    for i, r in enumerate(red_ratios):
        print(f"[DEBUG] ブロック{i+1}の赤密度: {r:.2%}")

    # 最も赤の密度が高いブロックの番号（1〜5）を返す
    max_ratio = max(red_ratios)
    if max_ratio < 0.05:
        print("❌ 赤色が検出されません（全ブロックで密度低）")
        return None  # 全体的に赤が少なすぎる場合
    else:
        print(f"一番密度の高いブロックは{red_ratios.index(max_ratio) + 1}です")
        return red_ratios.index(max_ratio) + 1

#マスクの作成
lower_red1 = np.array([0, 100, 100])
upper_red1 = np.array([10, 255, 255])
lower_red2 = np.array([160, 100, 100])
upper_red2 = np.array([180, 255, 255])

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

try:
    counter = 20
    print("ゴール誘導を開始します")
    #画面中央に写してからの誘導(画面外へ出ることはないと想定)
    while True:
        #画面割合、場所検知
        #タイムアウト()
        if counter <= 0:
            print("赤コーンが近くにありません。探索を行います")
            counter = 20
            while True:
                #照度条件が悪いかコーンが近くにないため、少し移動する。螺旋移動の一部をイメージ
                print("探索中")
                driver.changing_moving_forward(0, 90, 0, 70)
                time.sleep(2)
                driver.changing_moving_forward(90, 0, 70, 0)
                before_heading = bno.get_heading()
                delta_heading = 20
                #少し移動した場所において全方位のコーン探索を行う。
                while delta_heading > 5:
                    print("この場所でのコーン探索を行います")
                    after_heading = bno.get_heading()
                    delta_heading = min((after_heading -  before_heading) % 360, (before_heading -  after_heading) % 360)
                    frame = picam2.capture_array()
                    time.sleep(0.2)
                    percentage = get_percentage(frame)
                    if percentage > 15:
                        print("赤コーンの探索に成功しました")
                        break
                    print("視野角内にコーンを検知できませんでした。左回頭を行います")
                    driver.petit_left(0, 90)
                    time.sleep(0.3)
                    driver.motor_stop_brake()
                    time.sleep(0.2)
                else:
                    print("付近にはコーンを検知できなかったため、再度探索を行います")
                if percentage > 15:
                    print("15%以上の赤面積を検知したため、探索プログラムを終えます")
                    break
                    
        frame = picam2.capture_array()
        time.sleep(0.2)
        percentage = get_percentage(frame)
        number = get_block_number_by_density(frame)
        time.sleep(0.2)
        # 判定出力
        print(f"赤割合: {percentage:2f}%-----画面場所:{number}です ")

        if percentage >= 90:
            print("percentageでのゴール判定")
            break
        elif number == 3:
            if percentage > 40:
                print("petit_petitを1回実行します")
                driver.petit_petit(1)
                time.sleep(1.0)
                
            elif percentage > 20:
                print("petit_petitを3回実行します")
                driver.petit_petit(3)
                time.sleep(1.0)
                
            elif percentage > 10:
                print("petit_petitを5回実行します")
                driver.petit_petit(5)
                time.sleep(1.0)
                
            else:
                print("距離が遠いため、前身を行います")
                following.follow_forward(driver, bno, 70, 2)

        elif number == 1:
            driver.petit_right(0, 100)
            driver.motor_stop_brake()
            time.sleep(1.0)

        elif number == 2:
            driver.petit_right(0, 90)
            driver.motor_stop_brake()
            time.sleep(1.0)
            if percentage < 50:
                print("正面にとらえることができませんでしたが、検知割合が低いため、接近します")
                following.follow_forward(driver, bno, 70, 1)
            
        elif number == 4:
            driver.petit_left(0, 90)
            driver.motor_stop_brake()
            time.sleep(1.0)
            if percentage < 50:
                print("正面にとらえることができませんでしたが、検知割合が低いため、接近します")
                following.follow_forward(driver, bno, 70, 1)
            
        elif number == 5:
            driver.petit_left(0, 100)
            driver.motor_stop_brake()
            time.sleep(1.0)

        elif number is None:
            driver.petit_left(0, 80)
            driver.petit_left(80, 0)
            driver.motor_stop_brake()
            time.sleep(1.0)
        counter = counter - 1
finally:
    picam2.close()
    print("カメラを閉じました。")
    print("ゴール判定")
    driver.cleanup()
    print("GPIOクリーンアップが終了しました。プログラムを終了します")

