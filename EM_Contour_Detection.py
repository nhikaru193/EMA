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
from Flag_Detector import FlagDetector
import RPi.GPIO as GPIO

# --- 設定値 ---
TARGET_SHAPES = ["三角形", "長方形", "T字", "十字"] 
AREA_THRESHOLD_PERCENT = 50.0 

def find_target_flag(detected_data, target_name):
    """検出データから指定された図形(target_name)のフラッグを探して返す"""
    for flag in detected_data:
        for shape in flag['shapes']:
            if shape['name'] == target_name:
                return flag
    return None
 
# メインの実行ブロック
if __name__ == '__main__':
    # --- 初期化処理 ---
    detector = FlagDetector()
    driver = MotorDriver(
        PWMA=12, AIN1=23, AIN2=18,    # 左モーター
        PWMB=19, BIN1=16, BIN2=26,    # 右モーター
        STBY=21
    )
    screen_area = detector.width * detector.height
    
    # === BNO055 初期化 ===
    bno = BNO055()
    if not bno.begin():
        print("BNO055の初期化に失敗しました。")
        exit(1)
    time.sleep(1)
    bno.setExternalCrystalUse(True)
    bno.setMode(BNO055.OPERATION_MODE_NDOF)
    time.sleep(1)
    print("センサー類の初期化完了。")

    try:
        # --- 全てのターゲットに対してループ ---
        for target_name in TARGET_SHAPES:
            print(f"\n---====== 新しい目標: [{target_name}] の探索を開始します ======---")
            
            task_completed = False
            while not task_completed:
                
                # --- 探索 ---
                print(f"[{target_name}] を探しています...")
                detected_data = detector.detect()
                target_flag = find_target_flag(detected_data, target_name)

                # 見つからない場合は回転して探索
                if target_flag is None:
                    print(f"[{target_name}] が見つかりません。回転して探索します。")
                    search_count = 0
                    while target_flag is None and search_count < 40: # タイムアウト設定
                        driver.changing_right(0, 55)
                        driver.changing_right(55, 0)
                        time.sleep(0.15)
                        
                        detected_data = detector.detect()
                        target_flag = find_target_flag(detected_data, target_name)
                        search_count += 1
                
                # 回転しても見つからなかったら、このターゲットは諦めて次の輪郭検知　ここむずい　by中川
                if target_flag is None:
                    print(f"探索しましたが [{target_name}] は見つかりませんでした。次の目標に移ります。")
                    break # while not task_completed ループを抜ける

                # --- 追跡（中央寄せ＆接近）---
                print(f"[{target_name}] を発見！追跡を開始します。")
                while target_flag:
                    # --- 中央寄せ ---
                    if target_flag['location'] != '中央':
                        print(f"位置を調整中... (現在位置: {target_flag['location']})")
                        if target_flag['location'] == '左':
                            driver.changing_right(0, 55)
                            driver.changing_right(55, 0)
                            time.sleep(0.15)
                        elif target_flag['location'] == '右':
                            driver.changing_left(0, 55)
                            driver.changing_left(55, 0)
                            time.sleep(0.15)
                          
                        # 動かした直後に再検出
                        print("  再検出中...")
                        detected_data = detector.detect()
                        target_flag = find_target_flag(detected_data, target_name)
                        
                        if not target_flag:
                            print(f"調整中に [{target_name}] を見失いました。")
                            break # 追跡ループを抜ける
                        
                        # 位置を再評価するため、ループの最初に戻る
                        continue
                    
                    # --- 接近 ---
                    else: # 中央にいる場合
                        flag_area = cv2.contourArea(target_flag['flag_contour'])
                        area_percent = (flag_area / screen_area) * 100
                        print(f"中央に補足。接近中... (画面占有率: {area_percent:.1f}%)")

                        # 面積の比較
                        if area_percent >= AREA_THRESHOLD_PERCENT:
                            print(f"[{target_name}] に接近完了！")
                            task_completed = True
                            time.sleep(1)
                            break # 追跡ループを抜ける
                        else:
                            # しきい値未満なら、1秒前進
                            following.follow_forward(driver, bno, 90, 0.04)
                    
                    # 動作後に再検出（正しい位置）
                    print("  再検出中...")
                    detected_data = detector.detect()
                    target_flag = find_target_flag(detected_data, target_name)
                    
                    if not target_flag:
                        print(f"追跡中に [{target_name}] を見失いました。再探索します。")
                        break # 追跡ループ(while target_flag)を抜ける

        print("\n---====== 全ての目標の探索が完了しました ======---")

    finally:
        # --- 終了処理 ---
        print("--- 制御を終了します ---")
        driver.cleanup()
        detector.close()
        GPIO.cleanup() 
        cv2.destroyAllWindows()
