import math
import time
import RPi.GPIO as GPIO
import cv2
import numpy as np
from picamera2 import Picamera2
import color # ユーザーが作成したモジュール
import datetime # タイムスタンプのために追加
import os # ディレクトリ作成のために追加

def goal_detective_color_with_capture():
    """
    カメラに映る赤色の割合を検出し、条件に応じてタイムスタンプ付きで画像を保存する。
    """
    picam2 = color.init_camera() 
    
    try:
        while True:
            percentage = color.get_percentage()
            
            print(f"🔴 赤割合: {percentage:.2f}% → ", end="")
            
            if percentage >= 10.0:
                print("非常に近い（終了）")
                
                # --- 画像保存処理 ---
                # 1. 保存先のディレクトリを指定・作成
                save_dir = "captured_images"
                os.makedirs(save_dir, exist_ok=True)

                # 2. 現在時刻を取得し、ファイル名を作成
                now = datetime.datetime.now()
                filename = f"{save_dir}/goal_{now.strftime('%Y%m%d_%H%M%S')}.jpg"

                # 3. 画像をファイルとして保存
                picam2.capture_file(filename)
                print(f"画像を {filename} として保存しました。")
                # --- 画像保存処理ここまで ---
                
                break
            
            elif percentage >= 5.0:
                print("近い")
                
            elif percentage >= 2.0:
                print("遠い")
            
            else:
                print("範囲外")

            time.sleep(0.1)

    finally:
        if 'picam2' in locals() and picam2.is_open:
            picam2.close()
            print("カメラを閉じました。プログラム終了。")
        else:
            print("プログラム終了。")

# --- メイン処理 ---
if __name__ == '__main__':
    try:
        goal_detective_color_with_capture()
    except KeyboardInterrupt:
        print("\nプログラムが中断されました。")
