import time
import threading
import sys
# 同じディレクトリにあるem_gps_datalink.pyをインポート
from GPS_communication import EmGpsDatalink

# --- 擬似的なデバイス・センサー関数 ---
# 実際の環境に合わせて実装してください

def getEM_release():
    print("放出判定中...")
    time.sleep(1) # 処理に時間がかかることをシミュレート
    print("放出判定完了。")

def getEM_land():
    print("着地判定中...")
    time.sleep(1)
    print("着地判定完了。")

def getparakai():
    print("パラシュート回避中...")
    time.sleep(1.5)
    print("パラシュート回避完了。")

def getEM_excellent_gps():
    print("GPS誘導中（第1フラッグまで）...")
    time.sleep(3)
    print("第1フラッグ到達。")

def getEM_Flag_Navigate():
    print("フラッグ誘導中...")
    time.sleep(2)
    print("フラッグ誘導完了。")

def getcamera():
    print("物資設置中（カメラ操作を伴う）...")
    time.sleep(2.5)
    print("物資設置完了。")

def getmotor():
    print("ゴールまでGPS誘導中（モーター制御を伴う）...")
    time.sleep(4)
    print("ゴール付近に到達。")

def getEM_Goal_Detective_NOSHIRO():
    print("ゴール検知中...")
    time.sleep(1)
    print("ゴール検知完了。")

# --- メインシーケンス ---
def main_sequence():
    print("--- 制御シーケンス開始 ---")

    gps_datalink = None # EmGpsDatalinkのインスタンスを保持する変数

    try:
        print("放出判定を行います")
        getEM_release()
        
        # ★ここから変更★
        # 放出判定後、EmGpsDatalinkのインスタンスを作成し、GPS通信スレッドを開始
        print("放出判定完了。GPS通信の準備を開始します。")
        gps_datalink = EmGpsDatalink(
            rx_pin=17,
            tx_pin=27,
            baud_soft_uart=9600,
            baud_im920=19200,
            wireless_pin=22
        )
        gps_datalink.start() # GPSデータリンクのスレッドを起動
        print("GPS通信の準備が完了し、通信を開始しました。")

        # GPS通信開始後、すぐに最新のGPSデータを取得して表示
        # ただし、スレッド起動直後だとデータがまだ取得できていない場合もあります
        time.sleep(0.5) # GPSスレッドがデータを取得するまで少し待機
        current_gps_data = gps_datalink.get_current_gps()
        if current_gps_data:
            print(f"  (メイン: GPS通信開始直後のデータ: 緯度={current_gps_data['latitude']:.6f}, 経度={current_gps_data['longitude']:.6f})")
        else:
            print("  (メイン: GPS通信開始直後のデータはまだ取得されていません)")
        # ★ここまで変更★

        print("着地判定を行います")
        getEM_land()

        print("パラシュート回避を開始します")
        getparakai()

        print("GPS誘導を開始し、第1フラッグまで移動します")
        getEM_excellent_gps()

        print("フラッグ誘導を開始します")
        getEM_Flag_Navigate()

        print("物資を設置します")
        getcamera()

        print("ゴールまでGPS誘導させます")
        getmotor()

        print("ゴール検知を開始します")
        getEM_Goal_Detective_NOSHIRO()

    except ConnectionRefusedError as e:
        print(f"初期化エラーによりメインシーケンスを終了します: {e}")
        sys.exit(1) # プログラムを終了
    except IOError as e:
        print(f"ハードウェア初期化エラーによりメインシーケンスを終了します: {e}")
        sys.exit(1) # プログラムを終了
    except Exception as e:
        print(f"メインシーケンスで予期せぬエラーが発生しました: {e}")
    finally:
        # シーケンス終了時にGPS通信スレッドを停止し、リソースを解放
        if gps_datalink:
            gps_datalink.stop()

    print("--- すべてのシーケンスが完了しました。---")

if __name__ == "__main__":
    main_sequence()
