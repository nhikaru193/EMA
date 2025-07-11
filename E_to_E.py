import math
import time
import pigpio
import RPi.GPIO as GPIO
import threading
import sys 

# 各機能をモジュールとしてインポート
from motor import MotorDriver          # motor.py から MotorDriver クラスをインポート
from bno055_sensor import BNO055       # bno055_sensor.py から BNO055 クラスをインポート
import EM_GPS_datalink                 # EM_GPS_datalink.py (GPS通信とスタック検知)
import EM_stuck                        # EM_stuck.py (スタック回復)

# --- インスタンス生成 ---
# GPIOピン番号を正しく指定
driver = MotorDriver(
    PWMA=12, AIN1=23, AIN2=18,    # 左モーター用（モータA）
    PWMB=19, BIN1=16, BIN2=26,    # 右モーター用（モータB）
    STBY=21                       # STBYピン
)
bno = BNO055() # BNO055のインスタンス生成

# === pigpio 初期化 ===
TX_PIN = 17
RX_PIN = 27
BAUD = 9600

pi = pigpio.pi()
if not pi.connected:
    print("pigpio デーモンに接続できません。")
    sys.exit(1) 

err = pi.bb_serial_read_open(RX_PIN, BAUD, 8)
if err != 0:
    print(f"ソフトUART RX の設定に失敗：GPIO={RX_PIN}, {BAUD}bps", file=sys.stderr)
    pi.stop()
    sys.exit(1)

print(f"▶ ソフトUART RX を開始：GPIO={RX_PIN}, {BAUD}bps")


# --- メインモジュールで管理するグローバル変数と同期オブジェクト ---
is_gps_communication_active = False 
gps_data_lock = threading.Lock() 
current_gps_data = None 

is_rover_stuck = False          
is_recovery_in_progress = False 
last_gps_update_time = time.time() 
last_known_gps_position = None  


# === 各ミッションステップ関数 ===
def getEM_release():
    print("\n[ステップ] 放出判定を行います")
    time.sleep(1) 
    print("[ステップ] 放出判定完了。")

def getEM_land():
    print("\n[ステップ] 着地判定を行います")
    time.sleep(1)
    print("[ステップ] 着地判定完了。")

def getparakai(timeout=60): 
    print("\n[ステップ] パラシュート回避を開始します")
    start_time = time.time()
    
    try:
        while time.time() - start_time < timeout:
            if time.time() % 10 < 2: # 約20%の確率で早期成功とみなす
                print("[ステップ] パラシュート回避完了。")
                return True 

            print(f"[ステップ] パラシュート回避処理中... 経過時間: {int(time.time() - start_time)}秒")
            time.sleep(1) 
        
        print(f"[ステップ] パラシュート回避がタイムアウトしました（{timeout}秒）。")
        return False 

    except Exception as e:
        print(f"[ステップ] パラシュート回避中にエラーが発生しました: {e}", file=sys.stderr)
        return False 

def getEM_excellent_gps():
    print("\n[ステップ] GPS誘導を開始し、第1フラッグまで移動します")
    # この関数が失敗する可能性を考慮し、ランダムな失敗を追加
    if time.time() % 15 < 3: # 約20%の確率で失敗
        raise RuntimeError("GPS誘導中に予期せぬエラーが発生しました。")
    time.sleep(3)
    print("[ステップ] 第1フラッグ到達。")
    return True # 成功した場合はTrueを返す

def getEM_Flag_Navigate():
    print("\n[ステップ] フラッグ誘導を開始します")
    if time.time() % 12 < 2: # 約16%の確率で失敗
        raise RuntimeError("フラッグ誘導中にセンサーエラーが発生しました。")
    time.sleep(2)
    print("[ステップ] フラッグ誘導完了。")
    return True

def getcamera():
    print("\n[ステップ] 物資を設置します")
    if time.time() % 18 < 2: # 約11%の確率で失敗
        raise RuntimeError("カメラ/物資設置機構に問題が発生しました。")
    time.sleep(2.5)
    print("[ステップ] 物資設置完了。")
    return True

def getmotor():
    print("\n[ステップ] ゴールまでGPS誘導させます")
    if time.time() % 20 < 4: # 約20%の確率で失敗
        raise RuntimeError("最終GPS誘導中にモーター異常が発生しました。")
    time.sleep(4)
    print("[ステップ] ゴール付近に到達。")
    return True

def getEM_Goal_Detective_NOSHIRO():
    print("\n[ステップ] ゴール検知を開始します")
    if time.time() % 10 < 1: # 約10%の確率で失敗
        raise RuntimeError("ゴール検知センサーが応答しません。")
    time.sleep(1)
    print("[ステップ] ゴール検知完了。")
    return True


# --- メインシーケンス ---
def main_sequence():
    global is_gps_communication_active
    global is_rover_stuck 
    global is_recovery_in_progress
    global last_gps_update_time
    global last_known_gps_position
    global current_gps_data

    gps_monitor_thread = None
    
    MISSION_PHASE_TIMEOUT = 300 
    MAX_STEP_ATTEMPTS = 3 # 各ステップの最大再試行回数

    print("--- 制御シーケンス開始 ---")

    try:
        # 他のモジュールに、main.pyで定義した共有変数を参照渡しで設定
        EM_GPS_datalink.set_pigpio_globals(pi, RX_PIN, BAUD)
        EM_GPS_datalink.set_datalink_globals(
            is_active_ref=is_gps_communication_active,
            gps_lock_ref=gps_data_lock,
            current_gps_data_ref=current_gps_data,
            is_stuck_ref=is_rover_stuck, 
            is_recovery_ref=is_recovery_in_progress, 
            last_update_time_ref=last_gps_update_time,
            last_pos_ref=last_known_gps_position
        )
        EM_stuck.set_motor_driver_instance(driver_ref=driver) 

        getEM_release()
        
        print("\nGPS通信とスタック検知を開始します")
        EM_GPS_datalink._is_gps_communication_active = True 
        gps_monitor_thread = threading.Thread(target=EM_GPS_datalink.EM_GPS_datalink)
        gps_monitor_thread.start()
        
        time.sleep(0.5) 
        with gps_data_lock:
            if current_gps_data:
                print(f"  (メイン: GPS通信開始直後のデータ: {current_gps_data})")
            else:
                print("  (メイン: GPSデータがまだ取得できていません)")

        getEM_land()

        mission_phase_start_time = time.time()
        
        mission_phase_completed = False
        while not mission_phase_completed and \
              (time.time() - mission_phase_start_time < MISSION_PHASE_TIMEOUT):
            
            # --- 各ステップ実行前のスタックチェック ---
            if EM_GPS_datalink._is_rover_stuck: 
                print("メイン: 現在スタック状態です。回復処理を実行します。")
                if not EM_stuck.EM_stuck(): 
                    print("致命的エラー: スタック回復に失敗したため、シーケンスを中断します。")
                    return 
                # スタック回復成功後、再度このwhileループの先頭に戻り、次のステップに進む前に再度スタックチェックを行う

            # =========================================================================
            # 各ミッションステップに再試行とスタック回復ロジックを追加
            # =========================================================================

            # --- パラシュート回避処理 (既存のロジック) ---
            if not getattr(main_sequence, 'parakai_done', False):
                parakai_attempts = 0
                while not getattr(main_sequence, 'parakai_done', False) and parakai_attempts < MAX_STEP_ATTEMPTS:
                    print(f"\n--- パラシュート回避処理 (試行 {parakai_attempts + 1}/{MAX_STEP_ATTEMPTS}) ---")
                    parakai_succeeded = getparakai(timeout=30) 
                    
                    if parakai_succeeded:
                        print("メイン: パラシュート回避処理が正常に完了しました。")
                        main_sequence.parakai_done = True 
                        break 
                    else: 
                        print("メイン: パラシュート回避処理が完了しなかった、またはタイムアウトしました。")
                        parakai_attempts += 1 

                        if EM_GPS_datalink._is_rover_stuck: 
                            print("メイン: タイムアウト後スタック検出。回復を試みます。")
                            if not EM_stuck.EM_stuck():
                                print("致命的エラー: スタック回復失敗。シーケンス中断。")
                                return 
                            else:
                                print("メイン: スタック回復成功。パラシュート回避を再試行します。")
                        else:
                            print("メイン: タイムアウトしたがスタックはなし。パラシュート回避を再試行します。")
                    
                    if parakai_attempts >= MAX_STEP_ATTEMPTS:
                        print(f"メイン: パラシュート回避の最大再試行回数 ({MAX_STEP_ATTEMPTS}) を超えました。次のステップへ進みます。")
                        main_sequence.parakai_done = True 

                # パラシュート回避後のスタックチェック (保険)
                if getattr(main_sequence, 'parakai_done', False) and EM_GPS_datalink._is_rover_stuck:
                    print("メイン: パラシュート回避ステップ後、スタック検出。回復を試みます。")
                    if not EM_stuck.EM_stuck():
                        print("致命的エラー: スタック回復失敗。シーケンス中断。")
                        return

            # --- GPS誘導（第1フラッグまで） ---
            if getattr(main_sequence, 'parakai_done', False) and \
               not getattr(main_sequence, 'excellent_gps_done', False):
                excellent_gps_attempts = 0
                while not getattr(main_sequence, 'excellent_gps_done', False) and excellent_gps_attempts < MAX_STEP_ATTEMPTS:
                    print(f"\n--- GPS誘導 (第1フラッグ) (試行 {excellent_gps_attempts + 1}/{MAX_STEP_ATTEMPTS}) ---")
                    try:
                        getEM_excellent_gps()
                        main_sequence.excellent_gps_done = True
                        print("メイン: GPS誘導 (第1フラッグ) 正常に完了しました。")
                        break # 成功したのでループを抜ける
                    except Exception as e:
                        print(f"メイン: GPS誘導 (第1フラッグ) 中にエラーが発生しました: {e}", file=sys.stderr)
                        excellent_gps_attempts += 1

                        if EM_GPS_datalink._is_rover_stuck: 
                            print("メイン: エラー後スタック検出。回復を試みます。")
                            if not EM_stuck.EM_stuck():
                                print("致命的エラー: スタック回復失敗。シーケンス中断。")
                                return 
                            else:
                                print("メイン: スタック回復成功。GPS誘導 (第1フラッグ) を再試行します。")
                        else:
                            print("メイン: スタックはなし。GPS誘導 (第1フラッグ) を再試行します。")
                    
                    if excellent_gps_attempts >= MAX_STEP_ATTEMPTS:
                        print(f"メイン: GPS誘導 (第1フラッグ) の最大再試行回数 ({MAX_STEP_ATTEMPTS}) を超えました。次のステップへ進みます。")
                        main_sequence.excellent_gps_done = True # 強制的に完了とマーク
                
                if getattr(main_sequence, 'excellent_gps_done', False) and EM_GPS_datalink._is_rover_stuck:
                    print("メイン: GPS誘導 (第1フラッグ) 後、スタック検出。回復を試みます。")
                    if not EM_stuck.EM_stuck():
                        print("致命的エラー: スタック回復失敗。シーケンス中断。")
                        return

            # --- フラッグ誘導 ---
            if getattr(main_sequence, 'excellent_gps_done', False) and \
               not getattr(main_sequence, 'flag_navigate_done', False):
                flag_navigate_attempts = 0
                while not getattr(main_sequence, 'flag_navigate_done', False) and flag_navigate_attempts < MAX_STEP_ATTEMPTS:
                    print(f"\n--- フラッグ誘導 (試行 {flag_navigate_attempts + 1}/{MAX_STEP_ATTEMPTS}) ---")
                    try:
                        getEM_Flag_Navigate()
                        main_sequence.flag_navigate_done = True
                        print("メイン: フラッグ誘導 正常に完了しました。")
                        break
                    except Exception as e:
                        print(f"メイン: フラッグ誘導中にエラーが発生しました: {e}", file=sys.stderr)
                        flag_navigate_attempts += 1

                        if EM_GPS_datalink._is_rover_stuck: 
                            print("メイン: エラー後スタック検出。回復を試みます。")
                            if not EM_stuck.EM_stuck():
                                print("致命的エラー: スタック回復失敗。シーケンス中断。")
                                return 
                            else:
                                print("メイン: スタック回復成功。フラッグ誘導を再試行します。")
                        else:
                            print("メイン: スタックはなし。フラッグ誘導を再試行します。")
                    
                    if flag_navigate_attempts >= MAX_STEP_ATTEMPTS:
                        print(f"メイン: フラッグ誘導の最大再試行回数 ({MAX_STEP_ATTEMPTS}) を超えました。次のステップへ進みます。")
                        main_sequence.flag_navigate_done = True 

                if getattr(main_sequence, 'flag_navigate_done', False) and EM_GPS_datalink._is_rover_stuck:
                    print("メイン: フラッグ誘導後、スタック検出。回復を試みます。")
                    if not EM_stuck.EM_stuck():
                        print("致命的エラー: スタック回復失敗。シーケンス中断。")
                        return

            # --- 物資設置 ---
            if getattr(main_sequence, 'flag_navigate_done', False) and \
               not getattr(main_sequence, 'camera_done', False):
                camera_attempts = 0
                while not getattr(main_sequence, 'camera_done', False) and camera_attempts < MAX_STEP_ATTEMPTS:
                    print(f"\n--- 物資設置 (試行 {camera_attempts + 1}/{MAX_STEP_ATTEMPTS}) ---")
                    try:
                        getcamera()
                        main_sequence.camera_done = True
                        print("メイン: 物資設置 正常に完了しました。")
                        break
                    except Exception as e:
                        print(f"メイン: 物資設置中にエラーが発生しました: {e}", file=sys.stderr)
                        camera_attempts += 1

                        if EM_GPS_datalink._is_rover_stuck: 
                            print("メイン: エラー後スタック検出。回復を試みます。")
                            if not EM_stuck.EM_stuck():
                                print("致命的エラー: スタック回復失敗。シーケンス中断。")
                                return 
                            else:
                                print("メイン: スタック回復成功。物資設置を再試行します。")
                        else:
                            print("メイン: スタックはなし。物資設置を再試行します。")
                    
                    if camera_attempts >= MAX_STEP_ATTEMPTS:
                        print(f"メイン: 物資設置の最大再試行回数 ({MAX_STEP_ATTEMPTS}) を超えました。次のステップへ進みます。")
                        main_sequence.camera_done = True 
                
                if getattr(main_sequence, 'camera_done', False) and EM_GPS_datalink._is_rover_stuck:
                    print("メイン: 物資設置後、スタック検出。回復を試みます。")
                    if not EM_stuck.EM_stuck():
                        print("致命的エラー: スタック回復失敗。シーケンス中断。")
                        return

            # --- ゴールまでGPS誘導 ---
            if getattr(main_sequence, 'camera_done', False) and \
               not getattr(main_sequence, 'motor_done', False):
                motor_attempts = 0
                while not getattr(main_sequence, 'motor_done', False) and motor_attempts < MAX_STEP_ATTEMPTS:
                    print(f"\n--- ゴールまでGPS誘導 (試行 {motor_attempts + 1}/{MAX_STEP_ATTEMPTS}) ---")
                    try:
                        getmotor()
                        main_sequence.motor_done = True
                        print("メイン: ゴールまでGPS誘導 正常に完了しました。")
                        break
                    except Exception as e:
                        print(f"メイン: ゴールまでGPS誘導中にエラーが発生しました: {e}", file=sys.stderr)
                        motor_attempts += 1

                        if EM_GPS_datalink._is_rover_stuck: 
                            print("メイン: エラー後スタック検出。回復を試みます。")
                            if not EM_stuck.EM_stuck():
                                print("致命的エラー: スタック回復失敗。シーケンス中断。")
                                return 
                            else:
                                print("メイン: スタック回復成功。ゴールまでGPS誘導を再試行します。")
                        else:
                            print("メイン: スタックはなし。ゴールまでGPS誘導を再試行します。")
                    
                    if motor_attempts >= MAX_STEP_ATTEMPTS:
                        print(f"メイン: ゴールまでGPS誘導の最大再試行回数 ({MAX_STEP_ATTEMPTS}) を超えました。次のステップへ進みます。")
                        main_sequence.motor_done = True 
                
                if getattr(main_sequence, 'motor_done', False) and EM_GPS_datalink._is_rover_stuck:
                    print("メイン: ゴールまでGPS誘導後、スタック検出。回復を試みます。")
                    if not EM_stuck.EM_stuck():
                        print("致命的エラー: スタック回復失敗。シーケンス中断。")
                        return

            # --- ゴール検知 ---
            if getattr(main_sequence, 'motor_done', False) and \
               not getattr(main_sequence, 'goal_detective_done', False):
                goal_detective_attempts = 0
                while not getattr(main_sequence, 'goal_detective_done', False) and goal_detective_attempts < MAX_STEP_ATTEMPTS:
                    print(f"\n--- ゴール検知 (試行 {goal_detective_attempts + 1}/{MAX_STEP_ATTEMPTS}) ---")
                    try:
                        getEM_Goal_Detective_NOSHIRO()
                        main_sequence.goal_detective_done = True
                        mission_phase_completed = True # 全て完了したらループを抜ける
                        print("メイン: ゴール検知 正常に完了しました。")
                        break
                    except Exception as e:
                        print(f"メイン: ゴール検知中にエラーが発生しました: {e}", file=sys.stderr)
                        goal_detective_attempts += 1

                        # ゴール検知は移動を伴わないことが多いので、スタック回復は通常不要だが、エラー発生時の一般的な処理として残す
                        if EM_GPS_datalink._is_rover_stuck: 
                            print("メイン: エラー後スタック検出。回復を試みます。")
                            if not EM_stuck.EM_stuck():
                                print("致命的エラー: スタック回復失敗。シーケンス中断。")
                                return 
                            else:
                                print("メイン: スタック回復成功。ゴール検知を再試行します。")
                        else:
                            print("メイン: スタックはなし。ゴール検知を再試行します。")
                    
                    if goal_detective_attempts >= MAX_STEP_ATTEMPTS:
                        print(f"メイン: ゴール検知の最大再試行回数 ({MAX_STEP_ATTEMPTS}) を超えました。ミッションを完了できませんでした。")
                        main_sequence.goal_detective_done = True # 強制的に完了とマーク
                        mission_phase_completed = False # 完了できなかったのでFalseのまま
                
                # ゴール検知後のスタックチェック (この時点でのスタックは稀だが、保険として)
                if getattr(main_sequence, 'goal_detective_done', False) and EM_GPS_datalink._is_rover_stuck:
                    print("メイン: ゴール検知後、スタック検出。回復を試みます。")
                    if not EM_stuck.EM_stuck():
                        print("致命的エラー: スタック回復失敗。シーケンス中断。")
                        return

            # 各ステップ間に短いスリープを挟むことで、バックグラウンドスレッドが動作する機会を与える
            time.sleep(0.5) 
            
        # whileループを抜けた後の最終チェック
        if not mission_phase_completed:
            print(f"メイン: パラシュート回避以降のミッションフェーズがタイムアウトしました（{MISSION_PHASE_TIMEOUT}秒）。")
            if EM_GPS_datalink._is_rover_stuck:
                print("メイン: タイムアウト時スタック状態でした。回復を試みます。")
                if not EM_stuck.EM_stuck():
                    print("致命的エラー: スタック回復失敗。シーケンス中断。")
                    return
            else:
                print("メイン: タイムアウトしましたが、スタックは検知されませんでした。")
        else:
            print("メイン: パラシュート回避以降のミッションフェーズが正常に完了しました。")

    except Exception as e:
        print(f"メインシーケンスで致命的なエラーが発生しました: {e}", file=sys.stderr)
    finally:
        if gps_monitor_thread and gps_monitor_thread.is_alive():
            print("GPS/スタック検知スレッドを停止中...")
            EM_GPS_datalink._is_gps_communication_active = False 
            gps_monitor_thread.join() 

        if pi.connected:
            print("pigpio シリアルポートをクローズ中...")
            pi.bb_serial_read_close(RX_PIN)
            pi.stop()
            print("pigpio 終了処理完了。")
        
        driver.cleanup()

    print("--- すべてのシーケンスが完了しました。---")

if __name__ == "__main__":
    main_sequence()
