import RPi.GPIO as GPIO
import time
import socket
from pynput import keyboard
import threading
import serial
import pigpio

# motor.py から MotorDriver クラスをインポート
from motor import MotorDriver

# --- グローバル変数と初期設定 ---
# モーター制御ピン (ご自身の配線に合わせて変更してください)
# 例: L298Nなどのモータードライバーの場合を想定
PWMA = 12  # 左モーターの速度制御 (PWM)
AIN1 = 23  # 左モーターの方向制御1
AIN2 = 18  # 左モーターの方向制御2

PWMB = 19  # 右モーターの速度制御 (PWM)
BIN1 = 16  # 右モーターの方向制御1
BIN2 = 26  # 右モーターの方向制御2

STBY = 21  # スタンバイピン (モーターON/OFF)

# GPS (ソフトUART) 用のピンとボーレート
# ラズパイのGPIO17 (GPSモジュールのTXに接続)
GPS_RX_PIN = 17 
GPS_BAUD = 9600

# IM920SL 用のシリアルポート設定
IM920_PORT = '/dev/serial0' # 通常はハードウェアUART
IM920_BAUD = 19200

# グローバルなインスタンスと状態変数
motor = None
pi = None # pigpioインスタンス
im920 = None # IM920SLシリアルインスタンス
current_action = "stop"
current_speed = 50 # デフォルトの速度 (デューティ比 0-100)
# スレッド終了のためのイベント
exit_event = threading.Event()

# --- GPSデータ処理関数 ---
def convert_to_decimal(coord, direction):
    # 度分（ddmm.mmmm）形式を10進数に変換
    degrees = int(coord[:2]) if direction in ['N', 'S'] else int(coord[:3])
    minutes = float(coord[2:]) if direction in ['N', 'S'] else float(coord[3:])
    decimal = degrees + minutes / 60
    if direction in ['S', 'W']:
        decimal *= -1
    return decimal

# --- GPSとIM920SL通信を担当するスレッド関数 ---
def gps_and_im920_thread():
    global pi, im920

    print("GPS/IM920通信スレッドを開始します...")

    # pigpioデーモンに接続
    pi = pigpio.pi()
    if not pi.connected:
        print("エラー: pigpio デーモンに接続できません。pigpiodが実行されているか確認してください。")
        exit_event.set() # メインスレッドに終了を通知
        return

    # ソフトUART RX の設定
    err = pi.bb_serial_read_open(GPS_RX_PIN, GPS_BAUD, 8)
    if err != 0:
        print(f"エラー: ソフトUART RX の設定に失敗：GPIO={GPS_RX_PIN}, {GPS_BAUD}bps, エラーコード: {err}")
        exit_event.set() # メインスレッドに終了を通知
        pi.stop()
        return

    print(f"▶ ソフトUART RX を開始：GPIO={GPS_RX_PIN}, {GPS_BAUD}bps")

    # IM920SL のシリアルポート設定
    try:
        im920 = serial.Serial(IM920_PORT, IM920_BAUD, timeout=1)
        print(f"IM920SL を開きました: {IM920_PORT} @ {IM920_BAUD}bps")
    except serial.SerialException as e:
        print(f"エラー: IM920SL (Serial) のオープンに失敗しました: {e}")
        exit_event.set() # メインスレッドに終了を通知
        if pi and pi.connected:
            pi.bb_serial_read_close(GPS_RX_PIN)
            pi.stop()
        return

    try:
        while not exit_event.is_set(): # 終了イベントがセットされるまでループ
            (count, data) = pi.bb_serial_read(GPS_RX_PIN)
            if count and data:
                try:
                    text = data.decode("ascii", errors="ignore")
                    if "$GNRMC" in text:
                        lines = text.split("\n")
                        for line in lines:
                            if "$GNRMC" in line:
                                parts = line.strip().split(",")
                                if len(parts) > 6 and parts[2] == "A": # "A"は有効なデータを示す
                                    try:
                                        lat = convert_to_decimal(parts[3], parts[4])
                                        lon = convert_to_decimal(parts[5], parts[6])
                                        # GPSデータをIM920SLで送信
                                        data_to_send = f'{lat:.6f},{lon:.6f}' # 小数点以下6桁にフォーマット
                                        msg = f'TXDA 0003,{data_to_send}\r'
                                        if im920 and im920.is_open:
                                            im920.write(msg.encode())
                                            print(f"IM920SL 送信: {msg.strip()}")
                                        else:
                                            print("IM920SLがオープンされていません。")
                                    except ValueError as ve:
                                        print(f"GPS座標の変換エラー: {ve}, ライン: {line.strip()}")
                                except IndexError:
                                    print(f"GPSデータフォーマットエラー (長さ不足): {line.strip()}")
                except UnicodeDecodeError as ude:
                    print(f"GPSデータデコードエラー: {ude}, データ: {data}")
                except Exception as e:
                    print(f"GPSデータ処理中に予期せぬエラー: {e}")
            time.sleep(0.1) # ポーリング間隔

    except Exception as e:
        print(f"GPS/IM920通信スレッドでエラーが発生しました: {e}")
    finally:
        print("GPS/IM920通信スレッドを終了します。クリーンアップ中...")
        if im920 and im920.is_open:
            im920.close()
            print("IM920SLシリアルポートを閉じました。")
        if pi and pi.connected:
            pi.bb_serial_read_close(GPS_RX_PIN)
            pi.stop()
            print("pigpioリソースをクリーンアップしました。")

# --- キーボード入力処理 ---
def on_press(key):
    global current_action, current_speed, motor

    if exit_event.is_set(): # 終了イベントがセットされていたら何もしない
        return

    try:
        # pynputのキーオブジェクトがchar属性を持つかチェック
        char_key = getattr(key, 'char', None)

        if char_key == 'w':
            if current_action != "forward":
                print("前進")
                motor.motor_forward(current_speed)
                current_action = "forward"
        elif char_key == 's':
            if current_action != "backward":
                print("後退")
                motor.motor_retreat(current_speed)
                current_action = "backward"
        elif char_key == 'a':
            if current_action != "turn_left":
                print("左旋回")
                motor.motor_left(current_speed)
                current_action = "turn_left"
        elif char_key == 'd':
            if current_action != "turn_right":
                print("右旋回")
                motor.motor_right(current_speed)
                current_action = "turn_right"
        elif char_key == 'q': # 左斜め前進（左モーター前進、右モーター停止）
            if current_action != "diagonal_left_forward":
                print("左斜め前進")
                motor.motor_Lforward(current_speed)
                # 右モーターを停止
                GPIO.output(motor.B1, GPIO.LOW)
                GPIO.output(motor.B2, GPIO.LOW)
                motor.pwmb.ChangeDutyCycle(0)
                current_action = "diagonal_left_forward"
        elif char_key == 'e': # 右斜め前進（右モーター前進、左モーター停止）
            if current_action != "diagonal_right_forward":
                print("右斜め前進")
                motor.motor_Rforward(current_speed)
                # 左モーターを停止
                GPIO.output(motor.A1, GPIO.LOW)
                GPIO.output(motor.A2, GPIO.LOW)
                motor.pwma.ChangeDutyCycle(0)
                current_action = "diagonal_right_forward"
        elif char_key == 'u': # 速度アップ
            if current_speed < 100:
                current_speed += 10
                print(f"速度アップ: {current_speed}")
                # 現在の動作を継続するために、再度関数を呼び出す
                if current_action == "forward": motor.motor_forward(current_speed)
                elif current_action == "backward": motor.motor_retreat(current_speed)
                elif current_action == "turn_left": motor.motor_left(current_speed)
                elif current_action == "turn_right": motor.motor_right(current_speed)
                elif current_action == "diagonal_left_forward": motor.motor_Lforward(current_speed)
                elif current_action == "diagonal_right_forward": motor.motor_Rforward(current_speed)
        elif char_key == 'j': # 速度ダウン
            if current_speed > 0:
                current_speed -= 10
                print(f"速度ダウン: {current_speed}")
                if current_action == "forward": motor.motor_forward(current_speed)
                elif current_action == "backward": motor.motor_retreat(current_speed)
                elif current_action == "turn_left": motor.motor_left(current_speed)
                elif current_action == "turn_right": motor.motor_right(current_speed)
                elif current_action == "diagonal_left_forward": motor.motor_Lforward(current_speed)
                elif current_action == "diagonal_right_forward": motor.motor_Rforward(current_speed)
                elif current_speed == 0: # 速度が0になったら完全に停止
                    motor.motor_stop_free()
                    current_action = "stop"

    except AttributeError:
        # 特殊キー (e.g., Space, Esc) の処理
        if key == keyboard.Key.space:
            if current_action != "stop":
                print("停止 (スペースキー)")
                motor.motor_stop_free()
                current_action = "stop"
        elif key == keyboard.Key.esc:
            print("プログラム終了 (Escキーが押されました)")
            exit_event.set() # 終了イベントをセットして全てのスレッドに通知
            return False # リスナーを停止

def on_release(key):
    global current_action, motor
    # 速度調整キー ('u', 'j') 以外が離されたら停止
    # ESCキーが押されて終了処理中の場合は何もしない
    if not exit_event.is_set() and key not in [keyboard.KeyCode(char='u'), keyboard.KeyCode(char='j')]:
        if current_action != "stop":
            print("キーを離したので停止")
            motor.motor_stop_free()
            current_action = "stop"

# --- メイン処理 ---
if __name__ == "__main__":
    print("アプリケーションを開始します...")

    # 1. MotorDriverの初期化
    try:
        motor = MotorDriver(PWMA, AIN1, AIN2, PWMB, BIN1, BIN2, STBY)
        print("MotorDriverを初期化しました。")
    except Exception as e:
        print(f"エラー: MotorDriverの初期化中にエラーが発生しました: {e}")
        # GPIOが初期化されている可能性があるため、ここでcleanupを試みる
        # (pigpioはgps_and_im920_thread内で初期化されるためここでは参照しない)
        GPIO.cleanup()
        exit(1)

    # 2. GPS/IM920SL通信スレッドを開始
    gps_im920_thread = threading.Thread(target=gps_and_im920_thread, daemon=True)
    gps_im920_thread.start()

    # スレッドの起動を少し待つ（エラーチェックのため）
    time.sleep(2)
    if exit_event.is_set(): # GPSスレッドが起動直後にエラーで終了した場合
        print("GPS/IM920通信スレッドの起動に失敗したため、プログラムを終了します。")
        if motor:
            motor.cleanup()
        GPIO.cleanup()
        exit(1)

    print("\n---------------------------------------------------")
    print("ローバー制御開始。")
    print("操作キー:")
    print("  'w' : 前進")
    print("  's' : 後退")
    print("  'a' : 左旋回")
    print("  'd' : 右旋回")
    print("  'q' : 左斜め前進")
    print("  'e' : 右斜め前進")
    print("  'u' : 速度アップ")
    print("  'j' : 速度ダウン")
    print("  'Space' : 停止")
    print("  'Esc' : 全プログラム終了")
    print("---------------------------------------------------\n")

    # 3. キーボードリスナーを開始 (メインスレッドで実行)
    listener = None
    try:
        with keyboard.Listener(on_press=on_press, on_release=on_release) as listener:
            listener.join() # キーボードリスナーが終了するまで待機
    except Exception as e:
        print(f"メインスレッド(キーボードリスナー)でエラーが発生しました: {e}")
    finally:
        # プログラム終了処理
        print("メイン処理を終了します。全リソースをクリーンアップ中...")
        exit_event.set() # 全スレッドに終了を再度通知

        # GPSスレッドが終了するのを待つ（必要であればタイムアウトを設定）
        # daemom=True なので、メインスレッドが終了すれば自動的に終了するが、
        # クリーンアップ処理が完了するのを確実に待ちたい場合は join() を使う
        gps_im920_thread.join(timeout=5) # 最大5秒待つ

        if motor:
            motor.cleanup() # MotorDriverのPWMを停止
            print("MotorDriverをクリーンアップしました。")
        
        # pigpioとRPi.GPIOのクリーンアップは、それぞれのスレッド/finallyブロックで
        # 行われるように設計されていますが、念のため最終的なクリーンアップをここに追加。
        # ただし、pigpioのstop()はgps_and_im920_threadのfinallyで呼ばれるべき。
        # RPi.GPIOのcleanup()は複数回呼ばれても問題ないが、確実に最後に一度呼ぶのが理想。
        try:
            GPIO.cleanup()
            print("RPi.GPIOをクリーンアップしました。")
        except Exception as e:
            print(f"RPi.GPIOのクリーンアップ中にエラーが発生しました: {e}")
        
        print("アプリケーションが完全に終了しました。")
