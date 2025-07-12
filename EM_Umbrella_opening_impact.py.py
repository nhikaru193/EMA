import time
import math
from BNO055 import BNO055
import board               # CircuitPythonのハードウェア定義ライブラリ
# ★★★ ここを修正しました ★★★
from adafruit_bme280 import BME280_I2C # adafruit_bme280 モジュールから BME280_I2C クラスを直接インポート

def main():
    # BNO055センサーの初期化
    bno = BNO055()

    print("BNO055の初期化中...")
    if not bno.begin():
        print("BNO055の初期化に失敗しました。配線とI2Cアドレスを確認してください。")
        return

    print("BNO055の初期化に成功しました。")
    # 加速度計を含む最適なフュージョンモードであるNDOFモードに設定
    bno.setMode(BNO055.OPERATION_MODE_NDOF)
    time.sleep(1) # モード設定が反映されるのを待つ

    # BME280センサーの初期化 (Adafruit ライブラリを使用)
    bme280_present = False
    try:
        i2c = board.I2C()  # デフォルトのI2Cバスを使用

        # ★★★ ここを修正しました ★★★
        # BME280_I2C クラスを直接使用
        bme280 = BME280_I2C(i2c)
        
        # オプション: 海面気圧を設定（高度計算に影響）
        # bme280.sea_level_pressure = 1013.25 

        print("BME280の初期化に成功しました。")
        bme280_present = True
    except ValueError as e:
        print(f"BME280の初期化に失敗しました: {e}")
        print("BME280が接続されているか、I2Cアドレスを確認してください。")
        print("BME280の測定はスキップされます。")
        bme280_present = False
    except RuntimeError as e:
        print(f"BME280の初期化中にランタイムエラーが発生しました: {e}")
        print("BME280の測定はスキップされます。")
        bme280_present = False


    print("\n--- センサーデータ測定開始 ---")
    print("Ctrl+Cで終了します。")
    print("表示形式:")
    print("  全加速度 [X, Y, Z] (重力込み) | 大きさ")
    print("  直線加速度 [X, Y, Z] (重力なし) | 大きさ")
    if bme280_present:
        print("  気圧 [hPa] | 温度 [°C] | 湿度 [%]")
    print("-" * 50)

    try:
        while True:
            # --- BNO055データ取得 ---
            total_accel = bno.getVector(BNO055.VECTOR_ACCELEROMETER)
            ax, ay, az = total_accel
            total_accel_magnitude = math.hypot(ax, ay, az)

            linear_accel = bno.getVector(BNO055.VECTOR_LINEARACCEL)
            lx, ly, lz = linear_accel 
            linear_accel_magnitude = math.hypot(lx, ly, lz)

            # --- BME280データ取得 (Adafruit ライブラリを使用) ---
            temperature = None
            pressure = None
            humidity = None
            if bme280_present:
                try:
                    # Adafruit ライブラリはこれらの値を直接提供します
                    temperature = bme280.temperature
                    pressure = bme280.pressure
                    humidity = bme280.humidity
                except Exception as e:
                    print(f"BME280データの読み取り中にエラーが発生しました: {e}")
                    bme280_present = False 
            
            # --- データ出力 ---
            print(f"全加速度:    X={ax:7.2f}, Y={ay:7.2f}, Z={az:7.2f} | 大きさ={total_accel_magnitude:7.2f} m/s^2")
            print(f"直線加速度: X={lx:7.2f}, Y={ly:7.2f}, Z={lz:7.2f} | 大きさ={linear_accel_magnitude:7.2f} m/s^2")
            
            if bme280_present:
                output_bme = f"気圧: {pressure:7.2f} hPa | 温度: {temperature:6.2f} °C | 湿度: {humidity:6.2f} %"
                print(output_bme)
            
            print("-" * 50)

            time.sleep(0.5)

    except KeyboardInterrupt:
        print("\nプログラムを終了します。")
    finally:
        bno.setMode(BNO055.OPERATION_MODE_CONFIG)

if __name__ == '__main__':
    main()
