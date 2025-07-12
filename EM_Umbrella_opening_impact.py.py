import time
import math
from BNO055 import BNO055
import BME280_sensor # BME280_sensor.py から BME280_sensor モジュールをインポート

def main():
    bno = BNO055()

    print("BNO055の初期化中...")
    if not bno.begin():
        print("BNO055の初期化に失敗しました。配線とI2Cアドレスを確認してください。")
        return

    print("BNO055の初期化に成功しました。")
    bno.setMode(BNO055.OPERATION_MODE_NDOF)
    time.sleep(1)

    bme280_present = False
    try:
        BME280_sensor.init_bme280()    # BME280_sensor モジュール内の関数を呼び出す
        BME280_sensor.read_compensate() # BME280_sensor モジュール内の関数を呼び出す
        print("BME280の初期化に成功しました。")
        bme280_present = True
        
        # デバッグ: 補正値が正しく読み込まれているか確認
        # BME280_sensor モジュール内のグローバル変数にアクセス
        print(f"DEBUG: digT={BME280_sensor.digT}")
        print(f"DEBUG: digP={BME280_sensor.digP}")
        # 湿度関連は使わないので digH は表示しない

    except Exception as e:
        print(f"BME280の初期化に失敗しました: {e}")
        print("BME280が接続されているか、I2Cアドレスを確認してください。")
        print("BME280の測定はスキップされます。")
        bme280_present = False


    print("\n--- センサーデータ測定開始 ---")
    print("Ctrl+Cで終了します。")
    print("表示形式:")
    print("  全加速度 [X, Y, Z] (重力込み) | 大きさ")
    print("  直線加速度 [X, Y, Z] (重力なし) | 大きさ")
    if bme280_present:
        print("  気圧 [hPa]") 
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

            # --- BME280データ取得 (利用可能な場合) ---
            pressure = None
            if bme280_present:
                try:
                    pressure = BME280_sensor.get_pressure_only() # BME280_sensor モジュール内の関数を呼び出す
                except Exception as e:
                    print(f"BME280データの読み取り中にエラーが発生しました: {e}")
                    bme280_present = False 
            
            # --- データ出力 ---
            print(f"全加速度:    X={ax:7.2f}, Y={ay:7.2f}, Z={az:7.2f} | 大きさ={total_accel_magnitude:7.2f} m/s^2")
            print(f"直線加速度: X={lx:7.2f}, Y={ly:7.2f}, Z={lz:7.2f} | 大きさ={linear_accel_magnitude:7.2f} m/s^2")
            
            if bme280_present:
                output_bme = f"気圧: {pressure:7.2f} hPa"
                print(output_bme)
            
            print("-" * 50)

            time.sleep(0.5)

    except KeyboardInterrupt:
        print("\nプログラムを終了します。")
    finally:
        bno.setMode(BNO055.OPERATION_MODE_CONFIG)

if __name__ == '__main__':
    main()
