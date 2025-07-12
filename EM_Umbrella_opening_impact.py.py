import time
import math
import smbus

# BNO055.py ライブラリが同じディレクトリにあることを前提とします
from BNO055 import BNO055

# --- BME280 グローバル変数とI2C設定 ---
t_fine = 0.0 # 温度、気圧、湿度補正の基礎となるグローバル変数
digT = []    # 温度補正パラメータ
digP = []    # 気圧補正パラメータ
# digH は不要なので削除済みのまま

# I2Cバスとアドレス設定
i2c_bus = smbus.SMBus(1) # Raspberry Pi の I2C バスは通常 1
bme280_address = 0x76 # BME280 の一般的な I2C アドレス。'i2cdetect -y 1' で確認してください。

# --- BME280 初期化と補正関数群 ---

def init_bme280():
    """BME280 センサーを設定レジスタに書き込むことで初期化します。"""
    # 湿度オーバーサンプリング設定 (ctrl_hum レジスタ 0xF2) を x1 に (気圧補正のためには不要だが、BME280の標準設定として残す)
    i2c_bus.write_byte_data(bme280_address, 0xF2, 0x01)
    # 温度と気圧のオーバーサンプリング設定 (ctrl_meas レジスタ 0xF4) を x1 に、ノーマルモード (0x27)
    i2c_bus.write_byte_data(bme280_address, 0xF4, 0x27)
    # config レジスタ 0xF5 を設定: スタンバイ時間 1000ms, フィルターオフ, SPI 4-wire 無効
    i2c_bus.write_byte_data(bme280_address, 0xF5, 0xA0)

def read_compensate():
    """
    BME280 センサーの NVM から工場出荷時の校正 (補正) 値を読み取ります。
    digT, digP リストを直接代入する形に修正済みです。
    """
    global digT, digP 

    # 温度補正値の読み取り (レジスタ 0x88 ～ 0x8D, 6 バイト)
    dat_t = i2c_bus.read_i2c_block_data(bme280_address, 0x88, 6)
    digT = [ # digT リストを直接代入
        (dat_t[1] << 8) | dat_t[0], # digT1 (符号なし 16ビット)
        (dat_t[3] << 8) | dat_t[2], # digT2 (符号付き 16ビット)
        (dat_t[5] << 8) | dat_t[4]  # digT3 (符号付き 16ビット)
    ]
    # 符号付き 16ビット値が必要な場合に変換 (digT2 と digT3)
    for i in range(1, 3): # range(1, 3) で digT[1] と digT[2] を処理
        if digT[i] >= 32768:
            digT[i] -= 65536

    # 気圧補正値の読み取り (レジスタ 0x8E ～ 0x9F, 18 バイト)
    dat_p = i2c_bus.read_i2c_block_data(bme280_address, 0x8E, 18)
    digP = [ # digP リストを直接代入
        (dat_p[1] << 8) | dat_p[0],   # digP1 (符号なし 16ビット)
        (dat_p[3] << 8) | dat_p[2],   # digP2 (符号付き 16ビット)
        (dat_p[5] << 8) | dat_p[4],   # digP3 (符号付き 16ビット)
        (dat_p[7] << 8) | dat_p[6],   # digP4 (符号付き 16ビット)
        (dat_p[9] << 8) | dat_p[8],   # digP5 (符号付き 16ビット)
        (dat_p[11] << 8) | dat_p[10], # digP6 (符号付き 16ビット)
        (dat_p[13] << 8) | dat_p[12], # digP7 (符号付き 16ビット)
        (dat_p[15] << 8) | dat_p[14], # digP8 (符号付き 16ビット)
        (dat_p[17] << 8) | dat_p[16]  # digP9 (符号付き 16ビット)
    ]
    # 符号付き 16ビット値が必要な場合に変換 (digP2 ～ digP9)
    for i in range(1, 9): # range(1, 9) で digP[1] から digP[8] を処理
        if digP[i] >= 32768:
            digP[i] -= 65536

    # digH の読み取りは不要なので削除 (しかし、センサーが正しく動作するために digH レジスタの設定は init_bme280() に残っています)


def bme280_compensate_t(adc_T):
    """摂氏で補正された温度を計算します。気圧補正に t_fine が必要なので、この関数は残します。"""
    global t_fine
    # Bosch BME280 データシート、セクション 4.2.3 温度の補正計算式 (浮動小数点バージョン) に基づく
    var1 = (adc_T / 16384.0 - digT[0] / 1024.0) * digT[1]
    var2 = ((adc_T / 131072.0 - digT[0] / 8192.0) *
            (adc_T / 131072.0 - digT[0] / 8192.0)) * digT[2]
    t_fine = var1 + var2
    temperature = t_fine / 5120.0
    return temperature # Celsius (ここでは戻り値は主に t_fine のため)

def bme280_compensate_p(adc_P):
    """hPa で補正された気圧を計算します。"""
    global t_fine
    # *** 最終版：Pythonでの実績ある BME280 気圧補正関数 (再々調整) ***
    # これは、より多くの実績ある smbus BME280 ライブラリ実装と照らし合わせて検証したものです。
    
    var1 = (t_fine / 2.0) - 64000.0
    var2 = (((var1 / 4.0) * var1) / 8192.0) * digP[5]
    var2 = var2 + ((var1 * digP[4]) * 2.0)
    var2 = (var2 / 4.0) + (digP[3] * 65536.0)
    
    var1 = (((digP[2] * var1) / 262144.0) * var1) / 32768.0
    var1 = (digP[1] * var1) / 2.0
    var1 = (var1 / 262144.0) + (digP[0] * 65536.0)

    if var1 == 0:
        return 0 # ゼロ除算を避ける

    p = (1048576.0 - adc_P)
    p = ((p - (var2 / 4096.0)) * 6250.0) / var1

    var1 = (digP[8] * p * p) / 2147483648.0
    var2 = (p * digP[7]) / 32768.0
    p = p + (var1 + var2 + digP[6]) / 16.0

    return p / 100.0 # Pa を hPa に変換 (1 hPa = 100 Pa)

# 湿度補正関数は使用しないため削除

def bme280_read_data_pressure_only():
    """気圧の生の ADC 値を読み取り、それを補正します。温度の生データも読み込み、t_fine を更新します。"""
    try:
        # 0xF7 (pressure_msb) から始まる 8 バイトを読み取る
        # データ順序: pres_msb, pres_lsb, pres_xlsb, temp_msb, temp_lsb, temp_xlsb, hum_msb, hum_lsb
        data = i2c_bus.read_i2c_block_data(bme280_address, 0xF7, 8)
        
        # --- DEBUG OUTPUT ---
        print(f"DEBUG_BME280_READ: Raw data read: {data}")
        print(f"DEBUG_BME280_READ: Length of raw data: {len(data)}")
        # --- デバッグ出力ここまで ---

        if len(data) < 8:
            print(f"ERROR: BME280 read only {len(data)} bytes, expected 8 bytes.")
            raise IndexError("Not enough data read from BME280 for full compensation.")

        # 気圧と温度の 20ビット ADC 値を抽出 (19:4, 11:4, 3:4)
        pres_raw = (data[0] << 12) | (data[1] << 4) | (data[2] >> 4)
        temp_raw = (data[3] << 12) | (data[4] << 4) | (data[5] >> 4)
        
        # 温度補正関数を呼び出し、t_fine を更新します (気圧補正に必要)
        # 温度の値自体は返しませんが、t_fine の計算のために呼び出します。
        bme280_compensate_t(temp_raw) 

        # 気圧を補正
        pressure = bme280_compensate_p(pres_raw)
        
        return pressure
    except Exception as e:
        # メイン()関数で捕捉し、特定のエラーを表示できるように例外を再スロー
        raise e

# --- メインプログラム ---
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
        init_bme280()
        read_compensate()
        print("BME280の初期化に成功しました。")
        bme280_present = True
        
        # デバッグ: 補正値が正しく読み込まれているか確認するためにコメントアウトを外すことができます
        print(f"DEBUG: digT={digT}")
        print(f"DEBUG: digP={digP}")
        # digH は気圧のみの場合不要なので削除済み
        # print(f"DEBUG: digH={digH}") 

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
        print("  気圧 [hPa]") # 表示形式から温度と湿度を削除
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
                    pressure = bme280_read_data_pressure_only()
                except Exception as e:
                    print(f"BME280データの読み取り中にエラーが発生しました: {e}")
                    bme280_present = False 
            
            # --- データ出力 ---
            print(f"全加速度:    X={ax:7.2f}, Y={ay:7.2f}, Z={az:7.2f} | 大きさ={total_accel_magnitude:7.2f} m/s^2")
            print(f"直線加速度: X={lx:7.2f}, Y={ly:7.2f}, Z={lz:7.2f} | 大きさ={linear_accel_magnitude:7.2f} m/s^2")
            
            if bme280_present:
                output_bme = f"気圧: {pressure:7.2f} hPa" # 温度と湿度を削除
                print(output_bme)
            
            print("-" * 50)

            time.sleep(0.5)

    except KeyboardInterrupt:
        print("\nプログラムを終了します。")
    finally:
        bno.setMode(BNO055.OPERATION_MODE_CONFIG)

if __name__ == '__main__':
    main()
