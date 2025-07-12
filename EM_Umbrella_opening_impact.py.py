import time
import math
import smbus

# BNO055.py ライブラリが同じディレクトリにあることを前提とします
from BNO055 import BNO055

# --- BME280 グローバル変数とI2C設定 ---
t_fine = 0.0 # 温度、気圧、湿度補正の基礎となるグローバル変数
digT = []    # 温度補正パラメータ
digP = []    # 気圧補正パラメータ
digH = []    # 湿度補正パラメータ

# I2Cバスとアドレス設定
i2c_bus = smbus.SMBus(1) # Raspberry Pi の I2C バスは通常 1
bme280_address = 0x76 # BME280 の一般的な I2C アドレス。'i2cdetect -y 1' で確認してください。

# --- BME280 初期化と補正関数群 ---

def init_bme280():
    """BME280 センサーを設定レジスタに書き込むことで初期化します。"""
    # 湿度オーバーサンプリング設定 (ctrl_hum レジスタ 0xF2) を x1 に
    i2c_bus.write_byte_data(bme280_address, 0xF2, 0x01)
    # 温度と気圧のオーバーサンプリング設定 (ctrl_meas レジスタ 0xF4) を x1 に、ノーマルモード
    i2c_bus.write_byte_data(bme280_address, 0xF4, 0x27)
    # config レジスタ 0xF5 を設定: スタンバイ時間 1000ms, フィルターオフ, SPI 4-wire 無効
    i2c_bus.write_byte_data(bme280_address, 0xF5, 0xA0)

def read_compensate():
    """
    BME280 センサーの NVM (不揮発性メモリ) から工場出荷時の校正 (補正) 値を読み取ります。
    これらの値は、生 ADC データを意味のある物理単位に変換するために不可欠です。
    """
    global digT, digP, digH # リストを変更するためにグローバル宣言

    # 温度補正値の読み取り (レジスタ 0x88 ～ 0x8D, 6 バイト)
    dat_t = i2c_bus.read_i2c_block_data(bme280_address, 0x88, 6)
    digT = [
        (dat_t[1] << 8) | dat_t[0], # digT1 (符号なし 16ビット)
        (dat_t[3] << 8) | dat_t[2], # digT2 (符号付き 16ビット)
        (dat_t[5] << 8) | dat_t[4]  # digT3 (符号付き 16ビット)
    ]
    # 符号付き 16ビット値が必要な場合に変換 (digT2 と digT3)
    for i in range(1, 3):
        if digT[i] >= 32768:
            digT[i] -= 65536

    # 気圧補正値の読み取り (レジスタ 0x8E ～ 0x9F, 18 バイト)
    dat_p = i2c_bus.read_i2c_block_data(bme280_address, 0x8E, 18)
    digP = [
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
    for i in range(1, 9):
        if digP[i] >= 32768:
            digP[i] -= 65536

    # 湿度補正値の読み取り (レジスタ 0xA1 と 0xE1 ～ 0xE7, 合計 8 バイト)
    dh = i2c_bus.read_byte_data(bme280_address, 0xA1) # digH1
    dat_h = i2c_bus.read_i2c_block_data(bme280_address, 0xE1, 7) # digH2 ～ digH6

    # 6つの要素を持つ digH リストを構築
    digH = [0] * 6 # 正しいサイズを確保するためにゼロで初期化

    digH[0] = dh # H1
    digH[1] = (dat_h[1] << 8) | dat_h[0] # H2
    digH[2] = dat_h[2] # H3
    digH[3] = (dat_h[3] << 4) | (dat_h[4] & 0x0F) # H4
    digH[4] = (dat_h[5] << 4) | ((dat_h[4] >> 4) & 0x0F) # H5
    digH[5] = dat_h[6] # H6

    # 湿度補正の符号付き値を変換 (H2, H4, H5 は符号付き 16ビット; H6 は符号付き 8ビット)
    if digH[1] >= 32768:
        digH[1] -= 65536
    if digH[3] >= 32768:
        digH[3] -= 65536
    if digH[4] >= 32768:
        digH[4] -= 65536
    if digH[5] >= 128: # H6 は符号付き 8ビット
        digH[5] -= 256

def bme280_compensate_t(adc_T):
    """摂氏で補正された温度を計算します。"""
    global t_fine
    # Based on Bosch BME280 datasheet, Section 4.2.3 Compensation formula for temperature (floating-point version)
    var1 = (adc_T / 16384.0 - digT[0] / 1024.0) * digT[1]
    var2 = ((adc_T / 131072.0 - digT[0] / 8192.0) *
            (adc_T / 131072.0 - digT[0] / 8192.0)) * digT[2]
    t_fine = var1 + var2
    temperature = t_fine / 5120.0
    return temperature # Celsius

def bme280_compensate_p(adc_P):
    """hPa で補正された気圧を計算します。"""
    global t_fine
    # --- Python での浮動小数点精度とデータシートの C 言語実装に厳密に合わせた気圧補正関数 ---
    # 中間変数の型や計算順序が重要です。
    
    var1 = (t_fine / 2.0) - 64000.0
    var2 = (((var1 / 4.0) * var1) / 8192.0) * digP[5]
    var2 = var2 + ((var1 * digP[4]) * 2.0)
    var2 = (var2 / 4.0) + (digP[3] * 65536.0) # digP3 * 2^16
    
    var1 = (((digP[2] * var1) / 262144.0) * var1) / 32768.0
    var1 = (digP[1] * var1) / 2.0
    var1 = (var1 / 262144.0) + (digP[0] * 65536.0) # digP1 * 2^16

    if var1 == 0:
        return 0 # Avoid division by zero

    p = (1048576.0 - adc_P)
    p = ((p - (var2 / 4096.0)) * 6250.0) / var1 # この除算は非常に重要

    var1 = (digP[8] * p * p) / 2147483648.0 # (digP9 * p^2) / 2^31
    var2 = (p * digP[7]) / 32768.0       # (digP8 * p) / 2^15
    p = p + (var1 + var2 + digP[6]) / 16.0 # digP7 を加えてから 16 で割る

    return p / 100.0 # Pa を hPa に変換 (1 hPa = 100 Pa)

def bme280_compensate_h(adc_H):
    """%RH で補正された湿度を計算します。"""
    global t_fine
    # --- Python での浮動小数点精度とデータシートの C 言語実装に厳密に合わせた湿度補正関数 ---
    
    var_H = t_fine - 76800.0
    
    var_H = (adc_H - (digH[3] * 64.0 + digH[4] / 16384.0 * var_H)) * \
            (digH[1] / 1024.0 + digH[2] / 65536.0 * var_H)
    
    var_H = var_H * (1.0 + (digH[0] / 67108864.0 * var_H * \
            (1.0 + digH[5] / 67108864.0 * var_H))) # digH[5] は H6

    humidity = var_H # 結果が直接湿度値

    # 最終的な値を 0%～100% の範囲にクリッピング
    if humidity > 100.0:
        humidity = 100.0
    elif humidity < 0.0:
        humidity = 0.0
    
    return humidity # %RH


def bme280_read_data():
    """気圧、温度、湿度の生の ADC 値を読み取り、それらを補正します。"""
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

        # 気圧と温度の 20ビット ADC 値を抽出
        pres_raw = (data[0] << 12) | (data[1] << 4) | (data[2] >> 4)
        temp_raw = (data[3] << 12) | (data[4] << 4) | (data[5] >> 4)
        
        # 湿度の 16ビット ADC 値を抽出
        hum_raw  = (data[6] << 8)  | data[7]

        # 補正関数を正しい順序で呼び出す (t_fine を設定するため、温度が最初)
        temperature = bme280_compensate_t(temp_raw)
        pressure = bme280_compensate_p(pres_raw)
        humidity = bme280_compensate_h(hum_raw)

        return temperature, pressure, humidity
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
        print(f"DEBUG: digH={digH}")

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

            # --- BME280データ取得 (利用可能な場合) ---
            temperature = None
            pressure = None
            humidity = None
            if bme280_present:
                try:
                    temperature, pressure, humidity = bme280_read_data()
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
