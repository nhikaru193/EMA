import time
import math
import smbus  # smbusライブラリをインポート
from BNO055 import BNO055 # BNO055センサー用のカスタムライブラリ

# --- BME280関連のグローバル変数とI2C設定 ---
t_fine = 0.0
digT = []
digP = []
digH = []

# I2Cアドレスとバス設定
# Raspberry PiのI2Cバスは通常1
i2c_bus = smbus.SMBus(1)
bme280_address = 0x76 # BME280の一般的なI2Cアドレス

# --- BME280 初期化と補正関数群 ---
# 提供されたコードの関数をそのまま使用します。

def init_bme280():
    # センサー設定
    # 0xF2 (ctrl_hum): oversampling x1
    i2c_bus.write_byte_data(bme280_address, 0xF2, 0x01)
    # 0xF4 (ctrl_meas): Temp/Pres oversampling x1, Normal mode
    i2c_bus.write_byte_data(bme280_address, 0xF4, 0x27)
    # 0xF5 (config): T_standby 1000ms, filter off, SPI 4-wire
    i2c_bus.write_byte_data(bme280_address, 0xF5, 0xA0)

def read_compensate():
    global digT, digP, digH
    
    # 温度補正値の読み取り (0x88-0x8D)
    dat_t = i2c_bus.read_i2c_block_data(bme280_address, 0x88, 6)
    digT = [
        (dat_t[1] << 8) | dat_t[0],
        (dat_t[3] << 8) | dat_t[2],
        (dat_t[5] << 8) | dat_t[4]
    ]
    # 符号付き16ビット整数への変換
    for i in range(1, 2): # digT[1]とdigT[2]
        if digT[i] >= 32768:
            digT[i] -= 65536

    # 気圧補正値の読み取り (0x8E-0x9F)
    dat_p = i2c_bus.read_i2c_block_data(bme280_address, 0x8E, 18)
    digP = [
        (dat_p[1] << 8) | dat_p[0], (dat_p[3] << 8) | dat_p[2], (dat_p[5] << 8) | dat_p[4],
        (dat_p[7] << 8) | dat_p[6], (dat_p[9] << 8) | dat_p[8], (dat_p[11] << 8) | dat_p[10],
        (dat_p[13] << 8) | dat_p[12], (dat_p[15] << 8) | dat_p[14], (dat_p[17] << 8) | dat_p[16]
    ]
    # 符号付き16ビット整数への変換
    for i in range(1, 8): # digP[1]からdigP[8]
        if digP[i] >= 32768:
            digP[i] -= 65536

    # 湿度補正値の読み取り (0xA1, 0xE1-0xE7)
    dh = i2c_bus.read_byte_data(bme280_address, 0xA1)
    dat_h = i2c_bus.read_i2c_block_data(bme280_address, 0xE1, 7) # 0xE1-0xE7は7バイト
    digH = [
        dh,
        (dat_h[1] << 8) | dat_h[0],
        dat_h[2],
        (dat_h[3] << 4) | (0x0F & dat_h[4]),
        (dat_h[5] << 4) | ((dat_h[4] >> 4) & 0x0F),
        dat_h[6]
    ]
    # 符号付き整数への変換 (特定のdigH値)
    if digH[1] >= 32768:
        digH[1] -= 65536
    if digH[3] >= 32768: # digH[3]
        digH[3] -= 65536
    if digH[4] >= 32768: # digH[4] (上記digHの定義では5つ目の要素)
        digH[4] -= 65536
    if digH[5] >= 128:
        digH[5] -= 256

def bme280_compensate_t(adc_T):
    global t_fine
    var1 = (adc_T / 16384.0 - digT[0] / 1024.0) * digT[1]
    var2 = ((adc_T / 131072.0 - digT[0] / 8192.0) *
            (adc_T / 131072.0 - digT[0] / 8192.0)) * digT[2]
    t_fine = var1 + var2
    temperature = t_fine / 5120.0
    return temperature # Celsius

def bme280_compensate_p(adc_P):
    global t_fine
    p = 0.0
    var1 = t_fine / 2.0 - 64000.0
    var2 = var1 * var1 * digP[5] / 32768.0
    var2 = var2 + var1 * digP[4] * 2.0
    var2 = var2 + digP[3] * 131072.0
    var1 = (digP[2] * var1 * var1 / 524288.0 + digP[1] * var1) / 32768.0
    var1 = (1.0 + var1 / 32768.0) * digP[0]

    if var1 == 0:
        return 0 # avoid division by zero

    p = 1048576.0 - adc_P
    p = (p - var2 / 4096.0) * 6250.0 / var1
    var1 = digP[8] * p * p / 2147483648.0
    var2 = p * digP[7] / 32768.0
    p = p + (var1 + var2 + digP[6]) / 16.0
    return p / 100.0 # hPa (Pa / 100)

def bme280_compensate_h(adc_H):
    global t_fine
    var1 = t_fine - 76800.0
    var2 = (adc_H - (digH[3] * 64.0 + digH[4] / 16384.0 * var1)) * \
           (digH[1] / 1024.0 + digH[2] / 65536.0 * var1) * \
           (1.0 + digH[5] / 67108864.0 * var1 * (1.0 + digH[0] / 67108864.0 * var1))
    var2 = var2 * (1.0 - digH[6] * var2 / 1048576.0)
    if var2 > 100.0:
        var2 = 100.0
    elif var2 < 0.0:
        var2 = 0.0
    return var2 # %RH

def bme280_read_data():
    # ADC値の読み取り
    data = i2c_bus.read_i2c_block_data(bme280_address, 0xF7, 8)
    
    # 20ビットADC値の抽出
    pres_raw = (data[0] << 12) | (data[1] << 4) | (data[2] >> 4)
    temp_raw = (data[3] << 12) | (data[4] << 4) | (data[5] >> 4)
    hum_raw  = (data[6] << 8)  | data[7]

    # 補正関数を呼び出して実際の値を計算
    temperature = bme280_compensate_t(temp_raw)
    pressure = bme280_compensate_p(pres_raw)
    humidity = bme280_compensate_h(hum_raw)

    return temperature, pressure, humidity

# --- メインプログラム ---
def main():
    # BNO055センサーの初期化
    bno = BNO055()

    print("BNO055の初期化中...")
    if not bno.begin():
        print("BNO055の初期化に失敗しました。配線とI2Cアドレスを確認してください。")
        return

    print("BNO055の初期化に成功しました。")
    # BNO055をNDOFモードに設定 (加速度計、ジャイロスコープ、磁力計のデータフュージョン)
    bno.setMode(BNO055.OPERATION_MODE_NDOF)
    time.sleep(1) # モード設定が反映されるのを待つ

    # BME280センサーの初期化
    bme280_present = False
    try:
        init_bme280()      # BME280のレジスタを設定
        read_compensate()  # 補正値を読み込む
        print("BME280の初期化に成功しました。")
        bme280_present = True
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
            # 全加速度 (重力加速度を含む) のデータを取得 (m/s^2)
            total_accel = bno.getVector(BNO055.VECTOR_ACCELEROMETER)
            ax, ay, az = total_accel
            total_accel_magnitude = math.hypot(ax, ay, az) # 加速度の大きさ

            # 直線加速度 (重力加速度を除去) のデータを取得 (m/s^2)
            linear_accel = bno.getVector(BNO055.VECTOR_LINEARACCEL)
            lx, ly, lz = linear_accel
            linear_accel_magnitude = math.hypot(lx, ly, lz) # 直線加速度の大きさ

            # --- BME280データ取得 (BME280が利用可能な場合) ---
            temperature = None
            pressure = None
            humidity = None
            if bme280_present:
                try:
                    temperature, pressure, humidity = bme280_read_data()
                except Exception as e:
                    print(f"BME280データの読み取り中にエラーが発生しました: {e}")
                    bme280_present = False # エラーが続く場合は無効にする


            # --- データ出力 ---
            print(f"全加速度:    X={ax:7.2f}, Y={ay:7.2f}, Z={az:7.2f} | 大きさ={total_accel_magnitude:7.2f} m/s^2")
            print(f"直線加速度: X={lx:7.2f}, Y={ly:7.2f}, Z={lz:7.2f} | 大きさ={linear_accel_magnitude:7.2f} m/s^2")
            
            if bme280_present:
                output_bme = f"気圧: {pressure:7.2f} hPa | 温度: {temperature:6.2f} °C | 湿度: {humidity:6.2f} %"
                print(output_bme)
            
            print("-" * 50)

            time.sleep(0.5) # 0.5秒ごとにデータを更新

    except KeyboardInterrupt:
        print("\nプログラムを終了します。")
    finally:
        # プログラム終了時にBNO055をCONFIGモードに戻す (推奨)
        bno.setMode(BNO055.OPERATION_MODE_CONFIG)

if __name__ == '__main__':
    main()
