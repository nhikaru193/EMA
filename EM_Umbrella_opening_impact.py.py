import time
import math
import smbus

# Assuming BNO055.py is correctly set up and in the same directory
from BNO055 import BNO055

# --- BME280 Global Variables and I2C Setup ---
# t_fine is a global variable used by temperature, pressure, and humidity compensation
t_fine = 0.0
# Compensation parameters, read from the sensor's NVM (Non-Volatile Memory)
digT = []
digP = []
digH = []

# I2C bus and address settings
i2c_bus = smbus.SMBus(1) # Raspberry Pi's I2C bus is typically 1
bme280_address = 0x76 # Common I2C address for BME280. Verify with 'i2cdetect -y 1'.

# --- BME280 Initialization and Compensation Functions ---

def init_bme280():
    """Initializes the BME280 sensor by writing configuration registers."""
    # Set humidity oversampling (ctrl_hum register 0xF2) to x1
    i2c_bus.write_byte_data(bme280_address, 0xF2, 0x01)
    # Set temperature and pressure oversampling (ctrl_meas register 0xF4) to x1, Normal mode
    i2c_bus.write_byte_data(bme280_address, 0xF4, 0x27)
    # Set config register 0xF5: T_standby 1000ms, filter off, SPI 4-wire disable
    i2c_bus.write_byte_data(bme280_address, 0xF5, 0xA0)

def read_compensate():
    """
    Reads the factory calibration (compensation) values from the BME280 sensor's NVM.
    These values are crucial for converting raw ADC readings into meaningful physical units.
    """
    global digT, digP, digH # Declare global to modify the lists

    # Read temperature compensation values (reg 0x88 to 0x8D, 6 bytes)
    dat_t = i2c_bus.read_i2c_block_data(bme280_address, 0x88, 6)
    digT = [
        (dat_t[1] << 8) | dat_t[0], # digT1 (unsigned 16-bit)
        (dat_t[3] << 8) | dat_t[2], # digT2 (signed 16-bit)
        (dat_t[5] << 8) | dat_t[4]  # digT3 (signed 16-bit)
    ]
    # Convert signed 16-bit values if necessary (digT2 and digT3)
    for i in range(1, 3):
        if digT[i] >= 32768:
            digT[i] -= 65536

    # Read pressure compensation values (reg 0x8E to 0x9F, 18 bytes)
    dat_p = i2c_bus.read_i2c_block_data(bme280_address, 0x8E, 18)
    digP = [
        (dat_p[1] << 8) | dat_p[0],   # digP1 (unsigned 16-bit)
        (dat_p[3] << 8) | dat_p[2],   # digP2 (signed 16-bit)
        (dat_p[5] << 8) | dat_p[4],   # digP3 (signed 16-bit)
        (dat_p[7] << 8) | dat_p[6],   # digP4 (signed 16-bit)
        (dat_p[9] << 8) | dat_p[8],   # digP5 (signed 16-bit)
        (dat_p[11] << 8) | dat_p[10], # digP6 (signed 16-bit)
        (dat_p[13] << 8) | dat_p[12], # digP7 (signed 16-bit)
        (dat_p[15] << 8) | dat_p[14], # digP8 (signed 16-bit)
        (dat_p[17] << 8) | dat_p[16]  # digP9 (signed 16-bit)
    ]
    # Convert signed 16-bit values if necessary (digP2 to digP9)
    for i in range(1, 9):
        if digP[i] >= 32768:
            digP[i] -= 65536

    # Read humidity compensation values (reg 0xA1 and 0xE1 to 0xE7, total 8 bytes)
    dh = i2c_bus.read_byte_data(bme280_address, 0xA1) # digH1
    dat_h = i2c_bus.read_i2c_block_data(bme280_address, 0xE1, 7) # digH2 to digH6

    # Construct digH list with 6 elements
    digH = [0] * 6 # Initialize with zeros to ensure correct size

    digH[0] = dh # H1
    digH[1] = (dat_h[1] << 8) | dat_h[0] # H2
    digH[2] = dat_h[2] # H3
    digH[3] = (dat_h[3] << 4) | (dat_h[4] & 0x0F) # H4
    digH[4] = (dat_h[5] << 4) | ((dat_h[4] >> 4) & 0x0F) # H5
    digH[5] = dat_h[6] # H6

    # Convert signed values for humidity compensation (H2, H4, H5 are signed 16-bit; H6 is signed 8-bit)
    if digH[1] >= 32768:
        digH[1] -= 65536
    if digH[3] >= 32768:
        digH[3] -= 65536
    if digH[4] >= 32768:
        digH[4] -= 65536
    if digH[5] >= 128: # H6 is 8-bit signed
        digH[5] -= 256

def bme280_compensate_t(adc_T):
    """Calculates compensated temperature in degrees Celsius."""
    global t_fine
    # Based on Bosch BME280 datasheet, Section 4.2.3 Compensation formula for temperature (integer and floating-point versions)
    # This implementation is a standard floating-point translation.
    var1 = (adc_T / 16384.0 - digT[0] / 1024.0) * digT[1]
    var2 = ((adc_T / 131072.0 - digT[0] / 8192.0) *
            (adc_T / 131072.0 - digT[0] / 8192.0)) * digT[2]
    t_fine = var1 + var2
    temperature = t_fine / 5120.0
    return temperature # Celsius

def bme280_compensate_p(adc_P):
    """Calculates compensated pressure in hPa."""
    global t_fine
    # Based on Bosch BME280 datasheet, Section 4.2.3 Compensation formula for pressure (floating-point version)
    p = 0.0
    
    var1 = (t_fine / 2.0) - 64000.0
    var2 = var1 * var1 * digP[5] / 32768.0
    var2 = var2 + var1 * digP[4] * 2.0
    var2 = var2 + digP[3] * 131072.0
    var1 = (digP[2] * var1 * var1 / 524288.0) + (digP[1] * var1) / 32768.0
    var1 = (1.0 + var1 / 32768.0) * digP[0]

    if var1 == 0:
        return 0 # Avoid division by zero

    p = 1048576.0 - adc_P
    p = (p - (var2 / 4096.0)) * 6250.0 / var1
    var1 = (digP[8] * p * p) / 2147483648.0 # (digP9 * p^2) / (2^31)
    var2 = (p * digP[7]) / 32768.0       # (p * digP8) / (2^15)
    p = p + (var1 + var2 + digP[6]) / 16.0 # digP7 is added, then divided by 16

    return p / 100.0 # Convert Pa to hPa

def bme280_compensate_h(adc_H):
    """Calculates compensated humidity in %RH."""
    global t_fine
    # Based on Bosch BME280 datasheet, Section 4.2.3 Compensation formula for humidity (floating-point version)
    
    var_H = t_fine - 76800.0
    
    var_H = (adc_H - (digH[3] * 64.0 + digH[4] / 16384.0 * var_H)) * \
            (digH[1] / 1024.0 + digH[2] / 65536.0 * var_H)
    
    var_H = var_H * (1.0 + (digH[0] / 67108864.0 * var_H * \
            (1.0 + digH[5] / 67108864.0 * var_H))) # digH[5] is H6

    humidity = var_H
    
    # Final clamping to 0-100%
    if humidity > 100.0:
        humidity = 100.0
    elif humidity < 0.0:
        humidity = 0.0
    
    return humidity # %RH


def bme280_read_data():
    """Reads raw ADC values for pressure, temperature, and humidity, then compensates them."""
    try:
        # Read 8 bytes starting from 0xF7 (pressure_msb)
        # Data order: pres_msb, pres_lsb, pres_xlsb, temp_msb, temp_lsb, temp_xlsb, hum_msb, hum_lsb
        data = i2c_bus.read_i2c_block_data(bme280_address, 0xF7, 8)
        
        # --- DEBUG OUTPUT ---
        print(f"DEBUG_BME280_READ: Raw data read: {data}")
        print(f"DEBUG_BME280_READ: Length of raw data: {len(data)}")
        # --- END DEBUG OUTPUT ---

        if len(data) < 8:
            print(f"ERROR: BME280 read only {len(data)} bytes, expected 8 bytes.")
            raise IndexError("Not enough data read from BME280 for full compensation.")

        # Extract 20-bit ADC values for pressure and temperature
        pres_raw = (data[0] << 12) | (data[1] << 4) | (data[2] >> 4)
        temp_raw = (data[3] << 12) | (data[4] << 4) | (data[5] >> 4)
        
        # Extract 16-bit ADC value for humidity
        hum_raw  = (data[6] << 8)  | data[7]

        # Call compensation functions in the correct order (temperature first to set t_fine)
        temperature = bme280_compensate_t(temp_raw)
        pressure = bme280_compensate_p(pres_raw)
        humidity = bme280_compensate_h(hum_raw)

        return temperature, pressure, humidity
    except Exception as e:
        # Re-raise the exception so main() can catch it and print the specific error
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
        
        # デバッグ: 補正値を確認するためにコメントアウトを外すことができます
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

            # --- BME280データ取得 (BME280が利用可能な場合) ---
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
