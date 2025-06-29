import smbus
import time
from BNO055 import BNO055  # BNO055をインポート

# BME280関連のグローバル変数
t_fine = 0.0
digT = []
digP = []
digH = []

# I2Cアドレスとバス設定
i2c = smbus.SMBus(1)
address = 0x76 # BME280のアドレス

# --- BME280 初期化と補正関数群（変更なし） ---

def init_bme280():
    i2c.write_byte_data(address, 0xF2, 0x01)
    i2c.write_byte_data(address, 0xF4, 0x27)
    i2c.write_byte_data(address, 0xF5, 0xA0)

def read_compensate():
    global digT, digP, digH
    dat_t = i2c.read_i2c_block_data(address, 0x88, 6)
    digT = [(dat_t[1] << 8) | dat_t[0], (dat_t[3] << 8) | dat_t[2], (dat_t[5] << 8) | dat_t[4]]
    for i in range(1, 2):
        if digT[i] >= 32768:
            digT[i] -= 65536
    dat_p = i2c.read_i2c_block_data(address, 0x8E, 18)
    digP = [(dat_p[i+1] << 8) | dat_p[i] for i in range(0, 18, 2)]
    for i in range(1, 8):
        if digP[i] >= 32768:
            digP[i] -= 65536
    dh = i2c.read_byte_data(address, 0xA1)
    dat_h = i2c.read_i2c_block_data(address, 0xE1, 8)
    digH = [dh, (dat_h[1] << 8) | dat_h[0], dat_h[2],
            (dat_h[3] << 4) | (0x0F & dat_h[4]),
            (dat_h[5] << 4) | ((dat_h[4] >> 4) & 0x0F),
            dat_h[6]]
    if digH[1] >= 32768:
        digH[1] -= 65536
    for i in range(3, 4):
        if digH[i] >= 32768:
            digH[i] -= 65536
    if digH[5] >= 128:
        digH[5] -= 256

def bme280_compensate_t(adc_T):
    global t_fine
    var1 = (adc_T / 8.0 - digT[0] * 2.0) * digT[1] / 2048.0
    var2 = ((adc_T / 16.0 - digT[0]) ** 2) * digT[2] / 16384.0
    t_fine = var1 + var2
    t = (t_fine * 5 + 128) / 256 / 100
    return t

def bme280_compensate_p(adc_P):
    global t_fine
    p = 0.0
    var1 = t_fine - 128000.0
    var2 = var1 * var1 * digP[5]
    var2 += (var1 * digP[4]) * 131072.0
    var2 += digP[3] * 3.435973837e10
    var1 = (var1 * var1 * digP[2]) / 256.0 + (var1 * digP[1]) * 4096
    var1 = (1.407374884e14 + var1) * (digP[0] / 8589934592.0)
    if var1 == 0:
        return 0
    p = (1048576.0 - adc_P) * 2147483648.0 - var2
    p = (p * 3125) / var1
    var1 = digP[8] * (p / 8192.0)**2 / 33554432.0
    var2 = digP[7] * p / 524288.0
    p = (p + var1 + var2) / 256 + digP[6] * 16.0
    return p / 256 / 100

def get_pressure_and_temperature():
    """BME280から気圧と温度を読み込み、補正して返す"""
    dat = i2c.read_i2c_block_data(address, 0xF7, 8)
    adc_p = (dat[0] << 16 | dat[1] << 8 | dat[2]) >> 4
    adc_t = (dat[3] << 16 | dat[4] << 8 | dat[5]) >> 4
    
    temperature = bme280_compensate_t(adc_t)
    pressure = bme280_compensate_p(adc_p)
    return pressure, temperature

# --- 着地判定処理 ---

def check_landing(min_pressure_threshold=1029.0, max_pressure_threshold=1030.0, acc_threshold_abs=0.5, gyro_threshold_abs=0.5, consecutive_checks=3, timeout=60, calibrate_bno055=True):
    """
    気圧、加速度、角速度が絶対閾値内に収まる状態を監視し、着地条件が連続で満たされた場合に着地判定を行う。
    タイムアウトした場合、条件成立回数に関わらず着地成功とみなす。
    オプションでBNO055のキャリブレーション待機機能を含む。

    Args:
        min_pressure_threshold (float): 着地判定のための最小気圧閾値 (hPa)。
        max_pressure_threshold (float): 着地判定のための最大気圧閾値 (hPa)。
        acc_threshold_abs (float): 着地判定のための線形加速度の絶対値閾値 (m/s²)。
        gyro_threshold_abs (float): 着地判定のための角速度の絶対値閾値 (°/s)。
        consecutive_checks (int): 着地判定が連続して成立する必要のある回数。
        timeout (int): 判定を打ち切るタイムアウト時間 (秒)。
        calibrate_bno055 (bool): Trueの場合、BNO055の完全キャリブレーションを待機する。
    """
    # センサーの初期化
    init_bme280()
    read_compensate()

    bno = BNO055()
    if not bno.begin():
        print("🔴 BNO055 初期化失敗。プログラムを終了します。")
        return False # 失敗を明確に返す

    bno.setExternalCrystalUse(True)
    bno.setMode(BNO055.OPERATION_MODE_NDOF) # NDOFモードを明示的に設定

    # --- BNO055 キャリブレーション待機 ---
    if calibrate_bno055:
        print("\n⚙️ BNO055 キャリブレーション中... センサーをいろんな向きにゆっくり回してください。")
        print("   (システム、ジャイロ、加速度、地磁気が完全キャリブレーション(レベル3)になるのを待ちます)")
        calibration_start_time = time.time()
        while True:
            sys, gyro, accel, mag = bno.getCalibration()
            print(f"   現在のキャリブレーション状態 → システム:{sys}, ジャイロ:{gyro}, 加速度:{accel}, 地磁気:{mag} ", end='\r')
            
            # 加速度計もレベル3になるまで待つように条件を強化
            if sys == 3 and gyro == 3 and accel == 3 and mag == 3:
                print("\n✅ BNO055 キャリブレーション完了！")
                break
            time.sleep(0.5) # 0.5秒ごとに状態を確認
        print(f"   キャリブレーションにかかった時間: {time.time() - calibration_start_time:.1f}秒\n")
    else:
        print("\n⚠️ BNO055 キャリブレーション待機はスキップされました。")


    print("🛬 着地判定開始...")
    print(f"   気圧範囲: {min_pressure_threshold:.2f} hPa 〜 {max_pressure_threshold:.2f} hPa")
    print(f"   加速度絶対値閾値: < {acc_threshold_abs:.2f} m/s² (X, Y, Z軸)")
    print(f"   角速度絶対値閾値: < {gyro_threshold_abs:.2f} °/s (X, Y, Z軸)")
    print(f"   連続成立回数: {consecutive_checks}回")
    print(f"   タイムアウト: {timeout}秒\n")

    landing_count = 0 # 連続成立回数
    start_time = time.time()
    last_check_time = time.time() # 前回のチェック時刻

    # stable_変数は不要になったため削除
    # stable_pressure = None
    # stable_acc_x, stable_acc_y, stable_acc_z = None, None, None
    # stable_gyro_x, stable_gyro_y, stable_gyro_z = None, None, None

    try:
        while True:
            current_time = time.time()
            elapsed_total = current_time - start_time

            # タイムアウト判定
            if elapsed_total > timeout:
                print(f"\n⏰ タイムアウト ({timeout}秒経過)。条件成立回数 {landing_count} 回でしたが、強制的に着地判定を成功とします。")
                return True # タイムアウトしたら無条件で成功
            
            # データ取得と表示は一定間隔で行う
            if (current_time - last_check_time) < 0.2: # 約0.2秒間隔でデータ取得と表示
                time.sleep(0.01) # 短いスリープでCPU負荷軽減
                continue
            
            last_check_time = current_time

            # センサーデータの取得
            pressure, _ = get_pressure_and_temperature() # 温度はここでは使わないので_で受け取る
            acc_x, acc_y, acc_z = bno.getVector(BNO055.VECTOR_LINEARACCEL) # 線形加速度
            gyro_x, gyro_y, gyro_z = bno.getVector(BNO055.VECTOR_GYROSCOPE) # 角速度

            # 初回のデータで安定時の基準値を設定 (絶対値判定になったため不要)
            # if stable_pressure is None:
            #     print("--- 初期安定値設定完了。着地条件監視中... ---")
            #     continue # 初回は基準値設定のみで判定はスキップ

            print(f"経過: {elapsed_total:.1f}s | 気圧: {pressure:.2f} hPa | 加速度(X,Y,Z): ({acc_x:.2f}, {acc_y:.2f}, {acc_z:.2f}) m/s² | 角速度(X,Y,Z): ({gyro_x:.2f}, {gyro_y:.2f}, {gyro_z:.2f}) °/s ", end='\r')

            # 着地条件の判定 (絶対値での判定に変更)
            is_landing_condition_met = (
                min_pressure_threshold <= pressure <= max_pressure_threshold and  # 気圧が範囲内
                abs(acc_x) < acc_threshold_abs and                              # 各軸の加速度絶対値が閾値以下
                abs(acc_y) < acc_threshold_abs and
                abs(acc_z) < acc_threshold_abs and
                abs(gyro_x) < gyro_threshold_abs and                            # 各軸の角速度絶対値が閾値以下
                abs(gyro_y) < gyro_threshold_abs and
                abs(gyro_z) < gyro_threshold_abs
            )

            if is_landing_condition_met:
                landing_count += 1
                print(f"\n💡 条件成立！連続判定中: {landing_count}/{consecutive_checks} 回")
            else:
                if landing_count > 0:
                    print(f"\n--- 条件不成立。カウントリセット ({landing_count} -> 0) ---")
                landing_count = 0

            # 連続成立回数の確認
            if landing_count >= consecutive_checks:
                print(f"\n🎉 着地判定成功！連続 {consecutive_checks} 回条件成立！")
                return True # 着地判定成功で関数を終了

    except KeyboardInterrupt:
        print("\n\nプログラムがユーザーによって中断されました。")
        return False
    except Exception as e:
        print(f"\n\n🚨 エラーが発生しました: {e}")
        return False
    finally:
        print("\n--- 判定処理終了 ---")


# --- 実行例 ---
if __name__ == '__main__':
    # BNO055.py が test_land2.py と同じディレクトリにあることを確認してください。
    # 閾値とタイムアウトを設定して判定を開始
    is_landed = check_landing(
        min_pressure_threshold=1029.0, # 気圧の最小閾値
        max_pressure_threshold=1030.0, # 気圧の最大閾値
        acc_threshold_abs=0.5,         # 線形加速度の各軸の絶対値閾値 (m/s²)
        gyro_threshold_abs=0.5,        # 角速度の各軸の絶対値閾値 (°/s)
        consecutive_checks=3,          # 3回連続で条件が満たされたら着地とみなす
        timeout=120,                   # 2分以内に判定が行われなければタイムアウトで強制成功
        calibrate_bno055=True          # BNO055のキャリブレーション待機を有効にする (強く推奨)
    )

    if is_landed:
        print("\n=== ロケットの着地を確認しました！ ===")
    else:
        print("\n=== ロケットの着地は確認できませんでした。 ===")
