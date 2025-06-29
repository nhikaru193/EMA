import smbus
import time
from BNO055 import BNO055

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
    p = 0.0 # BME280の元のコードではpの初期化がなかったため追加
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

## 放出判定ロジック (キャリブレーションなし)

```python
def check_release(pressure_threshold=1030.0, acc_threshold=3.0, consecutive_checks=3, timeout=30):
    """
    気圧と線形加速度の変化を監視し、放出条件が連続で満たされた場合に放出判定を行う。
    キャリブレーションは行わないため、線形加速度の精度は低下する可能性がある。
    タイムアウトした場合、条件成立回数に関わらず放出成功とみなす。

    Args:
        pressure_threshold (float): 放出判定のための気圧閾値 (hPa)。
        acc_threshold (float): 放出判定のためのZ軸線形加速度絶対値閾値 (m/s²)。
        consecutive_checks (int): 放出判定が連続して成立する必要のある回数。
        timeout (int): 判定を打ち切るタイムアウト時間 (秒)。
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
    
    # --- BNO055 キャリブレーション待機部分は削除 ---
    print("\n⚠️ BNO055 キャリブレーションはスキップされました。線形加速度の精度が低下する可能性があります。")


    print("\n🚀 放出判定開始...")
    print(f"   気圧閾値: < {pressure_threshold:.2f} hPa")
    print(f"   Z軸線形加速度絶対値閾値: |Z| > {acc_threshold:.2f} m/s² (重力除去済み)")
    print(f"   連続成立回数: {consecutive_checks}回")
    print(f"   タイムアウト: {timeout}秒\n")

    release_count = 0 # 連続成立回数
    start_time = time.time()
    last_check_time = time.time() # 前回のチェック時刻

    try:
        while True:
            current_time = time.time()
            elapsed_total = current_time - start_time

            # タイムアウト判定
            if elapsed_total > timeout:
                print(f"\n⏰ タイムアウト ({timeout}秒経過)。条件成立回数 {release_count} 回でしたが、強制的に放出判定を成功とします。")
                return True # タイムアウトしたら無条件で成功

            # データ取得と表示は一定間隔で行う
            if (current_time - last_check_time) < 0.2: # 0.2秒間隔でデータ取得と表示
                time.sleep(0.01) # 短いスリープでCPU負荷軽減
                continue
            
            last_check_time = current_time

            # センサーデータの取得
            pressure, _ = get_pressure_and_temperature()
            acc_x, acc_y, acc_z = bno.getVector(BNO055.VECTOR_LINEARACCEL)

            print(f"経過: {elapsed_total:.1f}s | 気圧: {pressure:.2f} hPa | 線形加速度Z: {acc_z:.2f} m/s² ", end='\r')

            # 放出条件の判定
            if pressure < pressure_threshold and abs(acc_z) > acc_threshold:
                release_count += 1
                print(f"\n💡 条件成立！連続判定中: {release_count}/{consecutive_checks} 回")
            else:
                if release_count > 0:
                    print(f"\n--- 条件不成立。カウントリセット ({release_count} -> 0) ---")
                release_count = 0

            # 連続成立回数の確認
            if release_count >= consecutive_checks:
                print(f"\n🎉 放出判定成功！連続 {consecutive_checks} 回条件成立！")
                return True # 放出判定成功で関数を終了

    except KeyboardInterrupt:
        print("\n\nプログラムがユーザーによって中断されました。")
        return False
    except Exception as e:
        print(f"\n\n🚨 エラーが発生しました: {e}")
        return False
    finally:
        print("\n--- 判定処理終了 ---")

---
## 実行例

```python
if __name__ == '__main__':
    # BNO055.py が同じディレクトリにあることを確認してください。
    # 閾値とタイムアウトを設定して判定を開始
    is_released = check_release(
        pressure_threshold=1029.0, # 例: 高度上昇による気圧低下を検出 (約1000mの高度に相当)
        acc_threshold=2.5,        # 例: ロケット分離時の衝撃や加速を検出 (重力除去済み)
        consecutive_checks=3,
        timeout=30 # テスト期間を短めに設定
    )

    if is_released:
        print("\n=== ロケットの放出を確認しました！ ===")
    else:
        print("\n=== ロケットの放出は確認できませんでした。 ===")
