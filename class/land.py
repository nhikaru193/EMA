import smbus
import time
from BNO055 import BNO055 # BNO055をインポート

class RoverLandingDetector: # land.py のクラス名に合わせてください
    """
    BME280気圧センサーとBNO055慣性測定ユニットを使用して、
    ローバーの着地を検出するためのクラスです。
    """
    BME280_ADDRESS = 0x76
    # I2C_BUS = 1 # この行は残しても消してもOKですが、使用されません

    def __init__(self, bno_sensor, i2c_bus_instance, pressure_change_threshold=0.1, acc_threshold_abs=0.5, # <--- ここが変わる！
                 gyro_threshold_abs=0.5, consecutive_checks=3, timeout=60,
                 calibrate_bno055=True):
        """
        RoverLandingDetectorのコンストラクタです。

        Args:
            bno_sensor (BNO055): 既に初期化されたBNO055センサーのインスタンス。 <--- 新しい引数
            i2c_bus_instance (smbus.SMBus): 既に初期化されたSMBusのインスタンス。 <--- 新しい引数
            pressure_change_threshold (float): ...
            # ... その他の引数 ...
        """
        self.pressure_change_threshold = pressure_change_threshold
        self.acc_threshold_abs = acc_threshold_abs
        self.gyro_threshold_abs = gyro_threshold_abs
        self.consecutive_checks = consecutive_checks
        self.timeout = timeout
        self.calibrate_bno055 = calibrate_bno055
        self.bno = bno_sensor
        self.i2c = i2c_bus_instance

        self.i2c = i2c_bus_instance # <--- 外部から渡されたインスタンスを使う
        self.bno = bno_sensor       # <--- 外部から渡されたインスタンスを使う

        # BME280関連の補正データ (クラス内部でのみ使用)
        self._digT = []
        self._digP = []
        self._digH = []
        self._t_fine = 0.0

        # 着地検出の状態を保持する変数
        self.previous_pressure = None
        self.landing_count = 0
        self.start_time = None
        self.last_check_time = None

    # ... 後続のメソッド (_init_bme280, _read_compensate_bme280 など) は変更なし ...

    def _init_bme280(self):
        """BME280センサーを初期化します。"""
        self.i2c.write_byte_data(self.BME280_ADDRESS, 0xF2, 0x01)
        self.i2c.write_byte_data(self.BME280_ADDRESS, 0xF4, 0x27)
        self.i2c.write_byte_data(self.BME280_ADDRESS, 0xF5, 0xA0)

    def _read_compensate_bme280(self):
        """BME280の補正データを読み込みます。"""
        dat_t = self.i2c.read_i2c_block_data(self.BME280_ADDRESS, 0x88, 6)
        self._digT = [(dat_t[1] << 8) | dat_t[0], (dat_t[3] << 8) | dat_t[2], (dat_t[5] << 8) | dat_t[4]]
        for i in range(1, 2):
            if self._digT[i] >= 32768:
                self._digT[i] -= 65536

        dat_p = self.i2c.read_i2c_block_data(self.BME280_ADDRESS, 0x8E, 18)
        self._digP = [(dat_p[i+1] << 8) | dat_p[i] for i in range(0, 18, 2)]
        for i in range(1, 8):
            if self._digP[i] >= 32768:
                self._digP[i] -= 65536

        dh = self.i2c.read_byte_data(self.BME280_ADDRESS, 0xA1)
        dat_h = self.i2c.read_i2c_block_data(self.BME280_ADDRESS, 0xE1, 8)
        self._digH = [dh, (dat_h[1] << 8) | dat_h[0], dat_h[2],
                      (dat_h[3] << 4) | (0x0F & dat_h[4]),
                      (dat_h[5] << 4) | ((dat_h[4] >> 4) & 0x0F),
                      dat_h[6]]
        if self._digH[1] >= 32768:
            self._digH[1] -= 65536
        for i in range(3, 4):
            if self._digH[i] >= 32768:
                self._digH[i] -= 65536
        if self._digH[5] >= 128:
            self._digH[5] -= 256

    def _bme280_compensate_t(self, adc_T):
        """温度を補正します。"""
        var1 = (adc_T / 8.0 - self._digT[0] * 2.0) * self._digT[1] / 2048.0
        var2 = ((adc_T / 16.0 - self._digT[0]) ** 2) * self._digT[2] / 16384.0
        self._t_fine = var1 + var2
        t = (self._t_fine * 5 + 128) / 256 / 100
        return t

    def _bme280_compensate_p(self, adc_P):
        """気圧を補正します。"""
        p = 0.0
        var1 = self._t_fine - 128000.0
        var2 = var1 * var1 * self._digP[5]
        var2 += (var1 * self._digP[4]) * 131072.0
        var2 += self._digP[3] * 3.435973837e10
        var1 = (var1 * var1 * self._digP[2]) / 256.0 + (var1 * self._digP[1]) * 4096
        var1 = (1.407374884e14 + var1) * (self._digP[0] / 8589934592.0)
        if var1 == 0:
            return 0
        p = (1048576.0 - adc_P) * 2147483648.0 - var2
        p = (p * 3125) / var1
        var1 = self._digP[8] * (p / 8192.0)**2 / 33554432.0
        var2 = self._digP[7] * p / 524288.0
        p = (p + var1 + var2) / 256 + self._digP[6] * 16.0
        return p / 256 / 100

    def get_pressure_and_temperature(self):
        """BME280から気圧と温度を読み込み、補正して返します。"""
        dat = self.i2c.read_i2c_block_data(self.BME280_ADDRESS, 0xF7, 8)
        adc_p = (dat[0] << 16 | dat[1] << 8 | dat[2]) >> 4
        adc_t = (dat[3] << 16 | dat[4] << 8 | dat[5]) >> 4
        
        temperature = self._bme280_compensate_t(adc_t)
        pressure = self._bme280_compensate_p(adc_p)
        return pressure, temperature

    def check_landing(self):
        """
        着地条件を監視し、着地判定を行います。
        タイムアウトした場合、条件成立回数に関わらず着地成功とみなします。
        BNO055のキャリブレーションは行わないオプションもありますが、精度が低下する可能性があります。

        Returns:
            bool: 着地が成功した場合はTrue、それ以外はFalseを返します。
        """
        # センサーの初期化
        self._init_bme280()
        self._read_compensate_bme280()

        if not self.bno.begin():
            print("🔴 BNO055 の初期化に失敗しました。プログラムを終了します。")
            return False

        self.bno.setExternalCrystalUse(True)
        self.bno.setMode(BNO055.OPERATION_MODE_NDOF) # NDOFモードを明示的に設定

        # --- BNO055 キャリブレーション待機 ---
        if self.calibrate_bno055:
            print("\n⚙️ BNO055 キャリブレーション中... センサーをいろんな向きにゆっくり回してください。")
            print("    (ジャイロ、加速度、地磁気が完全キャリブレーション(レベル3)になるのを待ちます)")
            calibration_start_time = time.time()
            while True:
                sys, gyro, accel, mag = self.bno.getCalibration()
                print(f"    現在のキャリブレーション状態 → システム:{sys}, ジャイロ:{gyro}, 加速度:{accel}, 地磁気:{mag} ", end='\r')
                
                # ジャイロ、加速度、地磁気が全てレベル3になるまで待つ
                if gyro == 3 and accel == 3 and mag == 3:
                    print("\n✅ BNO055 キャリブレーション完了！")
                    break
                time.sleep(0.5) # 0.5秒ごとに状態を確認
            print(f"    キャリブレーションにかかった時間: {time.time() - calibration_start_time:.1f}秒\n")
        else:
            print("\n⚠️ BNO055 キャリブレーション待機はスキップされました。センサーデータの精度が低下する可能性があります。")

        print("🛬 着地判定を開始します...")
        print(f"  気圧変化量閾値: < {self.pressure_change_threshold:.2f} hPa")
        print(f"  加速度絶対値閾値: < {self.acc_threshold_abs:.2f} m/s² (X, Y, Z軸)")
        print(f"  角速度絶対値閾値: < {self.gyro_threshold_abs:.2f} °/s (X, Y, Z軸)")
        print(f"  連続成立回数: {self.consecutive_checks}回")
        print(f"  タイムアウト: {self.timeout}秒\n")

        self.landing_count = 0
        self.start_time = time.time()
        self.last_check_time = time.time()
        self.previous_pressure = None # 最初のループではNoneのまま

        try:
            # ヘッダーを一度だけ出力
            print(f"{'Timestamp(s)':<15}{'Elapsed(s)':<12}{'Pressure(hPa)':<15}{'Pressure_Chg(hPa)':<18}{'Acc_X':<8}{'Acc_Y':<8}{'Acc_Z':<8}{'Gyro_X':<8}{'Gyro_Y':<8}{'Gyro_Z':<8}")
            print("-" * 120)

            while True:
                current_time = time.time()
                elapsed_total = current_time - self.start_time

                # タイムアウト判定
                if elapsed_total > self.timeout:
                    print(f"\n\n⏰ タイムアウト ({self.timeout}秒経過)。条件成立回数 {self.landing_count} 回でしたが、強制的に着地判定を成功とします。")
                    return True
                
                # データ取得と表示は一定間隔で行う
                if (current_time - self.last_check_time) < 0.2: # 約0.2秒間隔でデータ取得と表示
                    time.sleep(0.01) # 短いスリープでCPU負荷軽減
                    continue
                
                self.last_check_time = current_time

                # センサーデータの取得
                current_pressure, _ = self.get_pressure_and_temperature()
                acc_x, acc_y, acc_z = self.bno.getVector(BNO055.VECTOR_LINEARACCEL)
                gyro_x, gyro_y, gyro_z = self.bno.getVector(BNO055.VECTOR_GYROSCOPE)

                # 気圧変化量の計算
                pressure_delta = float('inf') # 直前のデータがない場合は無限大に設定
                if self.previous_pressure is not None:
                    pressure_delta = abs(current_pressure - self.previous_pressure)
                
                # データをコンソールに整形して出力 (同じ行を上書き)
                print(f"{current_time:<15.3f}{elapsed_total:<12.1f}{current_pressure:<15.2f}{pressure_delta:<18.2f}{acc_x:<8.2f}{acc_y:<8.2f}{acc_z:<8.2f}{gyro_x:<8.2f}{gyro_y:<8.2f}{gyro_z:<8.2f}", end='\r')

                # 着地条件の判定
                is_landing_condition_met = (
                    pressure_delta <= self.pressure_change_threshold and  # 気圧の変化量が閾値以下
                    abs(acc_x) < self.acc_threshold_abs and              # 各軸の加速度絶対値が閾値以下
                    abs(acc_y) < self.acc_threshold_abs and
                    abs(acc_z) < self.acc_threshold_abs and
                    abs(gyro_x) < self.gyro_threshold_abs and              # 各軸の角速度絶対値が閾値以下
                    abs(gyro_y) < self.gyro_threshold_abs and
                    abs(gyro_z) < self.gyro_threshold_abs
                )

                # 次のループのために現在の気圧を保存
                self.previous_pressure = current_pressure

                if is_landing_condition_met:
                    self.landing_count += 1
                    # 画面表示が上書きされる前にメッセージを確実に出力するために改行
                    print(f"\n💡 条件成立！連続判定中: {self.landing_count}/{self.consecutive_checks} 回")
                else:
                    if self.landing_count > 0:
                        # 画面表示が上書きされる前にメッセージを確実に出力するために改行
                        print(f"\n--- 条件不成立。カウントをリセットします ({self.landing_count} -> 0) ---")
                    self.landing_count = 0

                # 連続成立回数の確認
                if self.landing_count >= self.consecutive_checks:
                    print(f"\n🎉 着地判定成功！連続 {self.consecutive_checks} 回条件が成立しました！")
                    return True # 着地判定成功で関数を終了

        except KeyboardInterrupt:
            print("\n\nプログラムがユーザーによって中断されました。")
            return False
        except Exception as e:
            print(f"\n\n🚨 エラーが発生しました: {e}")
            return False
        finally:
            print("\n--- 判定処理を終了します ---")
