import smbus
import time
import struct

class BNO055:
    BNO055_ADDRESS_A = 0x28
    BNO055_ADDRESS_B = 0x29
    BNO055_ID = 0xA0

    POWER_MODE_NORMAL = 0X00
    POWER_MODE_LOWPOWER = 0X01
    POWER_MODE_SUSPEND = 0X02

    OPERATION_MODE_CONFIG = 0X00
    OPERATION_MODE_NDOF = 0X0C # NDOFモードは線形加速度を含む融合データを提供します

    VECTOR_ACCELEROMETER = 0x08 # 全加速度（重力を含む）
    VECTOR_EULER = 0x1A
    VECTOR_MAGNETOMETER = 0x0E
    VECTOR_GYROSCOPE = 0x14
    VECTOR_LINEARACCEL = 0x28 # 線形加速度（重力補償済み）
    VECTOR_GRAVITY = 0x2E

    BNO055_CHIP_ID_ADDR = 0x00
    BNO055_SYS_TRIGGER_ADDR = 0x3F
    BNO055_PWR_MODE_ADDR = 0x3E
    BNO055_PAGE_ID_ADDR = 0x07
    BNO055_OPR_MODE_ADDR = 0x3D
    BNO055_TEMP_ADDR = 0x34
    BNO055_CALIB_STAT_ADDR = 0x35
    BNO055_SYS_STAT_ADDR = 0x39
    BNO055_SYS_ERR_ADDR = 0x3A
    BNO055_SELFTEST_RESULT_ADDR = 0x36
    BNO055_ACCEL_REV_ID_ADDR = 0x01
    BNO055_MAG_REV_ID_ADDR = 0x02
    BNO055_GYRO_REV_ID_ADDR = 0x03
    BNO055_SW_REV_ID_LSB_ADDR = 0x04
    BNO055_SW_REV_ID_MSB_ADDR = 0x05
    BNO055_BL_REV_ID_ADDR = 0x06
    BNO055_QUATERNION_DATA_W_LSB_ADDR = 0x20

    def __init__(self, sensorId=-1, address=0x28):
        self._sensorId = sensorId
        self._address = address
        self._mode = BNO055.OPERATION_MODE_NDOF

    def begin(self, mode=None):
        if mode is None:
            mode = BNO055.OPERATION_MODE_NDOF
        self._bus = smbus.SMBus(1)

        try:
            actual_chip_id = self.readBytes(BNO055.BNO055_CHIP_ID_ADDR)[0]
            print(f"DEBUG: BNO055のアドレス {hex(self._address)} から読み取ったチップID: {hex(actual_chip_id)}")
            print(f"DEBUG: 期待するチップID: {hex(BNO055.BNO055_ID)}")
            if actual_chip_id != BNO055.BNO055_ID:
                print("DEBUG: 期待するチップIDと異なります。1秒待機して再試行します。")
                time.sleep(1)
                actual_chip_id = self.readBytes(BNO055.BNO055_CHIP_ID_ADDR)[0]
                print(f"DEBUG: 1秒後、再度読み取ったチップID: {hex(actual_chip_id)}")
        except Exception as e:
            print(f"DEBUG: チップID読み取り中にエラーが発生しました: {e}")
            return False

        if actual_chip_id != BNO055.BNO055_ID:
            print(f"ERROR: BNO055デバイスのチップIDが期待値 {hex(BNO055.BNO055_ID)} と一致しませんでした ({hex(actual_chip_id)})。")
            return False

        self.setMode(BNO055.OPERATION_MODE_CONFIG)
        self.writeBytes(BNO055.BNO055_SYS_TRIGGER_ADDR, [0x20]) # RST_SYS (システムリセット)
        time.sleep(2) # リセットが完了するまで待機

        retries = 10
        for _ in range(retries):
            if self.readBytes(BNO055.BNO055_CHIP_ID_ADDR)[0] == BNO055.BNO055_ID:
                break
            time.sleep(0.1)
        else:
            print("ERROR: BNO055リセット後もチップIDが確認できませんでした。")
            return False

        time.sleep(0.05)

        self.writeBytes(BNO055.BNO055_PWR_MODE_ADDR, [BNO055.POWER_MODE_NORMAL])
        time.sleep(0.01)
        self.writeBytes(BNO055.BNO055_PAGE_ID_ADDR, [0])
        self.writeBytes(BNO055.BNO055_SYS_TRIGGER_ADDR, [0])
        time.sleep(0.01)
        self.setMode(mode)
        time.sleep(0.02)

        return True

    def setMode(self, mode):
        self._mode = mode
        self.writeBytes(BNO055.BNO055_OPR_MODE_ADDR, [self._mode])
        time.sleep(0.03)

    def setExternalCrystalUse(self, useExternalCrystal=True):
        prevMode = self._mode
        self.setMode(BNO055.OPERATION_MODE_CONFIG)
        time.sleep(0.025)
        self.writeBytes(BNO055.BNO055_PAGE_ID_ADDR, [0])
        self.writeBytes(BNO055.BNO055_SYS_TRIGGER_ADDR, [0x80] if useExternalCrystal else [0])
        time.sleep(0.01)
        self.setMode(prevMode)
        time.sleep(0.02)

    def getSystemStatus(self):
        self.writeBytes(BNO055.BNO055_PAGE_ID_ADDR, [0])
        (sys_stat, sys_err) = self.readBytes(BNO055.BNO055_SYS_STAT_ADDR, 2)
        self_test = self.readBytes(BNO055.BNO055_SELFTEST_RESULT_ADDR)[0]
        return (sys_stat, self_test, sys_err)

    def getRevInfo(self):
        (accel_rev, mag_rev, gyro_rev) = self.readBytes(BNO055.BNO055_ACCEL_REV_ID_ADDR, 3)
        sw_rev = self.readBytes(BNO055.BNO055_SW_REV_ID_LSB_ADDR, 2)
        sw_rev = sw_rev[0] | sw_rev[1] << 8
        bl_rev = self.readBytes(BNO055.BNO055_BL_REV_ID_ADDR)[0]
        return (accel_rev, mag_rev, gyro_rev, sw_rev, bl_rev)

    def getCalibration(self):
        calData = self.readBytes(BNO055.BNO055_CALIB_STAT_ADDR)[0]
        return (calData >> 6 & 0x03, calData >> 4 & 0x03, calData >> 2 & 0x03, calData & 0x03)

    def getTemp(self):
        return self.readBytes(BNO055.BNO055_TEMP_ADDR)[0]

    def getVector(self, vectorType):
        buf = self.readBytes(vectorType, 6)
        xyz = struct.unpack('hhh', struct.pack('BBBBBB', *buf)) # 'hhh'で3つのshort integer (2バイト) に変換

        scalingFactor = 1.0 # デフォルト値

        if vectorType == BNO055.VECTOR_MAGNETOMETER:
            scalingFactor = 16.0 # 1 LSB = 0.0625 uT
        elif vectorType == BNO055.VECTOR_GYROSCOPE:
            # データシートp.60: Gyro (dps) 1LSB = 16 dps. -> 16 LSB/dps
            # raw値を16.0で割るとdpsになる
            scalingFactor = 16.0 # 正しいスケーリングファクター
        elif vectorType == BNO055.VECTOR_EULER:
            scalingFactor = 16.0 # 1 LSB = 0.0625 degrees
        elif vectorType == BNO055.VECTOR_GRAVITY:
            scalingFactor = 100.0 # 1 LSB = 0.01 m/s^2 (データシートp.60, gravity vector default m/s^2)
        elif vectorType == BNO055.VECTOR_ACCELEROMETER or \
             vectorType == BNO055.VECTOR_LINEARACCEL:
            scalingFactor = 100.0 # BNO055のデフォルト設定で 1 LSB = 1 cm/s^2 (0.01 m/s^2)
                                  # m/s^2単位で返すため、raw値を100.0で割る
        else: # 未定義のvectorTypeの場合のフォールバック
            scalingFactor = 1.0

        return tuple([i / scalingFactor for i in xyz])

    def getQuat(self):
        buf = self.readBytes(BNO055.BNO055_QUATERNION_DATA_W_LSB_ADDR, 8)
        wxyz = struct.unpack('hhhh', struct.pack('BBBBBBBB', *buf))
        return tuple([i * (1.0 / (1 << 14)) for i in wxyz])

    def get_heading(self):
        return self.getVector(self.VECTOR_EULER)[0]

    def getLinearAcceleration(self):
        """
        重力成分を除いた線形加速度 (m/s^2) をX, Y, Zのタプルで返します。
        NDOFモードでのみ有効です。
        """
        return self.getVector(self.VECTOR_LINEARACCEL)

    def getTotalAcceleration(self):
        """
        重力成分を含む全加速度 (m/s^2) をX, Y, Zのタプルで返します。
        """
        return self.getVector(self.VECTOR_ACCELEROMETER)

    def readBytes(self, register, numBytes=1):
        return self._bus.read_i2c_block_data(self._address, register, numBytes)

    def writeBytes(self, register, byteVals):
        return self._bus.write_i2c_block_data(self._address, register, byteVals)

# ====================== 実行部 ======================

if __name__ == '__main__':
    bno = BNO055()
    if not bno.begin():
        print("デバイスの初期化に失敗しました。I2C接続と電源を確認してください。")
        exit()
    time.sleep(1)
    
    # 加速度データを取得するためにNDOFモードに設定します
    bno.setMode(BNO055.OPERATION_MODE_NDOF)
    print("BNO055をNDOFモードに設定しました。")
    time.sleep(0.1) 

    bno.setExternalCrystalUse(True)
    print("BNO055で外部水晶発振器の使用を設定しました。")
    time.sleep(0.1)

    print("\nセンサー値の読み取りを開始します。Ctrl+Cで終了します。")
    print("--------------------------------------------------")

    while True:
        try:
            # 重力加速度を考慮しない線形加速度を取得
            linear_accel = bno.getLinearAcceleration()
            # 重力加速度を考慮した全加速度を取得
            total_accel = bno.getTotalAcceleration()

            # 上方向の加速度は通常Z軸に対応します
            linear_accel_z = linear_accel[2] # 線形加速度のZ成分
            total_accel_z = total_accel[2]   # 全加速度のZ成分

            print(f"重力考慮なし (線形加速度) Z軸: {linear_accel_z: .3f} m/s^2")
            print(f"重力考慮あり (全加速度) Z軸: {total_accel_z: .3f} m/s^2")
            print("-" * 50)

            time.sleep(0.1) # 読み取り間隔

        except KeyboardInterrupt:
            print("\nプログラムを終了します。")
            break
        except Exception as e:
            print(f"データの読み取り中にエラーが発生しました: {e}")
            print("デバイスが正しく初期化されているか、I2C接続を確認してください。")
            time.sleep(1) # エラー時に少し待って再試行
