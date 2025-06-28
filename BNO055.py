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
    OPERATION_MODE_NDOF = 0X0C

    VECTOR_ACCELEROMETER = 0x08
    VECTOR_EULER = 0x1A
    VECTOR_MAGNETOMETER = 0x0E
    VECTOR_GYROSCOPE = 0x14
    VECTOR_LINEARACCEL = 0x28
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
            return False

        self.setMode(BNO055.OPERATION_MODE_CONFIG)
        self.writeBytes(BNO055.BNO055_SYS_TRIGGER_ADDR, [0x20])
        time.sleep(2)
        while self.readBytes(BNO055.BNO055_CHIP_ID_ADDR)[0] != BNO055.BNO055_ID:
            time.sleep(0.1)
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
        xyz = struct.unpack('hhh', struct.pack('BBBBBB', *buf))
        if vectorType == BNO055.VECTOR_MAGNETOMETER:
            scalingFactor = 16.0
        elif vectorType == BNO055.VECTOR_GYROSCOPE:
            scalingFactor = 900.0
        elif vectorType == BNO055.VECTOR_EULER:
            scalingFactor = 16.0
        elif vectorType == BNO055.VECTOR_GRAVITY:
            scalingFactor = 100.0
        else:
            scalingFactor = 1.0
        return tuple([i / scalingFactor for i in xyz])

    def getQuat(self):
        buf = self.readBytes(BNO055.BNO055_QUATERNION_DATA_W_LSB_ADDR, 8)
        wxyz = struct.unpack('hhhh', struct.pack('BBBBBBBB', *buf))
        return tuple([i * (1.0 / (1 << 14)) for i in wxyz])

    def get_heading(self):
        return self.getVector(self.VECTOR_EULER)[0]

    def readBytes(self, register, numBytes=1):
        return self._bus.read_i2c_block_data(self._address, register, numBytes)

    def writeBytes(self, register, byteVals):
        return self._bus.write_i2c_block_data(self._address, register, byteVals)

# ====================== 実行部 ======================

if __name__ == '__main__':
    bno = BNO055()
    if not bno.begin():
        print("Error initializing device")
        exit()
    time.sleep(1)
    bno.setMode(BNO055.OPERATION_MODE_NDOF)
    time.sleep(5)
    bno.setExternalCrystalUse(True)

    while True:
        for i in range(20):
            euler = bno.getVector(BNO055.VECTOR_EULER)
            print(euler)
            heading = euler[0]
            print(heading)
            print(bno.get_heading())
            time.sleep(0.1)
