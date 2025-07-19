import smbus
import time
from BNO055 import BNO055
import BME280

class RD:
    def __init__(self, bno: BNO055, p_counter = 3, p_threshold = 2, timeout = 300):
        self.bno = bno
        self.p_counter = p_counter
        self.p_threshold = p_threshold
        self.timeout = timeout

    def run(self):
        BME280.init_bme280()
        BME280.read_compensate()
        start_time = time.time()
        base_pressure = BME280.get_pressure()
        max_counter = self.p_counter
        print(f"!!!!!!圧力閾値:{self.p_threshold} | タイムアウト:{self.timeout} で放出判定を行います!!!!!!")
        while True:
            pressure = BME280.get_pressure()
            delta_pressure = pressure - base_pressure
            ax, ay, az = self.bno.getVector(BNO055.VECTOR_ACCELEROMETER)
            current_time = time.time()
            e_time = current_time - start_time
            print(f"t:{e_time} | p:{pressure} | ax:{ax} | ay:{ay} | az:{az} |")
            if delta_pressure > self.p_threshold:
                self.p_counter = 3
                if self.p_counter == 0:
                    print("気圧変化による放出判定に成功しました")
                    break
            else:
                self.p_counter = max_counter
            if e_time > self.timeout:
                print("タイムアウトによる放出判定に成功しました")
                break
            time.sleep(0.4)
        print("放出判定を終了します")
        #ここから先に無線機の電源とgpsの通信を開始するコードを作る   
