from BNO055 import BNO055
import BME280
import time

class Landing:
    def __init__(bno: BNO055, counter, timeout, p_threshold = 0.20, h_threshold = 0.10):
        self.bno = bno
        self.counter = counter
        self.timeout = timeout
        self.p_threshold = p_threshold
        self.h_threshold = h_threshold
        self.start_time = time.time()
        
    def run(self):
        print("着地判定を開始します")
        print("方位角変化量:第1シーケンス")
        self.start_time = time.time()
        while True:
            current_time = time.time()
            before_heading = self.bno.getVector(BNO055.VECTOR_EULER)[0]
            print(f"{before_heading}")
            time.sleep(1)
            after_heading = self.bno.getVector(BNO055.VECTOR_EULER)[0]
            delta_heading = after_heading - before_heading
            if delta_heading < self.h_threshold:
                print("方位角変化量による着地判定の終了")
