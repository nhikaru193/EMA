from BNO055 import BNO055
import BME280
import time
import fusing

class Landing:
    def __init__(bno: BNO055, p_counter = 3, h_counter = 3, timeout = 120, p_threshold = 0.20, h_threshold = 0.10):
        self.bno = bno
        self.p_counter = p_counter
        self.h_counter = h_counter
        self.timeout = timeout
        self.p_threshold = p_threshold
        self.h_threshold = h_threshold
        self.start_time = time.time()
        
    def run(self):
        print("着地判定を開始します")
        print("方位角変化量:第1シーケンス")
        self.start_time = time.time()
        max_counter =self.h_counter
        #heading着地判定
        while True:
            current_time = time.time()
            before_heading = self.bno.getVector(BNO055.VECTOR_EULER)[0]
            print(f"t = {current_time}||heading = {before_heading}")
            time.sleep(1)
            after_heading = self.bno.getVector(BNO055.VECTOR_EULER)[0]
            delta_heading = after_heading - before_heading
            if delta_heading > self.h_threshold:
                self.h_counter = self.h_counter - 1
                print(f"方位角着地判定{self.h_counter}回成功！")
                if self.h_counter == 0:
                    print("方位角:変化量による着地判定")
                    break
            else:
                self.h_counter = max_counter
            delta_time = current_time - self.start_time
            if delta_time > self.timeout:
                print("方位角:timeoutによる着地判定")

        #環境センサの初期設定
        
        #気圧着地判定
        while True:
            current_time = time.time()
            before_pressure = BME280.
            print(f"t = {current_time}||heading = {before_heading}")
            time.sleep(1)
            after_heading = self.bno.getVector(BNO055.VECTOR_EULER)[0]
            delta_heading = after_heading - before_heading
            if delta_heading > self.h_threshold:
                self.h_counter = self.h_counter - 1
                print(f"方位角着地判定{self.h_counter}回成功！")
                if self.h_counter == 0:
                    print("方位角:変化量による着地判定")
                    break
            else:
                self.h_counter = max_counter
            delta_time = current_time - self.start_time
            if delta_time > self.timeout:
                print("方位角:timeoutによる着地判定")
