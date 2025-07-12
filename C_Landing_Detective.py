from BNO055 import BNO055
import BME280
import time
import fusing
import RPi.GPIO as GPIO
import struct

class Landing:
    def __init__(bno: BNO055, p_counter = 3, h_counter = 3, timeout = 40, p_threshold = 0.20, h_threshold = 0.10):
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
            if delta_heading < self.h_threshold:
                self.h_counter = self.h_counter - 1
                print(f"方位角着地判定{self.h_counter}回成功！")
                if self.h_counter == 0:
                    print("方位角:変化量による着地判定")
                    break
            else:
                self.h_counter = max_counter
                print("着地判定失敗。再度判定を行います")
            delta_time = current_time - self.start_time
            if delta_time > self.timeout:
                print("方位角:timeoutによる着地判定")

        #環境センサの初期設定
        BME280.inin_bme280()
        BME280.read_compensate()
        print("気圧変化量:第2シーケンス")
        self.start_time = time.time()
        max_counter =self.p_counter
        
        #気圧着地判定
        while True:
            current_time = time.time()
            before_pressure = BME280.get_pressure()
            print(f"t = {current_time}||pressure = {before_pressure}")
            time.sleep(5)
            after_pressure = BME280.get_pressure()
            delta_pressure = after_pressure - before_pressure
            if delta_pressure < self.p_threshold:
                self.p_counter = self.p_counter - 1
                print(f"気圧着地判定{self.p_counter}回成功！")
                if self.p_counter == 0:
                    print("気圧:変化量による着地判定")
                    break
            else:
                self.p_counter = max_counter
                print("着地判定失敗。再度判定を行います")
            delta_time = current_time - self.start_time
            if delta_time > self.timeout:
                print("気圧:timeoutによる着地判定")

        #溶断回路作動
        print("着地判定正常終了。テグス溶断シーケンスに入ります")
        time.sleep(3)
        #fusing.circuit()
