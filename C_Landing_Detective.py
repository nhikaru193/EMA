from BNO055 import BNO055
import BME280

class Landing:
    def __init__(bno: BNO055, counter, timeout, p_threshold, h_threshold):
        self.bno = bno
        self.counter = counter
        self.timeout = timeout
        
def run(self):
    print("着地判定を開始します")
    while True:
        before_heading = self.bno.getVector(EULER_)[0]
        time.sleep(1)
        after_heading = self.bno.getVector()[0]
        
    

