import RPi.GPIO as GPIO
import time
import pigpio
import board
import busio
import numpy as np
import cv2
from picamera2 import Picamera2
import math
from motor import MotorDriver
from BNO055 import BNO055
import following

class Parakai:
    def __init__(self, bno: BNO055, goal_location :list):
        # GPIO
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        # Motor
        self.driver = MotorDriver(
            PWMA=12, AIN1=23, AIN2=18,
            PWMB=19, BIN1=16, BIN2=26,
            STBY=21
        )

        # GPS
        self.RX_PIN = 17
        self.pi = pigpio.pi()
        self.pi.bb_serial_read_close(self.RX_PIN)   #追加
        self.pi.bb_serial_read_open(self.RX_PIN, 9600, 8)
        self.bno = bno
        # Picamera2
        self.picam2 = Picamera2()
        config = self.picam2.create_still_configuration(main={"size": (320, 480)})
        self.picam2.configure(config)
        self.picam2.start()
        time.sleep(1)

        # Destination
        self.destination = goal_location

    def run(self):
        try:
            while True:
                # 1. GPS取得
                current = self._get_gps()
                # 2. 方位・距離
                bearing = self._calc_bearing(current, self.destination)
                dist = self._calc_distance(current, self.destination)
                # 3. 回頭
                self._turn_to(bearing)
                # 4. 赤色検出
                loc = self._detect_red_grid(min_red_pixel_ratio_per_cell=0.1)
                # 5. 回避
                self._avoid(loc)
                # 6. 確認スキャン
                if self._confirmation_scan(bearing):
                    break
        except Exception as e:
            print(f"Error: {e}")
            self.driver.motor_stop_brake()
        finally:
            self.driver.cleanup()
            self.picam2.close()

    def _get_gps(self):
        end = time.time()+5
        while time.time()<end:
            cnt, data = self.pi.bb_serial_read(self.RX_PIN)
            if cnt and data and b"$GNRMC" in data:
                txt = data.decode(errors="ignore")
                for line in txt.split("\n"):
                    if "$GNRMC" in line:
                        p = line.split(',')
                        if len(p)>6 and p[2]=='A':
                            return self._coord(p[3],p[4]), self._coord(p[5],p[6])
            time.sleep(0.1)
        print("GPS timeout")
        return (0.0,0.0)

    def _coord(self, coord, dir):
        d = int(coord[:2]) if dir in ['N','S'] else int(coord[:3])
        m = float(coord[2:]) if dir in ['N','S'] else float(coord[3:])
        val = d + m/60
        return -val if dir in ['S','W'] else val

    def _calc_bearing(self, cur, dst):
        lat1,lon1,lat2,lon2 = map(math.radians,(cur[0],cur[1],dst[0],dst[1]))
        dlon=lon2-lon1
        y=math.sin(dlon)*math.cos(lat2)
        x=math.cos(lat1)*math.sin(lat2)-math.sin(lat1)*math.cos(lat2)*math.cos(dlon)
        return (math.degrees(math.atan2(y,x))+360)%360

    def _calc_distance(self, cur, dst):
        lat1,lon1,lat2,lon2 = map(math.radians,(cur[0],cur[1],dst[0],dst[1]))
        r=6378137.0; dlat=lat2-lat1; dlon=lon2-lon1
        a=math.sin(dlat/2)**2+math.cos(lat1)*math.cos(lat2)*math.sin(dlon/2)**2
        return r*2*math.atan2(math.sqrt(a),math.sqrt(1-a))

    def _turn_to(self, target):
        # 簡易回頭：誤差20度以内になるまで旋回
        while True:
            cur = self.bno.get_heading()
            err = (target-cur+180+360)%360-180
            if abs(err)<20: break
            if err<0: self.driver.petit_left(0,80);self.driver.petit_left(80,0)
            else:    self.driver.petit_right(0,80);self.driver.petit_right(80,0)
            time.sleep(0.1)
            self.driver.motor_stop_brake()
        self.driver.motor_stop_brake()

    def _detect_red_grid(self, min_red_pixel_ratio_per_cell=0.1):
        frame = self.picam2.capture_array()
        img = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        h,w,_=img.shape; ch,hw=w//3,h//2
        hsv=cv2.cvtColor(cv2.GaussianBlur(img,(5,5),0),cv2.COLOR_BGR2HSV)
        mask=cv2.bitwise_or(cv2.inRange(hsv,(0,100,100),(10,255,255)),
                            cv2.inRange(hsv,(160,100,100),(180,255,255)))
        cells=[(h*0//2,h*1//2,0,ch),(h*0//2,h*1//2,ch,2*ch),(h*0//2,h*1//2,2*ch,w),
               (h*1//2,h*2//2,0,ch),(h*1//2,h*2//2,ch,2*ch),(h*1//2,h*2//2,2*ch,w)]
        bottom = cells[3:6]
        reds=[np.count_nonzero(mask[y0:y1,x0:x1])/( (y1-y0)*(x1-x0) )
              for y0,y1,x0,x1 in bottom]
        if max(reds)<min_red_pixel_ratio_per_cell: return 'none'
        if reds[0]>reds[2]: return 'left'
        if reds[2]>reds[0]: return 'right'
        return 'middle'

    def _avoid(self, loc):
        if loc=='left': self._turn_to((self.bno.get_heading()+90)%360)
        elif loc=='right': self._turn_to((self.bno.get_heading()-90)%360)
        elif loc=='middle': self._turn_to((self.bno.get_heading()+120)%360)
        following.follow_forward(self.driver, self.bno, base_speed=100, duration_time=5)

    def _confirmation_scan(self, orig):
        for angle in (0, -30, 30):
            self._turn_to((orig+angle)%360)
            res=self._detect_red_grid(min_red_pixel_ratio_per_cell=0.1)
            if res!='none': return False
        return True

    def _cleanup(self):
        self.driver.cleanup()
        self.pi.bb_serial_read_close(self.RX_PIN)
        self.pi.stop()
        self.picam2.close()
        self.driver.cleanup()

if __name__ == '__main__':
    rover = RoverControl()
    rover.run()
