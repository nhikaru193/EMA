import RPi.GPIO as GPIO
import time
import struct
import time
import pigpio

def circuit(t_melt = 1):
    """
    #われらの愛すべきポンコツコード
    NICHROME_PIN = 25
    HEATING_TIME = 2.0
    GPIO.setmode(GPIO.BCM)
    print("ニクロム線溶断シーケンスを開始します。")
    
    GPIO.setup(NICHROME_PIN, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
    GPIO.setup(NICHROME_PIN, GPIO.OUT, initial=GPIO.LOW)
    try:
        time.sleep(1)
        print(f"GPIO{NICHROME_PIN} をHIGHに設定し、ニクロム線をオンにします。")
        time.sleep(1)

        print(f"{HEATING_TIME}秒間、加熱します...")
        GPIO.output(NICHROME_PIN, GPIO.HIGH)
        time.sleep(HEATING_TIME)
        
        print(f"GPIO{NICHROME_PIN} をLOWに設定し、ニクロム線をオフにします。")
        GPIO.output(NICHROME_PIN, GPIO.LOW)
        time.sleep(0.2)
       
        print("シーケンスが正常に完了しました。")

    except KeyboardInterrupt:
        print("プログラムが中断されました。")
        GPIO.output(NICHROME_PIN, GPIO.LOW)

    finally:
        #GPIO.cleanup()
        print("GPIOのクリーンアップを実行しました。")
    """

    #2024の先輩コード
    meltPin = 25
    pi = pigpio.pi()
    pi.write(meltPin, 0)
    time.sleep(1)
    pi.write(meltPin, 1)
    time.sleep(t_melt)
    pi.write(meltPin, 0)
    time.sleep(1)

    pi.stop()
    
if __name__ == '__main__':
    # 許容誤差を調整したい場合は、ここで値を設定できます
    # 例: detector = FlagDetector(triangle_tolerance=0.8)
    circuit()
