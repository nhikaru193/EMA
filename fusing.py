import RPi.GPIO as GPIO
import time
import struct

def circuit():
    NICHROME_PIN = 25
    HEATING_TIME = 2.0
    GPIO.setmode(GPIO.BCM)
    print("ニクロム線溶断シーケンスを開始します。")
    GPIO.setup(NICHROME_PIN, GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(NICHROME_PIN, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
    
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
        GPIO.cleanup()
        print("GPIOのクリーンアップを実行しました。")

