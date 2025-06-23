import RPi.GPIO as GPIO
import time

# 回路図の「GPIO25」はBCMモードの25番ピンを指す
NICHROME_PIN = 25

# ニクロム線をオンにしておく時間（秒）
HEATING_DURATION_SECONDS = 3.0

# GPIOのモードをBCMに設定（GPIO番号で指定）
GPIO.setmode(GPIO.BCM)

# GPIO25を出力モードに設定し、初期状態をLOW(オフ)に設定
GPIO.setup(NICHROME_PIN, GPIO.OUT, initial=GPIO.LOW)

print("ニクロム線溶断シーケンスを開始します。")

# try...finally構文で、プログラムがエラーで中断しても必ずGPIOをクリーンアップする
try:
    # --- ニクロム線オン ---
    print(f"GPIO{NICHROME_PIN} をHIGHに設定し、ニクロム線をオンにします。")
    GPIO.output(NICHROME_PIN, GPIO.HIGH)

    # --- 指定時間待機 ---
    print(f"{HEATING_DURATION_SECONDS}秒間、加熱します...")
    time.sleep(HEATING_DURATION_SECONDS)

    # --- ニクロム線オフ ---
    print(f"GPIO{NICHROME_PIN} をLOWに設定し、ニクロム線をオフにします。")
    GPIO.output(NICHROME_PIN, GPIO.LOW)
    print("シーケンスが正常に完了しました。")

except KeyboardInterrupt:
    print("プログラムが中断されました。")
    GPIO.output(NICHROME_PIN, GPIO.LOW)

finally:
    # --- GPIOの設定リセット ---
    GPIO.cleanup()
    print("GPIOのクリーンアップを実行しました。")
