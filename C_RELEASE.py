import smbus
import time
from BNO055 import BNO055
import BME280
import csv
import os

class RD:
    def __init__(self, bno: BNO055, p_counter = 5, p_threshold = 0.15, timeout = 5400): #能代:p_threshold = 0.12　ARLISSは高高度からの投下であるため終端速度まで達することができると仮定し、ロケット内での誤検出を防ぐため、pthreshold, p_counterは厳しめに設定した
        self.bno = bno
        self.p_counter = p_counter
        self.p_threshold = p_threshold
        self.timeout = timeout

    def run(self):
        current_time_str = time.strftime("%m%d-%H%M%S") #現在時刻をファイル名に含める
        filename = f"bme280_data_{current_time_str}.csv"
        path_to = "/home/EMA/_csv"
        filename = os.path.join(path_to, filename)

        try:
            with open(filename, "w", newline='') as f: # newline='' はCSV書き込みのベストプラクティス #withでファイルを安全に開く
                writer = csv.writer(f)
                # CSVヘッダーを書き込む
                writer.writerow(["Time", "Pressure(hPa)", "Acceleration_X(m/s^2)", "Acceleration_Y(m/s^2)", "Acceleration_Z(m/s^2)"])
                print(f"データロギングを開始します。ファイル名: {filename}")
                BME280.init_bme280()
                BME280.read_compensate()
                time.sleep(0.5)
                start_time = time.time()
                max_counter = self.p_counter
                print(f"!!!!!!圧力閾値:{self.p_threshold} | タイムアウト:{self.timeout} で放出判定を行います!!!!!!")
                while True:
                    base_pressure = BME280.get_pressure()
                    time.sleep(1)
                    pressure = BME280.get_pressure()
                    ax, ay, az = self.bno.getVector(BNO055.VECTOR_ACCELEROMETER)
                    current_time = time.time()
                    e_time = current_time - start_time
                    print(f"t:{e_time:.2f} | p:{base_pressure:.2f} | ax:{ax:.2f} | ay:{ay:.2f} | az:{az:.2f} |")
                    writer.writerow([e_time, base_pressure, ax, ay, az])
                    f.flush() # データをすぐにファイルに書き出す (バッファリングさせない)
                    delta_pressure = pressure - base_pressure
                    if delta_pressure > self.p_threshold:
                        self.p_counter -= 1 # デクリメント演算子を使う
                        print(f"気圧による着地判定成功！残り{self.p_counter}回")
                        if self.p_counter == 0:
                            print("気圧変化による放出判定に成功しました")
                            break
                    else:
                        self.p_counter = max_counter # カウンターをリセット

                    if e_time > self.timeout:
                        print("タイムアウトによる放出判定に成功しました")
                        break

        except PermissionError:
            print(f"\nエラー: ファイル '{filename}' への書き込み権限がありません。")

        except Exception as e:
            # その他の予期せぬエラーをキャッチ
            print(f"\n予期せぬエラーが発生しました: {e}")

        finally:
            print("放出判定を終了します")
