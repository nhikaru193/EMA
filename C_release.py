import smbus
import time
from BNO055 import BNO055
import BME280

class Release:
    def __init__(self, bno: BNO055):
        self.bno = bno
        self.t_fine = 0.0
        self.digT = []
        self.digP = []
        self.digH = []
        self.i2c = smbus.SMBus(1)
        self.address = 0x76
        self.pressure_change_threshold = 0.3
        self.acc_z_threshold_abs = 0.5
        self.consecutive_checks = 3
        self.timeout = 60
        self.landing_count = 0
        self.start_time = time.time()
        self.current_time = time.time()
        self.last_check_time = time.time()
        self.initial_pressure = None
        
        
    def check_landing(self, self.pressure_change_threshold, self.acc_z_threshold_abs, self.consecutive_checks, self.timeout):
        BME280.init_bme280()
        BME280.read_compensate()
        print("\n🛬 着地判定開始...")
        print(f"  初期気圧からの変化量閾値: >= {pressure_change_threshold:.2f} hPa")
        print(f"  Z軸加速度絶対値閾値: > {acc_z_threshold_abs:.2f} m/s² (元の条件を維持)")
        print(f"  連続成立回数: {self.consecutive_checks}回")
        print(f"  タイムアウト: {timeout}秒\n")
        self.start_time = time.time()
        self.last_check_time = time.time()
        try:
            while True:
                self.current_time = time.time()
                elapsed_total = self.current_time - self.start_time
                if elapsed_total > timeout:
                    print(f"{self.current_time:<15.3f}{elapsed_total:<12.1f}{'TIMEOUT':<15}{'':<15}{'':<15}{'':<12}")
                    print(f"\n⏰ タイムアウト ({self.timeout}秒経過)。条件成立回数 {self.landing_count} 回でしたが、強制的に着地判定を成功とします。")
                if (self.current_time - self.last_check_time) < 0.2:
                    time.sleep(0.01)
                    continue
                self.last_check_time = self.current_time
                current_pressure, _ = BME280.get_pressure_and_temperature()
                _, _, acc_z = self.bno.getVector(BNO055.VECTOR_LINEARACCEL)
                if initial_pressure is None:
                    initial_pressure = current_pressure
                    print(f"{self.current_time:<15.3f}{elapsed_total:<12.1f}{current_pressure:<15.2f}{initial_pressure:<15.2f}{'-':<15}{acc_z:<12.2f}")
                    print("\n--- 初期気圧設定完了。着地条件監視中... ---")
                    continue # 初回は基準値設定のみで判定はスキップ
                pressure_delta_from_initial = abs(current_pressure - initial_pressure)
                print(f"{self.current_time:<15.3f}{elapsed_total:<12.1f}{current_pressure:<15.2f}{initial_pressure:<15.2f}{pressure_delta_from_initial:<15.2f}{acc_z:<12.2f}")
                is_landing_condition_met = (
                    pressure_delta_from_initial >= self.pressure_change_threshold and  
                    abs(acc_z) > self.acc_z_threshold_abs                 
                )
                if is_landing_condition_met:
                    landing_count += 1
                    print(f"\n💡 条件成立！連続判定中: {landing_count}/{self.consecutive_checks} 回")
                else:
                    if self.landing_count > 0:
                        print(f"\n--- 条件不成立。カウントリセット ({self.landing_count} -> 0) ---")
                    self.landing_count = 0
                if self.landing_count >= self.consecutive_checks:
                    print(f"\n🎉 着地判定成功！連続 {self.consecutive_checks} 回条件成立！")
                    return True
        except KeyboardInterrupt:
            print(f"\n{current_time:<15.3f}{elapsed_total:<12.1f}{current_pressure:<15.2f}{initial_pressure:<15.2f}{pressure_delta_from_initial:<15.2f}{acc_z:<12.2f}") # 中断時の最終データも表示
            print("\n\nプログラムがユーザーによって中断されました。")
            return False
        except Exception as e:
            print(f"\n{current_time:<15.3f}{elapsed_total:<12.1f}{current_pressure:<15.2f}{initial_pressure:<15.2f}{pressure_delta_from_initial:<15.2f}{acc_z:<12.2f}") # エラー時の最終データも表示
            print(f"\n\n🚨 エラーが発生しました: {e}")
            return False
        finally:
            print("\n--- 判定処理終了 ---")
    def run(self):
        is_landed = check_landing(
            self.pressure_change_threshold=0.3, # 気圧の変化量閾値 (hPa)。最初に測定した気圧から0.3hPa以上の変化があったら条件成立
            self.acc_z_threshold_abs=4.0,       # Z軸線形加速度の絶対値閾値 (m/s²)。元の値4.0を維持
            self.consecutive_checks=3,          # 3回連続で条件が満たされたら着地とみなす
            self.timeout=60                     # 30秒以内に判定が行われなければタイムアウトで強制成功
        )
        if is_landed:
            print("\n=== ローバーの放出を確認しました！ ===")
        else:
            print("\n=== ローバーの放出は確認できませんでした。 ===")
                
                
                    
                
        
        
        


        
        
        
