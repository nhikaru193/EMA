import time
import math
from BNO055 import BNO055

def main():
    bno = BNO055()

    print("BNO055の初期化中...")
    if not bno.begin():
        print("BNO055の初期化に失敗しました。配線とI2Cアドレスを確認してください。")
        return

    print("BNO055の初期化に成功しました。")
    # 加速度計を含む最適なフュージョンモードであるNDOFモードに設定
    bno.setMode(BNO055.OPERATION_MODE_NDOF)
    time.sleep(1)

    print("全加速度 (重力込み) と直線加速度 (重力なし) を測定中 (m/s^2)... Ctrl+Cで終了します。")
    print("形式:")
    print("  全加速度 [X, Y, Z] | 大きさ")
    print("  直線加速度 [X, Y, Z] | 大きさ")
    print("-" * 50)

    try:
        while True:
            # 全加速度 (重力加速度を含む) のデータを取得
            # BNO055.VECTOR_ACCELEROMETER は生の加速度センサーデータに近い値です
            total_accel = bno.getVector(BNO055.VECTOR_ACCELEROMETER)
            ax, ay, az = total_accel
            total_accel_magnitude = math.hypot(ax, ay, az)

            # 直線加速度 (重力加速度を除去) のデータを取得
            # BNO055.VECTOR_LINEARACCEL はBNO055内部で重力成分が除去された値です
            linear_accel = bno.getVector(BNO055.VECTOR_LINEARACCEL)
            lx, ly, lz = linear_accel
            linear_accel_magnitude = math.hypot(lx, ly, lz)

            # 出力
            print(f"全加速度:    X={ax:.2f}, Y={ay:.2f}, Z={az:.2f} | 大きさ={total_accel_magnitude:.2f} m/s^2")
            print(f"直線加速度: X={lx:.2f}, Y={ly:.2f}, Z={lz:.2f} | 大きさ={linear_accel_magnitude:.2f} m/s^2")
            print("-" * 50)

            time.sleep(0.1)

    except KeyboardInterrupt:
        print("\nプログラムを終了します。")
    finally:
        bno.setMode(BNO055.OPERATION_MODE_CONFIG)

if __name__ == '__main__':
    main()
