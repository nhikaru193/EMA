import time
# ご指定の通り、ファイル名 BNO055.py から BNO055 クラスをインポートします。
from BNO055 import BNO055

def main():
    # BNO055センサーのインスタンスを作成
    bno = BNO055()

    print("BNO055の初期化中...")
    # センサーの初期化
    if not bno.begin():
        print("BNO055の初期化に失敗しました。配線とI2Cアドレスを確認してください。")
        return

    print("BNO055の初期化に成功しました。")
    # 加速度計を含む最適なフュージョンモードであるNDOFモードに設定
    # BNO055.VECTOR_LINEARACCEL はNDOFモードで最も正確なデータを提供します。
    bno.setMode(BNO055.OPERATION_MODE_NDOF)
    time.sleep(1) # センサーが安定するのを待つ

    # オプション: 精度向上のために外部クリスタルを使用する場合
    # bno.setExternalCrystalUse(True)
    # time.sleep(0.1)

    print("Z軸方向の直線加速度 (m/s^2) を測定中... Ctrl+Cで終了します。")
    try:
        while True:
            # 直線加速度のデータを取得 (X, Y, Z軸)
            # getVectorメソッドは (x, y, z) のタプルを返します
            # VECTOR_LINEARACCEL は重力成分を除いた加速度です
            linear_accel = bno.getVector(BNO055.VECTOR_LINEARACCEL)

            # Z軸方向の加速度はタプルの3番目の要素 (インデックス2) です
            z_accel = linear_accel[2]

            print(f"Z軸直線加速度: {z_accel:.2f} m/s^2")

            time.sleep(0.1) # 100ミリ秒ごとに読み取り

    except KeyboardInterrupt:
        print("\nプログラムを終了します。")
    finally:
        # 終了前にセンサーをCONFIGモードに戻すのが良い慣例です
        bno.setMode(BNO055.OPERATION_MODE_CONFIG)

if __name__ == '__main__':
    main()
