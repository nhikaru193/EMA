#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import serial
import pynmea2
import time

def main():
    # シリアルポートのオープン (/dev/serial0 がシンボリックリンクで実際は ttyS0 か ttyAMA0)
    # ボーレートはデフォルト 9600 bps。必要に応じて変更可。
    port = "/dev/serial0"
    baudrate = 9600

    try:
        ser = serial.Serial(port, baudrate=baudrate, timeout=1)
    except serial.SerialException as e:
        print(f"シリアルポートオープンエラー: {e}")
        return

    print("L76K GPS モジュールからのデータを読み込み中...")

    while True:
        try:
            # 1行分のNMEAを読み込む
            line = ser.readline().decode('ascii', errors='replace').strip()

            # NMEAセンテンスである場合
            if line.startswith('$'):
                try:
                    msg = pynmea2.parse(line)
                except pynmea2.ParseError:
                    # パース失敗は無視
                    continue

                # GGAセンテンスから取得可能な緯度・経度・衛星数など
                if isinstance(msg, pynmea2.types.talker.GGA):
                    lat = msg.latitude    # 例: 35.123456 など
                    lon = msg.longitude   # 例: 139.123456 など
                    num_sats = msg.num_sats
                    altitude = msg.altitude  # 海抜高 (m)
                    # 緯度・経度が0.0の場合は未受信の可能性がある
                    if lat != 0.0 and lon != 0.0:
                        print(f"[GGA] 緯度: {lat:.6f}, 経度: {lon:.6f}, 衛星数: {num_sats}, 海抜: {altitude}m")
                # RMCセンテンスから取得可能なUTC時刻・ステータス・速度など
                elif isinstance(msg, pynmea2.types.talker.RMC):
                    status = msg.status  # 'A' = 有効, 'V' = 無効
                    lat = msg.latitude
                    lon = msg.longitude
                    speed_knots = msg.spd_over_grnd   # ノット
                    true_course = msg.true_course     # 方位角
                    # 有効なデータなら出力
                    if status == 'A':
                        print(f"[RMC] 緯度: {lat:.6f}, 経度: {lon:.6f}, 速度: {speed_knots} kn, 方位: {true_course}°")
        except KeyboardInterrupt:
            print("\n終了します。")
            break
        except Exception as e:
            # 不明な例外はログのみ
            print(f"エラー発生: {e}")
            time.sleep(1)
            continue

    ser.close()

if __name__ == "__main__":
    main()
