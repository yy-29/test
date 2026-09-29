#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
CanSat センサ・無線 実値読み取りテスト (Raspberry Pi 4 用)

sensor_wireless_test.py で「接続(検出)」がOKになったあと、
実際にセンサから値が読み取れるか／IM920で実データを送受信できるかを確認する。

対象 (CanSat_回路図.svg より):
  - 気圧 BME280     (0x76/0x77) : 気温・気圧・湿度 を生レジスタから直接計算 (追加ライブラリ不要)
  - IMU  BNO055     (0x28)      : オイラー角(Heading/Roll/Pitch)を生レジスタから直接読み取り
  - ToF  VL53L1X    (0x29)      : 距離[mm] を adafruit-circuitpython-vl53l1x 経由で読み取り
  - GPS  SAM-M10Q   (0x42)      : I2C(DDC)経由でNMEAセンテンスの生データを読み取り
  - 無線 IM920 (UART)           : テストデータを実際に送信 / 受信を一定時間リッスン

事前準備:
  sudo apt update
  sudo apt install -y python3-smbus2 python3-serial
  # VL53L1Xのみ追加ライブラリが必要:
  pip3 install adafruit-circuitpython-vl53l1x adafruit-blinka

使い方:
  python3 sensor_value_read_test.py
  python3 sensor_value_read_test.py --im920-send "HELLO" --im920-listen 5
  python3 sensor_value_read_test.py --only bme280,bno055
"""

import argparse
import struct
import sys
import time

I2C_BUS = 1

BME280_ADDR_CANDIDATES = [0x76, 0x77]
BNO055_ADDR = 0x28
VL53L1X_ADDR = 0x29
GPS_ADDR = 0x42


# ============================================================
# BME280 (気圧・気温・湿度) — 生レジスタ直読み、追加ライブラリ不要
# ============================================================
def read_bme280(bus_num=I2C_BUS):
    from smbus2 import SMBus

    last_err = None
    for addr in BME280_ADDR_CANDIDATES:
        try:
            with SMBus(bus_num) as bus:
                chip_id = bus.read_byte_data(addr, 0xD0)
                if chip_id not in (0x60, 0x58):  # 0x60=BME280, 0x58=BMP280
                    raise OSError(f"CHIP_IDが想定外(0x{chip_id:02X})")

                # --- キャリブレーションデータ読み出し ---
                calib1 = bus.read_i2c_block_data(addr, 0x88, 24)
                calib2 = bus.read_i2c_block_data(addr, 0xA1, 1)
                calib3 = bus.read_i2c_block_data(addr, 0xE1, 7)

                dig_T1, dig_T2, dig_T3, dig_P1, dig_P2, dig_P3, dig_P4, dig_P5, \
                    dig_P6, dig_P7, dig_P8, dig_P9 = struct.unpack("<Hhhhhhhhhhhh", bytes(calib1))
                dig_H1 = calib2[0]
                dig_H2, dig_H3 = struct.unpack("<hB", bytes(calib3[0:3]))
                e4, e5, e6 = calib3[3], calib3[4], calib3[5]
                dig_H4 = (e4 << 4) | (e5 & 0x0F)
                dig_H4 = dig_H4 - 4096 if dig_H4 > 2047 else dig_H4
                dig_H5 = (e6 << 4) | (e5 >> 4)
                dig_H5 = dig_H5 - 4096 if dig_H5 > 2047 else dig_H5
                dig_H6 = struct.unpack("<b", bytes([calib3[6]]))[0]

                # --- forcedモードで1回計測 (温湿度気圧すべてoversampling x1) ---
                bus.write_byte_data(addr, 0xF2, 0x01)          # ctrl_hum: osrs_h=1
                bus.write_byte_data(addr, 0xF4, 0x25)          # ctrl_meas: osrs_t=1,osrs_p=1,mode=forced
                time.sleep(0.1)

                data = bus.read_i2c_block_data(addr, 0xF7, 8)
                adc_p = (data[0] << 12) | (data[1] << 4) | (data[2] >> 4)
                adc_t = (data[3] << 12) | (data[4] << 4) | (data[5] >> 4)
                adc_h = (data[6] << 8) | data[7]

                # --- 温度 ---
                var1 = (adc_t / 16384.0 - dig_T1 / 1024.0) * dig_T2
                var2 = ((adc_t / 131072.0 - dig_T1 / 8192.0) ** 2) * dig_T3
                t_fine = var1 + var2
                temperature = t_fine / 5120.0

                # --- 気圧 ---
                var1 = t_fine / 2.0 - 64000.0
                var2 = var1 * var1 * dig_P6 / 32768.0
                var2 = var2 + var1 * dig_P5 * 2.0
                var2 = var2 / 4.0 + dig_P4 * 65536.0
                var1 = (dig_P3 * var1 * var1 / 524288.0 + dig_P2 * var1) / 524288.0
                var1 = (1.0 + var1 / 32768.0) * dig_P1
                if var1 == 0:
                    pressure = 0.0
                else:
                    p = 1048576.0 - adc_p
                    p = (p - var2 / 4096.0) * 6250.0 / var1
                    var1 = dig_P9 * p * p / 2147483648.0
                    var2 = p * dig_P8 / 32768.0
                    p = p + (var1 + var2 + dig_P7) / 16.0
                    pressure = p / 100.0  # hPa

                # --- 湿度 ---
                var_h = t_fine - 76800.0
                var_h = (adc_h - (dig_H4 * 64.0 + dig_H5 / 16384.0 * var_h)) * (
                    dig_H2 / 65536.0 * (1.0 + dig_H6 / 67108864.0 * var_h *
                                         (1.0 + dig_H3 / 67108864.0 * var_h)))
                var_h = var_h * (1.0 - dig_H1 * var_h / 524288.0)
                humidity = max(0.0, min(var_h, 100.0))

                return {"addr": addr, "temperature_C": round(temperature, 2),
                        "pressure_hPa": round(pressure, 2), "humidity_%": round(humidity, 2)}
        except Exception as e:
            last_err = e
            continue
    raise RuntimeError(f"BME280読み取り失敗 (0x76/0x77とも不可): {last_err}")


# ============================================================
# BNO055 (IMU) — 生レジスタ直読み、追加ライブラリ不要
# ============================================================
def read_bno055(bus_num=I2C_BUS, addr=BNO055_ADDR):
    from smbus2 import SMBus

    with SMBus(bus_num) as bus:
        chip_id = bus.read_byte_data(addr, 0x00)
        if chip_id != 0xA0:
            raise RuntimeError(f"CHIP_IDが想定外(0x{chip_id:02X}, 期待値0xA0)")

        # NDOFモード(0x0C)に設定してセンサフュージョンを有効化
        bus.write_byte_data(addr, 0x3D, 0x00)  # 一旦CONFIGモード
        time.sleep(0.03)
        bus.write_byte_data(addr, 0x3D, 0x0C)  # NDOFモード
        time.sleep(0.6)  # モード切替後の安定待ち

        # キャリブレーション状態 (0x35): 各2bit, 3で満点
        calib = bus.read_byte_data(addr, 0x35)
        sys_c = (calib >> 6) & 0x03
        gyro_c = (calib >> 4) & 0x03
        accel_c = (calib >> 2) & 0x03
        mag_c = calib & 0x03

        # オイラー角 (Heading/Roll/Pitch), 1/16 deg 単位, リトルエンディアン16bit signed
        raw = bus.read_i2c_block_data(addr, 0x1A, 6)
        heading, roll, pitch = struct.unpack("<hhh", bytes(raw))

        return {
            "heading_deg": round(heading / 16.0, 2),
            "roll_deg": round(roll / 16.0, 2),
            "pitch_deg": round(pitch / 16.0, 2),
            "calib_sys_gyro_accel_mag": f"{sys_c}/{gyro_c}/{accel_c}/{mag_c}",
        }


# ============================================================
# VL53L1X (ToF距離) — adafruit-circuitpython-vl53l1x を使用
# ============================================================
def read_vl53l1x():
    import board
    import busio
    import adafruit_vl53l1x

    i2c = busio.I2C(board.SCL, board.SDA)
    vl = adafruit_vl53l1x.VL53L1X(i2c, address=VL53L1X_ADDR)
    vl.start_ranging()
    try:
        for _ in range(20):
            if vl.data_ready:
                dist_mm = vl.distance
                vl.clear_interrupt()
                return {"distance_cm": dist_mm}
            time.sleep(0.05)
        raise RuntimeError("計測データが準備できませんでした(タイムアウト)")
    finally:
        vl.stop_ranging()


# ============================================================
# SAM-M10Q (GPS) — I2C(DDC)経由でNMEA生データをストリーム読み
# ============================================================
def read_gps_nmea(bus_num=I2C_BUS, addr=GPS_ADDR, duration=2.0):
    from smbus2 import SMBus

    lines = []
    buf = b""
    t_end = time.time() + duration
    with SMBus(bus_num) as bus:
        while time.time() < t_end:
            avail_h = bus.read_byte_data(addr, 0xFD)
            avail_l = bus.read_byte_data(addr, 0xFE)
            n = (avail_h << 8) | avail_l
            if n > 0:
                chunk_len = min(n, 32)
                data = bus.read_i2c_block_data(addr, 0xFF, chunk_len)
                buf += bytes(b for b in data if b != 0xFF)  # 0xFF=データなし埋め
            else:
                time.sleep(0.05)

    text = buf.decode("ascii", errors="ignore")
    for line in text.splitlines():
        line = line.strip()
        if line.startswith("$"):
            lines.append(line)
    return lines


# ============================================================
# IM920 (無線) — 実データ送信 / 受信リッスン
# ============================================================
def im920_send(ser, payload: bytes):
    hex_payload = payload.hex().upper()
    cmd = f"TXDA{hex_payload}\r\n".encode("ascii")
    ser.reset_input_buffer()
    ser.write(cmd)
    ser.flush()
    time.sleep(0.3)
    resp = ser.read(256)
    return cmd, resp


def im920_listen(ser, duration: float):
    ser.reset_input_buffer()
    t_end = time.time() + duration
    received = []
    buf = b""
    while time.time() < t_end:
        chunk = ser.read(256)
        if chunk:
            buf += chunk
    for line in buf.decode("ascii", errors="replace").splitlines():
        line = line.strip()
        if line:
            received.append(line)
    return received


# ============================================================
# 実行部
# ============================================================
def run_test(name, func, *args, **kwargs):
    print("-" * 60)
    print(f"[{name}]")
    try:
        result = func(*args, **kwargs)
        print(f"  結果: {result}")
        print(f"  → OK (値を読み取れました)")
        return True
    except ImportError as e:
        print(f"  [スキップ] 必要なライブラリがありません: {e}")
        return None
    except Exception as e:
        print(f"  [エラー] {e}")
        print(f"  → NG (値を読み取れませんでした)")
        return False


def main():
    parser = argparse.ArgumentParser(description="CanSat センサ・無線 実値読み取りテスト")
    parser.add_argument("--bus", type=int, default=I2C_BUS, help="I2Cバス番号")
    parser.add_argument("--port", type=str, default="/dev/serial0", help="IM920のシリアルポート")
    parser.add_argument("--baud", type=int, default=19200, help="IM920のボーレート")
    parser.add_argument("--gps-duration", type=float, default=2.0, help="GPS NMEA読み取りの観測秒数")
    parser.add_argument("--im920-send", type=str, default="HELLO", help="IM920で送信するテスト文字列")
    parser.add_argument("--im920-listen", type=float, default=3.0, help="IM920受信リッスン秒数(0で無効)")
    parser.add_argument("--only", type=str, default=None,
                         help="カンマ区切りで実行対象を限定 (bme280,bno055,vl53l1x,gps,im920)")
    args = parser.parse_args()

    targets = None
    if args.only:
        targets = {t.strip().lower() for t in args.only.split(",")}

    def enabled(key):
        return targets is None or key in targets

    print("CanSat センサ・無線 実値読み取りテスト")
    print("(接続確認ではなく、実際に値を取得できるかを確認します)\n")

    results = {}

    if enabled("bme280"):
        results["bme280"] = run_test("BME280 気圧/気温/湿度", read_bme280, args.bus)

    if enabled("bno055"):
        results["bno055"] = run_test("BNO055 IMU (オイラー角)", read_bno055, args.bus, BNO055_ADDR)

    if enabled("vl53l1x"):
        results["vl53l1x"] = run_test("VL53L1X ToF距離", read_vl53l1x)

    if enabled("gps"):
        def gps_wrapper():
            lines = read_gps_nmea(args.bus, GPS_ADDR, args.gps_duration)
            if not lines:
                raise RuntimeError("NMEAセンテンスを受信できませんでした(アンテナ未接続/屋内でも$GxTXT等は出るはず)")
            return f"{len(lines)}行受信, 例: {lines[0]}"
        results["gps"] = run_test(f"SAM-M10Q GPS (I2C NMEA, {args.gps_duration}s観測)", gps_wrapper)

    if enabled("im920") and (args.im920_send or args.im920_listen > 0):
        print("-" * 60)
        print("[IM920 無線 実データ送受信]")
        try:
            import serial
            ser = serial.Serial(args.port, baudrate=args.baud, timeout=0.3)
            with ser:
                if args.im920_send:
                    cmd, resp = im920_send(ser, args.im920_send.encode("ascii"))
                    print(f"  送信コマンド: {cmd!r}")
                    print(f"  送信直後の応答: {resp!r}")
                if args.im920_listen > 0:
                    print(f"  受信リッスン中... ({args.im920_listen}秒, 対向局からの送信を待ちます)")
                    lines = im920_listen(ser, args.im920_listen)
                    if lines:
                        print(f"  受信データ({len(lines)}件):")
                        for l in lines:
                            print(f"    {l}")
                        results["im920"] = True
                    else:
                        print("  受信データ: なし")
                        print("  ※対向局(地上局IM920など)からの送信がない場合は正常にNGとなります")
                        results["im920"] = None
        except Exception as e:
            print(f"  [エラー] {e}")
            results["im920"] = False

    print("=" * 60)
    print("結果サマリ")
    print("=" * 60)
    for name, ok in results.items():
        mark = "OK" if ok else ("SKIP" if ok is None else "NG")
        print(f"  {name:<10s}: {mark}")


if __name__ == "__main__":
    main()
