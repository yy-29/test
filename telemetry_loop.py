#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
CanSat センサ・無線 連続テレメトリ取得 (Raspberry Pi 4 用)

sensor_value_read_test.py は1回だけ値を読むテストだったが、
これは Ctrl+C で止めるまで一定間隔で値を取得し続ける常駐スクリプト。

対象 (CanSat_回路図.svg より):
  - BME280   : 気温・気圧・湿度
  - BNO055   : オイラー角 (Heading/Roll/Pitch) + キャリブレーション状態
  - VL53L1X  : 距離[mm] (adafruit-circuitpython-vl53l1x 使用)
  - SAM-M10Q : GPS I2C(DDC) 生NMEAストリーム
  - IM920    : 一定間隔でテレメトリを送信 + 常時受信リッスン

各センサはNGでもループ全体は止めず、そのサイクルだけ値なし(--)として継続する。

事前準備:
  sudo apt update
  sudo apt install -y python3-smbus2 python3-serial
  pip3 install adafruit-circuitpython-vl53l1x adafruit-blinka   # VL53L1Xのみ必要

使い方:
  python3 telemetry_loop.py                          # 1秒間隔で無限ループ
  python3 telemetry_loop.py --interval 0.5            # 0.5秒間隔
  python3 telemetry_loop.py --log telemetry.csv       # CSVにも保存
  python3 telemetry_loop.py --im920-tx-interval 5     # 5秒毎にIM920でテレメトリ送信
  python3 telemetry_loop.py --only bme280,bno055      # 一部センサだけ
  Ctrl+C で終了
"""

import argparse
import csv
import struct
import sys
import time
from datetime import datetime

I2C_BUS = 1
BME280_ADDR_CANDIDATES = [0x76, 0x77]
BNO055_ADDR = 0x28
VL53L1X_ADDR = 0x29
GPS_ADDR = 0x42


# ============================================================
# BME280
# ============================================================
class Bme280:
    def __init__(self, bus):
        self.bus = bus
        self.addr = None
        for addr in BME280_ADDR_CANDIDATES:
            try:
                chip_id = bus.read_byte_data(addr, 0xD0)
                if chip_id in (0x60, 0x58):
                    self.addr = addr
                    break
            except Exception:
                continue
        if self.addr is None:
            raise RuntimeError("BME280が見つかりません(0x76/0x77とも不可)")

        calib1 = bus.read_i2c_block_data(self.addr, 0x88, 24)
        calib2 = bus.read_i2c_block_data(self.addr, 0xA1, 1)
        calib3 = bus.read_i2c_block_data(self.addr, 0xE1, 7)
        (self.dig_T1, self.dig_T2, self.dig_T3, self.dig_P1, self.dig_P2, self.dig_P3,
         self.dig_P4, self.dig_P5, self.dig_P6, self.dig_P7, self.dig_P8, self.dig_P9) = \
            struct.unpack("<Hhhhhhhhhhhh", bytes(calib1))
        self.dig_H1 = calib2[0]
        self.dig_H2, self.dig_H3 = struct.unpack("<hB", bytes(calib3[0:3]))
        e4, e5, e6 = calib3[3], calib3[4], calib3[5]
        h4 = (e4 << 4) | (e5 & 0x0F)
        self.dig_H4 = h4 - 4096 if h4 > 2047 else h4
        h5 = (e6 << 4) | (e5 >> 4)
        self.dig_H5 = h5 - 4096 if h5 > 2047 else h5
        self.dig_H6 = struct.unpack("<b", bytes([calib3[6]]))[0]

    def read(self):
        bus, addr = self.bus, self.addr
        bus.write_byte_data(addr, 0xF2, 0x01)
        bus.write_byte_data(addr, 0xF4, 0x25)
        time.sleep(0.05)
        data = bus.read_i2c_block_data(addr, 0xF7, 8)
        adc_p = (data[0] << 12) | (data[1] << 4) | (data[2] >> 4)
        adc_t = (data[3] << 12) | (data[4] << 4) | (data[5] >> 4)
        adc_h = (data[6] << 8) | data[7]

        var1 = (adc_t / 16384.0 - self.dig_T1 / 1024.0) * self.dig_T2
        var2 = ((adc_t / 131072.0 - self.dig_T1 / 8192.0) ** 2) * self.dig_T3
        t_fine = var1 + var2
        temperature = t_fine / 5120.0

        var1 = t_fine / 2.0 - 64000.0
        var2 = var1 * var1 * self.dig_P6 / 32768.0
        var2 = var2 + var1 * self.dig_P5 * 2.0
        var2 = var2 / 4.0 + self.dig_P4 * 65536.0
        var1 = (self.dig_P3 * var1 * var1 / 524288.0 + self.dig_P2 * var1) / 524288.0
        var1 = (1.0 + var1 / 32768.0) * self.dig_P1
        if var1 == 0:
            pressure = 0.0
        else:
            p = 1048576.0 - adc_p
            p = (p - var2 / 4096.0) * 6250.0 / var1
            var1 = self.dig_P9 * p * p / 2147483648.0
            var2 = p * self.dig_P8 / 32768.0
            p = p + (var1 + var2 + self.dig_P7) / 16.0
            pressure = p / 100.0

        var_h = t_fine - 76800.0
        var_h = (adc_h - (self.dig_H4 * 64.0 + self.dig_H5 / 16384.0 * var_h)) * (
            self.dig_H2 / 65536.0 * (1.0 + self.dig_H6 / 67108864.0 * var_h *
                                       (1.0 + self.dig_H3 / 67108864.0 * var_h)))
        var_h = var_h * (1.0 - self.dig_H1 * var_h / 524288.0)
        humidity = max(0.0, min(var_h, 100.0))

        return round(temperature, 2), round(pressure, 2), round(humidity, 2)


# ============================================================
# BNO055
# ============================================================
class Bno055:
    def __init__(self, bus, addr=BNO055_ADDR):
        self.bus = bus
        self.addr = addr
        chip_id = bus.read_byte_data(addr, 0x00)
        if chip_id != 0xA0:
            raise RuntimeError(f"CHIP_IDが想定外(0x{chip_id:02X})")
        bus.write_byte_data(addr, 0x3D, 0x00)
        time.sleep(0.03)
        bus.write_byte_data(addr, 0x3D, 0x0C)  # NDOFモード
        time.sleep(0.6)

    def read(self):
        bus, addr = self.bus, self.addr
        calib = bus.read_byte_data(addr, 0x35)
        sys_c, gyro_c, accel_c, mag_c = (calib >> 6) & 3, (calib >> 4) & 3, (calib >> 2) & 3, calib & 3
        raw = bus.read_i2c_block_data(addr, 0x1A, 6)
        heading, roll, pitch = struct.unpack("<hhh", bytes(raw))
        return (round(heading / 16.0, 1), round(roll / 16.0, 1), round(pitch / 16.0, 1),
                f"{sys_c}{gyro_c}{accel_c}{mag_c}")


# ============================================================
# VL53L1X
# ============================================================
class Vl53l1x:
    def __init__(self):
        import board
        import busio
        import adafruit_vl53l1x
        i2c = busio.I2C(board.SCL, board.SDA)
        self.vl = adafruit_vl53l1x.VL53L1X(i2c, address=VL53L1X_ADDR)
        self.vl.start_ranging()

    def read(self):
        if self.vl.data_ready:
            d = self.vl.distance
            self.vl.clear_interrupt()
            return d
        return None

    def close(self):
        try:
            self.vl.stop_ranging()
        except Exception:
            pass


# ============================================================
# GPS (SAM-M10Q, I2C DDC) — 非ブロッキングでポーリングし生NMEAを蓄積
# ============================================================
class GpsI2c:
    def __init__(self, bus, addr=GPS_ADDR):
        self.bus = bus
        self.addr = addr
        self.linebuf = ""
        self.last_line = None

    def poll(self):
        bus, addr = self.bus, self.addr
        avail_h = bus.read_byte_data(addr, 0xFD)
        avail_l = bus.read_byte_data(addr, 0xFE)
        n = (avail_h << 8) | avail_l
        if n <= 0:
            return
        data = bus.read_i2c_block_data(addr, 0xFF, min(n, 32))
        text = bytes(b for b in data if b != 0xFF).decode("ascii", errors="ignore")
        self.linebuf += text
        while "\n" in self.linebuf:
            line, self.linebuf = self.linebuf.split("\n", 1)
            line = line.strip()
            if line.startswith("$"):
                self.last_line = line

    def latest(self):
        return self.last_line


# ============================================================
# IM920 (UART) — 常時受信 + 一定間隔で送信
# ============================================================
class Im920:
    def __init__(self, port, baud):
        import serial
        self.ser = serial.Serial(port, baudrate=baud, timeout=0)  # ノンブロッキング
        self.linebuf = ""
        self.last_rx = None

    def poll_rx(self):
        n = self.ser.in_waiting
        if n:
            chunk = self.ser.read(n).decode("ascii", errors="replace")
            self.linebuf += chunk
            while "\n" in self.linebuf:
                line, self.linebuf = self.linebuf.split("\n", 1)
                line = line.strip()
                if line:
                    self.last_rx = line

    def send(self, payload: bytes):
        hex_payload = payload.hex().upper()
        cmd = f"TXDA{hex_payload}\r\n".encode("ascii")
        self.ser.write(cmd)
        self.ser.flush()

    def close(self):
        try:
            self.ser.close()
        except Exception:
            pass


# ============================================================
# メインループ
# ============================================================
def main():
    parser = argparse.ArgumentParser(description="CanSat センサ・無線 連続テレメトリ取得")
    parser.add_argument("--bus", type=int, default=I2C_BUS)
    parser.add_argument("--interval", type=float, default=1.0, help="取得間隔(秒)")
    parser.add_argument("--duration", type=float, default=0.0, help="実行時間(秒), 0=無制限")
    parser.add_argument("--log", type=str, default=None, help="CSV保存先パス")
    parser.add_argument("--port", type=str, default="/dev/serial0")
    parser.add_argument("--baud", type=int, default=19200)
    parser.add_argument("--im920-tx-interval", type=float, default=0.0,
                         help="IM920でテレメトリを送信する間隔(秒), 0=送信しない(受信のみ)")
    parser.add_argument("--only", type=str, default=None,
                         help="カンマ区切りで対象を限定 (bme280,bno055,vl53l1x,gps,im920)")
    args = parser.parse_args()

    targets = None
    if args.only:
        targets = {t.strip().lower() for t in args.only.split(",")}

    def enabled(key):
        return targets is None or key in targets

    from smbus2 import SMBus
    bus = SMBus(args.bus)

    modules = {}
    print("初期化中...")

    if enabled("bme280"):
        try:
            modules["bme280"] = Bme280(bus)
            print("  BME280   : OK")
        except Exception as e:
            print(f"  BME280   : 無効化 ({e})")

    if enabled("bno055"):
        try:
            modules["bno055"] = Bno055(bus)
            print("  BNO055   : OK")
        except Exception as e:
            print(f"  BNO055   : 無効化 ({e})")

    if enabled("vl53l1x"):
        try:
            modules["vl53l1x"] = Vl53l1x()
            print("  VL53L1X  : OK")
        except Exception as e:
            print(f"  VL53L1X  : 無効化 ({e})")

    if enabled("gps"):
        try:
            modules["gps"] = GpsI2c(bus)
            print("  GPS      : OK (I2Cポーリング開始)")
        except Exception as e:
            print(f"  GPS      : 無効化 ({e})")

    if enabled("im920"):
        try:
            modules["im920"] = Im920(args.port, args.baud)
            print("  IM920    : OK (受信リッスン開始)")
        except Exception as e:
            print(f"  IM920    : 無効化 ({e})")

    print(f"\n計測開始 (interval={args.interval}s, Ctrl+Cで終了)\n")

    header = ["time"]
    if "bme280" in modules:
        header += ["temp_C", "pressure_hPa", "humidity_%"]
    if "bno055" in modules:
        header += ["heading_deg", "roll_deg", "pitch_deg", "calib"]
    if "vl53l1x" in modules:
        header += ["distance_mm"]
    if "gps" in modules:
        header += ["gps_nmea"]
    if "im920" in modules:
        header += ["im920_rx"]

    logfile = None
    writer = None
    if args.log:
        logfile = open(args.log, "w", newline="", encoding="utf-8")
        writer = csv.writer(logfile)
        writer.writerow(header)

    print(" | ".join(f"{h:<14s}" for h in header))

    seq = 0
    t_start = time.time()
    last_tx = 0.0
    try:
        while True:
            now = time.time()
            if args.duration > 0 and (now - t_start) > args.duration:
                break

            row = [datetime.now().strftime("%H:%M:%S.%f")[:-3]]

            if "bme280" in modules:
                try:
                    t, p, h = modules["bme280"].read()
                    row += [t, p, h]
                except Exception:
                    row += ["--", "--", "--"]

            if "bno055" in modules:
                try:
                    heading, roll, pitch, calib = modules["bno055"].read()
                    row += [heading, roll, pitch, calib]
                except Exception:
                    row += ["--", "--", "--", "--"]

            if "vl53l1x" in modules:
                try:
                    d = modules["vl53l1x"].read()
                    row += [d if d is not None else "--"]
                except Exception:
                    row += ["--"]

            if "gps" in modules:
                try:
                    modules["gps"].poll()
                    row += [modules["gps"].latest() or "--"]
                except Exception:
                    row += ["--"]

            if "im920" in modules:
                im = modules["im920"]
                try:
                    im.poll_rx()
                    if args.im920_tx_interval > 0 and (now - last_tx) >= args.im920_tx_interval:
                        im.send(f"SEQ{seq}".encode("ascii"))
                        last_tx = now
                    row += [im.last_rx or "--"]
                except Exception:
                    row += ["--"]

            print(" | ".join(f"{str(v):<14s}" for v in row))
            if writer:
                writer.writerow(row)
                logfile.flush()

            seq += 1
            time.sleep(max(0.0, args.interval - (time.time() - now)))

    except KeyboardInterrupt:
        print("\n中断されました。終了処理中...")
    finally:
        if "vl53l1x" in modules:
            modules["vl53l1x"].close()
        if "im920" in modules:
            modules["im920"].close()
        bus.close()
        if logfile:
            logfile.close()
            print(f"ログを保存しました: {args.log}")


if __name__ == "__main__":
    main()
