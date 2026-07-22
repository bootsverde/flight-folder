"""I2C sensor interface for the GY-91 (MPU9250 + AK8963 + BMP280) and the
Adafruit Mini GPS (PA1010D).

Both devices sit on a dedicated software I2C bus (bus 4, bit-banged on
GPIO23/GPIO24 via the i2c-gpio overlay) instead of the header's hardware
I2C1 (GPIO2/GPIO3), which is already in use by the touchscreen's touch
controller.

Register-level IMU/baro logic mirrors src/main.cpp so the Pi and ESP32
firmware behave identically.
"""

import math
import time

import pynmea2
from smbus2 import SMBus

IMU_ADDR = 0x68
MAG_ADDR = 0x0C
BMP_ADDR_CANDIDATES = (0x76, 0x77)
GPS_ADDR = 0x10


def _s16(hi, lo):
    v = (hi << 8) | lo
    return v - 65536 if v > 32767 else v


class _GPSReader:
    def __init__(self):
        self._buf = ""
        self.has_fix = False
        self.sats = 0
        self.lat = 0.0
        self.lon = 0.0
        self.speed_mph = 0.0
        self.altitude_m = 0.0

    def feed_byte(self, ch):
        if ch in ("\r", "\n"):
            line = self._buf.strip()
            self._buf = ""
            if line.startswith("$"):
                self._parse(line)
        else:
            self._buf += ch
            if len(self._buf) > 120:
                self._buf = ""

    def _parse(self, line):
        try:
            msg = pynmea2.parse(line)
        except pynmea2.ParseError:
            return

        if isinstance(msg, pynmea2.types.talker.GGA):
            self.has_fix = int(msg.gps_qual or 0) > 0
            if self.has_fix:
                self.sats = int(msg.num_sats or 0)
                self.lat = msg.latitude
                self.lon = msg.longitude
                if msg.altitude is not None:
                    self.altitude_m = float(msg.altitude)
        elif isinstance(msg, pynmea2.types.talker.RMC):
            self.has_fix = msg.status == "A"
            if msg.spd_over_grnd is not None:
                self.speed_mph = float(msg.spd_over_grnd) * 1.15078


class Sensors:
    def __init__(self, bus_num=4):
        self.bus = SMBus(bus_num)

        self.imu_ok = False
        self.mag_ok = False
        self.bmp_ok = False
        self.gps_ok = False
        self._bmp_addr = None
        self._bmp_calib = None

        self.pitch = 0.0
        self.roll = 0.0
        self.heading = 0.0
        self.altitude_m = 0.0
        self.speed_mph = 0.0
        self.gps_fix = False
        self.sats = 0
        self.lat = 0.0
        self.lon = 0.0
        self.sea_level_hpa = 1013.25

        self._gx_bias = self._gy_bias = self._gz_bias = 0.0
        self._pitch_offset = 0.0
        self._roll_offset = 0.0
        self._mag_x_off = self._mag_y_off = self._mag_z_off = 0.0

        self._prev_time = time.monotonic()
        self._gps = _GPSReader()

        self._init_imu()
        self._init_bmp()
        self._probe_gps()

    # ---------------- IMU ----------------
    def _imu_write(self, reg, val):
        self.bus.write_byte_data(IMU_ADDR, reg, val)

    def _init_imu(self):
        try:
            self._imu_write(0x6B, 0x80)  # reset
            time.sleep(0.1)
            self._imu_write(0x6B, 0x01)  # wake, clock from gyro X
            time.sleep(0.01)
            self._imu_write(0x1A, 0x03)  # DLPF
            self._imu_write(0x1B, 0x00)  # gyro +-250 dps
            self._imu_write(0x1C, 0x00)  # accel +-2g
            self._imu_write(0x1D, 0x03)

            who = self.bus.read_byte_data(IMU_ADDR, 0x75)
            self.imu_ok = True

            if who in (0x71, 0x73):
                self._imu_write(0x37, 0x02)  # I2C bypass -> AK8963 visible
                time.sleep(0.01)
                try:
                    self.bus.read_byte(MAG_ADDR)
                    self.bus.write_byte_data(MAG_ADDR, 0x0B, 0x01)  # reset
                    time.sleep(0.01)
                    self.bus.write_byte_data(MAG_ADDR, 0x0A, 0x16)  # 16-bit, 100Hz cont.
                    self.mag_ok = True
                except OSError:
                    self.mag_ok = False
        except OSError:
            self.imu_ok = False

    def calibrate_gyro(self, samples=200):
        if not self.imu_ok:
            return
        sx = sy = sz = 0
        n = 0
        for _ in range(samples):
            try:
                b = self.bus.read_i2c_block_data(IMU_ADDR, 0x43, 6)
            except OSError:
                continue
            sx += _s16(b[0], b[1])
            sy += _s16(b[2], b[3])
            sz += _s16(b[4], b[5])
            n += 1
            time.sleep(0.005)
        if n:
            self._gx_bias = (sx / n) / 131.0
            self._gy_bias = (sy / n) / 131.0
            self._gz_bias = (sz / n) / 131.0

    def zero_horizon(self):
        self._pitch_offset = self.pitch
        self._roll_offset = self.roll

    def zero_heading(self):
        self.heading = 0.0

    def adjust_qnh(self, delta):
        self.sea_level_hpa += delta

    def _read_imu(self):
        if not self.imu_ok:
            return
        try:
            b = self.bus.read_i2c_block_data(IMU_ADDR, 0x3B, 14)
        except OSError:
            return

        ax, ay, az = _s16(b[0], b[1]), _s16(b[2], b[3]), _s16(b[4], b[5])
        gx, gy, gz = _s16(b[8], b[9]), _s16(b[10], b[11]), _s16(b[12], b[13])

        if ax == 0 and ay == 0 and az == 0:
            return

        now = time.monotonic()
        dt = now - self._prev_time
        if dt <= 0 or dt > 0.5:
            dt = 0.01
        self._prev_time = now

        fax, fay, faz = ax / 16384.0, ay / 16384.0, az / 16384.0
        accel_pitch = math.degrees(math.atan2(faz, fay))
        accel_roll = math.degrees(math.atan2(-fax, fay))

        gx_rate = gx / 131.0 - self._gx_bias
        gy_rate = gy / 131.0 - self._gy_bias
        gz_rate = gz / 131.0 - self._gz_bias

        # GY-91 mounted vertical, pins at top: gx=pitch, gz=roll(inverted), gy=yaw
        self.pitch = 0.96 * (self.pitch + gx_rate * dt) + 0.04 * accel_pitch
        self.roll = 0.96 * (self.roll - gz_rate * dt) + 0.04 * accel_roll

        if not self.mag_ok:
            self.heading = (self.heading + gy_rate * dt) % 360.0

    def _read_mag(self):
        if not self.mag_ok:
            return
        try:
            status = self.bus.read_byte_data(MAG_ADDR, 0x02)
            if not (status & 0x01):
                return
            m = self.bus.read_i2c_block_data(MAG_ADDR, 0x03, 7)
        except OSError:
            return
        if m[6] & 0x08:
            return

        mx = _s16(m[1], m[0]) - self._mag_x_off
        my = _s16(m[3], m[2]) - self._mag_y_off
        mz = _s16(m[5], m[4]) - self._mag_z_off

        pr = math.radians(self.pitch - self._pitch_offset)
        rr = math.radians(self.roll - self._roll_offset)
        mx2 = mx * math.cos(pr) + mz * math.sin(pr)
        my2 = (mx * math.sin(rr) * math.sin(pr) + my * math.cos(rr)
               - mz * math.sin(rr) * math.cos(pr))

        self.heading = math.degrees(math.atan2(-my2, mx2)) % 360.0

    # ---------------- Barometer ----------------
    def _init_bmp(self):
        for addr in BMP_ADDR_CANDIDATES:
            try:
                chip_id = self.bus.read_byte_data(addr, 0xD0)
            except OSError:
                continue
            if chip_id in (0x58, 0x60):  # BMP280 / BME280
                self._bmp_addr = addr
                self._bmp_calib = self._read_bmp_calib(addr)
                self.bus.write_byte_data(addr, 0xF4, 0x27)  # normal mode, x1 osrs
                self.bus.write_byte_data(addr, 0xF5, 0x00)
                self.bmp_ok = True
                return
        self.bmp_ok = False

    def _read_bmp_calib(self, addr):
        c = self.bus.read_i2c_block_data(addr, 0x88, 24)

        def u16(i):
            return c[i] | (c[i + 1] << 8)

        def s16(i):
            v = u16(i)
            return v - 65536 if v > 32767 else v

        return {
            "T1": u16(0), "T2": s16(2), "T3": s16(4),
            "P1": u16(6), "P2": s16(8), "P3": s16(10), "P4": s16(12),
            "P5": s16(14), "P6": s16(16), "P7": s16(18), "P8": s16(20), "P9": s16(22),
        }

    def _read_baro(self):
        if not self.bmp_ok:
            return
        try:
            d = self.bus.read_i2c_block_data(self._bmp_addr, 0xF7, 6)
        except OSError:
            return

        adc_p = (d[0] << 12) | (d[1] << 4) | (d[2] >> 4)
        adc_t = (d[3] << 12) | (d[4] << 4) | (d[5] >> 4)

        c = self._bmp_calib
        var1 = (adc_t / 16384.0 - c["T1"] / 1024.0) * c["T2"]
        var2 = ((adc_t / 131072.0 - c["T1"] / 8192.0) ** 2) * c["T3"]
        t_fine = var1 + var2

        var1 = t_fine / 2.0 - 64000.0
        var2 = var1 * var1 * c["P6"] / 32768.0
        var2 = var2 + var1 * c["P5"] * 2.0
        var2 = var2 / 4.0 + c["P4"] * 65536.0
        var1 = (c["P3"] * var1 * var1 / 524288.0 + c["P2"] * var1) / 524288.0
        var1 = (1.0 + var1 / 32768.0) * c["P1"]
        if var1 == 0:
            return
        p = 1048576.0 - adc_p
        p = (p - var2 / 4096.0) * 6250.0 / var1
        var1 = c["P9"] * p * p / 2147483648.0
        var2 = p * c["P8"] / 32768.0
        pressure_hpa = (p + (var1 + var2 + c["P7"]) / 16.0) / 100.0

        self.altitude_m = 44330.0 * (1.0 - (pressure_hpa / self.sea_level_hpa) ** (1 / 5.255))

    # ---------------- GPS ----------------
    def _probe_gps(self):
        try:
            self.bus.read_byte(GPS_ADDR)
            self.gps_ok = True
        except OSError:
            self.gps_ok = False

    def _read_gps(self):
        if not self.gps_ok:
            return
        try:
            data = self.bus.read_i2c_block_data(GPS_ADDR, 0, 32)
        except OSError:
            return

        for byte in data:
            if byte in (0x00, 0xFF):
                continue
            self._gps.feed_byte(chr(byte))

        self.gps_fix = self._gps.has_fix
        self.sats = self._gps.sats
        self.lat = self._gps.lat
        self.lon = self._gps.lon
        self.speed_mph = self._gps.speed_mph
        if not self.bmp_ok:
            self.altitude_m = self._gps.altitude_m

    def update(self):
        self._read_imu()
        self._read_mag()
        self._read_baro()
        self._read_gps()
