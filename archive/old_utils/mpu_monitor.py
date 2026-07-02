#!/usr/bin/env python3
# mpu_monitor.py
# Reads MPU6050 on I2C bus 13, shows realtime plots, orientation (complementary filter),
# system stats, and optional GPS NMEA reading from /dev/ttyUSB0.

import sys, time, math, threading, serial
from collections import deque
from time import monotonic

# Hardware libs
from smbus2 import SMBus

# GUI libs
from PyQt5 import QtWidgets, QtCore
import pyqtgraph as pg

# System stats
import psutil

# Optional NMEA parser
import pynmea2

I2C_BUS = 13
MPU_ADDR = 0x68  # change to 0x69 if AD0 high

# MPU6050 registers
PWR_MGMT_1 = 0x6B
SMPLRT_DIV = 0x19
CONFIG = 0x1A
GYRO_CONFIG = 0x1B
ACCEL_CONFIG = 0x1C
ACCEL_XOUT_H = 0x3B
GYRO_XOUT_H = 0x43

# scale factors
ACCEL_SCALE = 16384.0    # for +/-2g
GYRO_SCALE = 131.0       # for +/-250deg/s

# complementary filter constant
ALPHA = 0.98

def twos_complement(val, bits=16):
    if val & (1 << (bits - 1)):
        val = val - (1 << bits)
    return val

class MPU6050:
    def __init__(self, bus_no=I2C_BUS, addr=MPU_ADDR):
        self.bus_no = bus_no
        self.addr = addr
        self.bus = SMBus(bus_no)
        # wake up
        self.bus.write_byte_data(addr, PWR_MGMT_1, 0x00)
        time.sleep(0.1)

    def read_all(self):
        # read 14 bytes accel(6), temp(2), gyro(6)
        data = self.bus.read_i2c_block_data(self.addr, ACCEL_XOUT_H, 14)
        ax = twos_complement((data[0] << 8) | data[1]) / ACCEL_SCALE
        ay = twos_complement((data[2] << 8) | data[3]) / ACCEL_SCALE
        az = twos_complement((data[4] << 8) | data[5]) / ACCEL_SCALE
        # temp raw = data[6]<<8 | data[7]  (not used here)
        gx = twos_complement((data[8] << 8) | data[9]) / GYRO_SCALE
        gy = twos_complement((data[10] << 8) | data[11]) / GYRO_SCALE
        gz = twos_complement((data[12] << 8) | data[13]) / GYRO_SCALE
        return (ax, ay, az, gx, gy, gz)

class GPSReader(threading.Thread):
    def __init__(self, device='/dev/ttyUSB0', baud=9600):
        super().__init__(daemon=True)
        self.device = device
        self.baud = baud
        self.lock = threading.Lock()
        self.lat = None
        self.lon = None
        self.fix = False
        try:
            self.ser = serial.Serial(device, baud, timeout=1)
            self.running = True
            self.start()
        except Exception:
            self.running = False
            self.ser = None

    def run(self):
        while self.running:
            try:
                line = self.ser.readline().decode('ascii', errors='ignore').strip()
                if line.startswith('$'):
                    msg = pynmea2.parse(line)
                    if hasattr(msg, 'latitude') and msg.latitude != 0.0:
                        with self.lock:
                            self.lat = msg.latitude
                            self.lon = msg.longitude
                            self.fix = True
            except Exception:
                time.sleep(0.2)

class MainWindow(QtWidgets.QMainWindow):
    def __init__(self, mpu, gps=None):
        super().__init__()
        self.setWindowTitle("IMU Monitor - MPU6050 (bus {})".format(I2C_BUS))
        self.resize(1000, 600)

        # central widget + layout
        w = QtWidgets.QWidget()
        v = QtWidgets.QVBoxLayout()
        w.setLayout(v)
        self.setCentralWidget(w)

        # top info labels
        self.info_label = QtWidgets.QLabel("No GPS")
        v.addWidget(self.info_label)

        # plot widget for accel (3 traces)
        self.plot_widget = pg.PlotWidget(title="Accelerometer (g)")
        self.plot_widget.addLegend()
        self.plot_widget.showGrid(x=True, y=True)
        self.ax_plot = self.plot_widget.plot(pen='r', name='ax')
        self.ay_plot = self.plot_widget.plot(pen='g', name='ay')
        self.az_plot = self.plot_widget.plot(pen='b', name='az')
        v.addWidget(self.plot_widget)

        # small status: gyro + orientation + cpu/mem
        h = QtWidgets.QHBoxLayout()
        self.orient_label = QtWidgets.QLabel("Pitch: 0.0  Roll: 0.0  Yaw: 0.0")
        h.addWidget(self.orient_label)
        self.sys_label = QtWidgets.QLabel("CPU: 0%  MEM: 0%")
        h.addWidget(self.sys_label)
        v.addLayout(h)

        # data buffers
        self.bufsize = 500
        self.ax_buf = deque([0]*self.bufsize, maxlen=self.bufsize)
        self.ay_buf = deque([0]*self.bufsize, maxlen=self.bufsize)
        self.az_buf = deque([0]*self.bufsize, maxlen=self.bufsize)

        # orientation state
        self.pitch = 0.0
        self.roll = 0.0
        self.yaw = 0.0
        self.gps = gps
        self.mpu = mpu

        # timer update
        self.last_t = monotonic()
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update_loop)
        self.timer.start(40)  # ~25Hz

    def update_loop(self):
        try:
            ax, ay, az, gx, gy, gz = self.mpu.read_all()
        except Exception as e:
            self.info_label.setText("MPU read error: {}".format(e))
            return

        # push to buffers
        self.ax_buf.append(ax)
        self.ay_buf.append(ay)
        self.az_buf.append(az)

        # integrate gyro for orientation (deg/s)
        t = monotonic()
        dt = t - self.last_t if self.last_t else 0.04
        self.last_t = t

        # accelerometer angles
        pitch_acc = math.degrees(math.atan2(ay, math.sqrt(ax*ax + az*az)))
        roll_acc  = math.degrees(math.atan2(-ax, az))

        # complementary filter
        self.pitch = ALPHA * (self.pitch + gx * dt) + (1 - ALPHA) * pitch_acc
        self.roll  = ALPHA * (self.roll  + gy * dt) + (1 - ALPHA) * roll_acc
        self.yaw   = self.yaw + gz * dt  # yaw needs magnetometer to be stable

        # update labels
        self.orient_label.setText(f"Pitch: {self.pitch:.2f}°  Roll: {self.roll:.2f}°  Yaw: {self.yaw:.2f}°")
        cpu = psutil.cpu_percent(interval=None)
        mem = psutil.virtual_memory().percent
        self.sys_label.setText(f"CPU: {cpu:.0f}%  MEM: {mem:.0f}%")

        # GPS if available
        gps_text = "No GPS"
        if self.gps and self.gps.running:
            with self.gps.lock:
                if self.gps.fix:
                    gps_text = f"GPS: {self.gps.lat:.6f}, {self.gps.lon:.6f}"
                else:
                    gps_text = "GPS: no fix yet"
        self.info_label.setText(gps_text)

        # update plots
        self.ax_plot.setData(list(self.ax_buf))
        self.ay_plot.setData(list(self.ay_buf))
        self.az_plot.setData(list(self.az_buf))

def main():
    # init MPU
    try:
        mpu = MPU6050(bus_no=I2C_BUS, addr=MPU_ADDR)
    except Exception as e:
        print("Failed to initialize MPU6050:", e)
        sys.exit(1)

    # start GPS reader (optional)
    gps = GPSReader('/dev/ttyUSB0', 9600)  # will disable itself if device missing

    app = QtWidgets.QApplication(sys.argv)
    mw = MainWindow(mpu, gps=gps)
    mw.show()
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()
