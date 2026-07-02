#!/usr/bin/env python3
"""
raspi_waterfall_monitor.py

A single-file PyQt5 + pyqtgraph application that shows:
 - an animated "waterfall" visualization driven by live system metrics
 - real-time plots for CPU, Memory, Network
 - a device panel listing USB and I2C devices (if available)
 - system info and temperature

Designed to run on Raspberry Pi (tested conceptually on Raspberry Pi OS).

Dependencies:
  sudo apt update && sudo apt install -y python3-pyqt5 python3-pip i2c-tools lm-sensors usbutils
  pip3 install --upgrade pip
  pip3 install psutil pyqtgraph numpy

Run:
  python3 raspi_waterfall_monitor.py

Notes:
 - If i2cdetect requires sudo, you will need to run the script with permissions or run i2cdetect separately and allow access.
 - lm-sensors may need to be configured (run `sudo sensors-detect`).

"""

import sys
import time
import subprocess
import shlex
import platform
from collections import deque

import numpy as np
import psutil

from PyQt5 import QtWidgets, QtCore
import pyqtgraph as pg

# ----------------------------- Config -----------------------------
REFRESH_MS = 500          # GUI refresh interval
WATERFALL_WIDTH = 200     # pixels (columns)
WATERFALL_HEIGHT = 160    # pixels (rows)
COLORMAP = pg.colormap.get('viridis')

# ----------------------------- Helpers -----------------------------

def run_cmd(cmd):
    try:
        out = subprocess.check_output(shlex.split(cmd), stderr=subprocess.DEVNULL, timeout=2).decode('utf-8')
        return out.strip()
    except Exception:
        return ''


def list_usb():
    out = run_cmd('lsusb')
    return out.splitlines() if out else []


def i2c_scan():
    # Requires i2cdetect from i2c-tools
    out = run_cmd('i2cdetect -y 1')
    return out if out else ''


def get_temperatures():
    temps = {}
    try:
        # psutil sensors_temperatures may be empty on some RPi installs
        st = psutil.sensors_temperatures()
        for k, vals in st.items():
            for v in vals:
                temps[v.label or k] = v.current
    except Exception:
        pass
    # Fallback: vcgencmd (Raspberry Pi) if available
    vcg = run_cmd('vcgencmd measure_temp')
    if vcg:
        try:
            t = float(vcg.split('=')[1].split("'C")[0])
            temps['cpu_temp_vcgencmd'] = t
        except Exception:
            pass
    return temps


def format_bytes(n):
    # human-readable
    for unit in ['B', 'KB', 'MB', 'GB', 'TB']:
        if abs(n) < 1024.0:
            return f"{n:3.1f}{unit}"
        n /= 1024.0
    return f"{n:.1f}PB"

# ----------------------------- Main Window -----------------------------

class WaterfallMonitor(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle('Raspberry Pi — Waterfall System Monitor')
        self.resize(1200, 720)
        self.setStyleSheet("background-color: #0f1115; color: #e6eef8; font-family: 'Segoe UI', Roboto, sans-serif;")

        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        layout = QtWidgets.QHBoxLayout(central)
        layout.setContentsMargins(12, 12, 12, 12)
        layout.setSpacing(10)

        # Left: waterfall + small controls
        left = QtWidgets.QVBoxLayout()
        layout.addLayout(left, 2)

        self._create_waterfall(left)
        left.addSpacing(8)
        self._create_summary_card(left)

        # Right: charts and device list
        right = QtWidgets.QVBoxLayout()
        layout.addLayout(right, 1)

        self._create_charts(right)
        right.addSpacing(8)
        self._create_devices_panel(right)

        # Timer
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update_all)
        self.timer.start(REFRESH_MS)

        # State
        self.net_prev = psutil.net_io_counters()
        self.last_update = time.time()

    # ---------------- UI Pieces ----------------
    def _create_waterfall(self, parent_layout):
        # Title
        t = QtWidgets.QLabel('Waterfall — live metric history')
        t.setStyleSheet('font-size:16px; font-weight:600;')
        parent_layout.addWidget(t)

        self.wf_view = pg.GraphicsLayoutWidget()
        parent_layout.addWidget(self.wf_view, 1)

        self.wf_img = pg.ImageItem()
        self.wf_plot = self.wf_view.addViewBox(row=0, col=0)
        self.wf_plot.addItem(self.wf_img)
        self.wf_plot.setAspectLocked(False)
        self.wf_plot.invertY(False)
        self.wf_buffer = np.zeros((WATERFALL_HEIGHT, WATERFALL_WIDTH), dtype=np.float32)

        # Colormap
        lut = (COLORMAP.getLookupTable(0.0, 1.0, 256) * 255).astype(np.uint8)
        self.wf_img.setLookupTable(lut)
        self.wf_img.setLevels([0, 1])

    def _create_summary_card(self, parent_layout):
        card = QtWidgets.QFrame()
        card.setStyleSheet('background: rgba(255,255,255,0.03); border-radius:10px; padding:10px;')
        v = QtWidgets.QVBoxLayout(card)
        self.info_label = QtWidgets.QLabel('...')
        self.info_label.setWordWrap(True)
        v.addWidget(self.info_label)
        parent_layout.addWidget(card, stretch=0)

    def _create_charts(self, parent_layout):
        # CPU plot
        self.cpu_plot = pg.PlotWidget(title='CPU %')
        self.cpu_plot.showGrid(x=True, y=True)
        self.cpu_plot.setYRange(0, 100)
        self.cpu_curve = self.cpu_plot.plot(pen=pg.mkPen(width=2))
        parent_layout.addWidget(self.cpu_plot)

        # Mem plot
        self.mem_plot = pg.PlotWidget(title='Memory %')
        self.mem_plot.showGrid(x=True, y=True)
        self.mem_plot.setYRange(0, 100)
        self.mem_curve = self.mem_plot.plot(pen=pg.mkPen(width=2))
        parent_layout.addWidget(self.mem_plot)

        # Net small text
        self.net_label = QtWidgets.QLabel('Network: ...')
        parent_layout.addWidget(self.net_label)

        # historic data buffers
        self.cpu_hist = deque(maxlen=300)
        self.mem_hist = deque(maxlen=300)

    def _create_devices_panel(self, parent_layout):
        title = QtWidgets.QLabel('Connected devices')
        title.setStyleSheet('font-size:14px; font-weight:600;')
        parent_layout.addWidget(title)

        self.devices_text = QtWidgets.QPlainTextEdit()
        self.devices_text.setReadOnly(True)
        self.devices_text.setMaximumBlockCount(2000)
        self.devices_text.setStyleSheet('background: rgba(255,255,255,0.02); border:none; padding:8px;')
        parent_layout.addWidget(self.devices_text, 1)

    # ---------------- Updates ----------------
    def update_all(self):
        now = time.time()
        dt = now - self.last_update if self.last_update else 0.1
        self.last_update = now

        cpu = psutil.cpu_percent(interval=None)
        mem = psutil.virtual_memory().percent
        disk = psutil.disk_usage('/').percent

        net = psutil.net_io_counters()
        bytes_sent = (net.bytes_sent - self.net_prev.bytes_sent) / max(dt, 1e-6)
        bytes_recv = (net.bytes_recv - self.net_prev.bytes_recv) / max(dt, 1e-6)
        self.net_prev = net

        temps = get_temperatures()

        # update simple history
        self.cpu_hist.append(cpu)
        self.mem_hist.append(mem)
        self.cpu_curve.setData(list(self.cpu_hist))
        self.mem_curve.setData(list(self.mem_hist))

        # Update net label
        self.net_label.setText(f'Network: ↑ {format_bytes(bytes_sent)}/s  ↓ {format_bytes(bytes_recv)}/s')

        # Compose one line for waterfall from metrics
        # create normalized vector of size WATERFALL_WIDTH
        metrics = np.array([cpu / 100.0, mem / 100.0, disk / 100.0, min(bytes_recv / (1024*1024), 10) / 10.0, min(bytes_sent / (1024*1024), 10) / 10.0])
        # add temps
        for k in temps:
            try:
                metrics = np.append(metrics, [temps[k] / 100.0])
            except Exception:
                pass

        # normalize and expand
        line = np.interp(np.linspace(0, len(metrics)-1, WATERFALL_WIDTH), np.arange(len(metrics)), metrics)
        # add noise/detail
        line = line + 0.02 * np.random.rand(WATERFALL_WIDTH)
        line = np.clip(line, 0, 1)

        # roll buffer down and add new line at top
        self.wf_buffer = np.roll(self.wf_buffer, 1, axis=0)
        self.wf_buffer[0, :] = line

        # apply slight vertical blur for aesthetics
        self.wf_img.setImage(self.wf_buffer, autoLevels=False)

        # update info card and devices
        sysinfo = f"Host: {platform.node()}  |  {platform.system()} {platform.release()}\n"
        sysinfo += f"Uptime: {self._get_uptime_str()}\n"
        sysinfo += f"CPU: {cpu:.1f}%  MEM: {mem:.1f}%  Disk: {disk:.1f}%\n"
        if temps:
            sysinfo += 'Temps: ' + ', '.join([f"{k}:{v:.1f}°C" for k, v in temps.items()]) + '\n'
        parent_layout_info = sysinfo
        self.info_label.setText(parent_layout_info)

        # devices
        usb_lines = list_usb()
        i2c = i2c_scan()
        dev_text = 'USB Devices:\n' + ('\n'.join(usb_lines) if usb_lines else 'none') + '\n\n'
        dev_text += 'I2C Scan:\n' + (i2c if i2c else 'i2c-tools not available or no devices') + '\n'
        self.devices_text.setPlainText(dev_text)

    def _get_uptime_str(self):
        try:
            with open('/proc/uptime', 'r') as f:
                s = float(f.read().split()[0])
                m, s = divmod(s, 60)
                h, m = divmod(m, 60)
                d, h = divmod(h, 24)
                return f"{int(d)}d {int(h)}h {int(m)}m"
        except Exception:
            return 'unknown'

# ----------------------------- Run -----------------------------

def main():
    app = QtWidgets.QApplication(sys.argv)
    # dark theme for pyqtgraph elements
    pg.setConfigOptions(antialias=True)
    win = WaterfallMonitor()
    win.show()
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()
