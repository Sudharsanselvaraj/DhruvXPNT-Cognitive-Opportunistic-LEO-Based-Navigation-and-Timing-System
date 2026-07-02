#!/usr/bin/env python3
"""
Enhanced logger_full.py
- same fields as before
- additionally monitors dmesg for USB-related lines and counts new USB reset messages
- writes CSV to ~/pnt_test/run1/data.csv
"""
import os, time, csv, json, psutil, subprocess, re
from datetime import datetime

RUN_DIR = os.path.expanduser("~/pnt_test/run1")
os.makedirs(RUN_DIR, exist_ok=True)
CSV_PATH = os.path.join(RUN_DIR, "data.csv")

ARDUINO_PATH = "/dev/ttyACM0"   # set to None if not used
SDR_PATH = "/dev/bus/usb"       # just used for checking /dev (not required)

num_cores = psutil.cpu_count(logical=True)

base_fields = [
    "timestamp","epoch_ms","uptime_s",
    "cpu_total_pct","cpu_freq_MHz","cpu_temp_C"
]
core_fields = [f"cpu_core_{i}" for i in range(num_cores)]
mem_fields = ["mem_used_MB","mem_total_MB","mem_pct","swap_used_MB","swap_total_MB","swap_pct"]
disk_fields = ["disk_usage_pct","disk_read_Bs","disk_write_Bs"]
net_fields = ["net_recv_Bs","net_sent_Bs"]
other_fields = ["throttled_flag","cpu_clock_MHz","dmesg_new_count","dmesg_usb_resets","arduino_json","sdr_event_json"]

fields = base_fields + core_fields + mem_fields + disk_fields + net_fields + other_fields

usb_reset_pattern = re.compile(r"USB.*reset", flags=re.IGNORECASE)
usb_error_pattern = re.compile(r"usb .* error|under-voltage|Voltage.*dropped", flags=re.IGNORECASE)

def read_vcgencmd_get_throttled():
    try:
        out = subprocess.check_output(["vcgencmd","get_throttled"], stderr=subprocess.DEVNULL).decode().strip()
        return out
    except Exception:
        return None

def read_vcgencmd_measure_clock():
    try:
        out = subprocess.check_output(["vcgencmd","measure_clock","arm"], stderr=subprocess.DEVNULL).decode().strip()
        return int(out.split('=')[1]) / 1e6
    except Exception:
        return None

def read_cpu_temp():
    try:
        return float(open("/sys/class/thermal/thermal_zone0/temp").read())/1000.0
    except:
        return None

def read_arduino_once():
    if not ARDUINO_PATH:
        return None
    try:
        import serial
        ser = serial.Serial(ARDUINO_PATH, 115200, timeout=0.05)
        line = ser.readline().decode(errors="ignore").strip()
        ser.close()
        if line:
            try:
                return json.loads(line)
            except:
                return {"raw": line}
    except Exception:
        return None

def dmesg_lines():
    try:
        out = subprocess.check_output(["dmesg","--ctime","--color=never"], stderr=subprocess.DEVNULL).decode(errors="ignore")
        return out.splitlines()
    except:
        return []

prev_disk = psutil.disk_io_counters()
prev_net = psutil.net_io_counters()
prev_time = time.time()

# baseline dmesg counts and usb-reset counts
dmesg_base_lines = dmesg_lines()
dmesg_base_count = len(dmesg_base_lines)
dmesg_base_usb_resets = sum(1 for l in dmesg_base_lines if usb_reset_pattern.search(l) or usb_error_pattern.search(l))

with open(CSV_PATH, "w", newline="") as csvfile:
    writer = csv.DictWriter(csvfile, fieldnames=fields)
    writer.writeheader()
    print("Logging to", CSV_PATH)
    try:
        while True:
            now = time.time()
            ts = datetime.utcnow().isoformat() + "Z"
            epoch_ms = int(now * 1000)
            uptime_s = None
            try:
                with open("/proc/uptime") as f:
                    uptime_s = float(f.readline().split()[0])
            except:
                pass

            cpu_total = psutil.cpu_percent(interval=0.2)
            cpu_freq = None
            try:
                cpu_freq = psutil.cpu_freq().current
            except:
                cpu_freq = None
            cpu_temp = read_cpu_temp()
            cpu_cores = psutil.cpu_percent(interval=0.0, percpu=True)

            vm = psutil.virtual_memory()
            mem_used_MB = (vm.total - vm.available) / (1024*1024)
            mem_total_MB = vm.total / (1024*1024)
            mem_pct = vm.percent
            swap = psutil.swap_memory()
            swap_used_MB = swap.used / (1024*1024)
            swap_total_MB = swap.total / (1024*1024)
            swap_pct = swap.percent

            du = psutil.disk_usage('/')
            curr_disk = psutil.disk_io_counters()
            dt = now - prev_time if prev_time else 1.0
            read_Bs = (curr_disk.read_bytes - prev_disk.read_bytes) / max(dt,1e-6)
            write_Bs = (curr_disk.write_bytes - prev_disk.write_bytes) / max(dt,1e-6)
            prev_disk = curr_disk

            curr_net = psutil.net_io_counters()
            recv_Bs = (curr_net.bytes_recv - prev_net.bytes_recv) / max(dt,1e-6)
            sent_Bs = (curr_net.bytes_sent - prev_net.bytes_sent) / max(dt,1e-6)
            prev_net = curr_net

            prev_time = now

            throttled = read_vcgencmd_get_throttled()
            cpu_clock_MHz = read_vcgencmd_measure_clock()

            # dmesg and usb resets since baseline
            lines = dmesg_lines()
            dmesg_new_count = len(lines) - dmesg_base_count if (len(lines) is not None and dmesg_base_count is not None) else None
            usb_resets = sum(1 for l in lines if usb_reset_pattern.search(l) or usb_error_pattern.search(l))
            dmesg_usb_resets = usb_resets - dmesg_base_usb_resets if (usb_resets is not None and dmesg_base_usb_resets is not None) else None

            arduino = read_arduino_once()

            # SDR event marker file read
            sdr_event_path = os.path.join(RUN_DIR, "sdr_event.json")
            sdr_event = None
            if os.path.exists(sdr_event_path):
                try:
                    with open(sdr_event_path,"r") as f:
                        sdr_event = f.read().strip()
                except:
                    sdr_event = None

            row = {
                "timestamp": ts,
                "epoch_ms": epoch_ms,
                "uptime_s": uptime_s,
                "cpu_total_pct": cpu_total,
                "cpu_freq_MHz": cpu_freq,
                "cpu_temp_C": cpu_temp,
                "throttled_flag": throttled,
                "cpu_clock_MHz": cpu_clock_MHz,
                "dmesg_new_count": dmesg_new_count,
                "dmesg_usb_resets": dmesg_usb_resets,
                "arduino_json": json.dumps(arduino) if arduino else None,
                "sdr_event_json": sdr_event
            }

            for i, v in enumerate(cpu_cores):
                row[f"cpu_core_{i}"] = v

            row.update({
                "mem_used_MB": round(mem_used_MB,2),
                "mem_total_MB": round(mem_total_MB,2),
                "mem_pct": mem_pct,
                "swap_used_MB": round(swap_used_MB,2),
                "swap_total_MB": round(swap_total_MB,2),
                "swap_pct": swap_pct,
                "disk_usage_pct": du.percent,
                "disk_read_Bs": int(read_Bs),
                "disk_write_Bs": int(write_Bs),
                "net_recv_Bs": int(recv_Bs),
                "net_sent_Bs": int(sent_Bs)
            })

            writer.writerow(row)
            csvfile.flush()
            print(row)
            time.sleep(0.6)
    except KeyboardInterrupt:
        print("Logger stopped by user")
