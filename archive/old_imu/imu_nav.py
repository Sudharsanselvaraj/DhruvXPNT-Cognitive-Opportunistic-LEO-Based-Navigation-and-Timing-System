#!/usr/bin/env python3
"""
realtime_server_pi_neon_with_header.py

Raspberry Pi IMU -> Live map neon HUD
- Auto-detect serial (or use FORCE_SERIAL_PORT)
- Accepts CSV with header, e.g.:
  t_ms,ax_g,ay_g,az_g,gx_dps,gy_dps,gz_dps,pitch_deg,roll_deg,yaw_deg,pressure_Pa,alt_m
- Builds header map and parses rows accordingly (falls back to fixed indices if no header)
- Emits: lat, lon, t, t_epoch, px,py,pz, vx,vy,vz, speed, distance, pressure, alt
- Frontend shows Lat/Lon, Time, Distance, Velocity, Speed, Altitude
"""

import os, glob, time, math, sys
from collections import deque
import numpy as np
from flask import Flask, render_template_string
from flask_socketio import SocketIO
import serial

# ---------------- USER CONFIG ----------------
BAUDRATE = 115200
LAT0 = 18.494146
LON0 = 74.019613
G = 9.80665
FORCE_SERIAL_PORT = None  # set to "/dev/ttyUSB0" to force
# ---------------------------------------------

app = Flask(__name__)
socketio = SocketIO(app, cors_allowed_origins="*", async_mode="threading")

# ---------------- global state ----------------
vx = vy = vz = 0.0
px = py = pz = 0.0
distance_total = 0.0
last_t = None
static_counter = 0

R_earth = 6378137.0

# filtering params
ACC_LPF_ALPHA = 0.25
ACC_DEADZONE_G = 0.05
VEL_SMOOTH_ALPHA = 0.80
STATIC_STD_WINDOW = 12
STATIC_STD_THRESHOLD_G = 0.05
STATIC_COUNT_REQUIRED = 12

_acc_mag_buf = deque(maxlen=STATIC_STD_WINDOW)
_acc_lpf = np.array([0.0, 0.0, 0.0])
_accel_bias = np.array([0.0, 0.0, 0.0])
_ACCEL_BIAS_LEARN_RATE = 0.002

# header -> index map (populated when header line seen)
HEADER_MAP = None  # dict: name -> index

# fallback indices (old positional format)
FALLBACK_IDX = {
    "t_ms": 0,
    "ax_g": 1, "ay_g": 2, "az_g": 3,
    "gx_dps": 4, "gy_dps": 5, "gz_dps": 6,
    "pitch_deg": 7, "roll_deg": 8, "yaw_deg": 9,
    "pressure_Pa": 10, "alt_m": 11
}


# ---------------- helpers ----------------
def auto_detect_serial():
    if FORCE_SERIAL_PORT:
        if os.path.exists(FORCE_SERIAL_PORT):
            print("[INFO] Using forced serial port:", FORCE_SERIAL_PORT)
            return FORCE_SERIAL_PORT
        else:
            print("[WARN] Forced serial port not found:", FORCE_SERIAL_PORT)

    candidates = sorted(glob.glob("/dev/ttyUSB*")) + sorted(glob.glob("/dev/ttyACM*"))
    if os.path.exists("/dev/serial0"):
        candidates.append("/dev/serial0")

    if not candidates:
        print("[ERROR] No serial candidates found. Plug in device or set FORCE_SERIAL_PORT.")
        return None

    for dev in candidates:
        try:
            ser = serial.Serial(dev, BAUDRATE, timeout=0.5)
            ser.close()
            print("[INFO] Auto-detected serial device:", dev)
            return dev
        except Exception:
            continue
    print("[ERROR] Serial candidates present but none opened successfully:", candidates)
    return None


def build_header_map(line):
    """Given a CSV header line, build a dict mapping column name -> index."""
    parts = [p.strip() for p in line.split(",")]
    hmap = {}
    for i, name in enumerate(parts):
        if name:
            hmap[name] = i
    return hmap


def get_field(parts, name):
    """Return float value for column 'name' from parts using HEADER_MAP if present else fallback index.
       Returns None on parse error."""
    try:
        idx = HEADER_MAP.get(name) if HEADER_MAP else None
        if idx is None:
            idx = FALLBACK_IDX.get(name)
        if idx is None or idx >= len(parts):
            return None
        return float(parts[idx])
    except Exception:
        return None


def R_body_to_nav(roll, pitch, yaw):
    cr = math.cos(roll); sr = math.sin(roll)
    cp = math.cos(pitch); sp = math.sin(pitch)
    cy = math.cos(yaw); sy = math.sin(yaw)
    return np.array([
        [cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr],
        [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr],
        [-sp,   cp*sr,            cp*cr]
    ])


def enu_to_latlon(px_local, py_local, lat0, lon0):
    lat = lat0 + (py_local / R_earth) * (180.0 / math.pi)
    lon = lon0 + (px_local / (R_earth * math.cos(math.radians(lat0)))) * (180.0 / math.pi)
    return lat, lon


# ---------------- process CSV row (robust to header) ----------------
def process_csv_row(parts):
    """
    parts: list of strings
    uses get_field() to extract values by name (header-aware)
    """
    global vx, vy, vz, px, py, pz, distance_total, last_t, static_counter
    global _acc_lpf, _acc_mag_buf, _accel_bias

    # parse required fields
    t_ms = get_field(parts, "t_ms")
    ax_g = get_field(parts, "ax_g")
    ay_g = get_field(parts, "ay_g")
    az_g = get_field(parts, "az_g")
    pitch_deg = get_field(parts, "pitch_deg")
    roll_deg = get_field(parts, "roll_deg")
    yaw_deg = get_field(parts, "yaw_deg")
    pressure = get_field(parts, "pressure_Pa")
    alt_m = get_field(parts, "alt_m")

    # if any required accel/time/orientation is missing -> skip
    if t_ms is None or ax_g is None or ay_g is None or az_g is None or pitch_deg is None or roll_deg is None or yaw_deg is None:
        return None

    t_s = float(t_ms) / 1000.0
    if last_t is None:
        dt = 0.01
    else:
        dt = max(0.001, min(0.1, t_s - last_t))
    last_t = t_s

    raw_acc = np.array([ax_g, ay_g, az_g])
    acc_unbiased = raw_acc - _accel_bias

    _acc_lpf = ACC_LPF_ALPHA * _acc_lpf + (1.0 - ACC_LPF_ALPHA) * acc_unbiased
    ax_f, ay_f, az_f = _acc_lpf.tolist()

    def dz(v): return 0.0 if abs(v) < ACC_DEADZONE_G else v
    ax_g_clean = dz(ax_f); ay_g_clean = dz(ay_f); az_g_clean = dz(az_f)

    a_body = np.array([ax_g_clean * G, ay_g_clean * G, az_g_clean * G])

    roll = math.radians(roll_deg)
    pitch = math.radians(pitch_deg)
    yaw = math.radians(yaw_deg)
    Rb2n = R_body_to_nav(roll, pitch, yaw)
    a_nav = Rb2n.dot(a_body) - np.array([0.0, 0.0, G])

    dead_acc_threshold = ACC_DEADZONE_G * G
    a_nav[0] = 0.0 if abs(a_nav[0]) < dead_acc_threshold else a_nav[0]
    a_nav[1] = 0.0 if abs(a_nav[1]) < dead_acc_threshold else a_nav[1]

    vx_raw = vx + a_nav[0] * dt
    vy_raw = vy + a_nav[1] * dt
    vz_raw = vz + a_nav[2] * dt

    vx = VEL_SMOOTH_ALPHA * vx + (1.0 - VEL_SMOOTH_ALPHA) * vx_raw
    vy = VEL_SMOOTH_ALPHA * vy + (1.0 - VEL_SMOOTH_ALPHA) * vy_raw
    vz = VEL_SMOOTH_ALPHA * vz + (1.0 - VEL_SMOOTH_ALPHA) * vz_raw

    speed = math.sqrt(vx*vx + vy*vy)

    acc_mag = math.sqrt(raw_acc[0]*raw_acc[0] + raw_acc[1]*raw_acc[1] + raw_acc[2]*raw_acc[2])
    _acc_mag_buf.append(acc_mag)
    std = np.std(np.array(_acc_mag_buf)) if len(_acc_mag_buf) >= 4 else 1.0

    if std < STATIC_STD_THRESHOLD_G:
        static_counter += 1
    else:
        static_counter = 0

    if static_counter >= STATIC_COUNT_REQUIRED:
        vx = 0.0
        vy = 0.0
        # learn bias slowly
        _accel_bias = (1.0 - _ACCEL_BIAS_LEARN_RATE) * _accel_bias + _ACCEL_BIAS_LEARN_RATE * raw_acc

    px += vx * dt
    py += vy * dt
    pz += vz * dt

    distance_total += speed * dt

    lat, lon = enu_to_latlon(px, py, LAT0, LON0)
    t_epoch = time.time()

    # ensure alt/pressure are floats or None
    pressure_f = float(pressure) if pressure is not None else None
    alt_f = float(alt_m) if alt_m is not None else None

    return float(lat), float(lon), float(t_s), float(t_epoch), float(px), float(py), float(pz), float(vx), float(vy), float(vz), float(speed), float(distance_total), pressure_f, alt_f


# ---------------- serial reader ----------------
def serial_reader_thread():
    global HEADER_MAP
    serial_port = auto_detect_serial()
    if serial_port is None:
        print("[FATAL] No serial device found. Serial thread terminating.")
        return

    try:
        ser = serial.Serial(serial_port, BAUDRATE, timeout=1)
    except Exception as e:
        print("[ERROR] Could not open serial port:", e)
        return

    print(f"[OK] Serial opened: {serial_port} @ {BAUDRATE}")

    header_found = False
    while not header_found:
        line = ser.readline().decode(errors="ignore").strip()
        if not line:
            time.sleep(0.01); continue

        # if header contains t_ms -> build header map
        if "t_ms" in line and "ax_g" in line:
            HEADER_MAP = build_header_map(line)
            print("[INFO] Header detected. Columns:", list(HEADER_MAP.keys()))
            header_found = True
            continue

        # if numeric rows start without header, proceed
        parts = line.split(",")
        if len(parts) >= 10:
            try:
                float(parts[0])
                print("[INFO] Numeric data found (no header). Starting stream.")
                header_found = True
                res = process_csv_row(parts)
                if res:
                    lat, lon, t_s, t_epoch, px, py, pz, vx_, vy_, vz_, speed, distance, pressure, alt = res
                    socketio.emit("new_point", {
                        'lat': lat, 'lon': lon, 't': t_s, 't_epoch': t_epoch,
                        'px': px, 'py': py, 'pz': pz,
                        'vx': vx_, 'vy': vy_, 'vz': vz_,
                        'speed': speed, 'distance': distance,
                        'pressure': pressure, 'alt': alt
                    })
                break
            except Exception:
                continue

    # main loop
    while True:
        raw = ser.readline().decode(errors="ignore").strip()
        if not raw:
            time.sleep(0.01); continue
        parts = raw.split(",")
        if len(parts) < 10:
            continue
        res = process_csv_row(parts)
        if not res:
            continue
        lat, lon, t_s, t_epoch, px, py, pz, vx_, vy_, vz_, speed, distance, pressure, alt = res
        socketio.emit("new_point", {
            'lat': lat, 'lon': lon, 't': t_s, 't_epoch': t_epoch,
            'px': px, 'py': py, 'pz': pz,
            'vx': vx_, 'vy': vy_, 'vz': vz_,
            'speed': speed, 'distance': distance,
            'pressure': pressure, 'alt': alt
        })
        time.sleep(0.001)


# ---------------- HTML / Frontend ----------------
INDEX_HTML = r"""
<!doctype html>
<html>
<head>
<meta charset="utf-8"/>
<title>Neon IMU Live Tracking (Header aware)</title>
<meta name="viewport" content="width=device-width,initial-scale=1.0"/>
<link rel="stylesheet" href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css"/>
<style>
  html,body,#map{ height:100%; margin:0; padding:0; background:#0b0f12; }
  .status {
    position:absolute; top:18px; left:18px; z-index:999;
    background: rgba(2,8,12,0.6); color:#bffcf2; padding:14px 18px;
    border-radius:10px; font-family: "Consolas", monospace; font-size:14px;
    border: 1px solid rgba(0,255,180,0.18);
    box-shadow: 0 8px 30px rgba(0,0,0,0.6), inset 0 1px 0 rgba(255,255,255,0.02);
    backdrop-filter: blur(6px);
    min-width:320px;
  }
  .status div { margin:6px 0; }
  .pulse-base {
      width: 18px; height: 18px; border-radius: 50%;
      background: linear-gradient(180deg,#ff5270,#ff002e);
      box-shadow: 0 0 18px rgba(255,80,120,0.85), 0 0 40px rgba(255,0,60,0.25);
      position: absolute; top:6px; left:6px;
  }
  .pulse-ring {
      position: absolute; width: 18px; height: 18px; border-radius:50%;
      border: 2px solid rgba(255,30,80,0.65); top:6px; left:6px;
      animation: neonPulse 1.4s infinite ease-out;
  }
  @keyframes neonPulse {
      0% { transform: scale(1); opacity: 0.95; }
      60% { transform: scale(2.6); opacity: 0.28; }
      100% { transform: scale(3.2); opacity: 0; }
  }
  .leaflet-control-attribution { opacity: 0.8; font-size:11px; }
  .start-pin { width:34px; height:40px; display:block; transform:translate(-17px,-40px); }
</style>
</head>
<body>
<div id="map"></div>

<div class="status">
  <div id="coord">Lat/Lon: -- , --</div>
  <div id="time">Time: --</div>
  <div id="dist">Distance: 0.00 m</div>
  <div id="vel">Velocity: (0.00, 0.00) m/s</div>
  <div id="speed">Speed: 0.00 m/s</div>
  <div id="alt">Alt: -- m</div>
</div>

<script src="https://cdn.socket.io/4.6.1/socket.io.min.js"></script>
<script src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js"></script>
<script>
const LAT0 = {{lat0}}, LON0 = {{lon0}};
var map = L.map('map', { zoomControl: true }).setView([LAT0, LON0], 19);
L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', { maxZoom: 20 }).addTo(map);

var poly = L.polyline([], { color: '#ff1f47', weight: 4, opacity: 0.95 }).addTo(map);
var startMarker = null, currMarker = null;
var pulsingHtml = '<div class="pulse-base"></div><div class="pulse-ring"></div>';

var startPinSVG = `
  <svg class="start-pin" viewBox="0 0 24 36" xmlns="http://www.w3.org/2000/svg">
    <path d="M12 0C7.03 0 3 4.03 3 9c0 6.63 9 20 9 20s9-13.37 9-20c0-4.97-4.03-9-9-9z" fill="#00ffd0" stroke="#00ffd0" stroke-opacity="0.9"/>
    <circle cx="12" cy="9" r="3.2" fill="#041617"/>
  </svg>
`;

var socket = io();
socket.on("connect", function(){ console.log("Socket.IO connected"); });
socket.on("connect_error", function(e){ console.error("Socket.IO connect_error", e); });

socket.on("new_point", function(d){
    if (!d || typeof d.lat !== "number" || typeof d.lon !== "number" || isNaN(d.lat) || isNaN(d.lon)) return;

    if (!startMarker){
        var startIcon = L.divIcon({ html: startPinSVG, className: "", iconSize: [34,40] });
        startMarker = L.marker([d.lat, d.lon], { icon: startIcon, interactive: true }).addTo(map);
        startMarker.bindPopup("<b>Start</b><br/>" + d.lat.toFixed(6) + ", " + d.lon.toFixed(6)).openPopup();
    }

    if (!currMarker){
        var icon = L.divIcon({ html: pulsingHtml, className: "" , iconSize: [36,36] });
        currMarker = L.marker([d.lat, d.lon], { icon: icon, interactive: false }).addTo(map);
    } else {
        currMarker.setLatLng([d.lat, d.lon]);
    }

    poly.addLatLng([d.lat, d.lon]);
    map.panTo([d.lat, d.lon]);

    document.getElementById("coord").innerText = "Lat/Lon: " + d.lat.toFixed(6) + " , " + d.lon.toFixed(6);

    if (d.t_epoch && !isNaN(Number(d.t_epoch))) {
        document.getElementById("time").innerText = "Time: " + new Date(Number(d.t_epoch)*1000).toLocaleString();
    } else if (d.t && !isNaN(Number(d.t))) {
        var tn = Number(d.t);
        if (tn > 1e9) document.getElementById("time").innerText = "Time: " + new Date(tn*1000).toLocaleString();
        else document.getElementById("time").innerText = "Time: " + new Date().toLocaleTimeString() + " (t=" + tn.toFixed(3) + "s)";
    } else {
        document.getElementById("time").innerText = "Time: --";
    }

    document.getElementById("dist").innerText = "Distance: " + (d.distance||0).toFixed(2) + " m";
    document.getElementById("vel").innerText = "Velocity: (" + (d.vx||0).toFixed(2) + ", " + (d.vy||0).toFixed(2) + ") m/s";
    document.getElementById("speed").innerText = "Speed: " + (d.speed||0).toFixed(2) + " m/s";
    document.getElementById("alt").innerText = "Alt: " + ((d.alt !== null && d.alt !== undefined) ? d.alt.toFixed(2) + " m" : "--");
});
</script>
</body>
</html>
"""

@app.route("/")
def index():
    return render_template_string(INDEX_HTML, lat0=LAT0, lon0=LON0)


# ---------------- main ----------------
if __name__ == "__main__":
    print("Python:", sys.version)
    socketio.start_background_task(serial_reader_thread)
    socketio.run(app, host="0.0.0.0", port=5000)
