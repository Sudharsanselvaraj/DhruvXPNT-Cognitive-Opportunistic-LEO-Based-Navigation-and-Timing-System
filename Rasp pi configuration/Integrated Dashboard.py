# integrated_dashboard.py
# Unified IMU + Satellite Navigation Dashboard
# One server, one interface - satellite data on left, IMU map on right

import os, glob, time, math, sys, csv
from collections import deque
import numpy as np
from flask import Flask, render_template_string, jsonify
from flask_socketio import SocketIO
import serial
import json
from pathlib import Path

# -------- USER CONFIG --------
SERIAL_PORT = "COM9"  # Will auto-detect if not found
BAUDRATE = 115200
LAT0 = None  # Will be set from satellite data
LON0 = None  # Will be set from satellite data
G = 9.80665
JSON_FILE = Path("data/live_navigation.json")
SATELLITE_DATA_READY = False  # Flag to start IMU processing

# Advanced IMU parameters
ACC_LPF_ALPHA = 0.22
ACC_DEADZONE_G = 0.04
VEL_SMOOTH_ALPHA = 0.78
STATIC_STD_WINDOW = 14
STATIC_STD_THRESHOLD_G = 0.04
STATIC_COUNT_REQUIRED = 10
MAX_SPEED_MPS = 12.0
VEL_DAMPING = 0.995
MAX_DT = 0.25
MIN_DT = 0.001
_ACCEL_BIAS_LEARN_RATE = 0.003

ALT_LOG_CSV = "alt_corrections_log.csv"
# -----------------------------

app = Flask(_name_)
socketio = SocketIO(app, cors_allowed_origins="*", async_mode='threading')

points = deque(maxlen=5000)

# ---------------- ROTATION MATRIX ----------------
def R_body_to_nav(roll, pitch, yaw):
    cr = math.cos(roll); sr = math.sin(roll)
    cp = math.cos(pitch); sp = math.sin(pitch)
    cy = math.cos(yaw); sy = math.sin(yaw)
    return np.array([
        [cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr],
        [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr],
        [-sp,   cp*sr,            cp*cr]
    ])

# ---------------- STATE VARIABLES ----------------
vx = vy = vz = 0.0
px = py = pz = 0.0
distance_total = 0.0
last_t = None
static_counter = 0
R_earth = 6378137.0

_acc_mag_buf = deque(maxlen=STATIC_STD_WINDOW)
_acc_lpf = np.array([0.0, 0.0, 0.0])
_accel_bias = np.array([0.0, 0.0, 0.0])

HEADER_MAP = None
FALLBACK_IDX = {
    "t_ms": 0,
    "ax_g": 1, "ay_g": 2, "az_g": 3,
    "gx_dps": 4, "gy_dps": 5, "gz_dps": 6,
    "pitch_deg": 7, "roll_deg": 8, "yaw_deg": 9,
    "pressure_Pa": 10, "alt_m": 11
}

# --------------- LAT/LON CONVERSION ---------------
def enu_to_latlon(px, py, lat0, lon0):
    lat = lat0 + (py / R_earth) * (180.0 / math.pi)
    lon = lon0 + (px / (R_earth * math.cos(math.radians(lat0)))) * (180.0 / math.pi)
    return lat, lon

# --------------- HELPER FUNCTIONS ------------------
def auto_detect_serial():
    """Auto-detect serial port on Windows/Linux"""
    # Try configured port first
    if os.path.exists(SERIAL_PORT):
        return SERIAL_PORT
    
    # Windows COM ports
    candidates = [f"COM{i}" for i in range(1, 20)]
    # Linux serial devices
    candidates += sorted(glob.glob("/dev/ttyUSB*")) + sorted(glob.glob("/dev/ttyACM*"))
    if os.path.exists("/dev/serial0"):
        candidates.append("/dev/serial0")
    
    for dev in candidates:
        try:
            ser = serial.Serial(dev, BAUDRATE, timeout=0.5)
            ser.close()
            print(f"[INFO] Auto-detected serial device: {dev}")
            return dev
        except Exception:
            continue
    print("[ERROR] No serial device found.")
    return None

def build_header_map(line):
    parts = [p.strip() for p in line.split(",")]
    return {name: i for i, name in enumerate(parts) if name}

def get_field(parts, name):
    try:
        if HEADER_MAP and name in HEADER_MAP:
            idx = HEADER_MAP[name]
        else:
            idx = FALLBACK_IDX.get(name)
        if idx is None or idx >= len(parts):
            return None
        return float(parts[idx])
    except Exception:
        return None

def barometric_altitude(pressure_pa):
    try:
        P0 = 101325.0
        P = float(pressure_pa)
        if P <= 0:
            return None
        h = 44330.0 * (1.0 - (P / P0) ** (1.0 / 5.255))
        return float(h)
    except Exception:
        return None

def correct_sensor_altitude(alt_sensor, alt_baro):
    """Auto-correct sensor altitude if implausible (cm->m, mm->m, etc.)"""
    if alt_sensor is None:
        return None, None
    try:
        a = float(alt_sensor)
    except Exception:
        return None, None

    # plausible raw
    if -500.0 < a < 15000.0:
        return a, "raw"

    # try candidates
    candidates = {
        'cm->m': a / 100.0,
        'mm->m': a / 1000.0,
        'div10->m': a / 10.0
    }

    if alt_baro is not None:
        best_method = None
        best_val = None
        best_err = None
        for m, v in candidates.items():
            err = abs(v - alt_baro)
            if best_err is None or err < best_err:
                best_err = err
                best_val = v
                best_method = m
        if best_err is not None and (best_err < 200.0 or best_err < 0.5 * abs(alt_baro)):
            return best_val, best_method
        else:
            return None, None
    else:
        for m, v in candidates.items():
            if -500.0 < v < 15000.0:
                return v, m
        return None, None

def init_alt_csv_log():
    write_header = not os.path.exists(ALT_LOG_CSV)
    try:
        f = open(ALT_LOG_CSV, "a", newline="")
        writer = csv.writer(f)
        if write_header:
            writer.writerow(["timestamp_epoch", "raw_alt", "corrected_alt", "method", "alt_baro", "alt_used", "pressure_Pa"])
        f.close()
    except Exception as e:
        print("[WARN] Could not initialize alt log:", e)

def log_alt_correction(ts_epoch, raw_alt, corrected_alt, method, alt_baro, alt_used, pressure):
    try:
        f = open(ALT_LOG_CSV, "a", newline="")
        writer = csv.writer(f)
        writer.writerow([ts_epoch, raw_alt if raw_alt is not None else "", corrected_alt if corrected_alt is not None else "", method if method else "", alt_baro if alt_baro is not None else "", alt_used if alt_used is not None else "", pressure if pressure is not None else ""])
        f.close()
    except Exception:
        pass

# --------------- PROCESS CSV ROW ------------------
def process_csv_row(parts):
    global vx, vy, vz, px, py, pz, distance_total, last_t, static_counter, _acc_lpf, _acc_mag_buf, _accel_bias

    # Wait for satellite data before processing IMU
    if not SATELLITE_DATA_READY or LAT0 is None or LON0 is None:
        return None

    t_ms = get_field(parts, "t_ms")
    ax_g = get_field(parts, "ax_g")
    ay_g = get_field(parts, "ay_g")
    az_g = get_field(parts, "az_g")
    pitch_deg = get_field(parts, "pitch_deg")
    roll_deg = get_field(parts, "roll_deg")
    yaw_deg = get_field(parts, "yaw_deg")
    pressure = get_field(parts, "pressure_Pa")
    alt_sensor = get_field(parts, "alt_m")

    if t_ms is None or ax_g is None or ay_g is None or az_g is None or pitch_deg is None or roll_deg is None or yaw_deg is None:
        return None

    t_s = float(t_ms) / 1000.0

    if last_t is None:
        dt = 0.01
    else:
        dt = t_s - last_t
        if dt <= 0:
            return None
        if dt > MAX_DT:
            dt = MAX_DT
    last_t = t_s
    dt = max(MIN_DT, dt)

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
    if abs(a_nav[0]) < dead_acc_threshold:
        a_nav[0] = 0.0
    if abs(a_nav[1]) < dead_acc_threshold:
        a_nav[1] = 0.0

    vx_raw = vx + a_nav[0] * dt
    vy_raw = vy + a_nav[1] * dt
    vz_raw = vz + a_nav[2] * dt

    vx = VEL_SMOOTH_ALPHA * vx + (1.0 - VEL_SMOOTH_ALPHA) * vx_raw
    vy = VEL_SMOOTH_ALPHA * vy + (1.0 - VEL_SMOOTH_ALPHA) * vy_raw
    vz = VEL_SMOOTH_ALPHA * vz + (1.0 - VEL_SMOOTH_ALPHA) * vz_raw

    vx *= VEL_DAMPING
    vy *= VEL_DAMPING
    vz *= VEL_DAMPING

    speed = math.sqrt(vx*vx + vy*vy)
    if speed > MAX_SPEED_MPS:
        scale = MAX_SPEED_MPS / speed
        vx *= scale; vy *= scale
        speed = MAX_SPEED_MPS

    acc_mag = math.sqrt(raw_acc[0]*2 + raw_acc[1]2 + raw_acc[2]*2)
    _acc_mag_buf.append(acc_mag)
    std = np.std(np.array(_acc_mag_buf)) if len(_acc_mag_buf) >= 4 else 1.0

    if std < STATIC_STD_THRESHOLD_G:
        static_counter += 1
    else:
        static_counter = 0

    if static_counter >= STATIC_COUNT_REQUIRED:
        vx = 0.0
        vy = 0.0
        _accel_bias = (1.0 - _ACCEL_BIAS_LEARN_RATE) * _accel_bias + _ACCEL_BIAS_LEARN_RATE * raw_acc

    px += vx * dt
    py += vy * dt
    pz += vz * dt

    distance_total += speed * dt

    lat, lon = enu_to_latlon(px, py, LAT0, LON0)

    alt_baro = None
    if pressure is not None:
        alt_baro = barometric_altitude(pressure)

    corrected_alt, method = correct_sensor_altitude(alt_sensor, alt_baro)
    if corrected_alt is not None:
        alt_used = corrected_alt
        alt_method = method
    elif alt_baro is not None:
        alt_used = alt_baro
        alt_method = "baro"
    else:
        alt_used = None
        alt_method = None

    t_epoch = time.time()

    try:
        log_alt_correction(t_epoch, alt_sensor, corrected_alt, method, alt_baro, alt_used, pressure)
    except Exception:
        pass

    return lat, lon, t_s, t_epoch, px, py, pz, vx, vy, vz, speed, distance_total, pressure, alt_sensor, alt_baro, alt_used, alt_method

# --------------- JSON MONITOR ---------------------
def json_monitor_thread():
    """Continuously monitor JSON file and update LAT0/LON0 when new satellite data arrives"""
    global LAT0, LON0, SATELLITE_DATA_READY
    last_timestamp = None
    
    print("JSON monitor started - waiting for satellite coordinates...")
    
    while True:
        try:
            if JSON_FILE.exists():
                with open(JSON_FILE, 'r') as f:
                    nav_data = json.load(f)
                    if nav_data and isinstance(nav_data, list) and len(nav_data) > 0:
                        latest = nav_data[-1]
                        new_timestamp = latest.get('timestamp')
                        
                        # Only update if we have a new data point
                        if new_timestamp != last_timestamp:
                            new_lat = latest.get('lat')
                            new_lon = latest.get('lon')
                            
                            if new_lat is not None and new_lon is not None:
                                LAT0 = new_lat
                                LON0 = new_lon
                                last_timestamp = new_timestamp
                                
                                if not SATELLITE_DATA_READY:
                                    SATELLITE_DATA_READY = True
                                    print(f"🛰 SATELLITE DATA ACQUIRED! LAT={LAT0:.6f}, LON={LON0:.6f}")
                                    print("✓ IMU tracking now enabled")
                                else:
                                    print(f"🛰 Updated reference from satellite: LAT={LAT0:.6f}, LON={LON0:.6f}")
        except Exception as e:
            pass  # Silent fail, keep monitoring
        
        time.sleep(1)  # Check every second

# --------------- SERIAL READER ---------------------
def serial_reader_thread():
    global HEADER_MAP
    
    serial_port = auto_detect_serial()
    if serial_port is None:
        print("[FATAL] No serial device found.")
        return
        
    try:
        ser = serial.Serial(serial_port, BAUDRATE, timeout=1)
    except Exception as e:
        print("[ERROR] Could not open serial:", e)
        return

    print(f"[OK] Serial opened: {serial_port} @ {BAUDRATE}")

    header_found = False
    while not header_found:
        line = ser.readline().decode(errors='ignore').strip()
        if not line:
            time.sleep(0.01); continue

        if "t_ms" in line and "ax_g" in line:
            HEADER_MAP = build_header_map(line)
            print("[INFO] Header detected. Columns:", list(HEADER_MAP.keys()))
            header_found = True
            continue

        parts = line.split(",")
        if len(parts) >= 10:
            try:
                float(parts[0])
                print("[INFO] Numeric data detected (no header). Starting stream.")
                header_found = True
                res = process_csv_row(parts)
                if res:
                    lat, lon, t_s, t_epoch, px, py, pz, vx_, vy_, vz_, speed, dist, pressure, alt_sensor, alt_baro, alt_used, alt_method = res
                    socketio.emit("new_point", {
                        'lat': lat, 'lon': lon, 't': t_s, 't_epoch': t_epoch,
                        'px': px, 'py': py, 'pz': pz,
                        'vx': vx_, 'vy': vy_, 'vz': vz_,
                        'speed': speed, 'distance': dist,
                        'pressure': pressure, 'alt_sensor': alt_sensor, 'alt_baro': alt_baro, 'alt_used': alt_used, 'alt_method': alt_method
                    })
                break
            except Exception:
                pass

    while True:
        raw = ser.readline().decode(errors='ignore').strip()
        if not raw:
            time.sleep(0.01); continue
        parts = raw.split(",")
        if len(parts) < 10: continue

        res = process_csv_row(parts)
        if not res: continue

        lat, lon, t_s, t_epoch, px, py, pz, vx_, vy_, vz_, speed, dist, pressure, alt_sensor, alt_baro, alt_used, alt_method = res

        socketio.emit("new_point", {
            'lat': lat, 'lon': lon, 't': t_s, 't_epoch': t_epoch,
            'px': px, 'py': py, 'pz': pz,
            'vx': vx_, 'vy': vy_, 'vz': vz_,
            'speed': speed, 'distance': dist,
            'pressure': pressure, 'alt_sensor': alt_sensor, 'alt_baro': alt_baro, 'alt_used': alt_used, 'alt_method': alt_method
        })

        time.sleep(0.001)

# ---------------- UNIFIED HTML UI -----------------
UNIFIED_HTML = """
<!DOCTYPE html>
<html>
<head>
    <title>DhruvX LEO-PNT Live</title>
    <meta charset="utf-8">
    <link rel="stylesheet" href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css" />
    <style>
        * { margin: 0; padding: 0; box-sizing: border-box; }
        body { 
            font-family: 'Courier New', monospace;
            background: #000;
            color: #00FF00;
            overflow: hidden;
        }
        
        #header {
            background: #111;
            padding: 15px 20px;
            border-bottom: 2px solid #00FF00;
        }
        
        h1 { 
            font-size: 16px; 
            font-weight: bold;
            letter-spacing: 2px;
        }
        
        #main-container {
            display: flex;
            height: calc(100vh - 60px);
        }
        
        #left-panel {
            width: 40%;
            background: #000;
            border-right: 2px solid #222;
            padding: 30px 40px;
        }
        
        #right-panel {
            width: 60%;
            background: #000;
            padding: 30px 40px;
        }
        
        .panel-header {
            font-size: 18px;
            text-decoration: underline;
            margin-bottom: 30px;
            letter-spacing: 1px;
        }
        
        .data-item {
            font-size: 20px;
            font-weight: bold;
            margin-bottom: 20px;
            letter-spacing: 1px;
        }
        
        #map-container {
            background: #0a0a0a;
            border: 2px solid #222;
            width: 100%;
            height: calc(100% - 100px);
            position: relative;
            display: flex;
            align-items: center;
            justify-content: center;
        }
        
        #map {
            width: 100%;
            height: 100%;
            background: #0a0a0a;
        }
        
        #map-waiting {
            position: absolute;
            color: #00FF00;
            font-size: 18px;
            font-weight: bold;
            text-align: center;
            z-index: 1000;
            animation: blink 2s infinite;
        }
        
        @keyframes blink {
            0%, 50%, 100% { opacity: 1; }
            25%, 75% { opacity: 0.3; }
        }
        
        #map.hidden {
            display: none;
        }
        
        .btn {
            background: #222;
            color: #00FF00;
            border: 1px solid #00FF00;
            padding: 10px 20px;
            font-family: 'Courier New', monospace;
            font-size: 12px;
            cursor: pointer;
            margin-top: 10px;
        }
        
        .btn:hover {
            background: #444;
        }
        
        .imu-status {
            position: absolute;
            bottom: 20px;
            left: 20px;
            z-index: 999;
            background: rgba(0,0,0,0.8);
            color: #00FF00;
            padding: 10px 14px;
            border-radius: 8px;
            font-family: 'Courier New', monospace;
            font-size: 11px;
            min-width: 200px;
            border: 1px solid #00FF00;
        }
    </style>
</head>
<body>
    <div id="header">
        <h1>SYSTEM HEALTH: <span id="status">NOMINAL</span></h1>
    </div>
    
    <div id="main-container">
        <div id="left-panel">
            <div class="panel-header">NAVIGATION DATA</div>
            
            <div class="data-item" id="lat">Lat : ACQUIRING...</div>
            <div class="data-item" id="lon">Long: ACQUIRING...</div>
            <div class="data-item" style="margin-top: 40px; font-size: 16px;" id="utc">UTC : WAITING FOR SIGNAL</div>
        </div>
        
        <div id="right-panel">
            <div class="panel-header">MAP / IMU VISUALIZATION</div>
            
            <div id="map-container">
                <div id="map-waiting">⏳ WAITING FOR SATELLITE DATA...</div>
                <div id="map" class="hidden"></div>
                <div class="imu-status">
                    <div id="imu-time">Time: --</div>
                    <div id="imu-dist">Distance: 0.00 m</div>
                    <div id="imu-vel">Velocity: (0.00, 0.00) m/s</div>
                    <div id="imu-speed">Speed: 0.00 m/s</div>
                    <div id="imu-alt">Alt(sensor/baro/used/method): -- / -- / -- / --</div>
                    <div id="imu-pressure">Pressure: -- Pa</div>
                </div>
            </div>
            
            <button class="btn" onclick="location.reload()">REFRESH MAP</button>
        </div>
    </div>
    
    <script src="https://cdn.socket.io/4.6.1/socket.io.min.js"></script>
    <script src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js"></script>
    <script>
        // Initialize Leaflet map - wait for satellite data
        var map = null;
        var poly = null;
        var startMarker = null;
        var currMarker = null;
        var mapInitialized = false;

        function initMap(lat, lon) {
            if (mapInitialized) return;
            
            // Hide waiting message, show map
            document.getElementById('map-waiting').style.display = 'none';
            document.getElementById('map').classList.remove('hidden');
            
            map = L.map('map').setView([lat, lon], 18);
            L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png',{maxZoom:19}).addTo(map);
            poly = L.polyline([], {color:'#00FF00',weight:3}).addTo(map);
            mapInitialized = true;
            console.log("Map initialized at", lat, lon);
        }

        function formatLatLon(lat, lon) {
            return lat.toFixed(6) + ", " + lon.toFixed(6);
        }

        // Socket.IO for IMU real-time updates
        var socket = io();
        socket.on("new_point", function(d){
            // Initialize map on first IMU data point
            if (!mapInitialized) {
                initMap(d.lat, d.lon);
            }
            
            if (!startMarker){
                startMarker = L.marker([d.lat, d.lon]).addTo(map);
                startMarker.bindPopup("<b>Start</b><br/>" + formatLatLon(d.lat, d.lon)).openPopup();
            }

            if (currMarker) {
                currMarker.setLatLng([d.lat,d.lon]);
                currMarker.bindPopup("<b>Now</b><br/>" + formatLatLon(d.lat, d.lon)).openPopup();
            } else {
                currMarker = L.marker([d.lat,d.lon]).addTo(map);
                currMarker.bindPopup("<b>Now</b><br/>" + formatLatLon(d.lat, d.lon)).openPopup();
            }

            poly.addLatLng([d.lat,d.lon]);
            map.panTo([d.lat,d.lon]);

            // Update IMU status
            document.getElementById("imu-time").innerText = "Time: " + new Date().toLocaleTimeString();
            document.getElementById("imu-dist").innerText = "Distance: " + d.distance.toFixed(2) + " m";
            document.getElementById("imu-vel").innerText = "Velocity: (" + d.vx.toFixed(2) + ", " + d.vy.toFixed(2) + ") m/s";
            document.getElementById("imu-speed").innerText = "Speed: " + d.speed.toFixed(2) + " m/s";
            
            var alt_sensor = (d.alt_sensor !== null && d.alt_sensor !== undefined) ? d.alt_sensor.toFixed(2) : "--";
            var alt_baro = (d.alt_baro !== null && d.alt_baro !== undefined) ? d.alt_baro.toFixed(2) : "--";
            var alt_used = (d.alt_used !== null && d.alt_used !== undefined) ? d.alt_used.toFixed(2) : "--";
            var alt_method = d.alt_method || "--";
            document.getElementById("imu-alt").innerText = "Alt(sensor/baro/used/method): " + alt_sensor + " / " + alt_baro + " / " + alt_used + " / " + alt_method;
            document.getElementById("imu-pressure").innerText = "Pressure: " + ((d.pressure !== null && d.pressure !== undefined) ? d.pressure.toFixed(2) + " Pa" : "--");
        });
        
        // Fetch satellite navigation data
        var lastSatelliteTimestamp = null;
        setInterval(fetchData, 2000);
        fetchData();
        
        function fetchData() {
            fetch('/api/data')
                .then(r => r.json())
                .then(data => {
                    if (data && data.length > 0) {
                        const latest = data[data.length - 1];
                        
                        // Check if this is new data
                        const currentTimestamp = latest.timestamp;
                        if (currentTimestamp !== lastSatelliteTimestamp) {
                            lastSatelliteTimestamp = currentTimestamp;
                            
                            document.getElementById('lat').textContent = Lat : ${latest.lat.toFixed(6)}°;
                            document.getElementById('lon').textContent = Long: ${latest.lon.toFixed(6)}°;
                            
                            const now = new Date();
                            document.getElementById('utc').textContent = UTC : ${now.toUTCString().slice(17, 25)};
                            
                            // Plot final position on map
                            if (latest.final && !window.finalMarkerPlaced && mapInitialized) {
                                L.marker([latest.lat, latest.lon])
                                    .addTo(map)
                                    .bindPopup(<b>🎯 FINAL POSITION</b><br/>Lat: ${latest.lat.toFixed(6)}°<br/>Lon: ${latest.lon.toFixed(6)}°<br/>Alt: ${latest.alt.toFixed(1)}m)
                                    .openPopup();
                                window.finalMarkerPlaced = true;
                                document.getElementById('status').textContent = 'POSITION LOCKED';
                            }
                        }
                    }
                })
                .catch(err => {
                    // Keep showing "ACQUIRING..." on error
                });
        }
    </script>
</body>
</html>
"""

# ---------------- ROUTES -----------------
@app.route("/")
def dashboard():
    return render_template_string(UNIFIED_HTML)

@app.route('/api/data')
def get_data():
    try:
        if JSON_FILE.exists():
            with open(JSON_FILE, 'r') as f:
                return jsonify(json.load(f))
        return jsonify([])
    except:
        return jsonify([])

# ---------------- MAIN -----------------
if _name_ == "_main_":
    print("\n" + "="*70)
    print("🚀 UNIFIED DASHBOARD - IMU + SATELLITE NAVIGATION")
    print("="*70)
    print("\n📡 Dashboard: http://localhost:5000")
    print("\n" + "="*70 + "\n")
    
    init_alt_csv_log()
    socketio.start_background_task(json_monitor_thread)
    socketio.start_background_task(serial_reader_thread)
    socketio.run(app, host="0.0.0.0", port=5000)
