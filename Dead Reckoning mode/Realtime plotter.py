#!/usr/bin/env python3
"""
Improved realtime IMU -> map server with:
 - robust ZUPT-based bias learning
 - gz (yaw) bias estimation & local yaw integration when no fused yaw provided
 - barometer sanity + stable baseline selection (relative altitude)
 - stronger deadzones and smoothing to reduce drift

Expected CSV:
 t_ms,ax_g,ay_g,az_g,gx_dps,gy_dps,gz_dps,pitch_deg,roll_deg,yaw_deg,pressure_Pa,alt_m
"""
import time, math, sys
from collections import deque
import numpy as np
from flask import Flask, render_template_string
from flask_socketio import SocketIO
import serial

# ---------- USER CONFIG ----------
SERIAL_PORT = "COM5"     # change as needed
BAUDRATE = 115200
LAT0 = 18.494146
LON0 = 74.019613
G = 9.80665

# smoothing / thresholds (tune these)
POS_ALPHA = 0.65
APPEND_THRESHOLD_M = 0.35
PAN_THRESHOLD_M = 0.8
ACC_LPF_ALPHA = 0.18
ACC_DEADZONE_G_BASE = 0.03
FORWARD_AXIS = 'x'
FORWARD_DEADZONE_MULT = 1.9

VEL_SMOOTH_ALPHA = 0.75
STATIC_STD_WINDOW = 16
STATIC_STD_THRESHOLD_G = 0.02    # tighter: require steadier accel
GYRO_STATIC_THRESHOLD_DPS = 3.5  # lower threshold for gyro mean

STATIC_COUNT_REQUIRED = 6        # need N consecutive static frames to apply "static" mode
VEL_BIAS_LEARN_ALPHA = 0.12      # slow continuous learning
VEL_BIAS_FAST_ALPHA = 0.45       # fast correction when static stabilised

BARO_VALID_MIN = 30000.0   # Pa
BARO_VALID_MAX = 110000.0  # Pa
BARO_BUFFER_LEN = 20
BARO_STABLE_STD = 10.0     # Pa; require stability before locking baseline

# ---------------------------------

app = Flask(_name_)
socketio = SocketIO(app, cors_allowed_origins="*", async_mode='threading')

# runtime state
vx = vy = vz = 0.0
px = py = pz = 0.0
distance_total = 0.0
last_t = None

_acc_lpf = np.array([0.0, 0.0, 0.0])
_acc_mag_buf = deque(maxlen=STATIC_STD_WINDOW)
_gyro_mag_buf = deque(maxlen=STATIC_STD_WINDOW)

_smoothed_lat = None
_smoothed_lon = None
_last_appended_lat = None
_last_appended_lon = None
_last_pan_lat = None
_last_pan_lon = None

vel_bias_x = 0.0
vel_bias_y = 0.0

# gz (yaw) local integration & bias estimate (useful when no fused yaw provided)
yaw_local = 0.0
gz_bias_est = 0.0
gz_bias_iir_alpha = 0.9995  # very slow update while dynamic; faster when static

# baro buffers
baro_buf = deque(maxlen=BARO_BUFFER_LEN)
baro_ref = None
baro_locked = False

static_count = 0

R_earth = 6378137.0

# ---------------- helpers ----------------
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

def safe_float(s, default=float('nan')):
    try:
        return float(s)
    except:
        return default

def haversine_m(lat1, lon1, lat2, lon2):
    if lat1 is None or lon1 is None or lat2 is None or lon2 is None:
        return float('inf')
    R = R_earth
    dlat = math.radians(lat2 - lat1)
    dlon = math.radians(lon2 - lon1)
    a = math.sin(dlat/2.0)*2 + math.cos(math.radians(lat1))*math.cos(math.radians(lat2))*math.sin(dlon/2.0)*2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(max(0.0,1.0-a)))
    return R * c

def heading_to_cardinals(heading_deg):
    if heading_deg is None or math.isnan(heading_deg):
        return (None, None)
    hd = heading_deg % 360.0
    sectors8 = ['N','NE','E','SE','S','SW','W','NW']
    idx8 = int((hd + 22.5) // 45) % 8
    card8 = sectors8[idx8]
    sectors4 = ['N','E','S','W']
    idx4 = int((hd + 45.0) // 90) % 4
    card4 = sectors4[idx4]
    return card8, card4

# ---------------- process CSV row ----------------
def process_csv_row(parts):
    global vx, vy, vz, px, py, pz, distance_total, last_t
    global _acc_lpf, _acc_mag_buf, _gyro_mag_buf, vel_bias_x, vel_bias_y
    global _smoothed_lat, _smoothed_lon, _last_appended_lat, _last_appended_lon, _last_pan_lat, _last_pan_lon
    global yaw_local, gz_bias_est, baro_buf, baro_ref, baro_locked, static_count

    if len(parts) < 10:
        return None

    t_ms = safe_float(parts[0], None)
    if t_ms is None:
        return None

    ax_g = safe_float(parts[1], 0.0); ay_g = safe_float(parts[2], 0.0); az_g = safe_float(parts[3], 0.0)
    gx = safe_float(parts[4], 0.0); gy = safe_float(parts[5], 0.0); gz = safe_float(parts[6], 0.0)
    pitch_deg = safe_float(parts[7], float('nan'))
    roll_deg  = safe_float(parts[8], float('nan'))
    fused_yaw = safe_float(parts[9], float('nan'))

    # pressure & alt may be present
    pressure = float('nan'); alt_m = float('nan')
    if len(parts) >= 12:
        p = safe_float(parts[10], float('nan'))
        if not math.isnan(p) and BARO_VALID_MIN <= p <= BARO_VALID_MAX:
            pressure = p
            baro_buf.append(p)
        alt_m = safe_float(parts[11], float('nan'))

    # time & dt
    t_s = t_ms / 1000.0
    if last_t is None:
        dt = 0.01
    else:
        dt = max(0.001, min(0.1, t_s - last_t))

    # if we have baro buffer and not locked, check stability to lock baseline
    if not baro_locked and len(baro_buf) >= BARO_BUFFER_LEN:
        arr = np.array(baro_buf)
        if float(np.std(arr)) < BARO_STABLE_STD:
            baro_ref = float(np.mean(arr))
            baro_locked = True
            # once locked, compute relative alt baseline by subtracting current alt
            # alt_m provided by Arduino may be absolute using 101325; we'll prefer relative formula:
            # nothing to do here besides locking; later we compute relative alt = current_alt_from_ref
    # compute relative altitude if pressure available and baro_ref locked
    rel_alt_m = float('nan')
    if baro_locked and not math.isnan(pressure):
        rel_alt_m = 44330.0 * (1.0 - (pressure / baro_ref)**0.1903)

    # LPF accel
    raw_acc = np.array([ax_g, ay_g, az_g])
    _acc_lpf = ACC_LPF_ALPHA * _acc_lpf + (1.0 - ACC_LPF_ALPHA) * raw_acc
    ax_f, ay_f, az_f = _acc_lpf.tolist()

    # per-axis deadzone
    def deadzone_for_axis(axis_name, v):
        dz = ACC_DEADZONE_G_BASE
        if axis_name == FORWARD_AXIS:
            dz *= FORWARD_DEADZONE_MULT
        return 0.0 if abs(v) < dz else v

    ax_clean = deadzone_for_axis('x', ax_f)
    ay_clean = deadzone_for_axis('y', ay_f)
    az_clean = deadzone_for_axis('z', az_f)

    # body accel in m/s^2
    a_body = np.array([ax_clean * G, ay_clean * G, az_clean * G])

    # derive pitch/roll if missing
    if math.isnan(pitch_deg) or math.isnan(roll_deg):
        try:
            pitch_deg = math.degrees(math.atan2(-ax_g, math.sqrt(ay_g*ay_g + az_g*az_g)))
            roll_deg  = math.degrees(math.atan2(ay_g, az_g))
        except:
            pitch_deg = 0.0; roll_deg = 0.0

    # yaw handling: prefer fused_yaw if valid; else integrate gz (after bias correction)
    use_local_yaw = False
    if math.isnan(fused_yaw) or abs(fused_yaw) > 1800:
        use_local_yaw = True
    # update gz bias estimator slowly when NOT static, and faster when static_count high
    gyro_mag = math.sqrt(gx*gx + gy*gy + gz*gz)
    _gyro_mag_buf.append(gyro_mag)
    gyro_mean = float(np.mean(list(_gyro_mag_buf))) if len(_gyro_mag_buf) >= 4 else 100.0

    acc_mag = math.sqrt(ax_g*ax_g + ay_g*ay_g + az_g*az_g)
    _acc_mag_buf.append(acc_mag)
    acc_std = float(np.std(list(_acc_mag_buf))) if len(_acc_mag_buf) >= 4 else 1.0

    # static detection (tight): accel magnitude close to 1g + low accel std + low gyro mean
    static_now = (abs(acc_mag - 1.0) < 0.04) and (acc_std < STATIC_STD_THRESHOLD_G) and (gyro_mean < GYRO_STATIC_THRESHOLD_DPS)
    if static_now:
        static_count += 1
    else:
        static_count = 0

    # update gz bias estimate:
    # - when dynamic: very slow IIR towards current gz (acts as long-term bias estimate)
    # - when static_count high: update faster towards measured gz (so we remove transient bias)
    if static_count >= STATIC_COUNT_REQUIRED:
        gz_bias_est = (1.0 - (1.0 - gz_bias_iir_alpha)*5.0) * gz_bias_est + (1.0 - gz_bias_iir_alpha)*5.0 * gz
    else:
        gz_bias_est = gz_bias_iir_alpha * gz_bias_est + (1.0 - gz_bias_iir_alpha) * gz

    gz_corr = gz - gz_bias_est

    if use_local_yaw:
        yaw_local += gz_corr * dt
        # normalize
        if yaw_local > 180.0: yaw_local -= 360.0
        if yaw_local < -180.0: yaw_local += 360.0
        heading = yaw_local
    else:
        heading = fused_yaw

    # compute nav accel
    roll = math.radians(roll_deg); pitch = math.radians(pitch_deg); yaw = math.radians(heading if heading is not None else 0.0)
    Rb2n = R_body_to_nav(roll, pitch, yaw)
    a_nav = Rb2n.dot(a_body) - np.array([0.0, 0.0, G])

    # remove tiny nav accelerations (prevents creeping)
    dead_acc_threshold = ACC_DEADZONE_G_BASE * G
    a_nav[0] = 0.0 if abs(a_nav[0]) < dead_acc_threshold else a_nav[0]
    a_nav[1] = 0.0 if abs(a_nav[1]) < dead_acc_threshold else a_nav[1]

    # integrate velocities (raw)
    vx_raw = vx + a_nav[0] * dt
    vy_raw = vy + a_nav[1] * dt
    vz_raw = vz + a_nav[2] * dt

    # ZUPT / velocity bias learning
    if static_count >= STATIC_COUNT_REQUIRED:
        # strong correction when we've been static for a while
        vel_bias_x = (1.0 - VEL_BIAS_FAST_ALPHA) * vel_bias_x + VEL_BIAS_FAST_ALPHA * vx_raw
        vel_bias_y = (1.0 - VEL_BIAS_FAST_ALPHA) * vel_bias_y + VEL_BIAS_FAST_ALPHA * vy_raw
        vx = 0.0
        vy = 0.0
        vz = 0.0
    else:
        # slow continuous learning away from bias
        vel_bias_x = (1.0 - VEL_BIAS_LEARN_ALPHA) * vel_bias_x + VEL_BIAS_LEARN_ALPHA * vx_raw
        vel_bias_y = (1.0 - VEL_BIAS_LEARN_ALPHA) * vel_bias_y + VEL_BIAS_LEARN_ALPHA * vy_raw
        vx_corr = vx_raw - vel_bias_x
        vy_corr = vy_raw - vel_bias_y
        vx = VEL_SMOOTH_ALPHA * vx + (1.0 - VEL_SMOOTH_ALPHA) * vx_corr
        vy = VEL_SMOOTH_ALPHA * vy + (1.0 - VEL_SMOOTH_ALPHA) * vy_corr
        vz = VEL_SMOOTH_ALPHA * vz + (1.0 - VEL_SMOOTH_ALPHA) * vz_raw

    speed = math.sqrt(vx*vx + vy*vy)
    px += vx * dt
    py += vy * dt
    pz += vz * dt
    distance_total += speed * dt

    # lat/lon conversion + smoothing
    lat_raw, lon_raw = enu_to_latlon(px, py, LAT0, LON0)
    if _smoothed_lat is None:
        _smoothed_lat = lat_raw; _smoothed_lon = lon_raw
    else:
        _smoothed_lat = POS_ALPHA * _smoothed_lat + (1.0 - POS_ALPHA) * lat_raw
        _smoothed_lon = POS_ALPHA * _smoothed_lon + (1.0 - POS_ALPHA) * lon_raw

    add_point = False
    if _last_appended_lat is None:
        add_point = True
    else:
        d = haversine_m(_last_appended_lat, _last_appended_lon, _smoothed_lat, _smoothed_lon)
        if d >= APPEND_THRESHOLD_M:
            add_point = True
    if add_point:
        _last_appended_lat = _smoothed_lat; _last_appended_lon = _smoothed_lon

    do_pan = False
    if _last_pan_lat is None:
        do_pan = True; _last_pan_lat = _smoothed_lat; _last_pan_lon = _smoothed_lon
    else:
        if haversine_m(_last_pan_lat, _last_pan_lon, _smoothed_lat, _smoothed_lon) >= PAN_THRESHOLD_M:
            do_pan = True; _last_pan_lat = _smoothed_lat; _last_pan_lon = _smoothed_lon

    last_t = t_s

    card8, card4 = heading_to_cardinals(heading) if heading is not None else (None, None)

    payload = {
        'lat': float(_smoothed_lat), 'lon': float(_smoothed_lon), 't': float(t_s),
        'px': float(px), 'py': float(py), 'pz': float(pz),
        'vx': float(vx), 'vy': float(vy), 'vz': float(vz),
        'speed': float(speed), 'distance': float(distance_total),
        'heading_deg': float(heading) if heading is not None else None,
        'card8': card8, 'card4': card4,
        'pitch': float(pitch_deg), 'roll': float(roll_deg),
        'vel_bias_x': float(vel_bias_x), 'vel_bias_y': float(vel_bias_y),
        'add': bool(add_point), 'pan': bool(do_pan),
        'is_static': bool(static_count >= STATIC_COUNT_REQUIRED),
        'acc_std_g': float(acc_std), 'gyro_mean_dps': float(gyro_mean),
        'pressure_Pa': float(pressure) if not math.isnan(pressure) else None,
        'alt_m': float(rel_alt_m) if not math.isnan(rel_alt_m) else None
    }
    return payload

# ---------------- serial reader ----------------
def serial_reader_thread():
    try:
        ser = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=1)
    except Exception as e:
        print("Could not open serial:", e)
        return
    print("Serial opened:", SERIAL_PORT, BAUDRATE)

    header_found = False
    while not header_found:
        line = ''
        try:
            line = ser.readline().decode(errors='ignore').strip()
        except Exception:
            pass
        if not line:
            time.sleep(0.01); continue
        if "t_ms" in line and "ax_g" in line:
            header_found = True; print("Header found. Starting...")
            continue
        parts = [p.strip() for p in line.split(",") if p.strip() != '']
        if len(parts) >= 7:
            try:
                float(parts[0]); header_found = True
                print("No header — numeric rows detected. Starting from first numeric row.")
                res = process_csv_row(parts)
                if res: socketio.emit("new_point", res)
                break
            except:
                pass

    while True:
        raw = ''
        try:
            raw = ser.readline().decode(errors='ignore').strip()
        except Exception:
            raw = ''
        if not raw:
            time.sleep(0.01); continue
        parts = [p.strip() for p in raw.split(",") if p.strip()!='']
        if len(parts) < 7:
            continue
        res = process_csv_row(parts)
        if not res:
            continue
        socketio.emit("new_point", res)
        time.sleep(0.0008)

# HTML front-end reused (omitted here for brevity - same as your previous)
INDEX_HTML = """... same frontend as before ..."""

from flask import render_template_string
@app.route("/")
def index():
    return render_template_string(INDEX_HTML, lat0=LAT0, lon0=LON0)

# ---------------- main ----------------
if _name_ == "_main_":
    print("Python:", sys.version)
    socketio.start_background_task(serial_reader_thread)
    socketio.run(app, host="0.0.0.0", port=5000)
