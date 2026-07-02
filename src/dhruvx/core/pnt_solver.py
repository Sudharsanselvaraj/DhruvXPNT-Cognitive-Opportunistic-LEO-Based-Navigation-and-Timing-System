#!/usr/bin/env python3
"""
iridium_pnt_from_bin.py
Prototype pipeline:
 - load complex64 IQ .bin
 - compute STFT spectrogram
 - detect bursts and extract frequency ridge
 - compute Doppler measurements (Hz) at times
 - load TLEs for Iridium constellation
 - non-linear least squares solve for receiver position+velocity
Outputs JSON with solution and diagnostics.
"""
import argparse, json, os, sys
from datetime import datetime, timezone, timedelta
import numpy as np
from scipy.signal import stft, medfilt
from scipy.optimize import least_squares
from sgp4.api import Satrec, jday
from tqdm import tqdm
from pyproj import Geod

# ---------- helper utilities ----------
C = 299792458.0

def load_complex64_bin(path):
    """Load raw complex64 interleaved (float32 real, float32 imag)"""
    x = np.fromfile(path, dtype=np.complex64)
    return x

def compute_spectrogram(iq, sr, nperseg=4096, noverlap=None):
    if noverlap is None:
        noverlap = nperseg // 2
    f, t, Z = stft(iq, fs=sr, nperseg=nperseg, noverlap=noverlap, boundary=None)
    # Z: shape (freq_bins, time_slices), complex
    return f, t, Z

def power_db(Z):
    P = 20*np.log10(np.abs(Z) + 1e-12)
    return P

def detect_bursts(P, f, t, power_thresh_db=None, min_width_seconds=0.05):
    """
    Simple burst detector:
      - integrate power across freq to find times with power spikes
      - threshold by median + offset
      - return list of (t_center_idx, time_seconds_indices_range)
    """
    time_power = P.mean(axis=0)  # average across freq
    med = np.median(time_power)
    std = np.std(time_power)
    if power_thresh_db is None:
        power_thresh_db = med + 2.5*std
    mask = time_power > power_thresh_db
    # group contiguous True segments
    bursts = []
    i = 0
    N = len(mask)
    while i < N:
        if not mask[i]:
            i += 1
            continue
        j = i
        while j+1 < N and mask[j+1]:
            j += 1
        dur = t[j] - t[i]
        if dur >= min_width_seconds:
            center_idx = (i+j)//2
            bursts.append((center_idx, i, j))
        i = j+1
    return bursts, time_power, power_thresh_db

def extract_ridge(Z, f, burst_slice, freq_window_hz=6000):
    """
    For a burst given by time indices (i0..i1), compute frequency centroid per time
    and return frequency (Hz) array and timestamps index array.
    """
    i0, i1 = burst_slice
    mags = np.abs(Z[:, i0:i1+1])
    # compute spectral centroid per time slice
    f_hz = f  # in Hz if f originally in Hz
    centroid = (mags * f_hz[:, None]).sum(axis=0) / (mags.sum(axis=0) + 1e-12)
    return centroid  # length i1-i0+1

def load_tles(tle_path):
    sats = []
    if not os.path.exists(tle_path):
        raise FileNotFoundError("TLE file not found: " + tle_path)
    with open(tle_path,'r') as fh:
        lines = [L.rstrip() for L in fh if L.strip()]
    i=0
    while i < len(lines)-2:
        # expecting name, line1, line2
        name = lines[i]
        l1 = lines[i+1]
        l2 = lines[i+2]
        try:
            sat = Satrec.twoline2rv(l1, l2)
            sats.append((name, sat))
        except Exception as e:
            # skip if parse error
            pass
        i += 3
    return sats

def satposvel_from_tle(satrec, dt_utc):
    # returns position (km) and velocity (km/s) ECI (TEME)
    year = dt_utc.year
    month = dt_utc.month
    day = dt_utc.day
    hour = dt_utc.hour
    minute = dt_utc.minute
    sec = dt_utc.second + dt_utc.microsecond*1e-6
    jd, fr = jday(year, month, day, hour, minute, sec)
    e, r, v = satrec.sgp4(jd, fr)
    if e != 0:
        raise RuntimeError("sgp4 propagation error code "+str(e))
    # r and v are km and km/s in TEME
    return np.array(r)*1000.0, np.array(v)*1000.0  # convert to meters

def geocentric_to_geodetic(x,y,z):
    # returns lat, lon, alt (deg, deg, meters) roughly via pyproj if needed
    # We'll use pyproj Geod for distance if needed; for now naive conversion:
    a = 6378137.0
    e2 = 6.69437999014e-3
    lon = np.degrees(np.arctan2(y,x))
    r = np.sqrt(x*x + y*y)
    lat = np.degrees(np.arctan2(z, r*(1-e2)))
    alt = np.sqrt(x*x+y*y+z*z) - a
    return lat, lon, alt

# ---------- model functions for solver ----------
def doppler_model(params, sats_r, sats_v, meas_times, meas_doppler_hz, f0):
    """
    params: [rx,ry,rz, vx,vy,vz] (meters, meters/sec) receiver ECI-like
    sats_r, sats_v: lists of satellite r (m) and v (m/s)
    meas_times: array same length as meas_doppler_hz, but note each measurement is associated with a particular satellite index
    meas_doppler_hz: measured doppler values (Hz)
    f0: nominal RF frequency (Hz)
    This model assumes simple relative line-of-sight range-rate relation:
    doppler = -(f0/c) * range_rate  where range_rate = ( (sat_v - rx_v)·u ) and u=(sat_r - rx_r)/|...|
    """
    rx = np.array(params[0:3])
    rv = np.array(params[3:6])
    preds = []
    for (sat_r, sat_v, t_idx) in sats_r:  # here sats_r is list of tuples containing measurements mapping; adapt below
        r = sat_r - rx
        dist = np.linalg.norm(r)
        u = r/dist
        rr = sat_v - rv
        range_rate = np.dot(rr, u)
        dop = -(f0/C)*range_rate
        preds.append(dop)
    preds = np.array(preds)
    return preds - meas_doppler_hz

# ---------- main pipeline ----------
def main():
    p = argparse.ArgumentParser()
    p.add_argument("--bin", required=True, help="complex64 interleaved IQ binary file")
    p.add_argument("--sr", required=True, type=float, help="sample rate (Hz)")
    p.add_argument("--center", required=True, type=float, help="center frequency (Hz) used when recording")
    p.add_argument("--start-utc", required=True, help="capture start UTC (ISO e.g. 2025-12-07T01:15:00Z)")
    p.add_argument("--tle", required=True, help="TLE file (three-line per sat: name, L1, L2)")
    p.add_argument("--out", default="pnt_result.json", help="output JSON")
    p.add_argument("--nperseg", type=int, default=4096)
    args = p.parse_args()

    # load IQ
    print("Loading", args.bin)
    iq = load_complex64_bin(args.bin)
    nsamps = len(iq)
    duration = nsamps / args.sr
    print(f"Samples: {nsamps:,} Duration(s): {duration:.3f}")

    # parse start time
    start_dt = datetime.fromisoformat(args.start_utc.replace("Z","+00:00")).astimezone(timezone.utc)

    # STFT
    print("Computing spectrogram...")
    f, t, Z = compute_spectrogram(iq, sr=args.sr, nperseg=args.nperseg, noverlap=args.nperseg//2)
    # convert f to Hz (scipy returns Hz already)
    Pdb = power_db(Z)

    # detect bursts
    bursts, time_power, pth = detect_bursts(Pdb, f, t, power_thresh_db=None, min_width_seconds=0.05)
    print("Detected bursts (count):", len(bursts))
    if len(bursts) == 0:
        print("No bursts found - increase sensitivity or check file.")
        return

    # for each burst extract centroid ridge and compute a Doppler measurement
    meas_times = []
    meas_dopplers = []
    meas_sat_index = []  # placeholder: not known yet - we will attempt matching against many sats
    # convert frequency bins (f) are offsets [0..fs/2] if one-sided; but STFT output has negative freqs depending on implementation.
    # SciPy's stft returns freq bins from -fs/2..fs/2 when input complex -> but by default for complex input return full band.
    # We'll assume f is centered around 0 and convert to absolute freq by adding center frequency.
    # check length:
    freq_hz = f  # already in Hz, center at 0 if complex

    for center_idx, i0, i1 in bursts:
        centroid = extract_ridge(Z, freq_hz, (i0, i1))
        # median-smoothed centroid
        cent_smooth = medfilt(centroid, kernel_size=5)
        # choose middle time index for this burst
        mid_rel = len(cent_smooth)//2
        mid_freq = cent_smooth[mid_rel]  # in Hz (absolute freq if f was absolute else offset)
        # Now compute time of mid:
        time_sec = t[i0 + mid_rel]  # seconds since file start
        abs_time = start_dt + timedelta(seconds=float(time_sec))
        # If f is offset around 0, convert to absolute by adding center freq argument:
        # We check typical f range; if mean(f) near 0 then add center
        if np.abs(np.mean(freq_hz)) < 1.0:
            abs_freq = args.center + mid_freq
        else:
            abs_freq = mid_freq
        # Doppler shift relative to center
        dop_hz = abs_freq - args.center
        meas_times.append(abs_time)
        meas_dopplers.append(dop_hz)
    meas_times = np.array(meas_times)
    meas_dopplers = np.array(meas_dopplers)
    print("Measurements (time, doppler Hz):")
    for tt, d in zip(meas_times, meas_dopplers):
        print(tt.isoformat(), f"{d:+.1f} Hz")

    # load TLEs and propagate sats to measurement times
    sats = load_tles(args.tle)
    if len(sats) == 0:
        print("No satellites parsed from TLE file.")
        return
    print("Loaded TLEs:", len(sats))

    # For each measurement, compute satellite r,v for all sats at that exact time and form a candidate measurement list
    # We'll produce candidate equations: for each measurement and each sat produce predicted doppler; then we'll use solver with sat mapping
    # But unknown: which satellite produced the burst. We'll attempt greedy matching: choose sat that gives predicted doppler closest in magnitude.
    sat_names = [s[0] for s in sats]
    sat_objs = [s[1] for s in sats]

    # compute sat r,v for each sat at each meas time
    meas_count = len(meas_times)
    sat_r_all = np.zeros((len(sats), meas_count, 3))
    sat_v_all = np.zeros((len(sats), meas_count, 3))
    for si, sat in enumerate(sat_objs):
        for mi, mt in enumerate(meas_times):
            try:
                r,v = satposvel_from_tle(sat, mt)
                sat_r_all[si,mi,:] = r
                sat_v_all[si,mi,:] = v
            except Exception as e:
                sat_r_all[si,mi,:] = np.nan
                sat_v_all[si,mi,:] = np.nan

    # Now for each measurement choose the best-fitting satellite initial guess by Doppler magnitude comparison assuming rx at Earth center & zero vel (quick heuristic)
    chosen_sat_idx = []
    for mi in range(meas_count):
        best_err = 1e12
        best_si = None
        for si in range(len(sats)):
            r = sat_r_all[si,mi,:]
            v = sat_v_all[si,mi,:]
            if np.any(np.isnan(r)):
                continue
            # approx range-rate assuming receiver at origin and rv=0: range_rate approx dot(v, u) with u = (r-0)/|r|
            u = r / np.linalg.norm(r)
            range_rate = np.dot(v, u)
            pred_dop = -(args.center/C)*range_rate
            err = abs(pred_dop - meas_dopplers[mi])
            if err < best_err:
                best_err = err
                best_si = si
        chosen_sat_idx.append(best_si)

    # Prepare solver arrays: for each measurement use the chosen satellite's r,v at that meas time
    meas_sat_r = []
    meas_sat_v = []
    for mi, si in enumerate(chosen_sat_idx):
        if si is None:
            print("Warning: measurement",mi,"has no sat candidate")
            continue
        meas_sat_r.append((sat_r_all[si,mi,:], sat_v_all[si,mi,:], mi))
        meas_sat_v.append((sat_r_all[si,mi,:], sat_v_all[si,mi,:], mi))
    # Build arrays
    sat_r_for_solver = [ (sat_r_all[si,mi,:], sat_v_all[si,mi,:], mi) for mi,si in enumerate(chosen_sat_idx) if si is not None ]
    meas_dop = meas_dopplers.copy()

    # initial guess: place near Earth surface under center lat/lon (approx)
    # Use simple: guess rx at radius Earth + 0 altitude, and zero velocity
    earth_radius = 6371000.0
    # choose a point at equator facing prime meridian
    x0 = np.array([earth_radius, 0.0, 0.0])
    v0 = np.array([0.0, 0.0, 0.0])
    xguess = np.hstack((x0, v0))

    print("Solving for receiver pos+vel (meters, m/s) — this may be unstable depending on measurements count/quality...")
    def fun(params):
        rx = params[0:3]
        rv = params[3:6]
        preds = []
        for (sat_r, sat_v, mi) in sat_r_for_solver:
            r = sat_r - rx
            dist = np.linalg.norm(r)
            if dist < 1:
                dist = 1.0
            u = r / dist
            rr = sat_v - rv
            range_rate = np.dot(rr, u)
            dop = -(args.center/C)*range_rate
            preds.append(dop)
        preds = np.array(preds)
        return preds - meas_dop

    res = least_squares(fun, xguess, method='lm', max_nfev=2000)
    if not res.success:
        print("Solver failed:", res.message)
    rx = res.x[0:3]
    rv = res.x[3:6]
    # convert rx to lat/lon/alt approximate
    lat, lon, alt = geocentric_to_geodetic(rx[0], rx[1], rx[2])
    print("Solution (approx): lat, lon, alt (deg,deg,m):", lat, lon, alt)
    print("Receiver velocity (m/s):", rv)

    out = {
        "success": bool(res.success),
        "message": res.message,
        "rx_m": rx.tolist(),
        "rv_mps": rv.tolist(),
        "lat_deg": float(lat),
        "lon_deg": float(lon),
        "alt_m": float(alt),
        "chosen_sat_idx": chosen_sat_idx,
        "meas_times_iso": [dt.isoformat() for dt in meas_times],
        "meas_dopplers_hz": meas_dopplers.tolist(),
        "tle_count": len(sats)
    }
    with open(args.out, 'w') as fh:
        json.dump(out, fh, indent=2)
    print("Wrote result to", args.out)
    print("Done.")

if __name__ == "__main__":
    main()
