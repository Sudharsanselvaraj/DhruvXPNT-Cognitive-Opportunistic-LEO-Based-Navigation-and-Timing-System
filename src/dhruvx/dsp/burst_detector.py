#!/usr/bin/env python3
# detect_doppler.py
import numpy as np, os, json
from datetime import datetime, timezone
from scipy.signal import windows
from numpy.fft import fftshift, fft

META = "ir_capture.meta.json"
IQFILE = "ir_capture.iq"
OUT_JSON = "bursts.json"

# parameters
SR = None
CENTER = None
CHUNK_S = 0.1          # STFT frame (seconds) for detection
NFFT = 4096

def read_meta():
    with open(META,"r") as f:
        m = json.load(f)
    return m

def load_iq(fname):
    # file contains complex64 interleaved float32
    data = np.fromfile(fname, dtype=np.complex64)
    return data

def detect_bursts(iq, sr, chunk_s=0.05, threshold_db=-30):
    frame_len = int(sr * chunk_s)
    hop = frame_len // 2
    energy = []
    times = []
    for i in range(0, len(iq)-frame_len, hop):
        frame = iq[i:i+frame_len]
        e = 10*np.log10(np.mean(np.abs(frame)**2)+1e-12)
        energy.append(e)
        times.append(i/sr)
    energy = np.array(energy)
    # dynamic threshold: mean + x dB
    thr = np.mean(energy) + 6.0  # tweak
    peaks = np.where(energy > thr)[0]
    bursts = []
    if len(peaks) == 0:
        return bursts
    # group contiguous peaks
    groups = []
    cur = [peaks[0]]
    for p in peaks[1:]:
        if p == cur[-1]+1:
            cur.append(p)
        else:
            groups.append(cur); cur=[p]
    groups.append(cur)
    for g in groups:
        idx = g[len(g)//2]  # center frame index
        t = times[idx]
        # take a 0.1-0.5s slice around t for frequency estimate
        center_sample = int(t*sr)
        half = int(sr*0.08)
        s0 = max(0, center_sample-half)
        s1 = min(len(iq), center_sample+half)
        slice_iq = iq[s0:s1]
        # FFT to find peak
        w = slice_iq * windows.hann(len(slice_iq))
        F = fftshift(fft(w, n=NFFT))
        freqs = np.linspace(-sr/2, sr/2, NFFT)
        mag = 20*np.log10(np.abs(F)+1e-12)
        idxmax = np.argmax(mag)
        f_offset = freqs[idxmax]  # Hz offset from center
        bursts.append({
            "time_s": t,
            "f_offset_hz": float(f_offset),
            "slice_start_sample": int(s0),
            "slice_end_sample": int(s1)
        })
    return bursts

def main():
    meta = read_meta()
    sr = meta["sample_rate"]
    center = meta["center_hz"]
    iq = load_iq(IQFILE)
    bursts = detect_bursts(iq, sr, chunk_s=0.05)
    print("Found bursts:", len(bursts))
    # convert to measured center frequency
    for b in bursts:
        b["measured_hz"] = center + b["f_offset_hz"]
    with open(OUT_JSON,"w") as f:
        json.dump({"meta":meta, "bursts":bursts}, f, indent=2)
    print("Saved", OUT_JSON)

if __name__ == "__main__":
    main()
