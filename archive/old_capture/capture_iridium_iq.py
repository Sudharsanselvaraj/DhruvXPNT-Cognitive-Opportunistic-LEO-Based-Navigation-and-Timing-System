#!/usr/bin/env python3
# capture_iridium_iq.py
# Records complex64 IQ from rtlsdr and writes metadata JSON (UTC start time).

import time, json, os
from datetime import datetime, timezone
import numpy as np

try:
    from rtlsdr import RtlSdr
except Exception as e:
    print("rtlsdr python bindings required (pip install pyrtlsdr).")
    raise

OUT = "ir_capture.iq"      # raw complex64 interleaved: float32(real),float32(imag)
META = "ir_capture.meta.json"
CENTER = 1622000000        # center freq in Hz
SR = 2_400_000.0           # sample rate
DURATION = 30.0            # seconds to capture
GAIN = 48.0                # dB
CHUNK = int(SR * 0.5)      # 0.5s chunks

def main():
    sdr = RtlSdr()
    sdr.sample_rate = SR
    sdr.center_freq = CENTER
    sdr.gain = GAIN
    # some drivers allow bias tee via sdr.set_biastee(1) — if available
    print("Start recording... make sure system time (UTC) is correct")
    start_utc = datetime.now(timezone.utc).isoformat()
    total_samples = int(SR * DURATION)
    remaining = total_samples
    with open(OUT, "wb") as f:
        while remaining > 0:
            toread = min(CHUNK, remaining)
            data = sdr.read_samples(toread)   # returns complex64
            # ensure complex64
            arr = np.array(data, dtype=np.complex64)
            arr.tofile(f)
            remaining -= len(arr)
            print(f"written {len(arr)} samples, remaining {remaining}")
    sdr.close()
    meta = {
        "filename": OUT,
        "start_utc": start_utc,
        "center_hz": CENTER,
        "sample_rate": SR,
        "duration_s": DURATION,
        "gain_db": GAIN
    }
    with open(META,"w") as mf:
        json.dump(meta, mf, indent=2)
    print("Finished. Files:", OUT, META)

if __name__ == "__main__":
    main()
