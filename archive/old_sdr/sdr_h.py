#!/usr/bin/env python3
"""
dashboard_from_json.py

Usage:
    python3 dashboard_from_json.py /path/to/capture_meta.json

Reads the JSON produced by your capture pipeline and displays a health dashboard
on the Raspberry Pi display (Tkinter). Auto-reloads when JSON mod time changes.
"""

import sys
import json
import os
import time
import math
import tkinter as tk
from tkinter import font
from datetime import datetime, timezone
from pathlib import Path
from PIL import ImageTk, Image  # pillow used only for nicer icon handling (optional)

REFRESH_MS = 1000  # UI refresh interval

# Thresholds (tune to your setup)
THRESH_RF_EXCELLENT = -35.0
THRESH_RF_GOOD = -50.0
THRESH_SNR_EXCELLENT = 20.0
THRESH_SNR_WARNING = 10.0
THRESH_CLOCK_DRIFT_MS_WARN = 1.0
THRESH_CLOCK_DRIFT_MS_FAIL = 10.0
THRESH_BURSTS_PER_SEC_WARN = 1.0  # bursts per second
THRESH_BURST_DURATION_MS_TOLERANCE = 5.0  # expected ~20 ms ± tol
THRESH_FREQ_VARIATION_HZ = 10000.0  # 10 kHz variation considered large

# Utility helpers
def safe_get(d, key, default=None):
    return d.get(key, default) if isinstance(d, dict) else default

def severity_color(sev):
    return {"OK":"#2ECC71","WARN":"#F1C40F","FAIL":"#E74C3C"}.get(sev, "#95A5A6")

def format_time(ts):
    try:
        return datetime.fromtimestamp(ts, tz=timezone.utc).isoformat()
    except:
        return str(ts)

# Health logic using only the JSON fields you provided
class HealthEvaluator:
    def __init__(self, j):
        self.j = j or {}
        self.events = []

    def evaluate(self):
        self.events.clear()
        j = self.j

        # Basic fields
        avg_power_db = safe_get(j, "avg_power_db", None)
        num_bursts = safe_get(j, "num_bursts_detected", 0)
        max_snr = safe_get(j, "max_burst_snr_db", None)
        sample_rate = safe_get(j, "sample_rate_hz", None)
        num_samples = safe_get(j, "num_samples", None)
        duration_seconds = safe_get(j, "duration_seconds", None)
        bursts = safe_get(j, "bursts", []) or []

        # RF Power health
        rf_health = "UNKNOWN"
        if avg_power_db is None:
            rf_health = "UNKNOWN"
        elif avg_power_db > THRESH_RF_EXCELLENT:
            rf_health = "OK"
        elif avg_power_db > THRESH_RF_GOOD:
            rf_health = "WARN"
            self.events.append(("RF_POWER_LOW", "WARN", f"avg_power_db={avg_power_db:.1f} dB"))
        else:
            rf_health = "FAIL"
            self.events.append(("RF_POWER_CRIT", "FAIL", f"avg_power_db={avg_power_db:.1f} dB"))

        # Burst health
        bursts_per_sec = (num_bursts / duration_seconds) if duration_seconds and duration_seconds>0 else 0
        burst_health = "OK"
        if bursts_per_sec < THRESH_BURSTS_PER_SEC_WARN:
            burst_health = "WARN"
            self.events.append(("BURST_RATE_LOW","WARN",f"bursts_per_sec={bursts_per_sec:.2f}"))
        if max_snr is None:
            pass
        elif max_snr < THRESH_SNR_WARNING:
            burst_health = "FAIL"
            self.events.append(("SNR_ALL_LOW","FAIL",f"max_snr={max_snr:.1f} dB"))
        elif max_snr < THRESH_SNR_EXCELLENT:
            if burst_health != "FAIL":
                burst_health = "WARN"

        # Clock drift calculation (using your fields)
        clock_health = "OK"
        clock_drift_ms = None
        if num_samples and sample_rate and duration_seconds:
            expected_duration = float(num_samples) / float(sample_rate)
            drift_sec = duration_seconds - expected_duration
            clock_drift_ms = drift_sec * 1000.0
            if abs(clock_drift_ms) > THRESH_CLOCK_DRIFT_MS_FAIL:
                clock_health = "FAIL"
                self.events.append(("CLOCK_DRIFT_CRIT","FAIL",f"drift_ms={clock_drift_ms:.3f}"))
            elif abs(clock_drift_ms) > THRESH_CLOCK_DRIFT_MS_WARN:
                clock_health = "WARN"
                self.events.append(("CLOCK_DRIFT_WARN","WARN",f"drift_ms={clock_drift_ms:.3f}"))

        # Burst duration check (Iridium ~20 ms)
        duration_health = "OK"
        durs = []
        for b in bursts:
            d = safe_get(b,"duration_ms", None)
            if d is not None:
                durs.append(d)
        if durs:
            mean_dur = sum(durs)/len(durs)
            if abs(mean_dur - 20.0) > THRESH_BURST_DURATION_MS_TOLERANCE:
                duration_health = "WARN"
                self.events.append(("BURST_DURATION_WEIRD","WARN",f"mean_dur_ms={mean_dur:.2f}"))
        else:
            mean_dur = None

        # Carrier stability: freq_center variation
        freq_centers = [safe_get(b,"freq_center_hz", None) for b in bursts if safe_get(b,"freq_center_hz", None) is not None]
        freq_health = "OK"
        if len(freq_centers) >= 2:
            fc_min = min(freq_centers); fc_max = max(freq_centers)
            if (fc_max - fc_min) > THRESH_FREQ_VARIATION_HZ:
                freq_health = "WARN"
                self.events.append(("CARRIER_VARIATION","WARN",f"range_hz={fc_max-fc_min:.1f}"))
        freq_range = (min(freq_centers), max(freq_centers)) if freq_centers else (None,None)

        # LNA inference: simple on/off guess
        lna_health = "UNKNOWN"
        # If max_snr exists and avg_power is low, suspect LNA off
        if max_snr is not None and avg_power_db is not None:
            if max_snr < 5 and avg_power_db < -60:
                lna_health = "FAIL"
                self.events.append(("LNA_SUSPECT","WARN","Low power and low SNR"))
            else:
                lna_health = "OK"

        # File integrity
        file_health = "OK"
        json_fname = safe_get(self.j,"files",{}).get("json")
        npy_fname = safe_get(self.j,"files",{}).get("npy")
        # if paths present, check existence
        if npy_fname:
            if not Path(npy_fname).exists():
                file_health = "WARN"
                self.events.append(("MISSING_IQ","WARN",f"{npy_fname} not found"))

        # Compose summary severity
        # Priority: any FAIL -> FAIL; else any WARN -> WARN; else OK
        levels = [rf_health, burst_health, clock_health, duration_health, freq_health, lna_health, file_health]
        if "FAIL" in levels:
            overall = "FAIL"
        elif "WARN" in levels:
            overall = "WARN"
        elif all(x=="OK" or x=="UNKNOWN" for x in levels):
            overall = "OK"
        else:
            overall = "UNKNOWN"

        # Build a dict to return
        result = {
            "overall": overall,
            "rf": {"value_db": avg_power_db, "status": rf_health},
            "bursts": {"count": num_bursts, "per_sec": bursts_per_sec, "status": burst_health, "max_snr_db": max_snr},
            "clock": {"drift_ms": clock_drift_ms, "status": clock_health},
            "burst_duration": {"mean_ms": mean_dur, "status": duration_health},
            "carrier": {"freq_range": freq_range, "status": freq_health},
            "lna": {"status": lna_health},
            "files": {"json": json_fname, "npy": npy_fname, "status": file_health},
            "events": list(self.events)
        }
        return result

# GUI
class JsonDashboard(tk.Tk):
    def __init__(self, json_path):
        super().__init__()
        self.json_path = Path(json_path)
        self.title("DhruvXPNT - Health Dashboard")
        self.configure(bg="#0b0b0b")
        # fullscreen on Pi
        try:
            self.attributes("-fullscreen", True)
        except:
            pass

        # big fonts
        self.large_font = font.Font(family="Helvetica", size=22, weight="bold")
        self.mid_font = font.Font(family="Helvetica", size=14)
        self.small_font = font.Font(family="Helvetica", size=11)

        # header
        self.header = tk.Label(self, text="DhruvXPNT — Capture Health", bg="#0b0b0b", fg="white", font=self.large_font)
        self.header.pack(pady=(8,4))

        # top summary
        self.summary_frame = tk.Frame(self, bg="#111111")
        self.summary_frame.pack(fill="x", padx=10, pady=5)

        self.overall_label = tk.Label(self.summary_frame, text="OVERALL: --", bg="#111111", fg="white", font=self.large_font)
        self.overall_label.pack(side="left", padx=12, pady=8)

        # right: timestamp and file
        self.info_label = tk.Label(self.summary_frame, text="JSON: --", bg="#111111", fg="white", font=self.mid_font)
        self.info_label.pack(side="right", padx=12)

        # main content frames
        self.left = tk.Frame(self, bg="#0b0b0b")
        self.left.pack(side="left", fill="y", padx=10, pady=6)

        self.center = tk.Frame(self, bg="#0b0b0b")
        self.center.pack(side="left", fill="both", expand=True, padx=6, pady=6)

        self.right = tk.Frame(self, bg="#0b0b0b")
        self.right.pack(side="right", fill="y", padx=10, pady=6)

        # LEFT widgets (RF, Bursts, Clock)
        self.rf_label = tk.Label(self.left, text="RF Power: -- dB", font=self.mid_font, bg="#0b0b0b", fg="white")
        self.rf_label.pack(anchor="w", pady=6)

        self.bursts_label = tk.Label(self.left, text="Bursts: --", font=self.mid_font, bg="#0b0b0b", fg="white")
        self.bursts_label.pack(anchor="w", pady=6)

        self.snr_label = tk.Label(self.left, text="Max SNR: -- dB", font=self.mid_font, bg="#0b0b0b", fg="white")
        self.snr_label.pack(anchor="w", pady=6)

        self.clock_label = tk.Label(self.left, text="Clock drift: -- ms", font=self.mid_font, bg="#0b0b0b", fg="white")
        self.clock_label.pack(anchor="w", pady=6)

        self.lna_label = tk.Label(self.left, text="LNA: --", font=self.mid_font, bg="#0b0b0b", fg="white")
        self.lna_label.pack(anchor="w", pady=6)

        # CENTER widgets (Burst table + carrier info)
        self.center_top = tk.Frame(self.center, bg="#111111")
        self.center_top.pack(fill="x", pady=4)
        self.meta_text = tk.Text(self.center_top, height=8, bg="#111111", fg="white", font=self.small_font)
        self.meta_text.pack(fill="x", padx=4, pady=4)

        self.center_bottom = tk.Frame(self.center, bg="#111111")
        self.center_bottom.pack(fill="both", expand=True, pady=2)
        self.carrier_label = tk.Label(self.center_bottom, text="Carrier Range: --", font=self.mid_font, bg="#111111", fg="white")
        self.carrier_label.pack(anchor="w", pady=6)
        self.duration_label = tk.Label(self.center_bottom, text="Burst mean duration: -- ms", font=self.mid_font, bg="#111111", fg="white")
        self.duration_label.pack(anchor="w", pady=6)

        # RIGHT widgets (events log)
        self.event_title = tk.Label(self.right, text="Events (most recent)", font=self.mid_font, bg="#0b0b0b", fg="white")
        self.event_title.pack(anchor="w")
        self.event_box = tk.Text(self.right, width=40, height=20, bg="#111111", fg="white", font=self.small_font)
        self.event_box.pack(padx=4, pady=6)

        # store last mod time
        self._last_mtime = 0
        self._last_read = None

        # start periodic update
        self.after(200, self._periodic_update)

    def _read_json(self):
        p = self.json_path
        if not p.exists():
            return None
        try:
            mtime = p.stat().st_mtime
            if mtime == self._last_mtime and self._last_read is not None:
                return self._last_read
            with open(p, "r") as f:
                j = json.load(f)
            self._last_mtime = mtime
            self._last_read = j
            return j
        except Exception as e:
            print("JSON read error:", e)
            return None

    def _periodic_update(self):
        j = self._read_json()
        if j is not None:
            self._update_from_json(j)
        # schedule next
        self.after(REFRESH_MS, self._periodic_update)

    def _update_from_json(self, j):
        # show meta header
        ts = safe_get(j, "timestamp_utc", None) or safe_get(j, "timestamp_unix", None)
        if isinstance(ts, (int,float)):
            try:
                ts = datetime.fromtimestamp(float(ts), tz=timezone.utc).isoformat()
            except:
                ts = str(ts)
        self.info_label.config(text=f"JSON: {self.json_path.name} | {ts}")

        # evaluate health
        he = HealthEvaluator(j)
        report = he.evaluate()

        # overall banner
        overall = report["overall"]
        color = severity_color("OK" if overall=="OK" else ("WARN" if overall=="WARN" else "FAIL"))
        self.overall_label.config(text=f"OVERALL: {overall}", bg=color)

        # left fields
        rf_val = report["rf"]["value_db"]
        self.rf_label.config(text=f"RF Power: {rf_val if rf_val is not None else '--'} dB")
        bursts = report["bursts"]
        self.bursts_label.config(text=f"Bursts: {bursts['count']}  ({bursts['per_sec']:.2f}/s)")
        self.snr_label.config(text=f"Max SNR: {bursts.get('max_snr_db', '--') if bursts.get('max_snr_db', None) is not None else '--'} dB")
        ck = report["clock"]
        self.clock_label.config(text=f"Clock drift: {ck['drift_ms']:.3f} ms" if ck["drift_ms"] is not None else "Clock drift: --")
        self.lna_label.config(text=f"LNA: {report['lna']['status']}")

        # center meta text - show json pretty a few keys
        meta_lines = []
        meta_lines.append(f"Center freq (MHz): {safe_get(j,'center_frequency_mhz','--')}")
        meta_lines.append(f"Sample rate (MHz): {safe_get(j,'sample_rate_mhz','--')}")
        meta_lines.append(f"Duration (s): {safe_get(j,'duration_seconds','--')}")
        meta_lines.append(f"Num samples: {safe_get(j,'num_samples','--')}")
        meta_lines.append(f"Avg power (dB): {safe_get(j,'avg_power_db','--')}")
        meta_lines.append(f"Num bursts: {safe_get(j,'num_bursts_detected','--')}")
        self.meta_text.delete("1.0", tk.END)
        self.meta_text.insert(tk.END, "\n".join(meta_lines))

        # carrier / duration
        fcmin, fcmax = report["carrier"]["freq_range"]
        if fcmin is None:
            self.carrier_label.config(text="Carrier Range: --")
        else:
            self.carrier_label.config(text=f"Carrier Range: {fcmin:.1f} Hz — {fcmax:.1f} Hz")
        bd = report["burst_duration"]["mean_ms"]
        self.duration_label.config(text=f"Burst mean duration: {bd:.2f} ms" if bd else "Burst mean duration: -- ms")

        # events log
        self.event_box.delete("1.0", tk.END)
        for ev in report["events"][-40:]:
            code, sev, msg = ev
            tstr = datetime.now().strftime("%H:%M:%S")
            self.event_box.insert(tk.END, f"[{tstr}] {sev} {code}: {msg}\n")

        # store last json for debugging if needed
        self._last_json = j

def main():
    if len(sys.argv) < 2:
        print("Usage: python3 dashboard_from_json.py /path/to/capture_meta.json")
        sys.exit(1)
    json_path = sys.argv[1]
    if not Path(json_path).exists():
        print("JSON file not found:", json_path)
        sys.exit(1)

    app = JsonDashboard(json_path)
    app.mainloop()

if __name__ == "__main__":
    main()
