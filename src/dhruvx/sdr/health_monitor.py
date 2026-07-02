#!/usr/bin/env python3
"""
sdr_healthv3.py

Unified RTL-SDR real-time health monitor with:
 - Dark "tech terminal" Tkinter GUI + Matplotlib (PSD + waterfall + time-series)
 - Auto-calibration for antenna thresholds
 - Robust heuristics to detect antenna disconnected and LNA problems
 - Curses terminal fallback for headless SSH
 - Logging to ~/sdr_healthv3.log

Run inside your project's venv:
    source venv/bin/activate
    python sdr_healthv3.py

Dependencies (venv):
    pip install pyrtlsdr numpy matplotlib

If GUI fails to start due to missing system Tk:
    sudo apt update
    sudo apt install python3-tk

Author: ChatGPT (GPT-5 Thinking mini)
"""
import os
import sys
import time
import math
import threading
import logging
from collections import deque
import traceback

# Configure logging
LOGFILE = os.path.expanduser("~/sdr_healthv3.log")
logging.basicConfig(
    filename=LOGFILE,
    level=logging.INFO,
    format="%(asctime)s %(levelname)s: %(message)s"
)
console = logging.StreamHandler()
console.setLevel(logging.INFO)
fmt = logging.Formatter("%(levelname)s: %(message)s")
console.setFormatter(fmt)
logging.getLogger().addHandler(console)

# Try importing SDR library and GUI libs
try:
    from rtlsdr import RtlSdr
except Exception as e:
    RtlSdr = None
    SDR_IMPORT_ERR = e
    logging.warning("rtlsdr import issue: %s", e)

try:
    import numpy as np
except Exception as e:
    logging.exception("numpy import failed")
    raise

# GUI libs (optional)
TKINTER_OK = True
try:
    import tkinter as tk
    from tkinter import ttk, messagebox
    from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
    from matplotlib.figure import Figure
    import matplotlib
except Exception as e:
    TKINTER_OK = False
    logging.info("Tkinter / Matplotlib GUI not usable: %s", e)

# curses fallback
try:
    import curses
    CURSES_OK = True
except Exception:
    CURSES_OK = False

# -------------------- User-configurable parameters --------------------
CENTER_FREQ = 1620e6         # Hz (Iridium band example)
SAMPLE_RATE = 2.4e6          # Hz
READ_SAMPLES = 128 * 1024    # reduce to 64k/32k on slow Pis
UPDATE_SEC = 1.0             # GUI / status update cadence (sec)
NFFT = 4096                  # FFT size for PSD/waterfall
PSD_AVG_BLOCKS = 2           # smoothing
WATERFALL_HISTORY = 120      # rows
HISTORY_LEN = 60             # timeseries length
CONSECUTIVE_FAILS_TO_OFFLINE = 4
# thresholds (will be tuned with auto-calibrate)
ABS_PEAK_DB_THRESHOLD = -65.0   # absolute-like threshold (amplitude-based) to mark disconnected
MIN_SNR_DB = 6.0                # minimum SNR to claim signal present
# ----------------------------------------------------------------------

# ------------------------- DSP helpers --------------------------------
def compute_psd_db(iq, nfft=NFFT, avg_blocks=PSD_AVG_BLOCKS):
    """Return PSD (relative dB) and freqs (Hz offset). If not enough samples returns (None,None)."""
    if iq is None or iq.size < nfft:
        return None, None
    iq = np.asarray(iq, dtype=np.complex64)
    accum = None
    start = 0
    count = 0
    while start + nfft <= iq.size and count < avg_blocks:
        seg = iq[start:start+nfft]
        win = np.hanning(nfft)
        spec = np.fft.fftshift(np.fft.fft(seg * win, n=nfft))
        ps = (np.abs(spec)**2) / (np.sum(win**2) + 1e-20)
        accum = ps if accum is None else accum + ps
        start += nfft
        count += 1
    if accum is None:
        return None, None
    psd = accum / max(1, count)
    with np.errstate(divide='ignore'):
        psd_db = 10.0 * np.log10(psd + 1e-20)
    psd_db = psd_db - np.max(psd_db)   # normalize so strongest bin ~ 0 dB
    freqs = np.linspace(-SAMPLE_RATE/2, SAMPLE_RATE/2, len(psd_db))
    return psd_db, freqs

def amplitude_peak_db(iq):
    """Return an amplitude-based peak (20*log10(|iq|)) max value (approx absolute)."""
    if iq is None or iq.size == 0:
        return None
    mag = np.abs(iq)
    with np.errstate(divide='ignore'):
        mag_db = 20.0 * np.log10(np.maximum(mag, 1e-12))
    return float(np.max(mag_db)), float(np.median(mag_db))

# ------------------------- SDR probe thread ----------------------------
class SDRProbe(threading.Thread):
    def __init__(self):
        super().__init__(daemon=True)
        self.sdr = None
        self.lock = threading.Lock()
        self.running = True
        # metrics
        self.psd = None
        self.freqs = None
        self.noise_rel = None
        self.peak_rel = None
        self.snr = None
        self.abs_peak_db = None
        self.abs_median_db = None
        self.gain = None
        self.fail_count = 0
        self.last_error = None
        self.waterfall = deque(maxlen=WATERFALL_HISTORY)
        # calibration thresholds
        self.abs_peak_threshold = ABS_PEAK_DB_THRESHOLD
        self.min_snr_db = MIN_SNR_DB

    def open_sdr(self):
        if RtlSdr is None:
            raise RuntimeError(f"pyrtlsdr not available: {SDR_IMPORT_ERR}")
        s = RtlSdr()
        s.sample_rate = SAMPLE_RATE
        s.center_freq = CENTER_FREQ
        # try auto gain; some versions accept string
        try:
            s.gain = 'auto'
        except Exception:
            pass
        return s

    def close_sdr(self):
        if self.sdr is not None:
            try:
                self.sdr.close()
            except Exception:
                pass
            self.sdr = None

    def run_once(self):
        try:
            if self.sdr is None:
                self.sdr = self.open_sdr()
            block = self.sdr.read_samples(READ_SAMPLES)
            psd_db, freqs = compute_psd_db(block, nfft=NFFT, avg_blocks=PSD_AVG_BLOCKS)
            abs_peak, abs_med = amplitude_peak_db(block)
            if psd_db is not None:
                noise = float(np.median(psd_db))
                peak = float(np.max(psd_db))
                snr = float(peak - noise)
            else:
                noise = peak = snr = None
            with self.lock:
                self.psd = psd_db
                self.freqs = freqs
                self.noise_rel = noise
                self.peak_rel = peak
                self.snr = snr
                self.abs_peak_db = abs_peak
                self.abs_median_db = abs_med
                try:
                    self.gain = float(self.sdr.gain)
                except Exception:
                    self.gain = None
                self.last_error = None
                self.fail_count = 0
                # waterfall row scaled 0..1 for plotting
                if psd_db is not None:
                    row = (psd_db - np.min(psd_db)) / (np.max(psd_db) - np.min(psd_db) + 1e-12)
                else:
                    row = None
                self.waterfall.appendleft(row)
        except Exception as e:
            logging.exception("SDR read error")
            with self.lock:
                self.last_error = str(e)
                self.fail_count += 1
            # close device to force reopen on next loop
            self.close_sdr()

    def run(self):
        while self.running:
            self.run_once()
            time.sleep(0.01)   # small sleep; GUI determines visible update cadence

    # heuristics for antenna/LNA health
    def get_health(self):
        with self.lock:
            noise = self.noise_rel
            peak = self.peak_rel
            snr = self.snr
            abs_peak = self.abs_peak_db
            abs_med = self.abs_median_db
            failc = self.fail_count
            gain = self.gain
            last_err = self.last_error
        # default
        status = {"connection": "UNKNOWN", "antenna": "UNKNOWN", "lna": "UNKNOWN", "reason": None}
        if failc >= CONSECUTIVE_FAILS_TO_OFFLINE:
            status["connection"] = "OFFLINE"
            status["reason"] = f"consecutive read failures: {failc}"
            return status
        status["connection"] = "ONLINE"
        # antenna disconnect detection: combine relative PSD and amplitude-based absolute
        if abs_peak is None:
            status["antenna"] = "NO_DATA"
        else:
            # absolute amplitude low -> disconnect
            if abs_peak < self.abs_peak_threshold:
                status["antenna"] = "DISCONNECTED"
                status["reason"] = f"abs_peak {abs_peak:.1f} dB < threshold {self.abs_peak_threshold:.1f}"
            elif snr is None:
                status["antenna"] = "UNKNOWN"
            elif snr < self.min_snr_db:
                status["antenna"] = "CONNECTED_NO_STRONG_SIGNAL"
                status["reason"] = f"SNR {snr:.1f} dB < min {self.min_snr_db:.1f}"
            else:
                status["antenna"] = "CONNECTED_SIGNAL_OK"
        # LNA heuristic using gain and noise floor
        if gain is None:
            status["lna"] = "UNKNOWN"
        else:
            if gain > 35 and (noise is not None and noise < -60):
                status["lna"] = "LNA_OK"
            elif gain > 40 and (noise is not None and noise > -45):
                status["lna"] = "LNA_SUSPICIOUS"
                status["reason"] = (status.get("reason","") + "; high gain but high noise").strip('; ')
            else:
                status["lna"] = "UNKNOWN"
        return status

    # calibration routine: user provides two samples: antenna disconnected then connected
    def calibrate(self, samples_no_ant, samples_with_ant):
        """Compute thresholds from two sample blocks (numpy arrays of IQ)."""
        try:
            ap_no, _ = amplitude_peak_db(samples_no_ant)
            ap_yes, _ = amplitude_peak_db(samples_with_ant)
            if ap_no is None or ap_yes is None:
                return False, "insufficient samples"
            # set threshold halfway between the two absolute peaks (but clamp)
            new_threshold = (ap_no + ap_yes) / 2.0
            # be a bit conservative: take 70% towards the higher (connected) reading
            new_threshold = ap_no + 0.7 * (ap_yes - ap_no)
            with self.lock:
                self.abs_peak_threshold = new_threshold
            logging.info("Calibration set abs_peak_threshold = %.2f dB (no_ant=%.2f, with_ant=%.2f)",
                         new_threshold, ap_no, ap_yes)
            return True, f"abs_peak_threshold={new_threshold:.2f} dB"
        except Exception as e:
            logging.exception("Calibration failed")
            return False, str(e)

# ------------------------ GUI Implementation --------------------------
def start_gui(probe: SDRProbe):
    """Start Tkinter GUI; returns when window closed."""
    if not TKINTER_OK:
        raise RuntimeError("Tkinter / Matplotlib not available for GUI")

    root = tk.Tk()
    root.title("RTL-SDR Tech Terminal")
    root.configure(bg='#0b0f12')
    root.geometry("1200x760")
    # style
    style = ttk.Style(root)
    try:
        style.theme_use('clam')
    except Exception:
        pass
    style.configure('TLabel', background='#0b0f12', foreground='#cfefff', font=('Consolas', 11))

    # Left info panel
    left = ttk.Frame(root, width=480)
    left.pack(side='left', fill='y', padx=8, pady=8)

    title = ttk.Label(left, text="RTL-SDR TECH TERMINAL", font=('Consolas', 14, 'bold'), foreground='#8be9fd', background='#0b0f12')
    title.pack(anchor='w', pady=(0,8))

    info_text = tk.Text(left, width=58, height=22, bg='#041018', fg='#cfefff', insertbackground='#cfefff',
                        font=('Consolas', 10), bd=0)
    info_text.pack(anchor='nw')
    info_text.insert('end', "Starting probe...\n")
    info_text.configure(state='disabled')

    # badges
    badges_frame = ttk.Frame(left)
    badges_frame.pack(anchor='w', pady=(6,0))
    conn_label = tk.Label(badges_frame, text="OFFLINE", bg='#e74c3c', fg='#fff', font=('Consolas',10,'bold'))
    conn_label.pack(side='left', padx=(0,8))
    ant_label = tk.Label(badges_frame, text="ANT: UNKNOWN", bg='#f39c12', fg='#000', font=('Consolas',10,'bold'))
    ant_label.pack(side='left', padx=(0,8))
    lna_label = tk.Label(badges_frame, text="LNA: ?", bg='#f39c12', fg='#000', font=('Consolas',10,'bold'))
    lna_label.pack(side='left', padx=(0,8))

    # calibrate controls
    cal_frame = ttk.Frame(left)
    cal_frame.pack(anchor='w', pady=(12,0))
    ttk.Label(cal_frame, text="Calibration:", font=('Consolas',11)).grid(row=0, column=0, sticky='w')
    cal_btn = ttk.Button(cal_frame, text="Run Auto-Calibration", width=24)
    cal_btn.grid(row=1, column=0, pady=(6,0))
    cal_status = tk.StringVar(value="Not calibrated")
    ttk.Label(cal_frame, textvariable=cal_status).grid(row=2, column=0, sticky='w', pady=(6,0))

    # Right plotting area
    right = ttk.Frame(root)
    right.pack(side='right', fill='both', expand=True, padx=8, pady=8)

    # Matplotlib figure with PSD and waterfall
    from matplotlib.figure import Figure
    fig = Figure(figsize=(8,6), dpi=100)
    ax_psd = fig.add_subplot(211)
    ax_psd.set_facecolor('#0b0f12')
    ax_psd.title.set_color('#8be9fd')
    ax_psd.tick_params(colors='#9fb7c9')
    line_psd, = ax_psd.plot([], [], linewidth=1.2)

    ax_wf = fig.add_subplot(212)
    ax_wf.set_facecolor('#0b0f12')

    canvas = FigureCanvasTkAgg(fig, master=right)
    canvas_widget = canvas.get_tk_widget()
    canvas_widget.pack(fill='both', expand=True)

    # Bottom timeseries small labels
    bottom = ttk.Frame(right)
    bottom.pack(fill='x', pady=(6,0))
    noise_var = tk.StringVar(value="Noise: -")
    peak_var = tk.StringVar(value="Peak: -")
    snr_var = tk.StringVar(value="SNR: -")
    gain_var = tk.StringVar(value="Gain: -")

    ttk.Label(bottom, textvariable=noise_var).pack(side='left', padx=8)
    ttk.Label(bottom, textvariable=peak_var).pack(side='left', padx=8)
    ttk.Label(bottom, textvariable=snr_var).pack(side='left', padx=8)
    ttk.Label(bottom, textvariable=gain_var).pack(side='left', padx=8)

    # tiny sparkline placeholders (optional)
    noise_hist = deque(maxlen=HISTORY_LEN)
    peak_hist = deque(maxlen=HISTORY_LEN)
    snr_hist = deque(maxlen=HISTORY_LEN)

    # prepare initial waterfall image
    wf_img = None

    def update_ui():
        nonlocal wf_img
        try:
            with probe.lock:
                psd = probe.psd
                freqs = probe.freqs
                noise = probe.noise_rel
                peak = probe.peak_rel
                snr = probe.snr
                abs_peak = probe.abs_peak_db
                abs_med = probe.abs_median_db
                gain = probe.gain
                failc = probe.fail_count
                last_err = probe.last_error
                wf_list = list(probe.waterfall)
                abs_threshold = probe.abs_peak_threshold
                min_snr_db = probe.min_snr_db

            # update left info box
            lines = []
            lines.append(f"Local time: {time.strftime('%Y-%m-%d %H:%M:%S')}")
            lines.append(f"Center freq: {CENTER_FREQ/1e6:.3f} MHz")
            lines.append(f"Sample rate: {SAMPLE_RATE/1e6:.3f} MS/s")
            lines.append(f"Read samples: {READ_SAMPLES}")
            lines.append("")
            if last_err:
                lines.append("Last error:")
                lines.append(f"  {last_err}")
            else:
                lines.append("Device read: OK" if failc==0 else f"Device read: intermittent (fails={failc})")
            lines.append("")
            lines.append(f"Gain: {gain if gain is not None else 'auto/unknown'}")
            lines.append(f"Noise (rel dB): {noise:.2f}" if noise is not None else "Noise (rel dB): -")
            lines.append(f"Peak (rel dB): {peak:.2f}" if peak is not None else "Peak (rel dB): -")
            lines.append(f"Est SNR (dB): {snr:.2f}" if snr is not None else "Est SNR (dB): -")
            lines.append(f"Abs peak (20log10|IQ|): {abs_peak:.2f}" if abs_peak is not None else "Abs peak: -")
            lines.append(f"Abs median (20log10|IQ|): {abs_med:.2f}" if abs_med is not None else "Abs median: -")
            lines.append("")
            lines.append(f"Abs peak threshold: {abs_threshold:.2f} dB")
            lines.append(f"Min SNR threshold: {min_snr_db:.2f} dB")
            # kernel driver note (best-effort)
            try:
                import subprocess
                lsmod = subprocess.check_output(['lsmod'], text=True)
                drv_found = any(m in lsmod for m in ('dvb_usb_rtl28xxu','rtl2832','rtl2830','dvb_usb_rtl2832u'))
                lines.append("Kernel driver hint: present" if drv_found else "Kernel driver hint: not present")
            except Exception:
                pass

            info_text.configure(state='normal')
            info_text.delete('1.0', 'end')
            info_text.insert('end', "\n".join(lines) + "\n")
            info_text.configure(state='disabled')

            # badges
            health = probe.get_health()
            if health["connection"] == "OFFLINE":
                conn_label.config(text="OFFLINE", bg='#e74c3c')
            else:
                conn_label.config(text="ONLINE", bg='#2ecc71')
            if health["antenna"].startswith("DISCONNECTED"):
                ant_label.config(text="ANT: DISCONNECTED", bg='#e74c3c')
            elif health["antenna"].startswith("CONNECTED") and "OK" in health["antenna"]:
                ant_label.config(text="ANT: OK", bg='#2ecc71')
            elif health["antenna"].startswith("CONNECTED"):
                ant_label.config(text="ANT: NO STRONG SIGNAL", bg='#f39c12')
            else:
                ant_label.config(text=f"ANT: {health['antenna']}", bg='#f39c12')
            lna_state = health.get("lna","UNKNOWN")
            if lna_state == "LNA_OK":
                lna_label.config(text="LNA: OK", bg='#2ecc71')
            elif lna_state == "LNA_SUSPICIOUS":
                lna_label.config(text="LNA: SUSP", bg='#f39c12')
            else:
                lna_label.config(text=f"LNA: {lna_state}", bg='#f39c12')

            # numeric bottom
            noise_var.set(f"Noise: {noise:.2f}" if noise is not None else "Noise: -")
            peak_var.set(f"Peak: {peak:.2f}" if peak is not None else "Peak: -")
            snr_var.set(f"SNR: {snr:.2f} dB" if snr is not None else "SNR: -")
            gain_var.set(f"Gain: {gain if gain is not None else 'auto/unknown'}")

            # update histories
            if noise is not None:
                noise_hist.append(noise)
                peak_hist.append(peak)
                snr_hist.append(snr)

            # update PSD curve
            if psd is not None and freqs is not None:
                line_psd.set_data(freqs, psd)
                ax_psd.set_xlim(freqs[0], freqs[-1])
                ymin = np.min(psd); ymax = np.max(psd)
                if ymax - ymin < 1.0:
                    ymin -= 1.0; ymax += 1.0
                ax_psd.set_ylim(ymin - 3.0, ymax + 3.0)

            # update waterfall
            if wf_list and wf_list[0] is not None:
                wf_arr = np.array([row for row in wf_list if row is not None])
                ax_wf.clear()
                ax_wf.imshow(wf_arr, aspect='auto', origin='lower',
                             extent=(freqs[0] if freqs is not None else -SAMPLE_RATE/2,
                                     freqs[-1] if freqs is not None else SAMPLE_RATE/2,
                                     0, wf_arr.shape[0]),
                             cmap='inferno')
                ax_wf.set_xlabel("Freq offset (Hz)")
                ax_wf.set_ylabel("Time")
            canvas.draw_idle()
        except Exception:
            logging.exception("GUI update failed")
        finally:
            root.after(int(UPDATE_SEC*1000), update_ui)

    # Calibration action: sample two blocks with user prompts
    def run_calibration():
        cal_btn.config(state='disabled')
        cal_status.set("Calibrating: please disconnect antenna and press OK")
        root.update()
        if not messagebox.askokcancel("Calibration", "Please disconnect antenna now and press OK when ready."):
            cal_btn.config(state='normal'); cal_status.set("Cancelled"); return
        # sample block 1
        try:
            b1 = probe.sdr.read_samples(READ_SAMPLES) if probe.sdr else None
        except Exception as e:
            logging.exception("Calibration sample no-ant failed")
            cal_status.set("Sample failed (no-ant)"); cal_btn.config(state='normal'); return
        cal_status.set("Now connect antenna and press OK")
        root.update()
        if not messagebox.askokcancel("Calibration", "Now connect antenna and press OK when ready."):
            cal_btn.config(state='normal'); cal_status.set("Cancelled"); return
        try:
            b2 = probe.sdr.read_samples(READ_SAMPLES) if probe.sdr else None
        except Exception as e:
            logging.exception("Calibration sample with-ant failed")
            cal_status.set("Sample failed (with-ant)"); cal_btn.config(state='normal'); return
        ok, msg = probe.calibrate(b1, b2)
        cal_status.set(msg if ok else f"Failed: {msg}")
        cal_btn.config(state='normal')

    cal_btn.config(command=run_calibration)

    # start updates
    root.after(1000, update_ui)
    try:
        root.mainloop()
    except KeyboardInterrupt:
        pass

# ------------------------ curses fallback UI ---------------------------
def start_curses_ui(probe: SDRProbe):
    if not CURSES_OK:
        raise RuntimeError("curses not available")
    def curses_main(stdscr):
        curses.curs_set(0)
        stdscr.nodelay(True)
        while True:
            stdscr.erase()
            stdscr.addstr(0,0,"RTL-SDR Health Monitor (curses)", curses.A_REVERSE)
            with probe.lock:
                noise = probe.noise_rel; peak = probe.peak_rel; snr = probe.snr
                abs_peak = probe.abs_peak_db; gain = probe.gain; failc = probe.fail_count; err = probe.last_error
            stdscr.addstr(2,0,f"Time: {time.strftime('%Y-%m-%d %H:%M:%S')}")
            stdscr.addstr(3,0,f"Center: {CENTER_FREQ/1e6:.3f} MHz  SR: {SAMPLE_RATE/1e6:.3f} MS/s")
            stdscr.addstr(5,0,f"Noise (rel): {noise:.2f}" if noise is not None else "Noise (rel): -")
            stdscr.addstr(6,0,f"Peak  (rel): {peak:.2f}" if peak is not None else "Peak (rel): -")
            stdscr.addstr(7,0,f"SNR (dB): {snr:.2f}" if snr is not None else "SNR: -")
            stdscr.addstr(8,0,f"Abs peak (20log10|IQ|): {abs_peak:.2f}" if abs_peak is not None else "Abs peak: -")
            stdscr.addstr(9,0,f"Gain: {gain if gain is not None else 'auto/unknown'}")
            if err:
                stdscr.addstr(11,0,"Last error: " + err[:curses.COLS-12], curses.A_BOLD)
            health = probe.get_health()
            stdscr.addstr(13,0,"Connection: " + health["connection"])
            stdscr.addstr(14,0,"Antenna: " + health["antenna"])
            stdscr.addstr(15,0,"LNA: " + health["lna"])
            stdscr.addstr(curses.LINES-1,0,"Press Ctrl-C to quit")
            stdscr.refresh()
            time.sleep(1.0)
    try:
        curses.wrapper(curses_main)
    except KeyboardInterrupt:
        pass

# ----------------------------- main -----------------------------------
def main():
    logging.info("Starting sdr_healthv3")
    probe = SDRProbe()
    probe.start()
    # choose GUI if available and a display exists, else curses
    display_env = os.environ.get("DISPLAY","")
    logging.info("DISPLAY=%r  TKINTER_OK=%s  CURSES_OK=%s", display_env, TKINTER_OK, CURSES_OK)
    # If tkinter available and display set, prefer GUI
    if TKINTER_OK and display_env:
        try:
            start_gui(probe)
            return
        except Exception:
            logging.exception("GUI failed, falling back to curses if available")
    # fallback to curses if available
    if CURSES_OK:
        start_curses_ui(probe)
    else:
        # neither GUI nor curses available — print periodic log to console
        logging.warning("No GUI or curses available. Printing status to console every second.")
        try:
            while True:
                with probe.lock:
                    noise = probe.noise_rel; peak = probe.peak_rel; snr = probe.snr
                    abs_peak = probe.abs_peak_db; gain = probe.gain; failc = probe.fail_count; err = probe.last_error
                health = probe.get_health()
                print(f"[{time.strftime('%H:%M:%S')}] conn={health['connection']} ant={health['antenna']} lna={health['lna']} noise={noise} peak={peak} snr={snr} abs_peak={abs_peak} gain={gain} err={err}")
                time.sleep(1.0)
        except KeyboardInterrupt:
            pass
    # cleanup
    probe.running = False
    probe.close_sdr()

if __name__ == "__main__":
    try:
        main()
    except Exception:
        logging.exception("Fatal error in main")
        print("See log:", LOGFILE)
        raise
