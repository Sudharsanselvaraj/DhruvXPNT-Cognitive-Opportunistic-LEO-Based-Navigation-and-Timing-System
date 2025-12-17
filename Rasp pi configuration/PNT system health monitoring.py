#!/usr/bin/env python3
"""
sdr_healthv4_layout.py

Improved RTL-SDR tech terminal with adjusted layout and spacing so plots
are well-separated and cover the display nicely.

Run inside your venv:
    source venv/bin/activate
    python sdr_healthv4_layout.py
"""
import os, sys, time, threading, logging
from collections import deque
import traceback

LOGFILE = os.path.expanduser("~/sdr_healthv3.log")
logging.basicConfig(filename=LOGFILE, level=logging.INFO,
                    format="%(asctime)s %(levelname)s: %(message)s")
console = logging.StreamHandler()
console.setLevel(logging.INFO)
console.setFormatter(logging.Formatter("%(levelname)s: %(message)s"))
logging.getLogger().addHandler(console)

# SDR / DSP
try:
    from rtlsdr import RtlSdr
except Exception as e:
    RtlSdr = None
    SDR_IMPORT_ERR = e
import numpy as np

# GUI / plotting
TKINTER_OK = True
try:
    import tkinter as tk
    from tkinter import ttk, messagebox
    import matplotlib
    matplotlib.use("TkAgg")
    from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
    from matplotlib.figure import Figure
    import matplotlib.ticker as mticker
except Exception as e:
    TKINTER_OK = False
    logging.info("Tkinter/Matplotlib not available: %s", e)

# curses fallback
try:
    import curses
    CURSES_OK = True
except Exception:
    CURSES_OK = False

# config
CENTER_FREQ = 1625e6
SAMPLE_RATE = 2.4e6
READ_SAMPLES = 128 * 1024
NFFT = 4096
PSD_AVG_BLOCKS = 2
WATERFALL_HISTORY = 120
HISTORY_LEN = 120
UPDATE_SEC = 1.0
RF_GAIN_DB = 48.0
ENABLE_BIAS_TEE = True
ABS_PEAK_DB_THRESHOLD = -65.0
MIN_SNR_DB = 6.0
CONSECUTIVE_FAILS_TO_OFFLINE = 4

# DSP helpers
def compute_psd_db(iq, nfft=NFFT, avg_blocks=PSD_AVG_BLOCKS):
    if iq is None or iq.size < nfft:
        return None, None
    iq = np.asarray(iq, dtype=np.complex64)
    acc = None
    start = 0
    count = 0
    while start + nfft <= iq.size and count < avg_blocks:
        seg = iq[start:start+nfft]
        win = np.hanning(nfft)
        spec = np.fft.fftshift(np.fft.fft(seg * win, n=nfft))
        ps = (np.abs(spec)**2) / (np.sum(win**2) + 1e-20)
        acc = ps if acc is None else acc + ps
        start += nfft
        count += 1
    if acc is None:
        return None, None
    psd = acc / max(1, count)
    with np.errstate(divide='ignore'):
        psd_db = 10.0 * np.log10(psd + 1e-20)
    psd_db = psd_db - np.max(psd_db)   # normalize strongest to 0
    freqs = np.linspace(-SAMPLE_RATE/2, SAMPLE_RATE/2, len(psd_db))
    return psd_db, freqs

def amplitude_peak_db(iq):
    if iq is None or iq.size == 0:
        return None, None
    mag = np.abs(iq)
    with np.errstate(divide='ignore'):
        mag_db = 20.0 * np.log10(np.maximum(mag, 1e-12))
    return float(np.max(mag_db)), float(np.median(mag_db))

# SDR probe
class SDRProbe(threading.Thread):
    def __init__(self):
        super().__init__(daemon=True)
        self.lock = threading.Lock()
        self.running = True
        self.sdr = None
        self.psd = None; self.freqs = None
        self.noise_rel = None; self.peak_rel = None; self.snr = None
        self.abs_peak_db = None; self.abs_med_db = None
        self.gain = None; self.last_error = None; self.fail_count = 0
        self.waterfall = deque(maxlen=WATERFALL_HISTORY)
        self.abs_peak_threshold = ABS_PEAK_DB_THRESHOLD
        self.min_snr_db = MIN_SNR_DB

    def open_sdr(self):
        if RtlSdr is None:
            raise RuntimeError(f"pyrtlsdr not available: {SDR_IMPORT_ERR}")
        s = RtlSdr()
        try: s.sample_rate = SAMPLE_RATE
        except: pass
        try: s.center_freq = CENTER_FREQ
        except: pass
        try:
            s.gain = RF_GAIN_DB
            logging.info("Set gain to %s dB", RF_GAIN_DB)
        except Exception:
            try: setattr(s, 'gain', RF_GAIN_DB); logging.info("Set gain via setattr")
            except Exception: logging.info("Could not set gain directly")
        # bias tee attempts
        if ENABLE_BIAS_TEE:
            try:
                if hasattr(s, 'set_bias_tee'):
                    s.set_bias_tee(True); logging.info("Bias-tee enabled via set_bias_tee")
                else:
                    for a in ('bias_tee','bias','enable_bias'):
                        if hasattr(s, a):
                            setattr(s, a, True); logging.info("Bias-tee set via attr %s", a); break
            except Exception:
                logging.info("Bias-tee enable failed (no API)")
        return s

    def close_sdr(self):
        if self.sdr is not None:
            try: self.sdr.close()
            except: pass
            self.sdr = None

    def run_once(self):
        try:
            if self.sdr is None:
                self.sdr = self.open_sdr()
            block = self.sdr.read_samples(READ_SAMPLES)
            psd_db, freqs = compute_psd_db(block)
            abs_peak, abs_med = amplitude_peak_db(block)
            if psd_db is not None:
                noise = float(np.median(psd_db)); peak = float(np.max(psd_db)); snr = float(peak - noise)
            else:
                noise = peak = snr = None
            with self.lock:
                self.psd = psd_db; self.freqs = freqs
                self.noise_rel = noise; self.peak_rel = peak; self.snr = snr
                self.abs_peak_db = abs_peak; self.abs_med_db = abs_med
                try: self.gain = float(self.sdr.gain)
                except: self.gain = None
                self.last_error = None; self.fail_count = 0
                row = (psd_db - np.min(psd_db)) / (np.max(psd_db) - np.min(psd_db) + 1e-12) if psd_db is not None else None
                self.waterfall.appendleft(row)
        except Exception as e:
            logging.exception("SDR read error")
            with self.lock:
                self.last_error = str(e); self.fail_count += 1
            self.close_sdr()

    def run(self):
        while self.running:
            self.run_once()
            time.sleep(0.01)

    def get_health(self):
        with self.lock:
            noise=self.noise_rel; peak=self.peak_rel; snr=self.snr; abs_peak=self.abs_peak_db
            failc=self.fail_count; gain=self.gain; last_err=self.last_error
        status = {"connection":"UNKNOWN","antenna":"UNKNOWN","lna":"UNKNOWN","reason":None}
        if failc >= CONSECUTIVE_FAILS_TO_OFFLINE:
            status["connection"]="OFFLINE"; status["reason"]=f"fails={failc}"; return status
        status["connection"]="ONLINE"
        if abs_peak is None: status["antenna"]="NO_DATA"
        else:
            if abs_peak < self.abs_peak_threshold:
                status["antenna"]="DISCONNECTED"; status["reason"]=f"abs_peak {abs_peak:.1f}<{self.abs_peak_threshold:.1f}"
            elif snr is None: status["antenna"]="UNKNOWN"
            elif snr < self.min_snr_db:
                status["antenna"]="CONNECTED_NO_STRONG_SIGNAL"; status["reason"]=f"SNR {snr:.1f}<{self.min_snr_db:.1f}"
            else: status["antenna"]="CONNECTED_SIGNAL_OK"
        if gain is None: status["lna"]="UNKNOWN"
        else:
            if gain>35 and (noise is not None and noise<-60): status["lna"]="LNA_OK"
            elif gain>40 and (noise is not None and noise>-45): status["lna"]="LNA_SUSPICIOUS"
            else: status["lna"]="UNKNOWN"
        return status

# GUI builder with improved spacing
def start_gui(probe):
    if not TKINTER_OK: raise RuntimeError("GUI libs not available")
    root = tk.Tk()
    root.title("RTL-SDR — Super Tech Terminal")
    # maximize window for good coverage
    try:
        root.state("zoomed")
    except Exception:
        # fallback: set a large geometry
        root.geometry("1360x768")
    root.configure(bg="#081016")

    # Left panel (info + controls)
    left = tk.Frame(root, bg="#071016", padx=12, pady=10)
    left.pack(side="left", fill="y")

    header = tk.Label(left, text="RTL-SDR TECH TERMINAL", font=("Consolas",16,"bold"), fg="#7be6ff", bg="#071016")
    header.pack(anchor="w", pady=(0,8))

    info = tk.Text(left, width=54, height=22, bg="#041018", fg="#dff6ff", bd=0, padx=8, pady=8, font=("Consolas",10))
    info.insert("end","Starting...\n"); info.config(state="disabled"); info.pack()

    badge_frame = tk.Frame(left, bg="#071016")
    badge_frame.pack(anchor="w", pady=(8,8))
    conn_badge = tk.Label(badge_frame, text="OFFLINE", bg="#e74c3c", fg="#fff", font=("Consolas",10,"bold"), padx=8, pady=4)
    conn_badge.pack(side="left", padx=(0,6))
    ant_badge = tk.Label(badge_frame, text="ANT: ?", bg="#f39c12", fg="#000", font=("Consolas",10,"bold"), padx=8, pady=4)
    ant_badge.pack(side="left", padx=(0,6))
    lna_badge = tk.Label(badge_frame, text="LNA: ?", bg="#f39c12", fg="#000", font=("Consolas",10,"bold"), padx=8, pady=4)
    lna_badge.pack(side="left", padx=(0,6))

    ctrl_frame = tk.Frame(left, bg="#071016")
    ctrl_frame.pack(anchor="w", pady=(8,0))
    tk.Label(ctrl_frame, text="Gain (dB):", fg="#cfefff", bg="#071016", font=("Consolas",10)).grid(row=0,column=0,sticky="w")
    gain_entry = tk.Entry(ctrl_frame, width=6, font=("Consolas",10))
    gain_entry.insert(0,str(RF_GAIN_DB))
    gain_entry.grid(row=0,column=1,padx=(6,12))
    def apply_gain():
        try:
            val = float(gain_entry.get())
        except:
            messagebox.showerror("Gain", "Enter numeric gain"); return
        with probe.lock:
            if probe.sdr:
                try:
                    probe.sdr.gain = val; logging.info("Gain set to %s", val); probe.gain = val; log("Gain set to %.1f dB" % val)
                except Exception as e:
                    logging.exception("Setting gain failed"); log("Setting gain failed: "+str(e))
            else:
                log("SDR not opened; reopen to apply gain")
    tk.Button(ctrl_frame, text="Apply", command=apply_gain).grid(row=0,column=2)

    bias_var = tk.BooleanVar(value=ENABLE_BIAS_TEE)
    def toggle_bias():
        with probe.lock:
            if probe.sdr:
                try:
                    if hasattr(probe.sdr,"set_bias_tee"):
                        probe.sdr.set_bias_tee(bool(bias_var.get())); log("Bias-tee toggled to %s" % bias_var.get())
                    else:
                        log("Bias-tee API not available on this build")
                except Exception as e:
                    log("Bias toggle failed: "+str(e))
            else:
                log("SDR not opened")
    tk.Checkbutton(ctrl_frame, text="Bias-Tee", var=bias_var, bg="#071016", fg="#cfefff", command=toggle_bias).grid(row=1,column=0,columnspan=2,sticky="w", pady=(8,0))

    def reopen_sdr():
        with probe.lock:
            probe.close_sdr()
        log("Reopening SDR...")
    tk.Button(ctrl_frame, text="Reopen SDR", command=reopen_sdr).grid(row=1,column=2,padx=(8,0))

    log_box = tk.Text(left, width=54, height=6, bg="#03060a", fg="#cfefff", bd=0, font=("Consolas",9))
    log_box.pack(pady=(8,0))
    def log(msg):
        logging.info(msg)
        ts = time.strftime("%H:%M:%S")
        log_box.configure(state="normal")
        log_box.insert("end", f"[{ts}] {msg}\n")
        log_box.see("end")
        log_box.configure(state="disabled")

    # Right: plotting area
    right = tk.Frame(root, bg="#081016", padx=6, pady=6)
    right.pack(side="right", fill="both", expand=True)

    from matplotlib.figure import Figure
    fig = Figure(figsize=(9,7), dpi=100, facecolor="#081016")
    # create 3 vertical rows: waterfall, PSD, sparklines
    ax_wf = fig.add_subplot(311)
    ax_psd = fig.add_subplot(312)
    ax_sparks = fig.add_subplot(313)
    # visuals
    for ax in (ax_wf, ax_psd, ax_sparks):
        ax.set_facecolor("#081016"); ax.title.set_color("#8be9fd")
        ax.tick_params(colors="#9fb7c9")
    ax_wf.set_title("Waterfall (recent)", pad=6)
    ax_psd.set_title("PSD (relative dB)", pad=6)
    ax_psd.set_ylabel("dB (rel)", color="#9fb7c9")
    ax_sparks.axis("off")

    # adjust spacing and margins to leave nice breathing room
    fig.subplots_adjust(hspace=0.55, left=0.12, right=0.98, top=0.96, bottom=0.06)

    canvas = FigureCanvasTkAgg(fig, master=right)
    canvas_widget = canvas.get_tk_widget()
    # add padding around the canvas so it doesn't touch edges
    canvas_widget.pack(fill="both", expand=True, padx=12, pady=12)

    # initial artists
    x_dummy = np.linspace(-SAMPLE_RATE/2, SAMPLE_RATE/2, NFFT)
    line_psd, = ax_psd.plot([], [], lw=1.2, color="#66c2ff")
    spark_noise_line, = ax_sparks.plot([], [], color="#7be6ff", lw=1)
    spark_peak_line, = ax_sparks.plot([], [], color="#ffb86b", lw=1)
    spark_snr_line, = ax_sparks.plot([], [], color="#7fff7f", lw=1)
    ax_psd.set_xlim(x_dummy[0], x_dummy[-1])
    ax_psd.set_ylim(-60, 5)

    hist_noise = deque(maxlen=HISTORY_LEN)
    hist_peak = deque(maxlen=HISTORY_LEN)
    hist_snr = deque(maxlen=HISTORY_LEN)

    def update_ui():
        try:
            with probe.lock:
                psd = probe.psd; freqs = probe.freqs
                noise = probe.noise_rel; peak = probe.peak_rel; snr = probe.snr
                abs_peak = probe.abs_peak_db; abs_med = probe.abs_med_db
                gain = probe.gain; last_err = probe.last_error; failc = probe.fail_count
                wf_rows = list(probe.waterfall)
                abs_threshold = probe.abs_peak_threshold; min_snr = probe.min_snr_db

            # info text
            lines = []
            lines.append(f"Local time: {time.strftime('%Y-%m-%d %H:%M:%S')}")
            lines.append(f"Center freq: {CENTER_FREQ/1e6:.3f} MHz")
            lines.append(f"Sample rate: {SAMPLE_RATE/1e6:.3f} MS/s    Read samples: {READ_SAMPLES}")
            lines.append("")
            if last_err: lines.append("Last error:"); lines.append(f"  {last_err}")
            else: lines.append("Device read: OK" if failc==0 else f"Device read: intermittent (fails={failc})")
            lines.append(f"Gain: {gain if gain is not None else 'auto/unknown'}")
            lines.append(f"Noise (rel dB): {noise:.2f}" if noise is not None else "Noise (rel dB): -")
            lines.append(f"Peak (rel dB): {peak:.2f}" if peak is not None else "Peak (rel dB): -")
            lines.append(f"Est SNR (dB): {snr:.2f}" if snr is not None else "Est SNR (dB): -")
            lines.append(f"Abs peak (20log10|IQ|): {abs_peak:.2f}" if abs_peak is not None else "Abs peak: -")
            lines.append(f"Abs median (20log10|IQ|): {abs_med:.2f}" if abs_med is not None else "Abs median: -")
            lines.append("")
            lines.append(f"Abs peak threshold: {abs_threshold:.2f} dB")
            lines.append(f"Min SNR threshold: {min_snr:.2f} dB")
            info.configure(state="normal"); info.delete("1.0","end"); info.insert("end", "\n".join(lines)+"\n"); info.configure(state="disabled")

            # badges
            health = probe.get_health()
            if health["connection"]=="OFFLINE": conn_badge.config(text="OFFLINE", bg="#e74c3c")
            else: conn_badge.config(text="ONLINE", bg="#2ecc71")
            if health["antenna"].startswith("DISCONNECTED"): ant_badge.config(text="ANT: DISCONNECTED", bg="#e74c3c")
            elif health["antenna"].startswith("CONNECTED") and "OK" in health["antenna"]: ant_badge.config(text="ANT: OK", bg="#2ecc71")
            elif health["antenna"].startswith("CONNECTED"): ant_badge.config(text="ANT: NO STRONG SIGNAL", bg="#f39c12")
            else: ant_badge.config(text=f"ANT: {health['antenna']}", bg="#f39c12")
            if health["lna"]=="LNA_OK": lna_badge.config(text="LNA: OK", bg="#2ecc71")
            elif health["lna"]=="LNA_SUSPICIOUS": lna_badge.config(text="LNA: SUSP", bg="#f39c12")
            else: lna_badge.config(text=f"LNA: {health['lna']}", bg="#f39c12")

            # histories
            if noise is not None:
                hist_noise.append(noise); hist_peak.append(peak); hist_snr.append(snr)
            x = np.arange(-len(hist_noise)+1, 1)
            if len(hist_noise)>0:
                spark_noise_line.set_data(x, list(hist_noise))
                spark_peak_line.set_data(x, list(hist_peak))
                spark_snr_line.set_data(x, list(hist_snr))
                ax_sparks.relim(); ax_sparks.autoscale_view()
                ax_sparks.set_ylim(min(min(hist_noise or [0])-5, -60), max(max(hist_peak or [0])+5, 5))

            # PSD update
            if psd is not None and freqs is not None:
                line_psd.set_data(freqs, psd)
                ax_psd.relim(); ax_psd.autoscale_view()
                ax_psd.xaxis.set_major_formatter(mticker.FuncFormatter(lambda v, pos: f"{v/1e3:.0f}k"))
                ax_psd.grid(True, color="#16303a", linestyle=":", linewidth=0.5)
            else:
                line_psd.set_data([], [])

            # waterfall update with robust vmin/vmax
            if wf_rows and wf_rows[0] is not None:
                wf_arr = np.array([r for r in wf_rows if r is not None])
                vmin = np.percentile(wf_arr, 2) if wf_arr.size>0 else 0.0
                vmax = np.percentile(wf_arr, 98) if wf_arr.size>0 else 1.0
                ax_wf.clear()
                ax_wf.set_facecolor("#081016")
                ax_wf.imshow(wf_arr, aspect='auto', origin='lower',
                             extent=(freqs[0], freqs[-1], 0, wf_arr.shape[0]),
                             cmap='inferno', vmin=vmin, vmax=vmax, interpolation='nearest')
                ax_wf.set_ylabel("Time")
                ax_wf.xaxis.set_major_formatter(mticker.FuncFormatter(lambda v,pos: f"{v/1e6:.2f}M"))
            canvas.draw_idle()
        except Exception:
            logging.exception("UI update failed")
        finally:
            root.after(int(UPDATE_SEC*1000), update_ui)

    root.after(1000, update_ui)
    root.mainloop()

def start_curses_ui(probe):
    if not CURSES_OK: raise RuntimeError("curses not available")
    def mainc(stdscr):
        curses.curs_set(0); stdscr.nodelay(True)
        while True:
            stdscr.erase()
            with probe.lock:
                n=probe.noise_rel; p=probe.peak_rel; s=probe.snr; ap=probe.abs_peak_db; g=probe.gain; err=probe.last_error
            stdscr.addstr(0,0,f"Time: {time.strftime('%Y-%m-%d %H:%M:%S')}")
            stdscr.addstr(1,0,f"Center: {CENTER_FREQ/1e6:.3f} MHz  SR: {SAMPLE_RATE/1e6:.3f} MS/s")
            stdscr.addstr(3,0,f"Noise: {n:.2f}" if n is not None else "Noise: -")
            stdscr.addstr(4,0,f"Peak: {p:.2f}" if p is not None else "Peak: -")
            stdscr.addstr(5,0,f"SNR: {s:.2f}" if s is not None else "SNR: -")
            stdscr.addstr(6,0,f"Abs peak: {ap:.2f}" if ap is not None else "Abs peak: -")
            stdscr.addstr(7,0,f"Gain: {g if g is not None else 'auto/unknown'}")
            if err: stdscr.addstr(9,0,"Err: "+str(err)[:curses.COLS-6])
            stdscr.refresh()
            time.sleep(1.0)
    import curses
    try: curses.wrapper(mainc)
    except KeyboardInterrupt: pass

def main():
    logging.info("Starting sdr_healthv4_layout")
    probe = SDRProbe()
    probe.start()
    display = os.environ.get("DISPLAY","")
    if TKINTER_OK and display:
        try:
            start_gui(probe); return
        except Exception:
            logging.exception("GUI failed, fallback to curses")
    if CURSES_OK:
        start_curses_ui(probe)
    else:
        try:
            while True:
                with probe.lock:
                    n=probe.noise_rel; p=probe.peak_rel; s=probe.snr; ap=probe.abs_peak_db; g=probe.gain; err=probe.last_error
                print(f"[{time.strftime('%H:%M:%S')}] noise={n} peak={p} snr={s} abs_peak={ap} gain={g} err={err}")
                time.sleep(1.0)
        except KeyboardInterrupt: pass
    probe.running = False; probe.close_sdr()

if __name__=="__main__":
    try: main()
    except Exception:
        logging.exception("Fatal")
        print("See", LOGFILE)
        raise
