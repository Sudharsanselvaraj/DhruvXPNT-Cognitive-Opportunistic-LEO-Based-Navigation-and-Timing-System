#!/usr/bin/env python3
import pandas as pd, os, matplotlib.pyplot as plt
RUN_DIR = os.path.expanduser("~/pnt_test/run1")
CSV = os.path.join(RUN_DIR,"data.csv")
df = pd.read_csv(CSV)
df = df.dropna(subset=["epoch_ms"])
df["t"] = (df["epoch_ms"] - df["epoch_ms"].iloc[0]) / 1000.0

def safe_plot(col, label):
    if col not in df.columns: 
        print("skip", col)
        return
    plt.figure()
    plt.plot(df["t"], df[col])
    plt.xlabel("Time (s)")
    plt.ylabel(label)
    plt.title(f"{label} vs time")
    plt.grid(True)
    out = os.path.join(RUN_DIR, f"{col}.png")
    plt.savefig(out)
    plt.close()
    print("Saved", out)

safe_plot("cpu_temp_C","CPU Temperature (°C)")
safe_plot("cpu_usage_pct","CPU Usage (%)")
safe_plot("clock_speed_MHz","Clock Speed (MHz)")
# If you have arduino_json or sdr_event_json, you can inspect manually
print("Done. PNGs in", RUN_DIR)
