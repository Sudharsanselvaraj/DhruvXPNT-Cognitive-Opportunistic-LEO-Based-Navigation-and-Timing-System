import numpy as np
import matplotlib.pyplot as plt
from scipy import signal

FILE = "iridium_iq.bin"     # IQ file to read
SR = 2_400_000              # Sample rate
OUT = "iridium_spectrogram.png"   # PNG output filename

print("[*] Loading IQ file...")
iq = np.fromfile(FILE, dtype=np.complex64)

print("[*] Generating spectrogram (this may take a moment)...")
f, t, Sxx = signal.spectrogram(
    iq,
    fs=SR,
    nperseg=4096,
    noverlap=2048,
    scaling='density',
    mode='magnitude'
)

Sxx_db = 10 * np.log10(Sxx + 1e-12)

plt.figure(figsize=(14, 6), dpi=150)
plt.pcolormesh(t, f/1e6, Sxx_db, shading="auto", cmap="viridis")
plt.xlabel("Time (s)")
plt.ylabel("Frequency Offset (MHz)")
plt.title("Iridium Spectrogram")
plt.colorbar(label="Power (dB)")

print("[*] Saving PNG:", OUT)
plt.savefig(OUT, dpi=150, bbox_inches="tight")

# Optional: comment out if you do NOT want the window to pop up
# plt.show()

plt.close()
print("[✓] Done. Saved spectrogram to:", OUT)
