#!/usr/bin/env python3
"""
Load a raw IQ file (auto-detect common formats) and save a centered spectrogram PNG.

Supported input formats:
 - complex64 (float32 complex, i.e. dtype=np.complex64)
 - float32 interleaved (I float32, Q float32, i.e. view as complex64)
 - uint8 interleaved (I,Q bytes, 0..255 with 128==0)
"""
import numpy as np
import os, sys, argparse
import matplotlib.pyplot as plt
from scipy.signal import stft

def detect_and_load(path):
    size = os.path.getsize(path)
    # Try complex64
    try:
        arr = np.fromfile(path, dtype=np.complex64)
        if arr.size > 1024:
            return arr.astype(np.complex64), 'complex64'
    except Exception:
        pass

    # Try float32 interleaved (I,Q float32)
    try:
        f = np.fromfile(path, dtype=np.float32)
        if f.size > 2048 and (f.size % 2 == 0):
            c = f.reshape(-1,2)
            iq = c[:,0] + 1j*c[:,1]
            return iq.astype(np.complex64), 'float32_interleaved'
    except Exception:
        pass

    # Try uint8 interleaved I,Q
    try:
        u = np.fromfile(path, dtype=np.uint8)
        if u.size > 2048 and (u.size % 2 == 0):
            I = (u[0::2].astype(np.float32) - 128.0) / 128.0
            Q = (u[1::2].astype(np.float32) - 128.0) / 128.0
            iq = I + 1j*Q
            return iq.astype(np.complex64), 'uint8_interleaved'
    except Exception:
        pass

    raise RuntimeError("Unknown/unsupported file format or file too small.")

def make_centered_spectrogram(iq, sr, nperseg=2048, noverlap=None, cmap='viridis'):
    if noverlap is None:
        noverlap = nperseg // 2
    # compute STFT (returns freqs 0..sr if complex input; we'll re-center)
    f, t, Z = stft(iq, fs=sr, nperseg=nperseg, noverlap=noverlap, boundary=None)
    # Z shape: (len(f), len(t))
    # center the frequency axis: shift rows
    Zs = np.fft.fftshift(Z, axes=0)
    # build frequency axis: -sr/2 .. +sr/2
    fcenter = np.linspace(-sr/2, sr/2, Zs.shape[0], endpoint=False)
    Pdb = 20.0 * np.log10(np.abs(Zs) + 1e-12)
    return fcenter, t, Pdb

def save_png(faxis, taxis, Pdb, center_hz, out_path, dpi=150):
    plt.figure(figsize=(12,6))
    plt.pcolormesh(taxis, faxis/1e6, Pdb, shading='auto')
    plt.xlabel("Time (s)")
    plt.ylabel("Frequency offset (MHz)")
    plt.title(f"Spectrogram (center {center_hz/1e6:.6f} MHz)")
    c = plt.colorbar(label="Power (dB)")
    plt.tight_layout()
    plt.savefig(out_path, dpi=dpi)
    plt.close()

def main():
    p = argparse.ArgumentParser()
    p.add_argument("file", help="raw IQ file")
    p.add_argument("--sr", type=float, default=2_400_000.0, help="sample rate (Hz)")
    p.add_argument("--center", type=float, default=1622000000.0, help="center freq (Hz)")
    p.add_argument("--out", default="spectrogram.png", help="output PNG filename")
    p.add_argument("--nperseg", type=int, default=2048)
    p.add_argument("--noverlap", type=int, default=None)
    args = p.parse_args()

    print("Loading", args.file)
    iq, fmt = detect_and_load(args.file)
    print("Detected format:", fmt)
    print("Samples:", iq.size, "Duration(s):", iq.size/args.sr)

    faxis, taxis, Pdb = make_centered_spectrogram(iq, sr=args.sr, nperseg=args.nperseg, noverlap=args.noverlap)
    print("Saving spectrogram to", args.out)
    save_png(faxis, taxis, Pdb, args.center, args.out)
    print("Done.")

if __name__ == "__main__":
    main()
