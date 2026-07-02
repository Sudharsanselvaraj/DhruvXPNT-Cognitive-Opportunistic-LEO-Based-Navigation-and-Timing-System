from rtlsdr import RtlSdr
import numpy as np
import time

CENTER_FREQ = 1622000000  # 1622 MHz
SAMPLE_RATE = 2_400_000   # 2.4 MSPS
GAIN = 48               # Good starting gain
DURATION = 10             # seconds
OUTFILE = "iridium_iq.bin"

def main():
    print("Opening RTL-SDR Blog V4...")
    sdr = RtlSdr()

    # Enable Bias-Tee (IMPORTANT for LNA!!)
    try:
        sdr.bias_tee = True
        print("Bias-Tee enabled.")
    except:
        print("Bias-Tee not supported here.")

    sdr.sample_rate = SAMPLE_RATE
    sdr.center_freq = CENTER_FREQ
    sdr.gain = GAIN

    print(f"Capturing {DURATION} sec at {SAMPLE_RATE/1e6} MSPS...")

    num_samples = int(SAMPLE_RATE * DURATION)
    samples = sdr.read_samples(num_samples)

    sdr.close()
    print("Capture complete.")

    samples.astype(np.complex64).tofile(OUTFILE)
    print(f"Saved to {OUTFILE}")

if __name__ == "__main__":
    main()
