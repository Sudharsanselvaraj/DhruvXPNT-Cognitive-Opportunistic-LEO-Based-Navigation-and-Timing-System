# doppler_predict.py
# Requires: numpy, pandas, skyfield, sgp4, requests (same environment as previous script)
# Drop this into same folder as your iridium propagation script and call compute_doppler_for_df()

import numpy as np
from datetime import datetime, timezone
from math import sin, cos, radians
import pandas as pd

# Constants
C = 299792458.0               # speed of light (m/s)
EARTH_OMEGA = 7.2921150e-5    # earth rotation rate (rad/s)
KM_PER_M = 1.0/1000.0

def geodetic_to_ecef_km(lat_deg, lon_deg, alt_m):
    """
    Convert geodetic (WGS84) to ECEF (km).
    lat_deg, lon_deg in degrees, alt_m in meters.
    Returns 3-vector in km.
    """
    # WGS84 ellipsoid
    a = 6378137.0             # semi-major axis (m)
    f = 1/298.257223563
    e2 = f * (2 - f)
    lat = radians(lat_deg)
    lon = radians(lon_deg)

    sin_lat = sin(lat)
    cos_lat = cos(lat)
    N = a / np.sqrt(1 - e2 * sin_lat * sin_lat)  # radius of curvature (m)

    x = (N + alt_m) * cos_lat * np.cos(lon)
    y = (N + alt_m) * cos_lat * np.sin(lon)
    z = (N * (1 - e2) + alt_m) * sin_lat

    return np.array([x, y, z]) * KM_PER_M  # km

def receiver_ecef_velocity_km_s(ecef_km):
    """
    Receiver velocity (km/s) due to Earth rotation: v = omega x r
    ecef_km: position vector in km
    """
    omega = np.array([0.0, 0.0, EARTH_OMEGA])  # rad/s
    # convert omega to km/s by cross product with km vector
    return np.cross(omega, ecef_km)  # km/s

def compute_range_and_range_rate(sat_pos_km, sat_vel_km_s, recv_ecef_km, recv_vel_km_s):
    """
    sat_pos_km, sat_vel_km_s: numpy arrays (3,)
    recv_*: same
    Returns: range_km, range_rate_m_s (positive if increasing distance)
    """
    rho_vec = sat_pos_km - recv_ecef_km      # km
    range_km = np.linalg.norm(rho_vec)
    rho_hat = rho_vec / (range_km + 1e-12)   # unit vector
    rel_vel_km_s = sat_vel_km_s - recv_vel_km_s
    range_rate_km_s = np.dot(rel_vel_km_s, rho_hat)  # km/s
    range_rate_m_s = range_rate_km_s * 1000.0
    return range_km, range_rate_m_s

def doppler_from_range_rate(range_rate_m_s, carrier_freq_hz):
    """
    Doppler (Hz) from range-rate: shift = - (range_rate / c) * f
    Negative range_rate (closing) gives positive shift; we adopt sign: predicted_shift = -range_rate/c * f
    """
    return - (range_rate_m_s / C) * carrier_freq_hz

def compute_doppler_for_df(df, receiver_lat_deg, receiver_lon_deg, receiver_alt_m, carrier_freq_hz):
    """
    Input:
      df: DataFrame produced by propagation step MUST include 'pos_x_km','pos_y_km','pos_z_km',
          'vel_x_km_s','vel_y_km_s','vel_z_km_s'
      receiver_* : geodetic coordinates & antenna height (m)
      carrier_freq_hz : frequency to predict (Hz), e.g., 1.62e9 for 1.62 GHz
    Returns: df with extra columns: range_km, range_rate_m_s, doppler_hz
    """
    # Validate columns
    required = ['pos_x_km','pos_y_km','pos_z_km','vel_x_km_s','vel_y_km_s','vel_z_km_s']
    for c in required:
        if c not in df.columns:
            raise ValueError(f"DataFrame missing required column: {c}")

    recv_ecef_km = geodetic_to_ecef_km(receiver_lat_deg, receiver_lon_deg, receiver_alt_m)
    recv_vel_km_s = receiver_ecef_velocity_km_s(recv_ecef_km)

    ranges = []
    range_rates = []
    dopplers = []

    for idx, row in df.iterrows():
        sat_pos = np.array([row['pos_x_km'], row['pos_y_km'], row['pos_z_km']], dtype=float)
        sat_vel = np.array([row['vel_x_km_s'], row['vel_y_km_s'], row['vel_z_km_s']], dtype=float)

        r_km, rr_m_s = compute_range_and_range_rate(sat_pos, sat_vel, recv_ecef_km, recv_vel_km_s)
        dop = doppler_from_range_rate(rr_m_s, carrier_freq_hz)

        ranges.append(float(r_km))
        range_rates.append(float(rr_m_s))
        dopplers.append(float(dop))

    df = df.copy()
    df['range_km'] = ranges
    df['range_rate_m_s'] = range_rates
    df['pred_doppler_hz'] = dopplers
    return df

# Example usage:
if __name__ == "__main__":
    # load your CSV produced earlier
    df = pd.read_csv("iridium_positions_iridium_20251208T091957Z.csv")  # update filename
    # Set your receiver coordinates (replace with your test site)
    receiver_lat = 12.9716   # deg (example: Bangalore)
    receiver_lon = 77.5946   # deg
    receiver_alt_m = 900.0   # meters

    # Carrier frequency to predict (Hz). Iridium L-band is roughly ~1.6 GHz; use the frequency you're measuring.
    carrier_freq = 1.62e9  # 1.62 GHz (example — change for the exact carrier you're tracking)

    df_out = compute_doppler_for_df(df, receiver_lat, receiver_lon, receiver_alt_m, carrier_freq)
    print(df_out[['name','norad_id','lat_deg','lon_deg','alt_km','range_km','range_rate_m_s','pred_doppler_hz']].to_string(index=False))
    # Save for inspection
    df_out.to_csv("iridium_doppler_predictions.csv", index=False)
    print("\nSaved: iridium_doppler_predictions.csv")
