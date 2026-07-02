# DhruvXPNT
## Cognitive Opportunistic LEO-Based Navigation & Timing System

DhruvXPNT is a **research-grade hardware–software system** for **GNSS-independent Position, Navigation, and Timing (PNT)** using **Signals of Opportunity (SoO)** from **Low Earth Orbit (LEO) satellites**.

This repository presents **system capability, physical realization, and validation evidence only**.  
**Technical algorithms, signal processing pipelines, and navigation logic are intentionally withheld.**

🏆 **Winner — Smart India Hackathon (SIH) 2025, Hardware Edition**

---

## Project Structure

```
DhruvXPNT/
├── README.md                    # This file
├── pyproject.toml              # Python package configuration
├── .gitignore                  # Git ignore rules
│
├── src/dhruvx/                 # Main Python package
│   ├── core/                   # PNT solver, Doppler prediction, TLE management
│   │   ├── pnt_solver.py
│   │   └── doppler_predictor.py
│   ├── dsp/                    # Digital signal processing
│   │   ├── capture.py          # RTL-SDR IQ capture
│   │   ├── burst_detector.py   # Burst detection & Doppler extraction
│   │   └── spectrogram.py      # Spectrogram generation
│   ├── imu/                    # Inertial measurement unit
│   │   ├── dead_reckoning.py   # IMU-based dead reckoning server
│   │   ├── sensor_reader.py    # MPU6050/ISM330DHCX reader
│   │   ├── mpu_probe.py        # MPU I2C probe utility
│   │   └── mpu_test.py         # MPU test script
│   ├── sdr/                    # SDR health & monitoring
│   │   ├── health_monitor.py   # RTL-SDR health GUI
│   │   └── waterfall_display.py # System waterfall monitor
│   ├── dashboard/              # Web dashboards
│   │   ├── integrated.py       # Unified IMU + Satellite dashboard
│   │   ├── realtime_plotter.py # Real-time plotter
│   │   └── templates/          # HTML templates
│   ├── utils/                  # Utilities
│   ├── cli/                    # Command-line interface
│   │   └── main.py             # Main CLI entry point
│   └── __init__.py
│
├── tests/                      # Unit tests
├── scripts/                    # Analysis & reporting scripts
│   ├── analyze.py              # Run analysis
│   ├── logger_final.py         # System health logger
│   └── generate_report.py      # Report generator
│
├── config/                     # Configuration files
│   ├── config.yaml
│   └── requirements.txt
│
├── data/                       # Data files (CSV, samples)
│   ├── samples/
│   ├── iridium_doppler_predictions.csv
│   └── iridium_positions_*.csv
│
├── firmware/                   # Embedded firmware
│   └── arduino/
│       └── imu_bmp_mmc.ino     # Arduino IMU + Barometer + Magnetometer
│
├── docs/                       # Documentation
│   ├── DhruvX_documentation.pdf
│   └── Implementation_and_stress_report.pdf
│
├── reports/                    # Reports & comparisons
├── images/                     # Project images
├── notebooks/                  # Jupyter notebooks (future)
└── archive/                    # Archived old versions
    ├── old_imu/
    ├── old_capture/
    ├── old_sdr/
    ├── old_dashboard/
    ├── old_dsp/
    └── old_utils/
```

---

## Quick Start

### Installation

```bash
# Clone the repository
git clone <repo-url>
cd DhruvXPNT

# Install as a package (editable mode)
pip install -e ".[dev]"

# Or install dependencies only
pip install -r config/requirements.txt
```

### Capture RF Signal

```bash
# Capture Iridium IQ data
dhruvx-capture

# Or use the module directly
python -m dhruvx.dsp.capture
```

### Process IQ Data

```bash
# Generate spectrogram
python -m dhruvx.dsp.spectrogram data/iridium_iq.bin

# Detect bursts and extract Doppler
python -m dhruvx.dsp.burst_detector
```

### Run PNT Solver

```bash
# Solve for position from IQ data
python -m dhruvx.core.pnt_solver --bin data/iridium_iq.bin --tle data/iridium.tle
```

### Start IMU Server

```bash
# Start dead reckoning server with web dashboard
python -m dhruvx.imu.dead_reckoning
```

### Monitor SDR Health

```bash
# Launch SDR health monitor GUI
python -m dhruvx.sdr.health_monitor
```

---

## Hardware Requirements

- **RTL-SDR Blog V4** (or compatible) for RF capture
- **Raspberry Pi 4** (or similar) for embedded processing
- **MPU6050 / ISM330DHCX** IMU for dead reckoning
- **BMP390 / BMP388** barometer for altitude
- **MMC5983MA** magnetometer for heading
- Active L-band antenna (Iridium: ~1621 MHz)

---

## Software Dependencies

Core:
- Python 3.9+
- NumPy, SciPy, Matplotlib
- pyrtlsdr (RTL-SDR bindings)
- sgp4 (TLE propagation)

Dashboard:
- Flask, Flask-SocketIO
- PyQt5, pyqtgraph (for SDR monitor)

IMU:
- pyserial, smbus2

Optional:
- pandas, pyproj, pillow, psutil

---

## Key Features

- **Passive RF-based navigation** — No GNSS payload decoding
- **LEO signal exploitation** — Uses Iridium (and extensible to Starlink, OneWeb)
- **Doppler-based positioning** — Sub-meter accuracy target
- **IMU dead reckoning** — Barometric altitude, static detection, bias learning
- **Real-time dashboards** — Web-based map tracking + SDR health monitoring
- **Cognitive layer ready** — ML module placeholders for future enhancement
- **Embedded processing** — Runs on Raspberry Pi with enclosed hardware

---

## Documentation

- [DhruvX Documentation](docs/DhruvX_documentation.pdf)
- [Implementation & Stress Report](docs/Implementation_and_stress_report.pdf)

---

## License

MIT License — See project details.

---

## Acknowledgments

**Smart India Hackathon 2025 — Hardware Edition**  
Evaluated via live hardware demonstration and real RF signal operation.

Ministry of Defence (MoD) — Defence Space Agency, Headquarters Integrated Defence Staff (IDS)

---

## Project Images

### Hardware Prototype
<table>
<tr>
<td><img src="images/dhruvx%20pnt%201.png" width="100%"></td>
<td><img src="images/dhruvx%20pnt%202.png" width="100%"></td>
</tr>
<tr>
<td colspan="2"><img src="images/dhruvx%20pnt%203.png" width="100%"></td>
</tr>
</table>

### RF Signal Observation (LEO Band)
<table>
<tr>
<td><img src="images/Airspy%20iridium%20signal%20capturing.jpg" width="100%"></td>
</tr>
</table>

### On-Device Processing & Detection
<table>
<tr>
<td><img src="images/Burst%20capturing%20in%20pi.jpg" width="100%"></td>
</tr>
</table>

### SDR Technical Monitoring Interface
<table>
<tr>
<td><img src="images/RTL%20SDR%20tech%20terminal.jpg" width="100%"></td>
</tr>
</table>

### System Health & Telemetry
<table>
<tr>
<td><img src="images/Waterfall%20system%20health.jpg" width="100%"></td>
<td><img src="images/PNT%20system%20health.jpg" width="100%"></td>
</tr>
</table>

### User Interaction & Display
<table>
<tr>
<td><img src="images/User%20interaction%20with%20system%20health.jpg" width="100%"></td>
<td><img src="images/Pi%20display%20.png" width="100%"></td>
</tr>
</table>

### Validation & Recognition (WINNING PROOF)
<table>
<tr>
<td><img src="images/winning%20moment%20team.jpg" width="100%"></td>
</tr>
<tr>
<td><img src="images/me%20with%20cup.jpg" width="100%"></td>
</tr>
<tr>
<td><img src="images/only%20cup%20with%20cheque.jpg" width="100%"></td>
</tr>
</table>
