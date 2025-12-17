# DhruvXPNT  
## Cognitive Opportunistic LEO-Based Navigation & Timing System

DhruvXPNT is a **research-grade hardware–software system** focused on **GNSS-independent Position, Navigation, and Timing (PNT)** using **Signals of Opportunity (SoO)** from **Low Earth Orbit (LEO) satellites**.

This repository documents the **existence, capability, and validation** of the system through controlled demonstrations and hardware deployment.  
**Implementation details, algorithms, and core methodologies are intentionally restricted.**

🏆 **Winner – Smart India Hackathon (SIH) 2025, Hardware Edition**

---

## Project Overview

DhruvXPNT demonstrates:
- Passive RF-based navigation capability
- Operation without reliance on GNSS payload decoding
- On-device processing and monitoring
- Portable, self-contained hardware deployment

The system has been **successfully built, tested, and demonstrated** on real RF signals under live conditions.

---

## Physical Hardware Prototype

### Enclosed System Unit

![DhruvXPNT Prototype](Images/dhruvx%20pnt%201.png)
![DhruvXPNT Enclosure](Images/dhruvx%20pnt%202.png)
![DhruvXPNT Internal Layout](Images/dhruvx%20pnt%203.png)

The prototype integrates RF front-end, processing, display, and thermal management into a single portable unit.

---

## RF Signal Observation (LEO Band)

### Live SDR Capture

![Iridium Signal Capture](Images/Airspy%20iridium%20signal%20capturing.jpg)

The system passively observes LEO-band RF activity and performs real-time signal assessment.

---

## On-Device Processing & Detection

### Burst Activity Capture

![Burst Capture on Raspberry Pi](Images/Burst%20capturing%20in%20pi.jpg)

The platform performs real-time signal evaluation directly on embedded hardware.

---

## SDR Technical Monitoring Interface

### RTL-SDR Diagnostic Terminal

![RTL SDR Technical Terminal](Images/RTL%20SDR%20tech%20terminal.jpg)

The interface provides live visibility into:
- RF conditions
- Signal quality metrics
- Device health status
- Processing state

---

## System Health & Telemetry

### Live Waterfall and Metrics

![Waterfall System Health](Images/Waterfall%20system%20health.jpg)
![PNT System Health](Images/PNT%20system%20health.jpg)

Continuous monitoring ensures operational stability during deployment.

---

## User Interaction & Display

![User Interaction](Images/User%20interaction%20with%20system%20health.jpg)
![Raspberry Pi Display](Images/Pi%20display%20.png)

The system includes an integrated display for live interaction and system status visualization.

---

## Repository Contents

This repository contains:
- System configuration files
- Demonstration artifacts
- Captured metadata samples
- Reports and documentation
- Visual evidence of system operation

