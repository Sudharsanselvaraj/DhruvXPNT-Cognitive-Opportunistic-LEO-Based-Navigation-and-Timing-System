#!/usr/bin/env python3
"""
DhruvXPNT Report Generator

Generates comprehensive PDF/HTML reports from test run data including:
- System health metrics (CPU, temperature, throttling)
- RF signal analysis (spectrograms, burst statistics)
- Navigation solution accuracy
- SDR health diagnostics

Usage:
    python generate_report.py /path/to/run_directory

Output:
    report_YYYYMMDD_HHMMSS.pdf
    report_YYYYMMDD_HHMMSS.html
"""

import argparse
import os
import sys
import json
import csv
from datetime import datetime
from pathlib import Path

import numpy as np
import pandas as pd
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages


def parse_args():
    parser = argparse.ArgumentParser(
        description="Generate DhruvXPNT test run report"
    )
    parser.add_argument(
        "run_dir",
        help="Directory containing test run data (CSV, JSON, PNG files)"
    )
    parser.add_argument(
        "--output", "-o",
        default=".",
        help="Output directory for reports (default: current directory)"
    )
    parser.add_argument(
        "--format", "-f",
        choices=["pdf", "html", "both"],
        default="both",
        help="Report output format"
    )
    return parser.parse_args()


def load_system_health(csv_path):
    """Load system health CSV and return DataFrame."""
    if not os.path.exists(csv_path):
        return None
    df = pd.read_csv(csv_path)
    df = df.dropna(subset=["epoch_ms"])
    df["t"] = (df["epoch_ms"] - df["epoch_ms"].iloc[0]) / 1000.0
    return df


def plot_system_health(df, ax_list):
    """Generate system health plots."""
    axes = iter(ax_list)
    
    # CPU Temperature
    if "cpu_temp_C" in df.columns:
        ax = next(axes)
        ax.plot(df["t"], df["cpu_temp_C"], color="#e74c3c", linewidth=1.2)
        ax.set_xlabel("Time (s)")
        ax.set_ylabel("Temperature (°C)")
        ax.set_title("CPU Temperature")
        ax.grid(True, alpha=0.3)
    
    # CPU Usage
    if "cpu_usage_pct" in df.columns:
        ax = next(axes)
        ax.plot(df["t"], df["cpu_usage_pct"], color="#3498db", linewidth=1.2)
        ax.set_xlabel("Time (s)")
        ax.set_ylabel("CPU Usage (%)")
        ax.set_title("CPU Utilization")
        ax.grid(True, alpha=0.3)
    
    # Clock Speed
    if "clock_speed_MHz" in df.columns:
        ax = next(axes)
        ax.plot(df["t"], df["clock_speed_MHz"], color="#2ecc71", linewidth=1.2)
        ax.set_xlabel("Time (s)")
        ax.set_ylabel("Clock Speed (MHz)")
        ax.set_title("CPU Clock Speed")
        ax.grid(True, alpha=0.3)
    
    # Memory Usage
    if "mem_pct" in df.columns:
        ax = next(axes)
        ax.plot(df["t"], df["mem_pct"], color="#9b59b6", linewidth=1.2)
        ax.set_xlabel("Time (s)")
        ax.set_ylabel("Memory Usage (%)")
        ax.set_title("Memory Utilization")
        ax.grid(True, alpha=0.3)


def plot_spectrogram_summary(run_dir, ax):
    """Find and display spectrogram image if available."""
    spectrograms = list(Path(run_dir).glob("*_spectrogram.png")) + \
                   list(Path(run_dir).glob("iridium_spectrogram.png")) + \
                   list(Path(run_dir).glob("ir_spectrogram.png"))
    
    if spectrograms:
        img = plt.imread(str(spectrograms[0]))
        ax.imshow(img)
        ax.set_title(f"Spectrogram: {spectrograms[0].name}")
        ax.axis('off')
    else:
        ax.text(0.5, 0.5, "No spectrogram found", ha='center', va='center')
        ax.set_title("Spectrogram")
        ax.axis('off')


def generate_pdf_report(run_dir, output_path):
    """Generate PDF report with all analysis."""
    run_path = Path(run_dir)
    csv_path = run_path / "data.csv"
    
    with PdfPages(output_path) as pdf:
        # Title page
        fig = plt.figure(figsize=(8, 10))
        fig.text(0.5, 0.9, "DhruvXPNT Test Run Report",
                ha='center', fontsize=24, weight='bold')
        fig.text(0.5, 0.85, f"Generated: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}",
                ha='center', fontsize=12)
        fig.text(0.5, 0.80, f"Run Directory: {run_dir}",
                ha='center', fontsize=10, style='italic')
        
        # System info
        fig.text(0.1, 0.70, "System Information", fontsize=14, weight='bold')
        info_text = []
        if csv_path.exists():
            df = load_system_health(csv_path)
            if df is not None:
                info_text.append(f"Data Points: {len(df)}")
                info_text.append(f"Duration: {df['t'].iloc[-1]:.1f} seconds")
                if 'cpu_temp_C' in df.columns:
                    info_text.append(f"Max CPU Temp: {df['cpu_temp_C'].max():.1f}°C")
                if 'cpu_usage_pct' in df.columns:
                    info_text.append(f"Avg CPU Usage: {df['cpu_usage_pct'].mean():.1f}%")
        
        fig.text(0.1, 0.65, "\n".join(info_text), fontsize=10, family='monospace')
        
        # Burst metadata
        burst_files = list(run_path.glob("burst_*.json"))
        fig.text(0.1, 0.50, "RF Signal Analysis", fontsize=14, weight='bold')
        fig.text(0.1, 0.45, f"Burst Metadata Files: {len(burst_files)}", fontsize=10)
        
        pdf.savefig(fig, bbox_inches='tight')
        plt.close(fig)
        
        # System health plots
        if csv_path.exists():
            df = load_system_health(csv_path)
            if df is not None:
                fig, axes = plt.subplots(2, 2, figsize=(12, 8))
                plot_system_health(df, axes.flatten())
                plt.tight_layout()
                pdf.savefig(fig, bbox_inches='tight')
                plt.close(fig)
        
        # Spectrogram page
        fig, ax = plt.subplots(figsize=(12, 6))
        plot_spectrogram_summary(run_dir, ax)
        pdf.savefig(fig, bbox_inches='tight')
        plt.close(fig)
        
        # Burst analysis page
        if burst_files:
            fig = plt.figure(figsize=(12, 8))
            ax = fig.add_subplot(111)
            
            burst_data = []
            for bf in burst_files:
                try:
                    with open(bf) as f:
                        data = json.load(f)
                        burst_data.append({
                            'freq': data.get('center_frequency_mhz', 0),
                            'power': data.get('avg_power_db', 0),
                            'bursts': data.get('num_bursts_detected', 0)
                        })
                except Exception:
                    pass
            
            if burst_data:
                df_bursts = pd.DataFrame(burst_data)
                ax.scatter(df_bursts['freq'], df_bursts['power'], 
                          s=df_bursts['bursts']*10, alpha=0.6)
                ax.set_xlabel("Frequency (MHz)")
                ax.set_ylabel("Power (dB)")
                ax.set_title("Burst Analysis")
                ax.grid(True, alpha=0.3)
            
            pdf.savefig(fig, bbox_inches='tight')
            plt.close(fig)
    
    print(f"PDF report saved: {output_path}")


def generate_html_report(run_dir, output_path):
    """Generate HTML report."""
    run_path = Path(run_dir)
    csv_path = run_path / "data.csv"
    
    html = f"""<!DOCTYPE html>
<html>
<head>
    <title>DhruvXPNT Test Report</title>
    <style>
        body {{ font-family: Arial, sans-serif; margin: 40px; background: #f5f5f5; }}
        .container {{ max-width: 1200px; margin: 0 auto; background: white; padding: 30px; }}
        h1 {{ color: #2c3e50; border-bottom: 3px solid #3498db; padding-bottom: 10px; }}
        h2 {{ color: #34495e; margin-top: 30px; }}
        .metric {{ display: inline-block; margin: 10px 20px 10px 0; padding: 15px; background: #ecf0f1; border-radius: 8px; }}
        .metric-label {{ font-size: 12px; color: #7f8c8d; }}
        .metric-value {{ font-size: 24px; font-weight: bold; color: #2c3e50; }}
        table {{ width: 100%; border-collapse: collapse; margin-top: 20px; }}
        th, td {{ padding: 12px; text-align: left; border-bottom: 1px solid #ddd; }}
        th {{ background: #3498db; color: white; }}
        tr:hover {{ background: #f5f5f5; }}
        .status-ok {{ color: #27ae60; }}
        .status-warn {{ color: #f39c12; }}
        .status-fail {{ color: #e74c3c; }}
    </style>
</head>
<body>
    <div class="container">
        <h1>🛰 DhruvXPNT Test Run Report</h1>
        <p><strong>Generated:</strong> {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}</p>
        <p><strong>Run Directory:</strong> {run_dir}</p>
        
        <h2>📊 System Metrics</h2>
"""
    
    if csv_path.exists():
        df = load_system_health(csv_path)
        if df is not None:
            html += f"""
        <div class="metric">
            <div class="metric-label">Data Points</div>
            <div class="metric-value">{len(df)}</div>
        </div>
        <div class="metric">
            <div class="metric-label">Duration</div>
            <div class="metric-value">{df['t'].iloc[-1]:.1f}s</div>
        </div>
"""
            if 'cpu_temp_C' in df.columns:
                html += f"""
        <div class="metric">
            <div class="metric-label">Max CPU Temp</div>
            <div class="metric-value">{df['cpu_temp_C'].max():.1f}°C</div>
        </div>
"""
            if 'cpu_usage_pct' in df.columns:
                html += f"""
        <div class="metric">
            <div class="metric-label">Avg CPU Usage</div>
            <div class="metric-value">{df['cpu_usage_pct'].mean():.1f}%</div>
        </div>
"""
    
    html += """
        <h2>📁 Files in Run Directory</h2>
        <table>
            <tr><th>File</th><th>Size</th><th>Type</th></tr>
"""
    
    for f in sorted(run_path.iterdir()):
        if f.is_file():
            size = f.stat().st_size
            size_str = f"{size/1024/1024:.2f} MB" if size > 1024*1024 else f"{size/1024:.2f} KB"
            html += f"<tr><td>{f.name}</td><td>{size_str}</td><td>{f.suffix}</td></tr>\n"
    
    html += """
        </table>
    </div>
</body>
</html>
"""
    
    with open(output_path, 'w') as f:
        f.write(html)
    
    print(f"HTML report saved: {output_path}")


def main():
    args = parse_args()
    
    run_dir = args.run_dir
    if not os.path.exists(run_dir):
        print(f"Error: Run directory not found: {run_dir}")
        sys.exit(1)
    
    output_dir = Path(args.output)
    output_dir.mkdir(parents=True, exist_ok=True)
    
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    
    if args.format in ("pdf", "both"):
        pdf_path = output_dir / f"dhruvx_report_{timestamp}.pdf"
        generate_pdf_report(run_dir, str(pdf_path))
    
    if args.format in ("html", "both"):
        html_path = output_dir / f"dhruvx_report_{timestamp}.html"
        generate_html_report(run_dir, str(html_path))
    
    print("\n✓ Report generation complete!")


if __name__ == "__main__":
    main()
