#!/usr/bin/env python3
"""
Master Orchestrator for GPSOpenCl Software Receiver Live Simulation & Web Analytics.
Spawns gps-sdr-sim signal generator, GPSOpenCl executable, and Plotly Dash Web Dashboard.
"""

import argparse
import os
import subprocess
import sys
import time

def parse_args():
    parser = argparse.ArgumentParser(description="Master Live Orchestrator for GPSOpenCl & Plotly Dashboard")
    parser.add_argument("--lat", type=float, default=48.1173, help="Simulated target latitude in degrees")
    parser.add_argument("--lon", type=float, default=11.5167, help="Simulated target longitude in degrees")
    parser.add_argument("--alt", type=float, default=545.4, help="Simulated target altitude in meters")
    parser.add_argument("--duration", type=int, default=30, help="Simulation duration in seconds")
    parser.add_argument("--sampling-freq", type=int, default=4096000, help="Sampling frequency in Hz")
    parser.add_argument("--port", type=int, default=8050, help="Dashboard HTTP port")
    return parser.parse_args()

def main():
    args = parse_args()
    project_root = os.path.abspath(os.path.dirname(__file__))

    sim_binary = os.path.join(project_root, "Tools", "gps-sdr-sim", "gps-sdr-sim")
    receiver_binary = os.path.join(project_root, "build", "Source", "GPSOpenCl")
    nav_file = os.path.join(project_root, "Tools", "gps-sdr-sim", "brdc0010.22n")
    sim_output_bin = os.path.join(project_root, "build", "live_stream.bin")
    dashboard_script = os.path.join(project_root, "Tools", "dashboard.py")

    print("=========================================================")
    print("   GPSOpenCl Master Live Simulation & Visual Analytics   ")
    print("=========================================================")

    # Step 1: Check & compile binaries
    if not os.path.exists(sim_binary):
        print(f"Building gps-sdr-sim...")
        subprocess.run(["gcc", "-O3", "Tools/gps-sdr-sim/gpssim.c", "-lm", "-o", sim_binary], check=True, cwd=project_root)

    print(f"Compiling GPSOpenCl software receiver...")
    subprocess.run(["cmake", "-S", ".", "-B", "build"], check=True, cwd=project_root)
    subprocess.run(["cmake", "--build", "build"], check=True, cwd=project_root)

    # Step 2: Generate Simulated RF Signal Data
    print(f"\n[1/3] Generating {args.duration}s live RF signal stream with gps-sdr-sim...")
    sim_cmd = [
        sim_binary,
        "-e", nav_file,
        "-l", f"{args.lat},{args.lon},{args.alt}",
        "-s", str(args.sampling_freq),
        "-b", "8",
        "-d", str(args.duration),
        "-o", sim_output_bin
    ]
    subprocess.run(sim_cmd, check=True)
    print(f"-> Signal stream ({os.path.getsize(sim_output_bin)} bytes) ready.")

    # Step 3: Launch Dashboard First
    print(f"\n[2/3] Launching Real-Time Plotly/Dash Web Dashboard on http://localhost:{args.port} ...")
    dashboard_proc = subprocess.Popen([sys.executable, dashboard_script, str(args.port)], cwd=project_root)
    time.sleep(2.0)

    # Step 4: Run GPSOpenCl Receiver Streaming Pipeline Concurrently
    print(f"\n[3/3] Executing GPSOpenCl Software Receiver Live Pipeline...")
    receiver_proc = subprocess.Popen([receiver_binary, sim_output_bin], cwd=project_root)

    print("\n=========================================================")
    print("   GPSOpenCl Software Receiver System is Live & Online   ")
    print("=========================================================")
    print(f"Dashboard URL : http://localhost:{args.port}")
    print(f"Telemetry File: {os.path.join(project_root, 'telemetry_stream.json')}")
    print("Press Ctrl+C to stop the system.\n")

    try:
        receiver_proc.wait()
        dashboard_proc.wait()
    except KeyboardInterrupt:
        print("\nStopping receiver and dashboard...")
        receiver_proc.terminate()
        dashboard_proc.terminate()

if __name__ == "__main__":
    main()
