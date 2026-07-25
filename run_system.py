#!/usr/bin/env python3
import argparse
import os
import signal
import subprocess
import sys
import time
import webbrowser

PROJECT_ROOT = os.path.dirname(os.path.abspath(__file__))
FIFO_DATA_PATH = "/tmp/gpsopencl/sim_data.fifo"
FIFO_CTRL_PATH = "/tmp/gpsopencl/sim_ctrl.fifo"
DEFAULT_NAV_FILE = os.path.join(PROJECT_ROOT, "Tools", "gps-sdr-sim", "brdc0010.22n")
BUILD_DIR = os.path.join(PROJECT_ROOT, "build")
RECEIVER_BIN = os.path.join(BUILD_DIR, "Source", "GPSOpenCl")
SIMULATOR_BIN = os.path.join(BUILD_DIR, "Tools", "gps-sdr-sim", "gps-sdr-sim")
DASHBOARD_SCRIPT = os.path.join(PROJECT_ROOT, "Tools", "dashboard.py")

processes = []

def cleanup(signum=None, frame=None):
    print("\n\n=========================================================")
    print("      Shutting down GPSOpenCl Real-Time Pipeline...     ")
    print("=========================================================")
    for proc in reversed(processes):
        if proc.poll() is None:
            try:
                proc.terminate()
                proc.wait(timeout=2.0)
            except Exception:
                proc.kill()
    
    for fifo in [FIFO_DATA_PATH, FIFO_CTRL_PATH]:
        if os.path.exists(fifo):
            try:
                os.remove(fifo)
            except Exception:
                pass
    print("All processes terminated. System shutdown complete.\n")
    sys.exit(0)

def ensure_build():
    if os.path.exists(RECEIVER_BIN) and os.path.exists(SIMULATOR_BIN):
        return

    print("[Build] Compiling GPSOpenCl software receiver and gps-sdr-sim-rt...")
    res = subprocess.run(
        ["cmake", "-B", "build", "-DGPSOPENCL_ENABLE_ZMQ=ON"],
        cwd=PROJECT_ROOT
    )
    if res.returncode != 0:
        print("[Build Error] CMake configuration failed.")
        sys.exit(1)

    res = subprocess.run(
        ["cmake", "--build", "build"],
        cwd=PROJECT_ROOT
    )
    if res.returncode != 0:
        print("[Build Error] CMake build failed.")
        sys.exit(1)

def main():
    parser = argparse.ArgumentParser(description="GPSOpenCl Continuous Real-Time System Launcher")
    parser.add_argument("--port", type=int, default=8050, help="Dashboard HTTP web port (default: 8050)")
    parser.add_argument("--lat", type=float, default=48.1173, help="Scenario Latitude (deg)")
    parser.add_argument("--lon", type=float, default=11.5167, help="Scenario Longitude (deg)")
    parser.add_argument("--alt", type=float, default=545.4, help="Scenario Altitude (m)")
    parser.add_argument("--sampling-freq", type=int, default=4096000, help="Sampling frequency in Hz (default: 4096000)")
    parser.add_argument("--no-browser", action="store_true", help="Do not automatically open web browser")
    args = parser.parse_args()

    signal.signal(signal.SIGINT, cleanup)
    signal.signal(signal.SIGTERM, cleanup)

    print("=========================================================")
    print("   GPSOpenCl Continuous Real-Time Software Receiver     ")
    print("=========================================================")

    # 1. Build Verification
    ensure_build()

    # 2. Setup Linux FIFOs
    os.makedirs("/tmp/gpsopencl", exist_ok=True)
    for fifo in [FIFO_DATA_PATH, FIFO_CTRL_PATH]:
        if os.path.exists(fifo):
            os.remove(fifo)
        os.mkfifo(fifo)

    # Launch real-time signal generator, streaming continuously over the FIFO
    print(f"[1/3] Starting real-time RF signal generator stream over FIFO ({FIFO_DATA_PATH})...")
    sim_cmd = [
        SIMULATOR_BIN,
        "-e", DEFAULT_NAV_FILE,
        "-l", f"{args.lat},{args.lon},{args.alt}",
        "-s", str(args.sampling_freq),
        "-b", "8",
        "-d", "86400",
        "-o", FIFO_DATA_PATH,
        "-C", FIFO_CTRL_PATH,
        "-R"
    ]
    sim_proc = subprocess.Popen(sim_cmd, cwd=PROJECT_ROOT)
    processes.append(sim_proc)

    # 4. Launch UI & Visual Analytics — Plotly/Dash Dashboard
    dashboard_url = f"http://localhost:{args.port}"
    print(f"[2/3] Launching Visual Analytics Dashboard on {dashboard_url} ...")
    dash_cmd = [
        sys.executable, DASHBOARD_SCRIPT,
        "--port", str(args.port),
        "--endpoint", "ipc:///tmp/gpsopencl/telemetry.sock"
    ]
    dash_proc = subprocess.Popen(dash_cmd, cwd=PROJECT_ROOT)
    processes.append(dash_proc)
    time.sleep(1.5)

    # Automatically open browser if requested
    if not args.no_browser:
        print(f"[UI] Opening web browser at {dashboard_url} ...")
        webbrowser.open(dashboard_url)

    # 5. Launch Executable 2 — OpenCL Software Receiver (GPSOpenCl)
    print(f"[3/3] Executing GPSOpenCl Receiver Pipeline on Real-Time FIFO Stream...")
    rx_cmd = [RECEIVER_BIN, FIFO_DATA_PATH]
    rx_proc = subprocess.Popen(rx_cmd, cwd=PROJECT_ROOT)
    processes.append(rx_proc)

    print("\n=========================================================")
    print("   GPSOpenCl Software Receiver Pipeline is ONLINE       ")
    print("=========================================================")
    print(f" Dashboard URL : {dashboard_url}")
    print(f" Data FIFO     : {FIFO_DATA_PATH}")
    print(f" Control FIFO  : {FIFO_CTRL_PATH}")
    print(" Press Ctrl+C to stop the system.\n")

    rx_proc.wait()

if __name__ == "__main__":
    main()
