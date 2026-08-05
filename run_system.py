#!/usr/bin/env python3
# Live demo launcher: builds the receiver, streams a simulated signal over the FIFO, and serves the dashboard
import argparse
import os
import subprocess
import sys
import time

FIFO_DIR = "/tmp/gpsopencl"
DATA_FIFO = os.path.join(FIFO_DIR, "sim_data.fifo")


def parse_args():
    parser = argparse.ArgumentParser(description="GPSOpenCl live demo: simulator -> FIFO -> receiver -> dashboard")
    parser.add_argument("--lat", type=float, default=48.1173, help="Simulated latitude in degrees")
    parser.add_argument("--lon", type=float, default=11.5167, help="Simulated longitude in degrees")
    parser.add_argument("--alt", type=float, default=545.4, help="Simulated altitude in meters")
    parser.add_argument("--port", type=int, default=8050, help="Dashboard web server port")
    parser.add_argument("--duration", type=int, default=300, help="Simulation length in seconds")
    parser.add_argument("--sampling-freq", type=int, default=4096000, help="Sampling frequency in Hz")
    parser.add_argument("--nav-file", type=str, default="Tools/gps-sdr-sim/brdc0010.22n", help="RINEX navigation file")
    parser.add_argument("--skip-build", action="store_true", help="Skip the cmake build step")
    return parser.parse_args()


def build_project(project_root):
    build_dir = os.path.join(project_root, "build")
    if not os.path.exists(os.path.join(build_dir, "CMakeCache.txt")):
        print("[run_system] Configuring build directory...")
        subprocess.run(["cmake", "-S", project_root, "-B", build_dir, "-DCMAKE_BUILD_TYPE=Release"], check=True)
    print("[run_system] Building...")
    subprocess.run(["cmake", "--build", build_dir, "-j"], check=True)


def find_sim_binary(project_root):
    candidates = [
        os.path.join(project_root, "build", "Tools", "gps-sdr-sim", "gps-sdr-sim"),
        os.path.join(project_root, "Tools", "gps-sdr-sim", "gps-sdr-sim"),
    ]
    for path in candidates:
        if os.path.exists(path):
            return path
    return None


def terminate(processes):
    for name, proc in reversed(processes):
        if proc.poll() is None:
            print(f"[run_system] Stopping {name}...")
            proc.terminate()
    deadline = time.time() + 5.0
    for name, proc in processes:
        remaining = max(0.1, deadline - time.time())
        try:
            proc.wait(timeout=remaining)
        except subprocess.TimeoutExpired:
            proc.kill()


def main():
    args = parse_args()
    project_root = os.path.abspath(os.path.dirname(__file__))

    if not args.skip_build:
        build_project(project_root)

    sim_binary = find_sim_binary(project_root)
    receiver_binary = os.path.join(project_root, "build", "Source", "GPSOpenCl")
    nav_file = os.path.abspath(os.path.join(project_root, args.nav_file))
    dashboard_script = os.path.join(project_root, "Tools", "dashboard.py")

    if sim_binary is None:
        print("Error: gps-sdr-sim binary not found; build the project first.", file=sys.stderr)
        return 1
    if not os.path.exists(receiver_binary):
        print(f"Error: receiver binary not found at {receiver_binary}.", file=sys.stderr)
        return 1
    if not os.path.exists(nav_file):
        print(f"Error: RINEX navigation file not found at {nav_file}.", file=sys.stderr)
        return 1

    os.makedirs(FIFO_DIR, exist_ok=True)

    # Remove the previous run's wire log so the dashboard does not replay stale telemetry
    wire_log = os.path.join(project_root, "build", "telemetry_wire.log")
    if os.path.exists(wire_log):
        os.remove(wire_log)

    print("[run_system] Scenario: lat %.4f, lon %.4f, alt %.1f m, %d s" % (args.lat, args.lon, args.alt, args.duration))

    processes = []
    try:
        sim_cmd = [
            sim_binary,
            "-e", nav_file,
            "-l", f"{args.lat},{args.lon},{args.alt}",
            "-s", str(args.sampling_freq),
            "-b", "8",
            "-d", str(args.duration),
            "-o", DATA_FIFO,
        ]
        print("[run_system] Starting gps-sdr-sim (real-time FIFO streaming)...")
        processes.append(("gps-sdr-sim", subprocess.Popen(sim_cmd, cwd=project_root)))

        print("[run_system] Starting GPSOpenCl receiver...")
        processes.append(("receiver", subprocess.Popen([receiver_binary, DATA_FIFO], cwd=project_root)))

        print("[run_system] Starting dashboard...")
        dash_cmd = [sys.executable, dashboard_script, "--port", str(args.port)]
        processes.append(("dashboard", subprocess.Popen(dash_cmd, cwd=project_root)))

        print(f"[run_system] Dashboard: http://localhost:{args.port}  (Ctrl+C to stop)")

        while True:
            time.sleep(1.0)
            for name, proc in processes[:2]:
                if proc.poll() is not None:
                    print(f"[run_system] {name} finished (exit {proc.returncode}); shutting down.")
                    terminate(processes)
                    return 0 if proc.returncode == 0 else proc.returncode
    except KeyboardInterrupt:
        print("\n[run_system] Interrupted; shutting down.")
        terminate(processes)
        return 0
    except Exception as exc:
        print(f"[run_system] Error: {exc}", file=sys.stderr)
        terminate(processes)
        return 1


if __name__ == "__main__":
    sys.exit(main())
