#!/usr/bin/env python3
import argparse
import json
import math
import os
import subprocess
import sys
import time

# WGS-84 geodetic -> ECEF, mirrors GPSOpenCl::PVTSolver::ecefToWgs84's inverse, for scoring the scenario's true position.
def geodetic_to_ecef(lat_deg, lon_deg, alt_m):
    a = 6378137.0
    f = 1.0 / 298.257223563
    e2 = f * (2.0 - f)
    lat_rad = math.radians(lat_deg)
    lon_rad = math.radians(lon_deg)
    sin_lat = math.sin(lat_rad)
    cos_lat = math.cos(lat_rad)
    n = a / math.sqrt(1.0 - e2 * sin_lat * sin_lat)
    x = (n + alt_m) * cos_lat * math.cos(lon_rad)
    y = (n + alt_m) * cos_lat * math.sin(lon_rad)
    z = (n * (1.0 - e2) + alt_m) * sin_lat
    return x, y, z

def parse_args():
    parser = argparse.ArgumentParser(description="End-to-End GPS Receiver Benchmark & Profiler Harness")
    parser.add_argument("--lat", type=float, default=48.1173, help="Simulated target latitude in degrees")
    parser.add_argument("--lon", type=float, default=11.5167, help="Simulated target longitude in degrees")
    parser.add_argument("--alt", type=float, default=545.4, help="Simulated target altitude in meters")
    parser.add_argument("--duration", type=int, default=2, help="Simulation duration in seconds")
    parser.add_argument("--sampling-freq", type=int, default=4096000, help="Sampling frequency in Hz")
    parser.add_argument("--bit-depth", type=int, default=8, help="IQ bit depth (8 or 16)")
    parser.add_argument("--nav-file", type=str, default="Tools/gps-sdr-sim/brdc0010.22n", help="RINEX navigation file path")
    parser.add_argument("--output-json", type=str, default="e2e_benchmark_report.json", help="Path to write LLM-parsable JSON report")
    parser.add_argument("--output-md", type=str, default="e2e_benchmark_report.md", help="Path to write Markdown report")
    parser.add_argument("--allow-debug-build", action="store_true", help="Skip the Release-build check (numbers will not reflect real-time performance)")
    parser.add_argument("--build-dir", type=str, default="build", help="CMake build directory holding the receiver binary")
    return parser.parse_args()

# Detects a Debug/_GLIBCXX_DEBUG binary, which is 5-50x slower than Release and would
# silently produce misleading real-time-speedup numbers.
def check_release_build(project_root, receiver_binary, build_dir):
    cache_path = os.path.join(project_root, build_dir, "CMakeCache.txt")
    build_type = None
    if os.path.exists(cache_path):
        with open(cache_path) as f:
            for line in f:
                if line.startswith("CMAKE_BUILD_TYPE:"):
                    build_type = line.strip().split("=", 1)[-1]
                    break

    debug_symbol_present = False
    with open(receiver_binary, "rb") as f:
        debug_symbol_present = b"__glibcxx_assert_fail" in f.read()

    if debug_symbol_present or (build_type and build_type != "Release"):
        print("=========================================================", file=sys.stderr)
        print("WARNING: build/Source/GPSOpenCl does not look like a Release build.", file=sys.stderr)
        if build_type:
            print(f"  CMAKE_BUILD_TYPE cache entry: '{build_type}'", file=sys.stderr)
        if debug_symbol_present:
            print("  Binary contains __glibcxx_assert_fail (libstdc++ debug-mode instrumentation).", file=sys.stderr)
        print("  Benchmark numbers from a Debug/_GLIBCXX_DEBUG binary understate real-time", file=sys.stderr)
        print("  performance by 5-50x and must not be used to judge algorithm speed.", file=sys.stderr)
        print("  Rebuild with: cmake -S . -B build -DCMAKE_BUILD_TYPE=Release && cmake --build build", file=sys.stderr)
        print("=========================================================", file=sys.stderr)
        sys.exit(1)

def main():
    args = parse_args()
    project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
    
    sim_binary = os.path.join(project_root, "build", "Tools", "gps-sdr-sim", "gps-sdr-sim")
    if not os.path.exists(sim_binary):
        sim_binary = os.path.join(project_root, "Tools", "gps-sdr-sim", "gps-sdr-sim")

    receiver_binary = os.path.join(project_root, args.build_dir, "Source", "GPSOpenCl")
    nav_file = os.path.abspath(os.path.join(project_root, args.nav_file))
    sim_output_bin = os.path.join(project_root, args.build_dir, "simulated_benchmark.bin")

    if not os.path.exists(sim_binary):
        print(f"Error: gps-sdr-sim binary not found at {sim_binary}", file=sys.stderr)
        sys.exit(1)

    if not os.path.exists(receiver_binary):
        print(f"Error: GPSOpenCl binary not found at {receiver_binary}. Please build the project first.", file=sys.stderr)
        sys.exit(1)

    if not args.allow_debug_build:
        check_release_build(project_root, receiver_binary, args.build_dir)

    print("=========================================================")
    print("   GPSOpenCl End-to-End Simulation & Profiling Suite     ")
    print("=========================================================")
    print(f"Scenario Location : Lat {args.lat:.4f} deg, Lon {args.lon:.4f} deg, Alt {args.alt:.1f} m")
    print(f"Sampling Frequency: {args.sampling_freq} Hz ({args.bit_depth}-bit IQ)")
    print(f"Simulation Length : {args.duration} seconds")

    # Step 1: Run gps-sdr-sim
    sim_cmd = [
        sim_binary,
        "-e", nav_file,
        "-l", f"{args.lat},{args.lon},{args.alt}",
        "-s", str(args.sampling_freq),
        "-b", str(args.bit_depth),
        "-d", str(args.duration),
        "-o", sim_output_bin
    ]

    print("\n[Step 1/2] Generating simulated GPS L1 C/A RF signal with gps-sdr-sim...")
    t_sim_start = time.time()
    sim_proc = subprocess.run(sim_cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    t_sim_end = time.time()
    sim_wall_time = t_sim_end - t_sim_start

    if sim_proc.returncode != 0:
        print(f"gps-sdr-sim failed: {sim_proc.stderr}", file=sys.stderr)
        sys.exit(1)

    print(f"-> Generated simulated binary signal ({os.path.getsize(sim_output_bin)} bytes) in {sim_wall_time:.2f} seconds.")

    # Step 2: Run GPSOpenCl Software Receiver
    print("\n[Step 2/2] Running GPSOpenCl Software Receiver Pipeline...")
    telemetry_path = os.path.join(project_root, "telemetry_stream.json")
    if os.path.exists(telemetry_path):
        os.remove(telemetry_path)

    t_rx_start = time.time()
    rx_proc = subprocess.run([receiver_binary, sim_output_bin], stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, cwd=project_root)
    t_rx_end = time.time()
    rx_wall_time = t_rx_end - t_rx_start

    rx_stdout = rx_proc.stdout

    # No PVT fix within short benchmark runs is reported explicitly, never silently omitted as zero error.
    true_ecef_x, true_ecef_y, true_ecef_z = geodetic_to_ecef(args.lat, args.lon, args.alt)
    accuracy_profile = {
        "pvt_fix_achieved": False,
        "reason": f"No PVT fix within {args.duration}s -- ephemeris decode and 4-satellite PVT typically need tens of seconds",
        "true_ecef_x": true_ecef_x,
        "true_ecef_y": true_ecef_y,
        "true_ecef_z": true_ecef_z
    }
    if os.path.exists(telemetry_path):
        try:
            with open(telemetry_path) as f:
                telemetry = json.load(f)
            pvt = telemetry.get("pvt", {})
            if pvt.get("valid"):
                dx = pvt["ecef_x"] - true_ecef_x
                dy = pvt["ecef_y"] - true_ecef_y
                dz = pvt["ecef_z"] - true_ecef_z
                position_error_m = math.sqrt(dx * dx + dy * dy + dz * dz)
                accuracy_profile = {
                    "pvt_fix_achieved": True,
                    "position_error_meters": round(position_error_m, 3),
                    "measured_latitude": pvt["latitude"],
                    "measured_longitude": pvt["longitude"],
                    "measured_altitude_meters": pvt["altitude"],
                    "hdop": pvt.get("hdop"),
                    "pdop": pvt.get("pdop"),
                    "vdop": pvt.get("vdop"),
                    "true_ecef_x": true_ecef_x,
                    "true_ecef_y": true_ecef_y,
                    "true_ecef_z": true_ecef_z
                }
        except (json.JSONDecodeError, KeyError) as e:
            print(f"Warning: failed to parse {telemetry_path}: {e}", file=sys.stderr)
        os.remove(telemetry_path)

    # Parse receiver metrics
    acquired_satellites = []
    for line in rx_stdout.splitlines():
        if "ACQUIRED!" in line:
            parts = line.strip().split()
            # Example line: --> SV ID 1 ACQUIRED! (C/N0: 38.5926 dB-Hz, Doppler: 3500 Hz)
            sv_id = int(parts[3])
            cn0 = float(parts[6])
            doppler = float(parts[9])
            acquired_satellites.append({
                "sv_id": sv_id,
                "cn0_db_hz": cn0,
                "doppler_hz": doppler
            })

    total_samples = args.duration * args.sampling_freq
    throughput_m_samples_sec = (total_samples / 1e6) / rx_wall_time if rx_wall_time > 0 else 0
    realtime_speedup_factor = args.duration / rx_wall_time if rx_wall_time > 0 else 0

    report_data = {
        "timestamp": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
        "scenario": {
            "target_lat": args.lat,
            "target_lon": args.lon,
            "target_alt_meters": args.alt,
            "duration_sec": args.duration,
            "sampling_frequency_hz": args.sampling_freq,
            "bit_depth": args.bit_depth
        },
        "performance_profile": {
            "signal_gen_wall_time_sec": round(sim_wall_time, 3),
            "receiver_processing_wall_time_sec": round(rx_wall_time, 3),
            "throughput_m_samples_per_sec": round(throughput_m_samples_sec, 2),
            "realtime_speedup_factor": round(realtime_speedup_factor, 2)
        },
        "acquisition_metrics": {
            "acquired_count": len(acquired_satellites),
            "acquired_satellites": acquired_satellites
        },
        "accuracy_profile": accuracy_profile,
        "receiver_status": "SUCCESS" if rx_proc.returncode == 0 else "FAILURE"
    }

    # Write JSON report
    with open(args.output_json, "w") as f:
        json.dump(report_data, f, indent=2)

    # Write Markdown report
    with open(args.output_md, "w") as f:
        f.write(f"# GPSOpenCl End-to-End Simulation & Profiling Report\n\n")
        f.write(f"- **Timestamp**: {report_data['timestamp']}\n")
        f.write(f"- **Status**: `{report_data['receiver_status']}`\n")
        f.write(f"- **Target Location**: Lat `{args.lat}`°, Lon `{args.lon}`°, Alt `{args.alt}` m\n\n")
        f.write(f"## Performance Profile\n\n")
        f.write(f"| Metric | Value |\n")
        f.write(f"| --- | --- |\n")
        f.write(f"| Signal Generation Time | `{sim_wall_time:.2f} s` |\n")
        f.write(f"| Receiver Execution Time | `{rx_wall_time:.2f} s` |\n")
        f.write(f"| Throughput | `{throughput_m_samples_sec:.2f} MSamples/s` |\n")
        f.write(f"| Real-Time Speedup | `{realtime_speedup_factor:.2f}x` |\n\n")
        f.write(f"## Acquired Satellite Channels ({len(acquired_satellites)} Satellites)\n\n")
        f.write(f"| SV ID | C/N0 (dB-Hz) | Doppler Shift (Hz) |\n")
        f.write(f"| --- | --- | --- |\n")
        for sat in acquired_satellites:
            f.write(f"| PRN {sat['sv_id']:02d} | `{sat['cn0_db_hz']:.2f}` | `{sat['doppler_hz']:.1f}` |\n")

        f.write(f"\n## Position Accuracy\n\n")
        if accuracy_profile["pvt_fix_achieved"]:
            f.write(f"| Metric | Value |\n")
            f.write(f"| --- | --- |\n")
            f.write(f"| Position Error (3D, vs. known truth) | `{accuracy_profile['position_error_meters']:.2f} m` |\n")
            f.write(f"| HDOP | `{accuracy_profile['hdop']:.2f}` |\n")
            f.write(f"| PDOP | `{accuracy_profile['pdop']:.2f}` |\n")
            f.write(f"| VDOP | `{accuracy_profile['vdop']:.2f}` |\n")
        else:
            f.write(f"No PVT fix achieved: {accuracy_profile['reason']}. ")
            f.write(f"Re-run with a longer `--duration` (60s+) to get a position accuracy measurement.\n")

    print("\n=========================================================")
    print("   End-to-End Benchmark Completed Successfully           ")
    print("=========================================================")
    print(f"Receiver Processing Time : {rx_wall_time:.3f} s")
    print(f"Processing Throughput    : {throughput_m_samples_sec:.2f} MSamples/s ({realtime_speedup_factor:.2f}x Realtime)")
    print(f"Acquired Satellite Channels: {len(acquired_satellites)} / 32")
    if accuracy_profile["pvt_fix_achieved"]:
        print(f"Position Error (3D)      : {accuracy_profile['position_error_meters']:.2f} m "
              f"(HDOP {accuracy_profile['hdop']:.2f}, PDOP {accuracy_profile['pdop']:.2f})")
    else:
        print(f"Position Error           : N/A ({accuracy_profile['reason']})")
    print(f"JSON LLM Report Written  : {args.output_json}")
    print(f"Markdown Report Written  : {args.output_md}")

    # Clean up temporary bin file
    if os.path.exists(sim_output_bin):
        os.remove(sim_output_bin)

if __name__ == "__main__":
    main()
