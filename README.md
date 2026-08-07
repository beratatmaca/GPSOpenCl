# GPSOpenCl - OpenCL Accelerated GPS L1 C/A Software Receiver

[![Build & Test Status](https://github.com/beratatmaca/GPSOpenCl/actions/workflows/debug.yml/badge.svg?branch=main)](https://github.com/beratatmaca/GPSOpenCl/actions/workflows/debug.yml)

**GPSOpenCl** is an OpenCL-accelerated GPS L1 C/A GNSS software-defined receiver (SDR) written in modern C++. It provides an end-to-end processing pipeline from raw digitized RF baseband signals to satellite acquisition, code/carrier tracking, navigation message bit decoding, 3D PVT position solving, atmospheric delay corrections, NMEA-0183 output generation, and interactive web visual analytics.

If no OpenCL-compatible GPU platform or vendor driver is detected on the host system, **GPSOpenCl** automatically falls back to standard C++ CPU compute routines (Radix-2 Cooley-Tukey FFT/IFFT, complex multiplication, NCO mixing, vector reduction) to guarantee seamless execution across all systems.

---

## Roadmap

This README describes the project as it exists today. GPSOpenCl is moving toward a hardware-deployable, real-time architecture built on an abstract `Source`/`Sink` contract, single-source-of-truth binary structs, a ZMQ publisher, and a dedicated profiler module.

- See [`CLAUDE.md`](CLAUDE.md) for the current vs. target architecture.

---

## Key Features & Architecture

A conceptual walkthrough of the receiver (signal path, thread topology, cold-start timeline) lives in [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md).

```
┌────────────────────────────────────────────────────────────────────────┐
│             Real-Time NMEA-0183 Output Stream ($GPGGA, $GPRMC)         │
└───────────────────────────────────┬────────────────────────────────────┘
                                    │
┌───────────────────────────────────▼────────────────────────────────────┐
│      PVT Receiver Position Solver (WLS Gauss-Jordan, DOP, WGS-84)     │
└─────────┬─────────────────────────┬──────────────────────────┬─────────┘
          │                         │                          │
┌─────────▼─────────────┐ ┌─────────▼──────────────┐ ┌─────────▼─────────┐
│ Keplerian Orbit ECEF  │ │ Atmospheric Delays     │ │ Navigation Bit    │
│ & Clock Corrections   │ │ (Klobuchar/Saastamoinen│ │ Subframes 1-5     │
└───────────────────────┘ └────────────────────────┘ └─────────▲─────────┘
                                                               │
┌──────────────────────────────────────────────────────────────┴─────────┐
│    Satellite Tracking Engine (0.5-Chip E/P/L, 2nd-Order PLL/DLL)      │
└───────────────────────────────────┬────────────────────────────────────┘
                                    │
┌───────────────────────────────────▼────────────────────────────────────┐
│    Satellite Acquisition Engine (Doppler Search, FFT Correlation)     │
└───────────────────────────────────┬────────────────────────────────────┘
                                    │
┌───────────────────────────────────▼────────────────────────────────────┐
│  GPU Compute & CPU Fallback (OpenCL Kernels / Radix-2 Cooley-Tukey)    │
└────────────────────────────────────────────────────────────────────────┘
```

- **OpenCL GPU Compute & CPU Fallbacks**: Parallelized kernel execution for FFT, IFFT, complex multiplication, NCO mixing, and magnitude computation, backed by CPU software fallback algorithms.
- **Satellite Acquisition Engine**: Parallel Doppler search grid ($\pm 4\text{ kHz}$) with frequency-domain circular cross-correlation and Carrier-to-Noise Ratio ($C/N_0$) peak estimation.
- **Satellite Tracking Engine**: 0.5-chip Early/Prompt/Late code replica generator with 2nd-order Phase-Locked Loop (PLL) and Delay-Locked Loop (DLL) discriminators & loop filters.
- **Navigation Preamble & Message Decoder**: Telemetry (TLM) preamble detection (`0x8B`/`0x74`), IS-GPS-200 30-bit Hamming parity verification, and Subframe 1–5 decoding, including Subframe 1–3 ephemeris parsing and Subframe 4 page 18 Klobuchar ionospheric parameters.
- **PVT Position & Orbit Solver**: Keplerian 3D Earth-Centered Earth-Fixed (ECEF) satellite orbit calculator, relativistic clock bias correction, Sagnac Earth rotation compensation, Gauss-Jordan Weighted Least Squares (WLS) receiver position solver, Dilution of Precision (GDOP, PDOP, HDOP, VDOP) matrix computation, and Geodetic WGS-84 (Latitude, Longitude, Altitude) conversion.
- **Atmospheric Delay Corrections**: Klobuchar Ionospheric model (Subframe 4 parameters) and Saastamoinen Tropospheric delay model.
- **NMEA-0183 Output Engine**: Standard `$GPGGA`, `$GPRMC`, `$GPGSA`, and `$GPGSV` sentence generation with 8-bit XOR checksums.
- **Interactive Plotly/Dash Dashboard**: Live web analytics interface displaying satellite skyplots, $C/N_0$ signal bars, Doppler shifts, WGS-84 location, and NMEA log streams.
- **Simulation & LLM Benchmarking Suite**: Powered by `gps-sdr-sim` for end-to-end scenario generation, throughput measurements ($\text{MSamples/s}$), and machine-parsable JSON performance logging.

---

## Build & Installation

### Prerequisites (Ubuntu/Debian)

```bash
sudo apt-get update
sudo apt-get install -y build-essential cmake git python3 python3-pip
```

*(Optional for GPU acceleration)*:
```bash
sudo apt-get install -y libopencl-dev opencl-headers ocl-icd-opencl-dev
```

### Compiling the Project

```bash
# Clone the repository
git clone https://github.com/beratatmaca/GPSOpenCl.git
cd GPSOpenCl

# Configure and build
cmake -S . -B ./build
cmake --build ./build
```

---

## Running the Application & Benchmark Suite

### 1. Live Interactive Web Dashboard & Simulation (Recommended)

Run the master real-time launcher to stream a simulated GPS signal over a Linux FIFO
(via the `gps-sdr-sim-rt` fork), process it through the software receiver in real time,
and launch the web dashboard:

```bash
python3 run_system.py
```

Then open your browser and navigate to:
👉 **`http://localhost:8050`**

#### Customizing Coordinates & Port:
```bash
python3 run_system.py --lat 40.7128 --lon -74.0060 --alt 10.0 --port 8060
```

---

### 2. End-to-End Automated Benchmark Suite (For LLMs & Profiling)

To run automated performance measurements and write machine-parsable JSON and Markdown reports:

```bash
python3 Tools/e2e_benchmark.py --duration 5
```

Output files generated:
- **`e2e_benchmark_report.json`**: Structured performance metrics and satellite acquisition details for AI/LLM monitoring.
- **`e2e_benchmark_report.md`**: Human-readable Markdown summary.

---

### 3. Running Unit & Integration Test Suite

The test suite has three tiers:

- **Unit tests** (default, ~15 s): 118 GoogleTest cases covering acquisition, tracking loops, nav decode, PVT math, queues, sources, and sinks.
- **Ground-truth integration tests** (gated, minutes): stream a full gps-sdr-sim scenario through the receiver and assert ephemeris, pseudorange, and fix accuracy against simulator truth. Enable with `GPSOPENCL_RUN_INTEGRATION_TESTS=1`.
- **Benchmark** (`Tools/e2e_benchmark.py`): measures throughput and position error; run it after any algorithm change.

```bash
ctest --test-dir build --output-on-failure
GPSOPENCL_RUN_INTEGRATION_TESTS=1 ctest --test-dir build -R GroundTruth --output-on-failure
```

---

### 4. Running Standalone Receiver Executable

```bash
./build/Source/GPSOpenCl [signal_file]
```

The first argument is the path to the input signal. A path containing `.fifo` selects the gps-sdr-sim streaming source; anything else is read as a file. Without an argument, the receiver probes `build/live_stream.bin`, `../build/live_stream.bin`, and `live_stream.bin`, then falls back to `inputSignal.txt` under `Tests/Scripts/` (see `Source/Main.cpp`).

#### Configuration

At startup the receiver loads `DefaultConf.ini`, searched in the working directory, `build/Source/`, and `Tests/Scripts/ConfigurationFile/`. If no file is found, built-in defaults apply. Recognized keys (see `Source/GPSOpenClSettings.cpp`):

| Key | Default | Unit / Meaning |
| --- | --- | --- |
| `DataSource` | `capture.dat` | Signal file or FIFO path. A `.fifo` path enables the gps-sdr-sim streaming source. |
| `SamplingFrequency` | `4096000` | Hz. Must yield a power-of-two samples per 1 ms code period. |
| `AcquisitionMinimumDoppler` | `-4000` | Hz. Lower edge of the Doppler search window. |
| `AcquisitionMaximumDoppler` | `4000` | Hz. Upper edge of the Doppler search window. |
| `AcquisitionDopplerSearchRange` | `500` | Hz. Doppler bin step. |
| `AcquisitionCn0Threshold` | `43.0` | dB-Hz. Minimum C/N0 to declare a satellite acquired. |
| `TrackingTelemetryIntervalBlocks` | `10` | Blocks between TrackingOutput publishes per channel. |
| `PllBandwidthHz` | `25.0` | Hz. PLL noise bandwidth. |
| `DllBandwidthHz` | `2.0` | Hz. DLL noise bandwidth. |
| `FllBandwidthHz` | `10.0` | Hz. FLL pull-in noise bandwidth. |
| `RateAidBandwidthHz` | `1.0` | Hz. Continuous Doppler-rate-aiding bandwidth. |
| `FllPullInBlocks` | `75` | Blocks of FLL-assisted pull-in before the PLL takes over. |
| `FixOutputIntervalBlocks` | `100` | Blocks between PVT solves and telemetry output. |
| `PvtTropoEnabled` | `1` | 0/1. Saastamoinen troposphere correction. Keep 0 for simulated signals. |
| `PvtElevationMaskDeg` | `0.0` | Degrees. Elevation mask once a fix exists. 0 disables. |
| `ProfilerEnabled` | `1` | 0/1. Per-stage timing telemetry. |

---

## OpenCL GPU vs. CPU Software Fallback

When running `GPSOpenCl`, you may notice the following log message:
```text
Couldn't identify a platform
```

### What This Means:
- OpenCL queries the system for GPU vendor drivers (NVIDIA CUDA, Intel OpenCL NEO, AMD ROCm, or POCL).
- If no GPU OpenCL driver is installed on the host system, OpenCL returns error code `-1001` (`CL_PLATFORM_NOT_FOUND_KHR`).

### Automatic Fallback Behavior:
- **GPSOpenCl** catches this condition and automatically switches to CPU software compute mode.
- All algorithms (Cooley-Tukey Radix-2 FFT, complex multiplication, NCO mixing, vector reduction) run seamlessly on the CPU.

### Enabling GPU Hardware Acceleration:
To enable hardware GPU acceleration, install the appropriate OpenCL driver for your system:
- **NVIDIA GPU**: `sudo apt install nvidia-cuda-toolkit`
- **Intel iGPU**: `sudo apt install intel-opencl-icd`
- **AMD GPU**: `sudo apt install rocm-opencl-runtime`
- **Generic CPU OpenCL**: `sudo apt install pocl-opencl-icd`

---

## License

This project is licensed under the terms of the GPL-3.0 License.
