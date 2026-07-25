# GPSOpenCl

OpenCL-accelerated GPS L1 C/A software receiver.

## Priorities

- Correctness first.
- Robustness over features.
- Profiling is required.
- Minimal dependencies.

## Current Architecture

Pipeline: compute -> acquisition -> tracking -> nav decode -> PVT -> NMEA.

- `Source/GPSOpenClGPUHandler.*`: OpenCL context, device, kernels.
- `Source/GPSOpenClGPUCompute.*`: FFT, mixing, GPU/CPU dispatch.
- `Source/GPSOpenClCode.*`: GPS C/A code generation.
- `Source/GPSOpenClAcquisition.*`: Doppler search, FFT correlation.
- `Source/GPSOpenClTracking.*`: PLL/DLL loops, E/P/L correlators.
- `Source/GPSOpenClChannel.*`: Per-satellite acquisition and tracking state.
- `Source/GPSOpenClNavigationDecoder.*`: Preamble search, parity, subframe parsing.
- `Source/GPSOpenClPVTSolver.*`: Satellite orbit calc, WLS position, DOP.
- `Source/GPSOpenClAtmosphericCorrections.*`: Klobuchar and Saastamoinen delay models.
- `Source/GPSOpenClNmeaGenerator.*`: NMEA-0183 sentence output.
- `Source/GPSOpenClSettings.*`: INI config parsing.
- `Source/GPSOpenClApplication.*`: Wires all modules together.
- `Source/GPSOpenClStructs.h`: Shared wire structs, single source of truth.
- `Source/GPSOpenClSource.h`, `GPSOpenClFileSource.*`, `GPSOpenClGpsSdrSimSource.*`: Abstract `Source`, plus file and gps-sdr-sim FIFO implementations.
- `Source/GPSOpenClSink.h`, `GPSOpenClFileSink.*`, `GPSOpenClZmqSink.*`: Abstract `Sink`, plus `NullSink`/`CompositeSink`, file, and ZMQ publisher implementations.
- `Source/GPSOpenClProfiler.*`: Per-block, per-stage timing, published through the Sink.
- `Source/GPSOpenClBoundedQueue.h`: Bounded blocking queue between producer and consumer threads.
- `Kernels/*.cl`: OpenCL kernels for FFT and NCO.

Data flow:

- `Main.cpp` reads from file or gps-sdr-sim FIFO.
- Producer thread (Source) and consumer thread (Application) linked by bounded queue.
- ZMQ publisher gated behind `GPSOPENCL_ENABLE_ZMQ`. FileSink and NullSink also exist.
- Profiler publishes per-block timing through Sink.
- Falls back to CPU if no OpenCL device.
- Struct-based wiring is incomplete. Verify call sites before assuming end-to-end connectivity.

## Target Architecture

Design direction, not yet fully built.

### Source abstraction

- Abstract `Source` base class. Concrete source reads gps-sdr-sim FIFO.
- Raw IQ samples stream as bytes, parsed into `ComplexFloatVector`.
- Only Source health telemetry goes through Sink.
- Designed for future hardware SDR plug-in (RTL-SDR, USRP, BladeRF).

### gps-sdr-sim real-time fork

- Fork at `beratatmaca/gps-sdr-sim-rt`, vendored in `Tools/gps-sdr-sim`.
- Streams samples over Linux FIFO: `/tmp/gpsopencl/sim_data.fifo`.
- Control FIFO: `/tmp/gpsopencl/sim_ctrl.fifo` (plain text: `START`, `STOP`, `SET_POS`).

### Module output contract

- Every module: one input struct (config), one output struct (telemetry).
- All structs in `Source/GPSOpenClStructs.h`. Single source of truth.
- Never mix config into telemetry or vice versa.

### Struct wire format

- First member: `uint32_t structVersion`.
- Fixed-width types only: `int32_t`, `uint32_t`, `double`. No `bool`, `size_t`, `int`.
- `#pragma pack(push, 1)`. Little-endian, no byte swap.
- Per-satellite telemetry: one message per SV, no arrays.
- Raw IQ samples are not wrapped in a struct.

### Publisher / Sink

- Abstract Sink. ZMQ PUB/SUB is default implementation.
- Endpoint: `ipc:///tmp/gpsopencl/<name>.sock`. TCP override allowed.
- Message: two frames (identifier + struct bytes).
- Fallback: NullSink or FileSink if ZMQ disabled.

### Profiler module

- Per-block, per-stage timing. Publishes through Sink.
- Must not distort the timing it measures.
- Disableable at compile time or runtime.

### Concurrency model

- Producer thread (Source) + consumer thread (Application) linked by bounded queue (8-16 blocks).
- Algorithm path backpressures. Sink path may drop.

### Visualization

- Visualizers are ZMQ subscribers, not core logic. Keep out of hot path.

### Deployment goal

- Linux only (x86_64, ARM64). Target: RPi 4/5, Jetson.
- GPU optional, CPU fallback mandatory.
- No dependency that blocks embedded deployment.

## Dependencies

- C++17 stdlib, OpenCL (GPU only), ZMQ (Sink only, gated by `GPSOPENCL_ENABLE_ZMQ`).
- FIFO for sim IPC, no library.
- Python tools may use additional packages.
- Justify any new dependency in PR.

## Build

```bash
cmake -S . -B build
cmake --build build
```

CMake 3.14+, C++17. OpenCL headers and GoogleTest auto-fetched.

## Test

```bash
ctest --test-dir build --output-on-failure
```

33 GoogleTest cases. Every new algorithm needs a test.

## Benchmarking

```bash
python3 Tools/e2e_benchmark.py --duration 5
```

Writes JSON and Markdown reports. Run after any algorithm change.

## Live Demo

```bash
python3 run_system.py
```

Builds, streams signal over FIFO, launches Dash dashboard on `localhost:8050`.

## Code Style

- `.clang-format` before committing.
- 4-space indent, Allman braces, 120 columns.
- Namespace: `GPSOpenCl`.
- File naming: `GPSOpenCl<ModuleName>.h` / `.cpp`.

## Commenting Rules

- **C++ headers (`.h`):** Doxygen only. `/** */` for classes, structs, functions. `///<` for fields. No `//` or `/* */`.
- **C++ sources (`.cpp`):** Zero comments.
- **Python (`.py`):** One `#` comment on the preceding line. No docstrings, no inline comments.
- **CMake (`CMakeLists.txt`):** One `#` comment on the preceding line.
- **OpenCL (`.cl`):** Doxygen `/** */` blocks on every kernel.
- Simple English. Short, direct, no filler.

## Working Rules

- Never fake or stub algorithm correctness.
- Verify PLL/DLL and PVT math against references.
- Prefer measured profiling over assumed performance.
- Ask before adding a new dependency.
- GPU and CPU compute paths must be identical.
- New modules must fit the Source/struct/Sink contract.
